// Copyright (c) 2026 NEWSLab, National Taiwan University
//
// SPDX-License-Identifier: GPL-2.0-only

#include "traced-channel-model.h"

#include "ns3/double.h"
#include "ns3/log.h"
#include "ns3/mobility-model.h"
#include "ns3/node.h"
#include "ns3/simulator.h"
#include "ns3/uinteger.h"

#include <arpa/inet.h>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("TracedChannelModel");
NS_OBJECT_ENSURE_REGISTERED(TracedChannelModel);

std::atomic<int64_t> g_tracedChannelFrame{-1};

namespace
{
const double kLight = 3e8; ///< as ThreeGppSpectrumPropagationLossModel's Doppler term

template <typename T>
T
Read(const uint8_t* p)
{
    T v;
    std::memcpy(&v, p, sizeof(T)); // little-endian hosts only
    return v;
}
} // namespace

TypeId
TracedChannelModel::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::TracedChannelModel")
            .SetParent<MatrixBasedChannelModel>()
            .SetGroupName("Nr")
            .AddConstructor<TracedChannelModel>()
            .AddAttribute("Port",
                          "UDP port on which the RT server's paths arrive",
                          UintegerValue(5601),
                          MakeUintegerAccessor(&TracedChannelModel::m_port),
                          MakeUintegerChecker<uint16_t>())
            .AddAttribute("Frequency",
                          "Carrier frequency, Hz (the traced band's)",
                          DoubleValue(3.5e9),
                          MakeDoubleAccessor(&TracedChannelModel::m_frequency),
                          MakeDoubleChecker<double>(0.0));
    return tid;
}

TracedChannelModel::TracedChannelModel()
    : m_port(5601),
      m_frequency(3.5e9),
      m_socket(-1),
      m_stop(false),
      m_gnbNode(0),
      m_frame(-1),
      m_pendingFrame(-1),
      m_cachedFrame(-2)
{
}

TracedChannelModel::~TracedChannelModel()
{
    DoDispose();
}

void
TracedChannelModel::DoDispose()
{
    m_stop = true;
    if (m_socket >= 0)
    {
        shutdown(m_socket, SHUT_RDWR);
    }
    if (m_thread.joinable())
    {
        m_thread.join();
    }
    if (m_socket >= 0)
    {
        close(m_socket);
        m_socket = -1;
    }
    MatrixBasedChannelModel::DoDispose();
}

void
TracedChannelModel::SetPaths(int64_t frame, const std::map<int, std::vector<Path>>& paths)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_paths = paths;
    m_frame = frame;
    g_tracedChannelFrame = frame;
}

void
TracedChannelModel::SetNodes(uint32_t gnbNode, const std::vector<uint32_t>& ueNodes, bool listen)
{
    m_gnbNode = gnbNode;
    for (size_t k = 0; k < ueNodes.size(); ++k)
    {
        m_ueByNode[ueNodes[k]] = static_cast<int>(k);
    }
    if (listen && m_socket < 0)
    {
        m_socket = socket(AF_INET, SOCK_DGRAM, 0);
        NS_ABORT_MSG_IF(m_socket < 0, "socket: " << std::strerror(errno));
        int big = 8 << 20;
        setsockopt(m_socket, SOL_SOCKET, SO_RCVBUF, &big, sizeof(big));
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(m_port);
        addr.sin_addr.s_addr = htonl(INADDR_ANY);
        NS_ABORT_MSG_IF(bind(m_socket, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0,
                        "bind to UDP port " << m_port << ": " << std::strerror(errno));
        m_thread = std::thread(&TracedChannelModel::Listen, this);
    }
}

void
TracedChannelModel::Listen()
{
    timeval tv{0, 200000}; // wake up to notice m_stop
    setsockopt(m_socket, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    std::vector<uint8_t> buf(65536);
    while (!m_stop)
    {
        ssize_t n = recv(m_socket, buf.data(), buf.size(), 0);
        if (n < 12)
        {
            continue;
        }
        const uint8_t* b = buf.data();
        uint64_t frame = Read<uint64_t>(b + 4);
        std::lock_guard<std::mutex> lock(m_mutex);
        if (std::memcmp(b, "RT5P", 4) == 0 && n >= 24)
        {
            uint64_t tNs = Read<uint64_t>(b + 12);
            uint16_t ue = Read<uint16_t>(b + 20);
            uint16_t count = Read<uint16_t>(b + 22);
            if (n < static_cast<ssize_t>(24 + 32 * count))
            {
                continue;
            }
            if (static_cast<int64_t>(frame) != m_pendingFrame)
            {
                m_pending.clear();
                m_pendingFrame = static_cast<int64_t>(frame);
                (void)tNs; // the world's time; ns-3 takes its own when it uses the frame
            }
            std::vector<Path>& paths = m_pending[ue];
            paths.clear();
            for (uint16_t k = 0; k < count; ++k)
            {
                const uint8_t* p = b + 24 + 32 * k;
                Path path;
                path.delayNs = Read<float>(p);
                path.gain = {Read<float>(p + 4), Read<float>(p + 8)};
                path.aod = Read<float>(p + 12);
                path.zod = Read<float>(p + 16);
                path.aoa = Read<float>(p + 20);
                path.zoa = Read<float>(p + 24);
                path.dopplerHz = Read<float>(p + 28);
                paths.push_back(path);
            }
        }
        else if (std::memcmp(b, "RT5E", 4) == 0 && static_cast<int64_t>(frame) == m_pendingFrame)
        {
            m_paths.swap(m_pending);
            m_pending.clear();
            m_frame = m_pendingFrame;
            g_tracedChannelFrame = m_frame;
        }
    }
}

int
TracedChannelModel::UeOf(uint32_t node) const
{
    auto it = m_ueByNode.find(node);
    return it == m_ueByNode.end() ? -1 : it->second;
}

Ptr<const MatrixBasedChannelModel::ChannelMatrix>
TracedChannelModel::GetChannel(Ptr<const MobilityModel> aMob,
                               Ptr<const MobilityModel> bMob,
                               Ptr<const PhasedArrayModel> aAntenna,
                               Ptr<const PhasedArrayModel> bAntenna)
{
    uint32_t aNode = aMob->GetObject<Node>()->GetId();
    uint32_t bNode = bMob->GetObject<Node>()->GetId();
    uint64_t key = GetKey(aNode, bNode);
    std::lock_guard<std::mutex> lock(m_mutex);
    if (m_cachedFrame != m_frame)
    {
        m_matrices.clear();
        m_params.clear();
        m_cachedFrame = m_frame;
        // The phase reference of this frame's gains: when ns-3 first uses them.
        m_frameTime = Simulator::Now();
    }
    auto cached = m_matrices.find(key);
    if (cached != m_matrices.end())
    {
        return cached->second; // reciprocal: the same matrix serves both directions
    }
    int ue = UeOf(aNode) >= 0 ? UeOf(aNode) : UeOf(bNode);
    bool aIsGnb = aNode == m_gnbNode;
    std::vector<Path> paths;
    if (ue >= 0 && (aIsGnb || bNode == m_gnbNode))
    {
        auto it = m_paths.find(ue);
        if (it != m_paths.end())
        {
            paths = it->second;
        }
    }
    size_t uSize = bAntenna->GetNumElems();
    size_t sSize = aAntenna->GetNumElems();
    size_t numClusters = std::max<size_t>(paths.size(), 1);
    Complex3DVector h(uSize, sSize, numClusters);
    Ptr<ChannelParams> params = Create<ChannelParams>();
    params->m_generatedTime = m_frameTime;
    params->m_nodeIds = std::make_pair(aNode, bNode);
    params->m_angle.assign(4, DoubleVector(numClusters, 0.0));
    params->m_cachedAngleSincos.assign(4,
                                       std::vector<std::pair<double, double>>(numClusters, {0, 1}));
    params->m_delay.assign(numClusters, 0.0);
    params->m_alpha.assign(numClusters, 0.0);
    params->m_D.assign(numClusters, 0.0);
    if (paths.empty())
    {
        // No path: one cluster 300 dB down. A zero channel makes the spectrum
        // model's beamforming normalisation divide 0 by 0, and the NaN SINR it
        // gives would read as in sync.
        Path none;
        none.delayNs = 0;
        none.gain = 1e-15;
        none.aod = none.zod = none.aoa = none.zoa = 0;
        none.dopplerHz = 0;
        paths.push_back(none);
    }
    double t = m_frameTime.GetSeconds();
    for (size_t n = 0; n < paths.size(); ++n)
    {
        const Path& p = paths[n];
        // s = a, u = b: departure at a, arrival at b. Gains and directions are the
        // gNB's to the UE's; on the uplink the UE departs along its arrival
        // direction and the gNB receives along its departure direction.
        double sAz = aIsGnb ? p.aod : p.aoa;
        double sZen = aIsGnb ? p.zod : p.zoa;
        double uAz = aIsGnb ? p.aoa : p.aod;
        double uZen = aIsGnb ? p.zoa : p.zod;
        // The spectrum model adds the phase 2 pi fD t; at the frame's time the
        // phase must be the traced one.
        std::complex<double> g = p.gain * std::polar(1.0, -2 * M_PI * p.dopplerHz * t);
        Angles sAngle(sAz, sZen);
        Angles uAngle(uAz, uZen);
        for (size_t u = 0; u < uSize; ++u)
        {
            Vector uLoc = bAntenna->GetElementLocation(u);
            double rxPhase = 2 * M_PI *
                             (std::sin(uZen) * std::cos(uAz) * uLoc.x +
                              std::sin(uZen) * std::sin(uAz) * uLoc.y + std::cos(uZen) * uLoc.z);
            auto [rxPhi, rxTheta] =
                bAntenna->GetElementFieldPattern(uAngle, bAntenna->GetElemPol(u));
            for (size_t s = 0; s < sSize; ++s)
            {
                Vector sLoc = aAntenna->GetElementLocation(s);
                double txPhase =
                    2 * M_PI *
                    (std::sin(sZen) * std::cos(sAz) * sLoc.x +
                     std::sin(sZen) * std::sin(sAz) * sLoc.y + std::cos(sZen) * sLoc.z);
                auto [txPhi, txTheta] =
                    aAntenna->GetElementFieldPattern(sAngle, aAntenna->GetElemPol(s));
                h(u, s, n) = (rxTheta * txTheta - rxPhi * txPhi) * g *
                             std::polar(1.0, rxPhase) * std::polar(1.0, txPhase);
            }
        }
        params->m_delay[n] = p.delayNs;
        double angles[4] = {uAz, uZen, sAz, sZen}; // AOA, ZOA, AOD, ZOD of s -> u
        for (int i = 0; i < 4; ++i)
        {
            params->m_angle[i][n] = angles[i] * 180.0 / M_PI;
            params->m_cachedAngleSincos[i][n] = {std::sin(angles[i]), std::cos(angles[i])};
        }
        // The spectrum model's Doppler phase is 2 pi t f / c (... + 2 alpha D):
        // with the nodes still, alpha = 1 and D = fD c / (2 f) give 2 pi fD t.
        params->m_alpha[n] = 1.0;
        params->m_D[n] = p.dopplerHz * kLight / (2 * m_frequency);
    }
    Ptr<ChannelMatrix> m = Create<ChannelMatrix>();
    m->m_channel = h;
    m->m_generatedTime = m_frameTime;
    m->m_nodeIds = std::make_pair(aNode, bNode);
    m->m_antennaPair = std::make_pair(aAntenna->GetId(), bAntenna->GetId());
    m_matrices[key] = m;
    m_params[key] = params;
    return m;
}

Ptr<const MatrixBasedChannelModel::ChannelParams>
TracedChannelModel::GetParams(Ptr<const MobilityModel> aMob, Ptr<const MobilityModel> bMob) const
{
    uint64_t key = GetKey(aMob->GetObject<Node>()->GetId(), bMob->GetObject<Node>()->GetId());
    std::lock_guard<std::mutex> lock(m_mutex);
    auto it = m_params.find(key);
    NS_ABORT_MSG_IF(it == m_params.end(), "GetParams before GetChannel for this pair");
    return it->second;
}

} // namespace ns3
