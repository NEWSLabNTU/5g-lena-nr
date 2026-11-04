// Copyright (c) 2026 NEWSLab, National Taiwan University
//
// SPDX-License-Identifier: GPL-2.0-only

/**
 * @file nr-handover-repro.cc
 * @brief Handover between a site's sectors on TracedChannelModel, with UDP both ways.
 *
 * --sectors co-located gNBs (bearings --gnbBearing + 360/N apart), X2 and A3-RSRP
 * handover as nr-v2x-edge; UE 0 and UE 1 drive past the site (--speed), so they cross
 * sector boundaries; each frame's paths follow their positions.
 *
 * nr-v2x-edge's radio (one site, a 4x8 3GPP panel, 100 MHz numerology 1, UEs
 * with a 1x2 isotropic array) with the traced channel fed directly: a new
 * frame every --framePeriod, each UE's path count changing from frame to
 * frame, UE 1 without paths until --lateAt (the model's "none" fallback), UE
 * 2 losing its paths every third frame. With --threaded the frames also
 * arrive from a second thread in wall-clock time, as the RT server's do.
 *
 * Before NrGnbMac dropped UL PDUs of detached UEs, heavy UL (--ulInterval=500)
 * crashed it at the first handover's context release, as nr-v2x-edge --sectors=3 did.
 *
 *   ./ns3 run "nr-handover-repro --sectors=3 --ulInterval=500"
 */

#include "ns3/antenna-module.h"
#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/nr-module.h"
#include "ns3/point-to-point-module.h"

#include "traced-channel-model.h"

#include <atomic>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <iostream>
#include <thread>

using namespace ns3;

namespace
{

std::ofstream g_tb; ///< --tbTrace
uint64_t g_dl = 0;
uint64_t g_ul = 0;
std::vector<uint64_t> g_dlUe(3, 0);
std::vector<uint64_t> g_ulUe(3, 0);
std::vector<Ptr<Node>> g_ueNodes;
double g_freq = 3.5e9;
Vector g_gnb(0, 0, 25);
std::vector<Vector> g_ues;
double g_lateAt = 1.0;
double g_allLateAt = -1; ///< no UE has paths before this
uint32_t g_numPaths = 0; ///< paths per UE (0: 1 to 5, changing per frame)
double g_cutAt = -1;  ///< UE 0 loses its paths from here (radio link failure)
double g_backAt = -1; ///< and has them again from here

void
OnRx(uint64_t* counter, Ptr<const Packet>, const Address&, const Address&)
{
    ++*counter;
}

void
OnRxUe(std::vector<uint64_t>* v, uint32_t k, Ptr<const Packet>, const Address&, const Address&)
{
    ++(*v)[k];
}

void
OnTb(std::string dir, RxPacketTraceParams p)
{
    g_tb << Simulator::Now().GetTimeStep() << "," << dir << "," << p.m_cellId << "," << p.m_rnti << ","
         << std::setprecision(17) << p.m_sinr << "," << p.m_tbler << "," << p.m_corrupt << ","
         << +p.m_mcs << "," << p.m_tbSize << "\n";
}

void
Report(Time period)
{
    std::cout << Simulator::Now().As(Time::S) << " received: DL " << g_dl << ", UL " << g_ul;
    for (size_t k = 0; k < g_ueNodes.size(); ++k)
    {
        std::cout << " | UE" << k << " " << g_dlUe[k] << "/" << g_ulUe[k];
        g_dlUe[k] = g_ulUe[k] = 0;
    }
    std::cout << std::endl;
    g_dl = g_ul = 0;
    Simulator::Schedule(period, &Report, period);
}

/// A free-space path from g to u, `lossDb` weaker, its directions turned by `spread` rad.
TracedChannelModel::Path
MakePath(Vector g, Vector u, double lossDb, double spread, double extraNs)
{
    Vector d = u - g;
    double r = d.GetLength();
    double lambda = 299792458.0 / g_freq;
    TracedChannelModel::Path p;
    p.delayNs = r / 0.299792458 + extraNs;
    p.gain = std::polar(lambda / (4 * M_PI * r) * std::pow(10.0, -lossDb / 20), spread * 7.0);
    p.aod = std::atan2(d.y, d.x) + spread;
    p.zod = std::acos(d.z / r);
    p.aoa = std::atan2(-d.y, -d.x) - spread;
    p.zoa = std::acos(-d.z / r);
    p.dopplerHz = 30.0 * spread;
    return p;
}

std::map<int, std::vector<TracedChannelModel::Path>>
MakeFrame(int64_t frame, bool late)
{
    std::map<int, std::vector<TracedChannelModel::Path>> f;
    double now = Simulator::Now().GetSeconds();
    for (size_t k = 0; k < g_ues.size(); ++k)
    {
        std::vector<TracedChannelModel::Path> paths;
        bool none = (g_allLateAt >= 0 && now < g_allLateAt) || (k == 1 && !late) || (k == 2 && frame % 3 == 0) ||
                    (k == 0 && g_cutAt >= 0 && now >= g_cutAt && now < g_backAt);
        if (!none)
        {
            int n = g_numPaths > 0 ? static_cast<int>(g_numPaths) : 1 + static_cast<int>((frame + k) % 5);
            for (int i = 0; i < n; ++i)
            {
                paths.push_back(MakePath(g_gnb, g_ueNodes[k]->GetObject<MobilityModel>()->GetPosition(), 0.5 * i, 0.03 * i, 5.0 * i));
            }
        }
        if (k != 1 || late)
        {
            f[static_cast<int>(k)] = paths; // UE 1 before lateAt: absent from the map
        }
    }
    return f;
}

void
Feed(Ptr<TracedChannelModel> m, int64_t frame, Time period)
{
    m->SetPaths(frame, MakeFrame(frame, Simulator::Now().GetSeconds() >= g_lateAt));
    Simulator::Schedule(period, &Feed, m, frame + 1, period);
}

} // namespace

int
main(int argc, char* argv[])
{
    std::string beamforming = "cellscan";
    uint32_t numUes = 3;
    uint32_t sectors = 3;
    double gnbBearing = 30;
    double speed = 20;
    bool attachFacing = true;
    bool ofdmaRr = true;
    bool skipSameKind = true;
    std::string tbTrace;
    double simTime = 4.0;
    double framePeriodMs = 50;
    bool threaded = false;
    uint32_t feedPeriodUs = 500;
    uint32_t ulIntervalUs = 5000;
    uint16_t rows = 4;
    uint16_t cols = 8;
    double bandwidth = 100e6;
    uint16_t numerology = 1;

    CommandLine cmd(__FILE__);
    cmd.AddValue("beamforming", "direct or cellscan", beamforming);
    cmd.AddValue("numUes", "UEs (1-3)", numUes);
    cmd.AddValue("sectors", "co-located gNBs", sectors);
    cmd.AddValue("gnbBearing", "first sector's bearing, deg", gnbBearing);
    cmd.AddValue("speed", "UE 0 and 1 speed, m/s", speed);
    cmd.AddValue("skipSameKind", "drop gNB-gNB and UE-UE signals (TracedSameKindFilter)", skipSameKind);
    cmd.AddValue("tbTrace", "file for every TB's time, cell, RNTI, SINR, TBLER, corrupt", tbTrace);
    cmd.AddValue("ofdmaRr", "NrMacSchedulerOfdmaRR as nr-v2x-edge (else the default)", ofdmaRr);
    cmd.AddValue("attachFacing", "attach to the sector facing each UE (else the closest gNB)", attachFacing);
    cmd.AddValue("simTime", "s", simTime);
    cmd.AddValue("framePeriod", "ms between traced frames", framePeriodMs);
    cmd.AddValue("lateAt", "when UE 1's paths first arrive, s", g_lateAt);
    cmd.AddValue("allLateAt", "no UE has paths before this, s (negative: off)", g_allLateAt);
    cmd.AddValue("numPaths", "paths per UE (0: 1 to 5, changing per frame)", g_numPaths);
    cmd.AddValue("cutAt", "UE 0 loses its paths, s (negative: never)", g_cutAt);
    cmd.AddValue("backAt", "UE 0 has its paths again, s", g_backAt);
    cmd.AddValue("ulInterval", "UL packet interval per UE, us", ulIntervalUs);
    cmd.AddValue("threaded", "also feed frames from a second thread", threaded);
    cmd.AddValue("feedPeriodUs", "wall-clock period of the second thread's frames, us", feedPeriodUs);
    cmd.AddValue("gnbRows", "panel rows", rows);
    cmd.AddValue("gnbCols", "panel columns", cols);
    cmd.AddValue("bandwidth", "Hz", bandwidth);
    cmd.AddValue("numerology", "numerology", numerology);
    cmd.Parse(argc, argv);
    NS_ABORT_IF(numUes < 1 || numUes > 3);

    Config::SetDefault("ns3::NrGnbRrc::EpsBearerToRlcMapping", EnumValue(NrGnbRrc::RLC_UM_ALWAYS));
    // nr-v2x-edge's radio link failure detection.
    Config::SetDefault("ns3::NrUePhy::EnableRlfDetection", BooleanValue(true));
    Config::SetDefault("ns3::NrUePhy::Qout", DoubleValue(-5.0));
    Config::SetDefault("ns3::NrUeRrc::T310", TimeValue(MilliSeconds(1000)));
    Config::SetDefault("ns3::NrUeRrc::N310", UintegerValue(1));
    Config::SetDefault("ns3::NrUeRrc::N311", UintegerValue(1));

    NodeContainer gnbNodes;
    gnbNodes.Create(sectors);
    NodeContainer ueNodes;
    ueNodes.Create(numUes);
    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(gnbNodes);
    mobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    mobility.Install(ueNodes);
    for (uint32_t s = 0; s < sectors; ++s)
    {
        gnbNodes.Get(s)->GetObject<MobilityModel>()->SetPosition(g_gnb);
    }
    // UE 0 drives north past the site at x = 100, UE 1 south at x = -80, UE 2 stands.
    std::vector<Vector> all = {Vector(100, -150, 1.5), Vector(-80, 150, 1.5), Vector(-60, -90, 1.5)};
    std::vector<Vector> vel = {Vector(0, speed, 0), Vector(0, -speed, 0), Vector(0, 0, 0)};
    for (uint32_t k = 0; k < numUes; ++k)
    {
        g_ues.push_back(all[k]);
        g_ueNodes.push_back(ueNodes.Get(k));
        ueNodes.Get(k)->GetObject<MobilityModel>()->SetPosition(all[k]);
        ueNodes.Get(k)->GetObject<ConstantVelocityMobilityModel>()->SetVelocity(vel[k]);
    }

    Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper>();
    Ptr<IdealBeamformingHelper> bf = CreateObject<IdealBeamformingHelper>();
    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
    nrHelper->SetBeamformingHelper(bf);
    nrHelper->SetEpcHelper(epcHelper);
    if (ofdmaRr)
    {
        nrHelper->SetSchedulerTypeId(NrMacSchedulerOfdmaRR::GetTypeId());
    }

    CcBwpCreator ccBwpCreator;
    CcBwpCreator::SimpleOperationBandConf bandConf(g_freq, bandwidth, 1);
    OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);
    Ptr<NrChannelHelper> channelHelper = CreateObject<NrChannelHelper>();
    channelHelper->ConfigureFactories("UMa", "LOS", "ThreeGpp");
    channelHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(false));
    channelHelper->AssignChannelsToBands({band}, NrChannelHelper::INIT_FADING);
    Ptr<TracedChannelModel> traced = CreateObject<TracedChannelModel>();
    traced->SetAttribute("Frequency", DoubleValue(g_freq));
    for (auto& cc : band.m_cc)
    {
        for (auto& bwp : cc->m_bwp)
        {
            bwp->GetChannel()->GetPhasedArraySpectrumPropagationLossModel()->SetAttribute(
                "ChannelModel",
                PointerValue(traced));
            if (skipSameKind)
            {
                bwp->GetChannel()->AddSpectrumTransmitFilter(CreateObject<TracedSameKindFilter>());
            }
        }
    }
    BandwidthPartInfoPtrVector allBwps = CcBwpCreator::GetAllBwps({band});

    nrHelper->SetGnbAntennaAttribute("NumRows", UintegerValue(rows));
    nrHelper->SetGnbAntennaAttribute("NumColumns", UintegerValue(cols));
    nrHelper->SetGnbAntennaAttribute("AntennaElement",
                                     PointerValue(CreateObject<ThreeGppAntennaModel>()));
    nrHelper->SetGnbAntennaAttribute("DowntiltAngle", DoubleValue(6 * M_PI / 180.0));
    nrHelper->SetUeAntennaAttribute("NumRows", UintegerValue(1));
    nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(2));
    nrHelper->SetUeAntennaAttribute("AntennaElement",
                                    PointerValue(CreateObject<IsotropicAntennaModel>()));
    NS_ABORT_IF(beamforming != "direct" && beamforming != "cellscan");
    bf->SetAttribute("BeamformingMethod",
                     TypeIdValue(beamforming == "cellscan" ? CellScanBeamforming::GetTypeId()
                                                           : DirectPathBeamforming::GetTypeId()));
    nrHelper->SetGnbPhyAttribute("TxPower", DoubleValue(43.0));
    nrHelper->SetGnbPhyAttribute("Numerology", UintegerValue(numerology));
    nrHelper->SetUePhyAttribute("TxPower", DoubleValue(23.0));

    if (sectors > 1)
    {
        nrHelper->SetHandoverAlgorithmType("ns3::NrA3RsrpHandoverAlgorithm");
        nrHelper->SetHandoverAlgorithmAttribute("Hysteresis", DoubleValue(3.0));
        nrHelper->SetHandoverAlgorithmAttribute("TimeToTrigger", TimeValue(MilliSeconds(256)));
    }
    NetDeviceContainer gnbDevs;
    for (uint32_t s = 0; s < sectors; ++s)
    {
        nrHelper->SetGnbAntennaAttribute("BearingAngle", DoubleValue(std::remainder(gnbBearing + 360.0 * s / sectors, 360.0) * M_PI / 180));
        gnbDevs.Add(nrHelper->InstallGnbDevice(gnbNodes.Get(s), allBwps));
    }
    NetDeviceContainer ueDevs = nrHelper->InstallUeDevice(ueNodes, allBwps);

    Ptr<Node> pgw = epcHelper->GetPgwNode();
    NodeContainer remote;
    remote.Create(1);
    InternetStackHelper internet;
    internet.Install(remote);
    PointToPointHelper p2p;
    p2p.SetDeviceAttribute("DataRate", StringValue("10Gbps"));
    p2p.SetChannelAttribute("Delay", TimeValue(MicroSeconds(1)));
    NetDeviceContainer link = p2p.Install(pgw, remote.Get(0));
    Ipv4AddressHelper ip;
    ip.SetBase("1.0.0.0", "255.0.0.0");
    Ipv4InterfaceContainer linkIps = ip.Assign(link);
    Ipv4StaticRoutingHelper routing;
    routing.GetStaticRouting(remote.Get(0)->GetObject<Ipv4>())
        ->AddNetworkRouteTo(Ipv4Address("7.0.0.0"), Ipv4Mask("255.0.0.0"), 1);
    internet.Install(ueNodes);
    Ipv4InterfaceContainer ueIps = epcHelper->AssignUeIpv4Address(ueDevs);
    for (uint32_t k = 0; k < numUes; ++k)
    {
        routing.GetStaticRouting(ueNodes.Get(k)->GetObject<Ipv4>())
            ->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);
    }

    // As nr-v2x-edge: attach first (the first beamforming runs before the
    // model knows the nodes), then name the nodes and feed the frames.
    if (sectors > 1)
    {
        nrHelper->AddX2Interface(gnbNodes);
    }
    if (sectors > 1 && attachFacing)
    {
        for (uint32_t k = 0; k < numUes; ++k)
        {
            double dir = std::atan2(all[k].y - g_gnb.y, all[k].x - g_gnb.x) * 180 / M_PI;
            uint32_t best = 0;
            for (uint32_t s = 1; s < sectors; ++s)
            {
                if (std::abs(std::remainder(dir - gnbBearing - 360.0 * s / sectors, 360.0)) <
                    std::abs(std::remainder(dir - gnbBearing - 360.0 * best / sectors, 360.0)))
                {
                    best = s;
                }
            }
            std::cout << "UE " << k << " attaches to sector " << best << std::endl;
            nrHelper->AttachToGnb(ueDevs.Get(k), gnbDevs.Get(best));
        }
    }
    else
    {
        nrHelper->AttachToClosestGnb(ueDevs, gnbDevs);
    }
    std::vector<uint32_t> gnbIds;
    for (uint32_t s = 0; s < sectors; ++s)
    {
        gnbIds.push_back(gnbNodes.Get(s)->GetId());
    }
    std::vector<uint32_t> ueIds;
    for (uint32_t k = 0; k < numUes; ++k)
    {
        ueIds.push_back(ueNodes.Get(k)->GetId());
    }
    traced->SetNodes(gnbIds, ueIds, false);
    Simulator::Schedule(MilliSeconds(1), &Feed, traced, 1, MilliSeconds(framePeriodMs));

    std::atomic<bool> stop{false};
    std::thread feeder;
    if (threaded)
    {
        feeder = std::thread([&]() {
            int64_t frame = 1000000;
            while (!stop)
            {
                traced->SetPaths(frame, MakeFrame(frame, true));
                ++frame;
                std::this_thread::sleep_for(std::chrono::microseconds(feedPeriodUs));
            }
        });
    }

    uint16_t port = 1234;
    ApplicationContainer sinks;
    ApplicationContainer sources;
    for (uint32_t k = 0; k < numUes; ++k)
    {
        UdpServerHelper dlSink(port + 2 * k);
        UdpServerHelper ulSink(port + 2 * k + 1);
        sinks.Add(dlSink.Install(ueNodes.Get(k)));
        sinks.Add(ulSink.Install(remote.Get(0)));
        sinks.Get(2 * k)->TraceConnectWithoutContext("RxWithAddresses",
                                                     MakeBoundCallback(&OnRx, &g_dl));
        sinks.Get(2 * k + 1)->TraceConnectWithoutContext("RxWithAddresses",
                                                         MakeBoundCallback(&OnRx, &g_ul));
        sinks.Get(2 * k)->TraceConnectWithoutContext("RxWithAddresses",
                                                     MakeBoundCallback(&OnRxUe, &g_dlUe, k));
        sinks.Get(2 * k + 1)->TraceConnectWithoutContext("RxWithAddresses",
                                                         MakeBoundCallback(&OnRxUe, &g_ulUe, k));
        UdpClientHelper dl(ueIps.GetAddress(k), port + 2 * k);
        dl.SetAttribute("Interval", TimeValue(MilliSeconds(5)));
        dl.SetAttribute("MaxPackets", UintegerValue(1000000));
        dl.SetAttribute("PacketSize", UintegerValue(1000));
        UdpClientHelper ul(linkIps.GetAddress(1), port + 2 * k + 1);
        ul.SetAttribute("Interval", TimeValue(MicroSeconds(ulIntervalUs)));
        ul.SetAttribute("MaxPackets", UintegerValue(1000000));
        ul.SetAttribute("PacketSize", UintegerValue(1000));
        sources.Add(dl.Install(remote.Get(0)));
        sources.Add(ul.Install(ueNodes.Get(k)));
    }
    sinks.Start(Seconds(0.3));
    sources.Start(Seconds(0.5));

    Config::Connect("/NodeList/*/DeviceList/*/$ns3::NrUeNetDevice/NrUeRrc/RadioLinkFailure",
                    MakeCallback(+[](std::string, uint64_t imsi, uint16_t cell, uint16_t rnti) {
                        std::cout << Simulator::Now().As(Time::S) << " IMSI " << imsi
                                  << " RNTI " << rnti << ": radio link failure" << std::endl;
                    }));
    Config::Connect("/NodeList/*/DeviceList/*/$ns3::NrUeNetDevice/NrUeRrc/StateTransition",
                    MakeCallback(+[](std::string, uint64_t imsi, uint16_t cell, uint16_t rnti,
                                     NrUeRrc::State from, NrUeRrc::State to) {
                        std::cout << Simulator::Now().As(Time::S) << " IMSI " << imsi << " cell "
                                  << cell << " RNTI " << rnti << ": " << NrUeRrc::ToString(from)
                                  << " -> " << NrUeRrc::ToString(to) << std::endl;
                    }));
    Config::Connect("/NodeList/*/DeviceList/*/$ns3::NrUeNetDevice/NrUeRrc/HandoverStart",
                    MakeCallback(+[](std::string, uint64_t imsi, uint16_t cell, uint16_t rnti, uint16_t target) {
                        std::cout << Simulator::Now().As(Time::S) << " IMSI " << imsi << " RNTI "
                                  << rnti << ": handover " << cell << " -> " << target << std::endl;
                    }));
    Config::Connect("/NodeList/*/DeviceList/*/$ns3::NrUeNetDevice/NrUeRrc/HandoverEndOk",
                    MakeCallback(+[](std::string, uint64_t imsi, uint16_t cell, uint16_t rnti) {
                        std::cout << Simulator::Now().As(Time::S) << " IMSI " << imsi << " RNTI "
                                  << rnti << ": handover done, cell " << cell << std::endl;
                    }));
    if (!tbTrace.empty())
    {
        g_tb.open(tbTrace);
        for (uint32_t s = 0; s < sectors; ++s)
        {
            DynamicCast<NrGnbNetDevice>(gnbDevs.Get(s))->GetPhy(0)->GetSpectrumPhy()->TraceConnectWithoutContext(
                "RxPacketTraceGnb", MakeBoundCallback(&OnTb, std::string("UL")));
        }
        for (uint32_t k = 0; k < numUes; ++k)
        {
            DynamicCast<NrUeNetDevice>(ueDevs.Get(k))->GetPhy(0)->GetSpectrumPhy()->TraceConnectWithoutContext(
                "RxPacketTraceUe", MakeBoundCallback(&OnTb, std::string("DL")));
        }
    }
    Simulator::Schedule(MilliSeconds(500), &Report, MilliSeconds(500));
    Simulator::Stop(Seconds(simTime));
    Simulator::Run();
    stop = true;
    if (feeder.joinable())
    {
        feeder.join();
    }
    Simulator::Destroy();
    std::cout << "done" << std::endl;
    return 0;
}
