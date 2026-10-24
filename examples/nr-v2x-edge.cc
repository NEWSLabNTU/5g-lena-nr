// Copyright (c) 2026 NEWSLab, National Taiwan University
//
// SPDX-License-Identifier: GPL-2.0-only

/**
 * @file nr-v2x-edge.cc
 * @brief One gNB, N tapped UEs and one tapped edge host, in real time.
 *
 * The radio of the SEPS 5G V2X emulation (crdt-tensor, phase 021 R1). Each UE
 * stands for one vehicle's modem and the edge host for the RSU at the cell
 * site. Every UE and the edge host are joined to an external Linux host (a
 * container's interface) through a TapBridge in UseBridge mode:
 *
 *   vehicle k wwan0 -- br_ue_k -- tap_ue_k -- ghost -- CSMA -- UE k
 *     UE k ~~ NR Uu ~~ gNB -- core (PGW/UPF) -- P2P -- edge server
 *   edge server -- CSMA -- ghost -- tap_edge -- br_edge -- RSU bh0
 *
 * Vehicle k sits on 7.0.(k+1).0/24 with UE k's CSMA side at .1 as its
 * gateway; the RSU sits on 10.1.1.0/24 with the edge server at .1. Packets
 * between a vehicle and the RSU cross the NR network inside the IP-in-UDP
 * tunnel of UeTunnelApp and EdgeTunnelApp, which keeps the core's GTP filter
 * (it drops packets whose source is not a UE address) out of the way.
 * Vehicles never reach each other directly; they meet at the edge.
 *
 * With --simClock, ns-3 runs on the external simulator's clock, fed as UDP
 * ticks (SimClockSynchronizer, phase 022 R2), instead of the wall clock.
 *
 * UE positions are fixed in R1 (uePositions); phase 022 sets them from
 * CARLA through the radio bridge. Status lines go to stdout, since NS_LOG is
 * compiled out of optimized builds.
 */

#include "edge-tunnel-app.h"
#include "external-channel-condition-model.h"
#include "sim-clock-synchronizer.h"
#include "ue-tunnel-app.h"

#include "ns3/antenna-module.h"
#include "ns3/core-module.h"
#include "ns3/csma-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/nr-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/tap-bridge-module.h"

#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("NrV2xEdge");

namespace
{

/// Move each UE to the position the radio bridge last sent (phase 022 R3) and
/// set its link's condition (R4), every period of simulation time.
void
ApplyPositions(Ptr<SimClockSynchronizer> clock, NodeContainer ues, Time period, uint64_t seen)
{
    std::vector<Vector> p;
    uint64_t tick = clock->GetPositions(p);
    std::vector<uint8_t> c;
    clock->GetConditions(c);
    for (uint32_t k = 0; k < ues.GetN() && k < c.size(); ++k)
    {
        static const ChannelCondition::LosConditionValue value[] = {ChannelCondition::LOS,
                                                                   ChannelCondition::NLOS,
                                                                   ChannelCondition::NLOSv};
        if (c[k] < 3)
        {
            ExternalChannelConditionModel::SetCondition(ues.Get(k)->GetId(), value[c[k]]);
        }
    }
    if (tick != seen)
    {
        for (uint32_t k = 0; k < ues.GetN() && k < p.size(); ++k)
        {
            if (std::isfinite(p[k].x) && std::isfinite(p[k].y) && std::isfinite(p[k].z))
            {
                ues.Get(k)->GetObject<MobilityModel>()->SetPosition(p[k]);
            }
        }
    }
    Simulator::Schedule(period, &ApplyPositions, clock, ues, period, tick);
}

/// Each UE's radio between two trace rows, and its state in the reattach model.
struct UeRadio
{
    double ctrlSinrSum = 0;   ///< linear, over the DL control SINR reports
    uint32_t ctrlSinrN = 0;
    double lossSum = 0;       ///< dB, DL control coupling loss (path loss less gains)
    uint32_t lossN = 0;
    double lastLoss = NAN;    ///< dB, the latest coupling loss
    Time belowSince = Time::Max(); ///< since when the link quality is below Qout
    Time goodSince = Time::Max();  ///< out of service: since when it is back above Qout
    bool outOfService = false;
    uint32_t failures = 0;
    Ptr<UeTunnelApp> tunnel;
};

std::vector<UeRadio> g_radio;
std::map<uint32_t, uint32_t> g_ueByNode; ///< node id -> UE index
double g_qoutDb = -8.0;
Time g_t310 = Seconds(1);
Time g_reattach = Seconds(0); ///< zero: the model is off
double g_ueTxPowerDbm = 23.0;
double g_ulNoiseDbm = -174.0 + 10 * std::log10(360e3) + 5.0; ///< thermal over one RB, gNB NF 5 dB

/// The uplink's SNR in one resource block, estimated from the DL coupling loss
/// by reciprocity (UE power less loss less noise): what the UE has for a
/// scheduling request or a small grant, with all its power in that block. The
/// uplink is the weaker direction (23 dBm against the gNB's 30 dBm), and ns-3
/// reports no uplink SINR when the UE sends nothing.
double
UlSnrEstimateDb(const UeRadio& r)
{
    return g_ueTxPowerDbm - r.lastLoss - g_ulNoiseDbm;
}

/// While out of service, every 10 ms: back in service once the link has been
/// good again for the reattach time (cell found again, then RRC set-up).
void
Reattach(uint32_t k)
{
    UeRadio& r = g_radio[k];
    if (r.goodSince == Time::Max() || Simulator::Now() - r.goodSince < g_reattach)
    {
        Simulator::Schedule(MilliSeconds(10), &Reattach, k);
        return;
    }
    r.outOfService = false;
    r.tunnel->SetOutOfService(false);
}

/// DL control SINR (linear), every slot: the trace, and radio link monitoring
/// for the reattach model. A UE whose link quality, the weaker of the DL
/// control SINR and the estimated UL SNR, stays below Qout for T310 declares a
/// radio link failure and is out of service until the link has been good again
/// for the reattach time (own simplification of TS 38.331's N310/T310, of
/// uplink failure through scheduling requests and random access, and of RRC
/// re-establishment).
void
OnCtrlSinr(uint32_t k, uint16_t, uint16_t, double sinr, uint16_t)
{
    UeRadio& r = g_radio[k];
    r.ctrlSinrSum += sinr;
    ++r.ctrlSinrN;
    Time now = Simulator::Now();
    double quality = 10 * std::log10(sinr);
    if (std::isfinite(r.lastLoss))
    {
        quality = std::min(quality, UlSnrEstimateDb(r));
    }
    if (quality >= g_qoutDb)
    {
        r.belowSince = Time::Max();
        if (r.outOfService && r.goodSince == Time::Max())
        {
            r.goodSince = now;
        }
        return;
    }
    r.goodSince = Time::Max();
    if (r.belowSince == Time::Max())
    {
        r.belowSince = now;
    }
    if (g_reattach.IsStrictlyPositive() && !r.outOfService && now - r.belowSince >= g_t310)
    {
        r.outOfService = true;
        ++r.failures;
        r.tunnel->SetOutOfService(true);
        Simulator::Schedule(MilliSeconds(10), &Reattach, k);
    }
}

void
OnCtrlLoss(uint16_t, uint8_t, uint32_t node, double lossDb)
{
    auto it = g_ueByNode.find(node);
    if (it != g_ueByNode.end())
    {
        g_radio[it->second].lossSum += lossDb;
        ++g_radio[it->second].lossN;
        g_radio[it->second].lastLoss = lossDb;
    }
}

/// One row per UE every period of simulation time: ns-3's time, the world's
/// (the last tick's, relative to the first), the UE's position and link
/// condition, its distance to the gNB, mean DL control SINR and coupling loss
/// over the period, the UL SNR that loss implies, its service state, failures
/// and drops.
void
TraceRadio(std::ofstream* out,
           Ptr<SimClockSynchronizer> clock,
           NodeContainer ues,
           Vector gnb,
           Time period)
{
    double world = clock ? clock->GetLastTick() / 1e9 : NAN;
    for (uint32_t k = 0; k < ues.GetN(); ++k)
    {
        UeRadio& r = g_radio[k];
        Vector p = ues.Get(k)->GetObject<MobilityModel>()->GetPosition();
        int cond = ExternalChannelConditionModel::GetCondition(ues.Get(k)->GetId());
        double sinrDb = r.ctrlSinrN ? 10 * std::log10(r.ctrlSinrSum / r.ctrlSinrN) : NAN;
        double loss = r.lossN ? r.lossSum / r.lossN : NAN;
        double ulSnr = g_ueTxPowerDbm - loss - g_ulNoiseDbm;
        *out << Simulator::Now().GetSeconds() << "," << world << "," << k << "," << p.x << ","
             << p.y << "," << p.z << "," << cond << "," << CalculateDistance(p, gnb) << ","
             << sinrDb << "," << loss << "," << ulSnr << "," << (r.outOfService ? 0 : 1) << "," << r.failures
             << "," << r.tunnel->GetOutOfServiceDrops() << "\n";
        r.ctrlSinrSum = 0;
        r.ctrlSinrN = 0;
        r.lossSum = 0;
        r.lossN = 0;
    }
    out->flush();
    Simulator::Schedule(period, &TraceRadio, out, clock, ues, gnb, period);
}

/// Parse "x,y;x,y;..." into ground positions at height z.
std::vector<Vector>
ParsePositions(const std::string& s, double z)
{
    std::vector<Vector> out;
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, ';'))
    {
        if (item.empty())
        {
            continue;
        }
        double x = 0;
        double y = 0;
        char comma = 0;
        std::stringstream is(item);
        is >> x >> comma >> y;
        NS_ABORT_MSG_IF(comma != ',' || is.fail(), "bad position '" << item << "'");
        out.emplace_back(x, y, z);
    }
    return out;
}

} // namespace

int
main(int argc, char* argv[])
{
    uint32_t numUes = 2;
    std::string uePositions = "";
    double ueHeight = 1.5;
    double gnbX = 0.0;
    double gnbY = 0.0;
    double gnbHeight = 6.0;
    std::string scenario = "UMi";
    std::string condition = "LOS";
    double centralFrequency = 3.5e9;
    double bandwidth = 20e6;
    uint16_t numerology = 1;
    double gnbTxPower = 30.0;
    double ueTxPower = 23.0;
    double simTime = 86400.0;
    std::string tapUePrefix = "tap_ue";
    std::string tapEdge = "tap_edge";
    bool realtime = true;
    bool simClock = false;
    bool rlfDetection = false;
    uint16_t simClockPort = 5600;
    uint32_t rngRun = 1;
    std::string traceFile = "";
    double tracePeriod = 0.1;
    double reattachTime = 0.0;
    double qout = -8.0;
    double t310 = 1.0;

    CommandLine cmd(__FILE__);
    cmd.AddValue("numUes", "Number of UEs, each tapped as <tapUePrefix><k>", numUes);
    cmd.AddValue("uePositions", "UE ground positions 'x,y;x,y;...' in m (default: a 50 m ring)", uePositions);
    cmd.AddValue("ueHeight", "UE antenna height in m", ueHeight);
    cmd.AddValue("gnbX", "gNB x in m", gnbX);
    cmd.AddValue("gnbY", "gNB y in m", gnbY);
    cmd.AddValue("gnbHeight", "gNB antenna height in m", gnbHeight);
    cmd.AddValue("scenario", "3GPP TR 38.901 scenario (UMi, UMa, RMa)", scenario);
    cmd.AddValue("condition",
                 "Channel condition: LOS, NLOS, Default (3GPP draws it) or External (the radio "
                 "bridge's ray casts; needs --simClock)",
                 condition);
    cmd.AddValue("centralFrequency", "Central frequency in Hz", centralFrequency);
    cmd.AddValue("bandwidth", "Bandwidth in Hz", bandwidth);
    cmd.AddValue("numerology", "NR numerology (0-4)", numerology);
    cmd.AddValue("gnbTxPower", "gNB TX power in dBm", gnbTxPower);
    cmd.AddValue("ueTxPower", "UE TX power in dBm (23: power class 3; ns-3's own default is 2)", ueTxPower);
    cmd.AddValue("simTime", "Simulation time in s", simTime);
    cmd.AddValue("tapUePrefix", "TAP device name prefix for the UEs", tapUePrefix);
    cmd.AddValue("tapEdge", "TAP device name for the edge host", tapEdge);
    cmd.AddValue("realtime", "Run in real time (needed with TAPs)", realtime);
    cmd.AddValue("simClock", "Follow the external simulator's clock (ticks over UDP) instead of the wall clock", simClock);
    cmd.AddValue("simClockPort", "UDP port of the external clock's ticks", simClockPort);
    cmd.AddValue("rlfDetection",
                 "Detect radio link failures; ns-3's UE then stays detached, where a real modem "
                 "would reattach, so it is off by default and a lost link comes back with the signal",
                 rlfDetection);
    cmd.AddValue("rngRun", "RNG run number", rngRun);
    cmd.AddValue("traceFile", "CSV of each UE's radio every tracePeriod of simulation time (empty: none)", traceFile);
    cmd.AddValue("tracePeriod", "Trace period in s of simulation time", tracePeriod);
    cmd.AddValue("reattachTime",
                 "Reattach model: a UE whose link quality (DL control SINR, estimated UL SNR in one "
                 "RB) stays below qout for t310 is out of service until it has been above qout "
                 "again for this many s (0: off)",
                 reattachTime);
    cmd.AddValue("qout", "Reattach model: out-of-sync threshold on the DL control SINR in dB", qout);
    cmd.AddValue("t310", "Reattach model: time below qout before a radio link failure, in s", t310);
    cmd.Parse(argc, argv);

    // The simulator implementation must be chosen before any ns-3 object exists.
    if (realtime)
    {
        GlobalValue::Bind("SimulatorImplementationType", StringValue("ns3::RealtimeSimulatorImpl"));
        Config::SetDefault("ns3::RealtimeSimulatorImpl::SynchronizationMode", StringValue("HardLimit"));
        Config::SetDefault("ns3::RealtimeSimulatorImpl::HardLimit", TimeValue(MilliSeconds(500)));
        if (simClock)
        {
            // Needs the SynchronizerType attribute of NEWSLabNTU/ns-3-dev-git,
            // branch seps-sim-clock.
            Config::SetDefault("ns3::SimClockSynchronizer::Port", UintegerValue(simClockPort));
            Config::SetDefault("ns3::RealtimeSimulatorImpl::SynchronizerType",
                               TypeIdValue(SimClockSynchronizer::GetTypeId()));
        }
    }
    GlobalValue::Bind("ChecksumEnabled", BooleanValue(true));
    RngSeedManager::SetRun(rngRun);

    std::vector<Vector> positions = ParsePositions(uePositions, ueHeight);
    if (positions.empty())
    {
        for (uint32_t k = 0; k < numUes; ++k)
        {
            double a = 2.0 * M_PI * k / numUes;
            positions.emplace_back(gnbX + 50.0 * std::cos(a), gnbY + 50.0 * std::sin(a), ueHeight);
        }
    }
    NS_ABORT_MSG_IF(positions.size() != numUes,
                    "uePositions has " << positions.size() << " entries for " << numUes << " UEs");

    //--------------------------------------------------------------------------
    // Nodes and positions
    //--------------------------------------------------------------------------
    NodeContainer gnbNodes;
    gnbNodes.Create(1);
    NodeContainer ueNodes;
    ueNodes.Create(numUes);

    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(gnbNodes);
    gnbNodes.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(gnbX, gnbY, gnbHeight));
    mobility.Install(ueNodes);
    for (uint32_t k = 0; k < numUes; ++k)
    {
        ueNodes.Get(k)->GetObject<MobilityModel>()->SetPosition(positions[k]);
    }

    //--------------------------------------------------------------------------
    // NR network
    //--------------------------------------------------------------------------
    Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper>();
    Ptr<IdealBeamformingHelper> beamforming = CreateObject<IdealBeamformingHelper>();
    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
    nrHelper->SetBeamformingHelper(beamforming);
    nrHelper->SetEpcHelper(epcHelper);
    nrHelper->SetSchedulerTypeId(NrMacSchedulerOfdmaRR::GetTypeId());

    CcBwpCreator ccBwpCreator;
    CcBwpCreator::SimpleOperationBandConf bandConf(centralFrequency, bandwidth, 1);
    OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);

    Ptr<NrChannelHelper> channelHelper = CreateObject<NrChannelHelper>();
    bool external = condition == "External";
    NS_ABORT_MSG_IF(external && !simClock, "--condition=External needs --simClock");
    channelHelper->ConfigureFactories(scenario, external ? "Default" : condition, "ThreeGpp");
    if (external)
    {
        channelHelper->ConfigureChannelConditionFactory(ExternalChannelConditionModel::GetTypeId());
    }
    channelHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(false));
    channelHelper->AssignChannelsToBands({band});
    BandwidthPartInfoPtrVector allBwps = CcBwpCreator::GetAllBwps({band});

    nrHelper->SetGnbAntennaAttribute("NumRows", UintegerValue(4));
    nrHelper->SetGnbAntennaAttribute("NumColumns", UintegerValue(4));
    nrHelper->SetGnbAntennaAttribute("AntennaElement", PointerValue(CreateObject<IsotropicAntennaModel>()));
    nrHelper->SetUeAntennaAttribute("NumRows", UintegerValue(1));
    nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(2));
    nrHelper->SetUeAntennaAttribute("AntennaElement", PointerValue(CreateObject<IsotropicAntennaModel>()));
    beamforming->SetAttribute("BeamformingMethod", TypeIdValue(DirectPathBeamforming::GetTypeId()));

    nrHelper->SetGnbPhyAttribute("TxPower", DoubleValue(gnbTxPower));
    nrHelper->SetGnbPhyAttribute("Numerology", UintegerValue(numerology));
    nrHelper->SetUePhyAttribute("TxPower", DoubleValue(ueTxPower));

    Config::SetDefault("ns3::NrUePhy::EnableRlfDetection", BooleanValue(rlfDetection));

    // Unacknowledged RLC: a lost transport block is lost, not delayed.
    Config::SetDefault("ns3::NrGnbRrc::EpsBearerToRlcMapping", EnumValue(NrGnbRrc::RLC_UM_ALWAYS));
    Config::SetDefault("ns3::NrRlcUm::MaxTxBufferSize", UintegerValue(1024 * 1024));

    NetDeviceContainer gnbDevs = nrHelper->InstallGnbDevice(gnbNodes, allBwps);
    NetDeviceContainer ueDevs = nrHelper->InstallUeDevice(ueNodes, allBwps);
    g_qoutDb = qout;
    g_t310 = Seconds(t310);
    g_reattach = Seconds(reattachTime);
    g_ueTxPowerDbm = ueTxPower;
    g_ulNoiseDbm = -174.0 + 10 * std::log10(12 * 15e3 * (1 << numerology)) + 5.0; // one RB
    g_radio.resize(numUes);
    for (uint32_t k = 0; k < numUes; ++k)
    {
        g_ueByNode[ueNodes.Get(k)->GetId()] = k;
        Ptr<NrUePhy> phy = DynamicCast<NrUeNetDevice>(ueDevs.Get(k))->GetPhy(0);
        phy->TraceConnectWithoutContext("DlCtrlSinr", MakeBoundCallback(&OnCtrlSinr, k));
        phy->GetSpectrumPhy()->EnableDlCtrlPathlossTrace();
        phy->GetSpectrumPhy()->TraceConnectWithoutContext("DlCtrlPathloss", MakeCallback(&OnCtrlLoss));
    }

    InternetStackHelper internet;
    internet.Install(ueNodes);
    Ipv4InterfaceContainer ueIps = epcHelper->AssignUeIpv4Address(ueDevs);
    Ipv4StaticRoutingHelper routing;
    for (uint32_t k = 0; k < numUes; ++k)
    {
        routing.GetStaticRouting(ueNodes.Get(k)->GetObject<Ipv4>())
            ->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);
    }

    //--------------------------------------------------------------------------
    // Edge server behind the PGW, and its ghost node towards the RSU's TAP
    //--------------------------------------------------------------------------
    Ptr<Node> pgw = epcHelper->GetPgwNode();
    Ptr<Node> edge = CreateObject<Node>();
    Ptr<Node> edgeGhost = CreateObject<Node>();
    internet.Install(edge);

    PointToPointHelper p2p;
    p2p.SetDeviceAttribute("DataRate", StringValue("10Gbps"));
    p2p.SetChannelAttribute("Delay", TimeValue(MicroSeconds(1)));
    NetDeviceContainer pgwEdge = p2p.Install(pgw, edge);
    Ipv4AddressHelper ip;
    ip.SetBase("10.1.0.0", "255.255.255.0");
    Ipv4InterfaceContainer pgwEdgeIps = ip.Assign(pgwEdge);
    Ipv4Address edgeIp = pgwEdgeIps.GetAddress(1);

    CsmaHelper csma;
    csma.SetChannelAttribute("DataRate", DataRateValue(DataRate("10Gbps")));
    csma.SetChannelAttribute("Delay", TimeValue(MicroSeconds(0)));
    NetDeviceContainer edgeLan = csma.Install(NodeContainer(edge, edgeGhost));
    ip.SetBase("10.1.1.0", "255.255.255.0");
    ip.Assign(edgeLan.Get(0));

    routing.GetStaticRouting(edge->GetObject<Ipv4>())->SetDefaultRoute(pgwEdgeIps.GetAddress(0), 1);
    routing.GetStaticRouting(pgw->GetObject<Ipv4>())
        ->AddNetworkRouteTo(Ipv4Address("10.1.1.0"),
                            Ipv4Mask("255.255.255.0"),
                            edgeIp,
                            pgw->GetObject<Ipv4>()->GetInterfaceForAddress(pgwEdgeIps.GetAddress(0)));

    TapBridgeHelper tap;
    tap.SetAttribute("Mode", StringValue("UseBridge"));
    tap.SetAttribute("DeviceName", StringValue(tapEdge));
    tap.Install(edgeGhost, edgeLan.Get(1));

    Ptr<EdgeTunnelApp> edgeTunnel = CreateObject<EdgeTunnelApp>();
    edgeTunnel->SetOuterDevice(pgwEdge.Get(1));
    edgeTunnel->SetInnerDevice(edgeLan.Get(0));
    edgeTunnel->SetInnerSubnet(Ipv4Address("10.1.1.0"), Ipv4Mask("255.255.255.0"));
    edgeTunnel->SetLocalPort(5000);
    edgeTunnel->SetTunnelPort(5000);
    edgeTunnel->SetEdgeNodeId(0);

    //--------------------------------------------------------------------------
    // One ghost node, CSMA link, TAP and tunnel per UE
    //--------------------------------------------------------------------------
    for (uint32_t k = 0; k < numUes; ++k)
    {
        Ptr<Node> ghost = CreateObject<Node>();
        NetDeviceContainer lan = csma.Install(NodeContainer(ueNodes.Get(k), ghost));
        std::ostringstream subnet;
        subnet << "7.0." << (k + 1) << ".0";
        ip.SetBase(subnet.str().c_str(), "255.255.255.0");
        ip.Assign(lan.Get(0));

        // The tunnel app forwards the vehicle's packets; plain forwarding on the
        // CSMA side would send them into GTP with a non-UE source and lose them.
        Ptr<Ipv4> ueIpv4 = ueNodes.Get(k)->GetObject<Ipv4>();
        ueIpv4->SetAttribute("IpForward", BooleanValue(true));
        int32_t lanIf = ueIpv4->GetInterfaceForDevice(lan.Get(0));
        ueIpv4->SetForwarding(lanIf, false);

        std::ostringstream tapName;
        tapName << tapUePrefix << k;
        tap.SetAttribute("DeviceName", StringValue(tapName.str()));
        tap.Install(ghost, lan.Get(1));

        Ptr<UeTunnelApp> ueTunnel = CreateObject<UeTunnelApp>();
        ueTunnel->SetInnerDevice(lan.Get(0));
        ueTunnel->SetClientSubnet(Ipv4Address(subnet.str().c_str()), Ipv4Mask("255.255.255.0"));
        ueTunnel->SetTunnelEndpoint(edgeIp, 5000);
        ueTunnel->SetLocalPort(5000);
        ueNodes.Get(k)->AddApplication(ueTunnel);
        g_radio[k].tunnel = ueTunnel;
        ueTunnel->SetStartTime(Seconds(1.0));
        ueTunnel->SetStopTime(Seconds(simTime));

        edgeTunnel->AddTunnelMapping(Ipv4Address(subnet.str().c_str()),
                                     Ipv4Mask("255.255.255.0"),
                                     ueIps.GetAddress(k));
        std::cout << "UE " << k << ": " << tapName.str() << ", vehicle subnet " << subnet.str()
                            << "/24, NR address " << ueIps.GetAddress(k) << ", position "
                            << positions[k] << std::endl;
    }
    edge->AddApplication(edgeTunnel);
    edgeTunnel->SetStartTime(Seconds(1.0));
    edgeTunnel->SetStopTime(Seconds(simTime));
    std::cout << "edge: " << tapEdge << ", RSU subnet 10.1.1.0/24, edge server " << edgeIp << std::endl;

    nrHelper->AttachToClosestGnb(ueDevs, gnbDevs);

    Ptr<SimClockSynchronizer> clock;
    if (simClock)
    {
        // The synchronizer the simulator created from SynchronizerType.
        PointerValue syncPtr;
        Simulator::GetImplementation()->GetAttribute("Synchronizer", syncPtr);
        clock = syncPtr.Get<SimClockSynchronizer>();
        NS_ABORT_MSG_IF(!clock, "the simulator does not run a SimClockSynchronizer");
        Simulator::Schedule(MilliSeconds(10), &ApplyPositions, clock, ueNodes, MilliSeconds(10), uint64_t{0});
    }
    std::ofstream trace;
    if (!traceFile.empty())
    {
        trace.open(traceFile);
        NS_ABORT_MSG_IF(!trace, "cannot write " << traceFile);
        trace << "t_ns3,t_world,ue,x,y,z,condition,distance_m,ctrl_sinr_db,coupling_loss_db,"
                 "ul_snr_rb_db,in_service,failures,oos_drops\n";
        Simulator::Schedule(Seconds(tracePeriod),
                            &TraceRadio,
                            &trace,
                            clock,
                            ueNodes,
                            Vector(gnbX, gnbY, gnbHeight),
                            Seconds(tracePeriod));
    }

    std::cout << "nr-v2x-edge running: " << numUes << " UEs, " << scenario << " " << condition
                                          << ", " << centralFrequency / 1e9 << " GHz, "
                                          << bandwidth / 1e6 << " MHz, realtime " << realtime
                                          << ", clock " << (simClock ? "external" : "wall")
                                          << ", reattach model "
                                          << (reattachTime > 0 ? "on" : "off") << std::endl;
    Simulator::Stop(Seconds(simTime));
    Simulator::Run();
    Simulator::Destroy();
    return 0;
}
