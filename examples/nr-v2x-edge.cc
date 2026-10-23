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
    double simTime = 3600.0;
    std::string tapUePrefix = "tap_ue";
    std::string tapEdge = "tap_edge";
    bool realtime = true;
    bool simClock = false;
    uint16_t simClockPort = 5600;
    uint32_t rngRun = 1;

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
    cmd.AddValue("simTime", "Simulation time in s", simTime);
    cmd.AddValue("tapUePrefix", "TAP device name prefix for the UEs", tapUePrefix);
    cmd.AddValue("tapEdge", "TAP device name for the edge host", tapEdge);
    cmd.AddValue("realtime", "Run in real time (needed with TAPs)", realtime);
    cmd.AddValue("simClock", "Follow the external simulator's clock (ticks over UDP) instead of the wall clock", simClock);
    cmd.AddValue("simClockPort", "UDP port of the external clock's ticks", simClockPort);
    cmd.AddValue("rngRun", "RNG run number", rngRun);
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

    // Unacknowledged RLC: a lost transport block is lost, not delayed.
    Config::SetDefault("ns3::NrGnbRrc::EpsBearerToRlcMapping", EnumValue(NrGnbRrc::RLC_UM_ALWAYS));
    Config::SetDefault("ns3::NrRlcUm::MaxTxBufferSize", UintegerValue(1024 * 1024));

    NetDeviceContainer gnbDevs = nrHelper->InstallGnbDevice(gnbNodes, allBwps);
    NetDeviceContainer ueDevs = nrHelper->InstallUeDevice(ueNodes, allBwps);

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

    if (simClock)
    {
        // The synchronizer the simulator created from SynchronizerType.
        PointerValue syncPtr;
        Simulator::GetImplementation()->GetAttribute("Synchronizer", syncPtr);
        Ptr<SimClockSynchronizer> clock = syncPtr.Get<SimClockSynchronizer>();
        NS_ABORT_MSG_IF(!clock, "the simulator does not run a SimClockSynchronizer");
        Simulator::Schedule(MilliSeconds(10), &ApplyPositions, clock, ueNodes, MilliSeconds(10), uint64_t{0});
    }

    std::cout << "nr-v2x-edge running: " << numUes << " UEs, " << scenario << " " << condition
                                          << ", " << centralFrequency / 1e9 << " GHz, "
                                          << bandwidth / 1e6 << " MHz, realtime " << realtime
                                          << ", clock " << (simClock ? "external" : "wall") << std::endl;
    Simulator::Stop(Seconds(simTime));
    Simulator::Run();
    Simulator::Destroy();
    return 0;
}
