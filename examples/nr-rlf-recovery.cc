// Copyright (c) 2026 NEWSLab, National Taiwan University
//
// SPDX-License-Identifier: GPL-2.0-only

/**
 * @file nr-rlf-recovery.cc
 * @brief One UE that loses its link and comes back: does traffic resume?
 *
 * A gNB, one UE 100 m away and a remote host behind the PGW exchange UDP both
 * ways (every 10 ms). At --awayAt the UE is moved --awayDistance m out, where
 * the link fails; at --backAt it returns. With RLF detection on, the UE's RRC
 * declares a radio link failure, leaves connected mode, reselects the cell and
 * connects again (TS 38.331's idle path). The program prints the RRC state
 * transitions, the failures and, every 100 ms, the packets received each way,
 * so that one run shows whether, and when, the traffic comes back.
 *
 *   ./ns3 run "nr-rlf-recovery --awayAt=2 --backAt=5 --simTime=10"
 */

#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/nr-module.h"
#include "ns3/point-to-point-module.h"

#include <iostream>

using namespace ns3;

namespace
{

uint64_t g_dl = 0; ///< packets the UE received this period
uint64_t g_ul = 0; ///< packets the remote host received this period

void
OnState(std::string, uint64_t imsi, uint16_t cell, uint16_t rnti, NrUeRrc::State from, NrUeRrc::State to)
{
    std::cout << Simulator::Now().As(Time::S) << " IMSI " << imsi << " cell " << cell << " RNTI "
              << rnti << ": " << NrUeRrc::ToString(from) << " -> " << NrUeRrc::ToString(to)
              << std::endl;
}

void
OnRlf(std::string, uint64_t imsi, uint16_t cell, uint16_t rnti)
{
    std::cout << Simulator::Now().As(Time::S) << " IMSI " << imsi << " cell " << cell << " RNTI "
              << rnti << ": radio link failure" << std::endl;
}

void
OnSync(std::string, uint64_t imsi, uint16_t rnti, uint16_t cell, std::string what, uint8_t count)
{
    std::cout << Simulator::Now().As(Time::S) << " IMSI " << imsi << ": " << what << " ("
              << +count << ")" << std::endl;
}

void
OnDlCtrlSinr(uint16_t, uint16_t, double sinr, uint16_t)
{
    static Time last;
    if (Simulator::Now() - last >= MilliSeconds(500))
    {
        std::cout << Simulator::Now().As(Time::S) << " DL CTRL SINR " << 10 * std::log10(sinr)
                  << " dB" << std::endl;
        last = Simulator::Now();
    }
}

void
OnRx(uint64_t* counter, Ptr<const Packet>, const Address&, const Address&)
{
    ++*counter;
}

void
Report(Time period)
{
    std::cout << Simulator::Now().As(Time::S) << " received: DL " << g_dl << ", UL " << g_ul
              << std::endl;
    g_dl = g_ul = 0;
    Simulator::Schedule(period, &Report, period);
}

void
Move(Ptr<Node> ue, Vector to)
{
    ue->GetObject<MobilityModel>()->SetPosition(to);
    std::cout << Simulator::Now().As(Time::S) << " UE moved to " << to << std::endl;
}

} // namespace

int
main(int argc, char* argv[])
{
    double awayAt = 2.0;
    double backAt = 5.0;
    double awayDistance = 5000.0;
    double simTime = 10.0;
    bool rlfDetection = true;

    CommandLine cmd(__FILE__);
    cmd.AddValue("awayAt", "When the UE is moved out of coverage, s", awayAt);
    cmd.AddValue("backAt", "When it returns, s", backAt);
    cmd.AddValue("awayDistance", "How far out, m", awayDistance);
    cmd.AddValue("simTime", "Simulation time, s", simTime);
    cmd.AddValue("rlfDetection", "Detect radio link failures", rlfDetection);
    cmd.Parse(argc, argv);

    Config::SetDefault("ns3::NrUePhy::EnableRlfDetection", BooleanValue(rlfDetection));
    Config::SetDefault("ns3::NrGnbRrc::EpsBearerToRlcMapping", EnumValue(NrGnbRrc::RLC_UM_ALWAYS));

    NodeContainer gnbNodes;
    gnbNodes.Create(1);
    NodeContainer ueNodes;
    ueNodes.Create(1);
    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(gnbNodes);
    mobility.Install(ueNodes);
    gnbNodes.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(0, 0, 10));
    ueNodes.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(100, 0, 1.5));

    Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper>();
    Ptr<IdealBeamformingHelper> beamforming = CreateObject<IdealBeamformingHelper>();
    beamforming->SetAttribute("BeamformingMethod", TypeIdValue(DirectPathBeamforming::GetTypeId()));
    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
    nrHelper->SetBeamformingHelper(beamforming);
    nrHelper->SetEpcHelper(epcHelper);

    CcBwpCreator ccBwpCreator;
    CcBwpCreator::SimpleOperationBandConf bandConf(3.5e9, 20e6, 1);
    OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);
    Ptr<NrChannelHelper> channelHelper = CreateObject<NrChannelHelper>();
    channelHelper->ConfigureFactories("UMi", "LOS", "ThreeGpp");
    channelHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(false));
    channelHelper->AssignChannelsToBands({band});
    BandwidthPartInfoPtrVector allBwps = CcBwpCreator::GetAllBwps({band});
    nrHelper->SetGnbPhyAttribute("TxPower", DoubleValue(30.0));
    nrHelper->SetUePhyAttribute("TxPower", DoubleValue(23.0));

    NetDeviceContainer gnbDevs = nrHelper->InstallGnbDevice(gnbNodes, allBwps);
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
    routing.GetStaticRouting(ueNodes.Get(0)->GetObject<Ipv4>())
        ->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);
    nrHelper->AttachToClosestGnb(ueDevs, gnbDevs);

    // UDP both ways, every 10 ms.
    uint16_t dlPort = 1234;
    uint16_t ulPort = 1235;
    UdpServerHelper dlSink(dlPort);
    UdpServerHelper ulSink(ulPort);
    ApplicationContainer sinks;
    sinks.Add(dlSink.Install(ueNodes.Get(0)));
    sinks.Add(ulSink.Install(remote.Get(0)));
    UdpClientHelper dl(ueIps.GetAddress(0), dlPort);
    dl.SetAttribute("Interval", TimeValue(MilliSeconds(10)));
    dl.SetAttribute("MaxPackets", UintegerValue(1000000));
    dl.SetAttribute("PacketSize", UintegerValue(200));
    UdpClientHelper ul(linkIps.GetAddress(1), ulPort);
    ul.SetAttribute("Interval", TimeValue(MilliSeconds(10)));
    ul.SetAttribute("MaxPackets", UintegerValue(1000000));
    ul.SetAttribute("PacketSize", UintegerValue(200));
    ApplicationContainer sources;
    sources.Add(dl.Install(remote.Get(0)));
    sources.Add(ul.Install(ueNodes.Get(0)));
    sinks.Start(Seconds(0.3));
    sources.Start(Seconds(0.5));
    sinks.Get(0)->TraceConnectWithoutContext("RxWithAddresses", MakeBoundCallback(&OnRx, &g_dl));
    sinks.Get(1)->TraceConnectWithoutContext("RxWithAddresses", MakeBoundCallback(&OnRx, &g_ul));

    Config::Connect("/NodeList/*/DeviceList/*/$ns3::NrUeNetDevice/NrUeRrc/StateTransition",
                    MakeCallback(&OnState));
    Config::Connect("/NodeList/*/DeviceList/*/$ns3::NrUeNetDevice/NrUeRrc/RadioLinkFailure",
                    MakeCallback(&OnRlf));
    Config::Connect("/NodeList/*/DeviceList/*/$ns3::NrUeNetDevice/NrUeRrc/PhySyncDetection",
                    MakeCallback(&OnSync));
    DynamicCast<NrUeNetDevice>(ueDevs.Get(0))
        ->GetPhy(0)
        ->TraceConnectWithoutContext("DlCtrlSinr", MakeCallback(&OnDlCtrlSinr));

    Simulator::Schedule(Seconds(awayAt), &Move, ueNodes.Get(0), Vector(awayDistance, 0, 1.5));
    Simulator::Schedule(Seconds(backAt), &Move, ueNodes.Get(0), Vector(100, 0, 1.5));
    Simulator::Schedule(MilliSeconds(100), &Report, MilliSeconds(100));
    Simulator::Stop(Seconds(simTime));
    Simulator::Run();
    Simulator::Destroy();
    return 0;
}
