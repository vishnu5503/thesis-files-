/*
 * phaqm-dumbbell.cc  — Multi-run version with pcap + seed support
 * Place at:  ~/ns-3.45/scratch/phaqm-dumbbell.cc
 */
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/applications-module.h"
#include "ns3/traffic-control-module.h"
#include "ns3/flow-monitor-module.h"

using namespace ns3;
NS_LOG_COMPONENT_DEFINE ("PhaqmDumbbell");

static std::ofstream g_qLog;
void QueueLengthTrace (uint32_t, uint32_t newVal) {
  g_qLog << Simulator::Now().GetSeconds() << "," << newVal << "\n";
}

int main (int argc, char *argv[])
{
  uint32_t nFlows     = 200;
  double   simTime    = 120.0;
  uint32_t seed       = 1;
  bool     enablePcap = false;
  std::string outDir  = "";

  CommandLine cmd;
  cmd.AddValue ("nFlows",     "Total flows",               nFlows);
  cmd.AddValue ("simTime",    "Simulation time (s)",       simTime);
  cmd.AddValue ("seed",       "RNG seed / run number",     seed);
  cmd.AddValue ("enablePcap", "Write .pcap for Wireshark", enablePcap);
  cmd.AddValue ("outDir",     "Output directory",          outDir);
  cmd.Parse (argc, argv);

  RngSeedManager::SetSeed (seed * 100 + 7);
  RngSeedManager::SetRun  (seed);

  uint32_t nBulk = (uint32_t)(nFlows * 0.60);
  uint32_t nWeb  = (uint32_t)(nFlows * 0.30);
  uint32_t nUdp  = nFlows - nBulk - nWeb;
  NS_LOG_UNCOND ("PHAQM seed=" << seed << " " << nBulk << "B+" << nWeb << "W+" << nUdp << "U");

  NodeContainer senders, receivers, routers;
  senders.Create(nFlows); receivers.Create(nFlows); routers.Create(2);
  InternetStackHelper stack; stack.InstallAll();

  PointToPointHelper bottleneck;
  bottleneck.SetDeviceAttribute  ("DataRate", StringValue ("45Mbps"));
  bottleneck.SetChannelAttribute ("Delay",    StringValue ("50ms"));
  NetDeviceContainer bottleneckDevs = bottleneck.Install(routers.Get(0), routers.Get(1));

  TrafficControlHelper tch;
  tch.SetRootQueueDisc("ns3::PhaqmQueueDisc", "MaxSize",
      QueueSizeValue(QueueSize(QueueSizeUnit::PACKETS, 1000)));
  QueueDiscContainer qdiscs = tch.Install(bottleneckDevs.Get(0));

  if (enablePcap) {
    bottleneck.EnablePcap("phaqm-bottleneck", bottleneckDevs.Get(0), true);
    NS_LOG_UNCOND ("PCAP → phaqm-bottleneck-0-0.pcap");
  }

  const char* home = std::getenv("HOME");
  std::string qPath = outDir.empty()
      ? (home ? std::string(home)+"/phaqm-queue-length.csv" : "phaqm-queue-length.csv")
      : outDir + "/queue-length.csv";
  g_qLog.open(qPath);
  g_qLog << "time,queue\n";
  qdiscs.Get(0)->TraceConnectWithoutContext("PacketsInQueue", MakeCallback(&QueueLengthTrace));

  Ipv4AddressHelper addr;
  addr.SetBase("10.1.0.0","255.255.255.0"); addr.Assign(bottleneckDevs); addr.NewNetwork();

  Ptr<UniformRandomVariable> startRv = CreateObject<UniformRandomVariable>();
  startRv->SetAttribute("Min", DoubleValue(0.0)); startRv->SetAttribute("Max", DoubleValue(30.0));
  Ptr<ParetoRandomVariable> paretoRv = CreateObject<ParetoRandomVariable>();
  paretoRv->SetAttribute("Shape", DoubleValue(1.2)); paretoRv->SetAttribute("Scale", DoubleValue(50000.0));

  const uint16_t basePort = 5000;
  PointToPointHelper access;
  access.SetDeviceAttribute("DataRate", StringValue("100Mbps"));

  for (uint32_t i = 0; i < nFlows; i++) {
    if      (i%3==0) access.SetChannelAttribute("Delay", StringValue("5ms"));
    else if (i%3==1) access.SetChannelAttribute("Delay", StringValue("20ms"));
    else             access.SetChannelAttribute("Delay", StringValue("50ms"));

    NetDeviceContainer ld = access.Install(senders.Get(i), routers.Get(0));
    addr.Assign(ld); addr.NewNetwork();
    NetDeviceContainer rd = access.Install(routers.Get(1), receivers.Get(i));
    Ipv4InterfaceContainer ri = addr.Assign(rd); addr.NewNetwork();

    Ipv4Address recvAddr = ri.GetAddress(1);
    uint16_t port = basePort + (uint16_t)i;
    double t0 = startRv->GetValue();

    if (i < nBulk+nWeb) {
      PacketSinkHelper s("ns3::TcpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(),port));
      auto a = s.Install(receivers.Get(i)); a.Start(Seconds(0)); a.Stop(Seconds(simTime));
    } else {
      PacketSinkHelper s("ns3::UdpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(),port));
      auto a = s.Install(receivers.Get(i)); a.Start(Seconds(0)); a.Stop(Seconds(simTime));
    }

    if (i < nBulk) {
      BulkSendHelper b("ns3::TcpSocketFactory", InetSocketAddress(recvAddr,port));
      b.SetAttribute("MaxBytes", UintegerValue((uint32_t)paretoRv->GetValue()));
      auto a = b.Install(senders.Get(i)); a.Start(Seconds(t0)); a.Stop(Seconds(simTime));
    } else if (i < nBulk+nWeb) {
      OnOffHelper w("ns3::TcpSocketFactory", InetSocketAddress(recvAddr,port));
      w.SetAttribute("DataRate",   StringValue("5Mbps"));
      w.SetAttribute("PacketSize", UintegerValue(1000));
      w.SetAttribute("OnTime",  StringValue("ns3::ExponentialRandomVariable[Mean=1.0]"));
      w.SetAttribute("OffTime", StringValue("ns3::ExponentialRandomVariable[Mean=2.0]"));
      auto a = w.Install(senders.Get(i)); a.Start(Seconds(t0)); a.Stop(Seconds(simTime));
    } else {
      OnOffHelper v("ns3::UdpSocketFactory", InetSocketAddress(recvAddr,port));
      v.SetAttribute("DataRate",   StringValue("2Mbps"));
      v.SetAttribute("PacketSize", UintegerValue(1200));
      v.SetAttribute("OnTime",  StringValue("ns3::ConstantRandomVariable[Constant=1e9]"));
      v.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0.0]"));
      auto a = v.Install(senders.Get(i)); a.Start(Seconds(t0)); a.Stop(Seconds(simTime));
    }
  }

  Ipv4GlobalRoutingHelper::PopulateRoutingTables();
  FlowMonitorHelper fmh; Ptr<FlowMonitor> fm = fmh.InstallAll();
  NS_LOG_UNCOND("Running " << simTime << "s ...");
  Simulator::Stop(Seconds(simTime)); Simulator::Run();
  std::string xmlPath = outDir.empty() ? "phaqm-flow-results.xml" : outDir+"/flow-results.xml";
  fm->SerializeToXmlFile(xmlPath, true, true);
  Simulator::Destroy(); g_qLog.close();
  NS_LOG_UNCOND("Done → " << qPath);
  return 0;
}
