/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2009 IITP RAS
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * This is an example script for AODV manet routing protocol. 
 *
 * Authors: Pavel Boyko <boyko@iitp.ru>
 */

#include <iostream>
#include <cmath>
#include "ns3/aodv-module.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/v4ping-helper.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/wifi-radio-energy-model-helper.h"
#include "ns3/basic-energy-source-helper.h"
#include "ns3/flow-monitor-module.h"


using namespace ns3;

/**
 * \ingroup aodv-examples
 * \ingroup examples
 * \brief Test script.
 * 
 * This script creates 1-dimensional grid topology and then ping last node from the first one:
 * 
 * [10.0.0.1] <-- step --> [10.0.0.2] <-- step --> [10.0.0.3] <-- step --> [10.0.0.4]
 * 
 * ping 10.0.0.4
 *
 * When 1/3 of simulation time has elapsed, one of the nodes is moved out of
 * range, thereby breaking the topology.  By default, this will result in
 * only 34 of 100 pings being received.  If the step size is reduced
 * to cover the gap, then all pings can be received.
 */
class AodvExample 
{
public:
  AodvExample ();
  /**
   * \brief Configure script parameters
   * \param argc is the command line argument count
   * \param argv is the command line arguments
   * \return true on successful configuration
  */
  bool Configure (int argc, char **argv);
  /// Run simulation
  void Run ();
  /**
   * Report results
   * \param os the output stream
   */
  void Report (std::ostream & os);

private:

  // parameters
  /// Number of nodes
  uint32_t size;
  /// Distance between nodes, meters
  double step;
  /// Simulation time, seconds
  double totalTime;
  /// Write per-device PCAP traces if true
  bool pcap;
  /// Print routes if true
  bool printRoutes;

  uint32_t port = 9;
  uint32_t bytesTotal = 0;
  uint32_t packetsReceived = 0;

  uint32_t SentPackets = 0;
  uint32_t ReceivedPackets = 0;
  uint32_t LostPackets = 0;

  int num_nodes = 0;

  // network
  /// nodes used in the example
  NodeContainer nodes;
  /// devices used in the example
  NetDeviceContainer devices;
  /// interfaces used in the example
  Ipv4InterfaceContainer interfaces;

  EnergySourceContainer energySources;

  FlowMonitorHelper flowmon;
  Ptr<FlowMonitor> monitor;

  /// Create the nodes
  void CreateNodes ();
  /// Create the devices
  void CreateDevices ();
  /// Create the network
  void InstallInternetStack ();
  /// Create the simulation applications
  void InstallApplications ();
  /// FlowMonitor statistics
  void PrintFlowMonitorStats ();

  void ReceivePacket(Ptr<Socket> socket);
  void CheckThroughput();
  void CollectPacketStatistics ();
  Ptr<Socket> SetupPacketReceive(Ipv4Address addr, Ptr<Node> node);
};

int main (int argc, char **argv)
{
  AodvExample test;
  if (!test.Configure (argc, argv))
    NS_FATAL_ERROR ("Configuration failed. Aborted.");

//  LogComponentEnable ("YansWifiPhy", LOG_LEVEL_ALL);
//  LogComponentEnable ("WifiPhy", LOG_LEVEL_ALL);

  NS_LOG_UNCOND("Run Simulation.");

  test.Run ();
//  test.Report (std::cout);
  return 0;
}

//-----------------------------------------------------------------------------
AodvExample::AodvExample () :
  size (10),
  step (2),
  totalTime (100),
  pcap (true),
  printRoutes (true)
{
}

bool
AodvExample::Configure (int argc, char **argv)
{
  // Enable AODV logs by default. Comment this if too noisy
  // LogComponentEnable("AodvRoutingProtocol", LOG_LEVEL_ALL);

  SeedManager::SetSeed (12345);
  CommandLine cmd;

  cmd.AddValue ("pcap", "Write PCAP traces.", pcap);
  cmd.AddValue ("printRoutes", "Print routing table dumps.", printRoutes);
  cmd.AddValue ("size", "Number of nodes.", size);
  cmd.AddValue ("time", "Simulation time, s.", totalTime);
  cmd.AddValue ("step", "Grid step, m", step);

  cmd.Parse (argc, argv);
  return true;
}

void
AodvExample::Run ()
{
//  Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", UintegerValue (1)); // enable rts cts all the time.
  CreateNodes ();
  CreateDevices ();
  InstallInternetStack ();
  InstallApplications ();
  CollectPacketStatistics ();
  monitor = flowmon.InstallAll();
  CheckThroughput();

  std::cout << "Starting simulation for " << totalTime << " s ...\n";
  Simulator::Stop (Seconds (totalTime));

  for(uint32_t i = 0 ; i < size ; ++i) {
      std::cout << "Energy at node : " << i << " " << energySources.Get(i)->GetRemainingEnergy() << " " << std::endl;
  }
  Simulator::Run ();

  PrintFlowMonitorStats();

  double sum = 0;
  for(uint32_t i = 0 ; i < size; ++i) {
      std::cout << "Energy at node : " << i << " " << energySources.Get(i)->GetRemainingEnergy() << " " << std::endl;
      sum += energySources.Get(i)->GetRemainingEnergy();
  }
   std::cout << "Average energy = " << sum/size << std::endl;

//  Simulator::Destroy ();
}

void
AodvExample::Report (std::ostream &)
{ 
}

//Receiving packets
void
AodvExample::ReceivePacket(Ptr<Socket> socket)
{
  Ptr<Packet> packet;
  Address senderAddress;
  while ((packet = socket->RecvFrom(senderAddress)))
  {
    bytesTotal += packet->GetSize();
    packetsReceived += 1;
//    NS_LOG_UNCOND(PrintReceivedPacket(socket, packet, senderAddress));
  }
}
//txp is transmission power
void
AodvExample::CheckThroughput()
{
  double kbs = (bytesTotal * 8.0) / 1000;
  bytesTotal = 0;

  std::ofstream out("manet_project.csv", std::ios::app);

  out << (Simulator::Now()).GetSeconds() << ","
      << kbs << ","
      << packetsReceived << ","
      << std::endl;

  out.close();
  packetsReceived = 0;
  Simulator::Schedule(Seconds(1.0), &AodvExample::CheckThroughput, this);
}

//set up function for receiving the packet. Binding sockets
Ptr<Socket>
AodvExample::SetupPacketReceive(Ipv4Address addr, Ptr<Node> node)
{
  TypeId tid = TypeId::LookupByName("ns3::UdpSocketFactory");
  Ptr<Socket> sink = Socket::CreateSocket(node, tid);
  InetSocketAddress local = InetSocketAddress(addr, port);
  sink->Bind(local);
  sink->SetRecvCallback(MakeCallback(&AodvExample::ReceivePacket, this)); // SetupPacketReceive will call ReceivePacket function

  return sink;
}

void
AodvExample::CreateNodes ()
{
  std::cout << "Creating " << (unsigned)size << " nodes " << step << " m apart.\n";
  nodes.Create (size);
  // Name nodes
  for (uint32_t i = 0; i < size; ++i)
    {
      std::ostringstream os;
      os << "node-" << i;
      Names::Add (os.str (), nodes.Get (i));
    }
  // Create static grid
  MobilityHelper mobility;
  mobility.SetPositionAllocator ("ns3::GridPositionAllocator",
                                 "MinX", DoubleValue (0.0),
                                 "MinY", DoubleValue (0.0),
                                 "DeltaX", DoubleValue (step),
                                 "DeltaY", DoubleValue (10),
                                 "GridWidth", UintegerValue (size),
                                 "LayoutType", StringValue ("RowFirst"));
  //mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");

  //mobility.SetMobilityModel ("ns3::RandomWalk2dMobilityModel", "Bounds", RectangleValue (Rectangle (-50, 50, -50, 50)));
  mobility.Install (nodes);
}

void
AodvExample::CreateDevices ()
{
  WifiMacHelper wifiMac;
  wifiMac.SetType ("ns3::AdhocWifiMac");
  YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default ();
  YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
  wifiPhy.SetChannel (wifiChannel.Create ());
  WifiHelper wifi;
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager", "DataMode", StringValue ("OfdmRate6Mbps"), "RtsCtsThreshold", UintegerValue (0));
  devices = wifi.Install (wifiPhy, wifiMac, nodes);

  // Energy Sources
  BasicEnergySourceHelper basicSourceHelper;
  basicSourceHelper.Set ("BasicEnergySourceInitialEnergyJ", DoubleValue (100));
  this->energySources = basicSourceHelper.Install (this->nodes);
  WifiRadioEnergyModelHelper radioEnergyHelper;
  radioEnergyHelper.Set ("TxCurrentA", DoubleValue (0.0174));
  DeviceEnergyModelContainer deviceModels = radioEnergyHelper.Install (devices, energySources);

  if (pcap) {
    wifiPhy.EnablePcapAll (std::string ("aodv"));
  }
}

void
AodvExample::InstallInternetStack ()
{
  AodvHelper aodv;
  // you can configure AODV attributes here using aodv.Set(name, value)
  InternetStackHelper stack;
  //stack.SetRoutingHelper (aodv); // has effect on the next Install ()
  stack.Install (nodes);
  Ipv4AddressHelper address;
  address.SetBase ("10.0.0.0", "255.0.0.0");
  interfaces = address.Assign (devices);

  if (printRoutes)
    {
      Ptr<OutputStreamWrapper> routingStream = Create<OutputStreamWrapper> ("aodv.routes", std::ios::out);
      aodv.PrintRoutingTableAllAt (Seconds (8), routingStream);
    }
}

void
AodvExample::InstallApplications ()
{
  // Ping to last node
  V4PingHelper ping (interfaces.GetAddress (size - 1));
  ping.SetAttribute ("Verbose", BooleanValue (true));

  // First Node
  ApplicationContainer p = ping.Install (nodes.Get (0));
  p.Start (Seconds (0));
  p.Stop (Seconds (totalTime) - Seconds (0.001));

  // move node away
  //Ptr<Node> node = nodes.Get (size/2);
  //Ptr<MobilityModel> mob = node->GetObject<MobilityModel> ();
  //Simulator::Schedule (Seconds (totalTime/3), &MobilityModel::SetPosition, mob, Vector (1e5, 1e5, 1e5));
}

void
AodvExample::CollectPacketStatistics ()
{
  for (int i = 0; i < num_nodes; i++)
  {
    Ptr<Socket> sink = SetupPacketReceive(interfaces.GetAddress(i), nodes.Get(i));
  }
}

void
AodvExample::PrintFlowMonitorStats ()
{
  int j = 0;
  float AvgThroughput = 0;
  Time Jitter;
  Time Delay;

  Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier> (flowmon.GetClassifier ());
  std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats ();

  for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator iter = stats.begin ();
       iter != stats.end (); ++iter)
    {
      Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow (iter->first);

      NS_LOG_UNCOND ("----Flow ID:" << iter->first);
      NS_LOG_UNCOND ("Src Addr" << t.sourceAddress << "Dst Addr " << t.destinationAddress);
      NS_LOG_UNCOND ("Sent Packets=" << iter->second.txPackets);
      NS_LOG_UNCOND ("Received Packets =" << iter->second.rxPackets);
      NS_LOG_UNCOND ("Lost Packets =" << iter->second.txPackets - iter->second.rxPackets);
      if (iter->second.txPackets > 0)
        {
          NS_LOG_UNCOND ("Packet delivery ratio ="
                         << iter->second.rxPackets * 100 / iter->second.txPackets << "%");
          NS_LOG_UNCOND ("Packet loss ratio ="
                         << (iter->second.txPackets - iter->second.rxPackets) * 100 /
                                iter->second.txPackets
                         << "%");
        }
      else
        {
          NS_LOG_UNCOND ("Packet delivery ratio =0%");
          NS_LOG_UNCOND ("Packet loss ratio =0%");
        }
      NS_LOG_UNCOND ("Delay =" << iter->second.delaySum);
      NS_LOG_UNCOND ("Jitter =" << iter->second.jitterSum);
      NS_LOG_UNCOND ("Throughput =" << iter->second.rxBytes * 8.0 /
                                           (iter->second.timeLastRxPacket.GetSeconds () -
                                            iter->second.timeFirstTxPacket.GetSeconds ()) /
                                           1024
                                    << "Kbps");

      SentPackets = SentPackets + (iter->second.txPackets);
      ReceivedPackets = ReceivedPackets + (iter->second.rxPackets);
      LostPackets = LostPackets + (iter->second.txPackets - iter->second.rxPackets);
      AvgThroughput = AvgThroughput + (iter->second.rxBytes * 8.0 /
                                       (iter->second.timeLastRxPacket.GetSeconds () -
                                        iter->second.timeFirstTxPacket.GetSeconds ()) /
                                       1024);
      Delay = Delay + (iter->second.delaySum);
      Jitter = Jitter + (iter->second.jitterSum);

      j = j + 1;
    }

  AvgThroughput = AvgThroughput / j;
  NS_LOG_UNCOND ("--------Total Results of the simulation----------" << std::endl);
  NS_LOG_UNCOND ("Total sent packets  =" << SentPackets);
  NS_LOG_UNCOND ("Total Received Packets =" << ReceivedPackets);
  NS_LOG_UNCOND ("Total Lost Packets =" << LostPackets);
  if (SentPackets > 0)
    {
      NS_LOG_UNCOND ("Packet Loss ratio =" << ((LostPackets * 100) / SentPackets) << "%");
      NS_LOG_UNCOND ("Packet delivery ratio =" << ((ReceivedPackets * 100) / SentPackets) << "%");
    }
  else
    {
      NS_LOG_UNCOND ("Packet Loss ratio =0%");
      NS_LOG_UNCOND ("Packet delivery ratio =0%");
    }
  NS_LOG_UNCOND ("Average Throughput =" << AvgThroughput << "Kbps");
  NS_LOG_UNCOND ("End to End Delay =" << Delay);
  NS_LOG_UNCOND ("End to End Jitter delay =" << Jitter);
  NS_LOG_UNCOND ("Total Flow id " << j);

  monitor->SerializeToXmlFile("manet_project.xml", true, true);
}
