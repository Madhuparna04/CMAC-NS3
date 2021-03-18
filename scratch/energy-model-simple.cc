/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2010 Network Security Lab, University of Washington, Seattle.
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
 * Author: Sidharth Nabar <snabar@uw.edu>, He Wu <mdzz@u.washington.edu>
 */

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/config-store-module.h"
#include "ns3/energy-module.h"
#include "ns3/internet-module.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/wifi-radio-energy-model-helper.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("EnergyExample");

static inline std::string
PrintReceivedPacket (Address& from)
{
  InetSocketAddress iaddr = InetSocketAddress::ConvertFrom (from);

  std::ostringstream oss;
  oss << "--\nReceived one packet! Socket: " << iaddr.GetIpv4 ()
      << " port: " << iaddr.GetPort ()
      << " at time = " << Simulator::Now ().GetSeconds ()
      << "\n--";

  return oss.str ();
}

/**
 * \param socket Pointer to socket.
 *
 * Packet receiving sink.
 */
void
ReceivePacket (Ptr<Socket> socket)
{
  Ptr<Packet> packet;
  Address from;
  while ((packet = socket->RecvFrom (from)))
    {
      if (packet->GetSize () > 0)
        {
          NS_LOG_UNCOND (PrintReceivedPacket (from));
        }
    }
}

/**
 * \param socket Pointer to socket.
 * \param pktSize Packet size.
 * \param n Pointer to node.
 * \param pktCount Number of packets to generate.
 * \param pktInterval Packet sending interval.
 *
 * Traffic generator.
 */
static void
GenerateTraffic (Ptr<Socket> socket, uint32_t pktSize, Ptr<Node> n,
                 uint32_t pktCount, Time pktInterval)
{
  if (pktCount > 0)
    {
      socket->Send (Create<Packet> (pktSize));
      Simulator::Schedule (pktInterval, &GenerateTraffic, socket, pktSize, n,
                           pktCount - 1, pktInterval);
    }
  else
    {
      socket->Close ();
    }
}

/// Trace function for remaining energy at node.
/*void
RemainingEnergy (double oldValue, double remainingEnergy)
{
  NS_LOG_UNCOND (Simulator::Now ().GetSeconds ()
              << "s Current remaining energy = " << remainingEnergy << "J");
}

/// Trace function for total energy consumption at node.
void
TotalEnergy (double oldValue, double totalEnergy)
{
 NS_LOG_UNCOND (Simulator::Now ().GetSeconds ()
                 << "s Total energy consumed by radio = " << totalEnergy << "J");
}*/

int
main (int argc, char *argv[])
{
  LogComponentEnable ("EnergySource", LOG_LEVEL_DEBUG);
  LogComponentEnable ("BasicEnergySource", LOG_LEVEL_DEBUG);
  LogComponentEnable ("DeviceEnergyModel", LOG_LEVEL_DEBUG);
  LogComponentEnable ("WifiRadioEnergyModel", LOG_LEVEL_DEBUG);
  

  LogComponentEnable ("EnergyExample", LogLevel (LOG_PREFIX_TIME | LOG_PREFIX_NODE | LOG_LEVEL_INFO));

  std::string phyMode ("DsssRate1Mbps");
  double Prss = -80;            // dBm
  uint32_t PpacketSize = 200;  
  bool verbose = false;

  // simulation parameters
  uint32_t numPackets = 1;  // number of packets to send.  Change this value and check energy consumed readings.
  double interval = 0;          // seconds
  double startTime = 0.0;       // seconds
  double distanceToRx = 50.0;  // meters. Change this value and check energy consumed readings.

  CommandLine cmd;
  cmd.AddValue ("phyMode", "Wifi Phy mode", phyMode);
  cmd.AddValue ("Prss", "Intended primary RSS (dBm)", Prss);
  cmd.AddValue ("PpacketSize", "size of application packet sent", PpacketSize);
  cmd.AddValue ("numPackets", "Total number of packets to send", numPackets);
  cmd.AddValue ("startTime", "Simulation start time", startTime);
  cmd.AddValue ("distanceToRx", "X-Axis distance between nodes", distanceToRx);
  cmd.AddValue ("verbose", "Turn on all device log components", verbose);
  cmd.Parse (argc, argv);

  // Convert to time object
  Time interPacketInterval = Seconds (interval);

  Config::SetDefault ("ns3::WifiRemoteStationManager::NonUnicastMode",
                      StringValue (phyMode));
  
  NodeContainer c;
  c.Create (2);     // create 2 nodes
  NodeContainer networkNodes;
  networkNodes.Add (c.Get (0));
  networkNodes.Add (c.Get (1));




  // The below set of helpers will help us to put together the wifi NICs we want
  WifiHelper wifi;
  if (verbose)
    {
      wifi.EnableLogComponents ();
    }
  wifi.SetStandard (WIFI_PHY_STANDARD_80211b);

  /** Wifi PHY **/
  /***************************************************************************/
  YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default ();

  /** wifi channel **/
  YansWifiChannelHelper wifiChannel;
  wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
  wifiChannel.AddPropagationLoss ("ns3::FriisPropagationLossModel");
  
  // create wifi channel
  Ptr<YansWifiChannel> wifiChannelPtr = wifiChannel.Create ();
  wifiPhy.SetChannel (wifiChannelPtr);



  /** MAC layer **/
  // Add a MAC and disable rate control
  WifiMacHelper wifiMac;
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager", "DataMode",
                                StringValue (phyMode), "ControlMode",
                                StringValue (phyMode));
  // Set it to ad-hoc mode
  wifiMac.SetType ("ns3::AdhocWifiMac");

  /** install PHY + MAC **/
  NetDeviceContainer devices = wifi.Install (wifiPhy, wifiMac, networkNodes);



  /** mobility **/
  MobilityHelper mobility;
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  positionAlloc->Add (Vector (0.0, 0.0, 0.0));
  positionAlloc->Add (Vector (2 * distanceToRx, 0.0, 0.0));
  mobility.SetPositionAllocator (positionAlloc);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (c);

  /** Energy Model **/
//Create and install energy source and a single basic radio energy model on the node using helpers.
  /***************************************************************************/
  /* energy source */
  BasicEnergySourceHelper basicSourceHelper; ////Creates a BasicEnergySource (inherits from EnergySource class) object. BasicEnergySource decreases/increases remaining energy stored in itself in linearly. Energy source base class:This is the base class for energy sources. Energy sources keep track of remaining energy. Device energy models will be updating the remaining energy in the energy source. The energy source itself does not update the remaining energy. Energy source also keeps a list of device energy models installed on the same node. When the remaining energy level reaches 0, the energy source will notify all device energy models stored in the list.
  // configure energy source
  basicSourceHelper.Set ("BasicEnergySourceInitialEnergyJ", DoubleValue (0.1)); //Initial energy stored in basic energy source. set with class: ns3::DoubleValue. Initial value: 10
  // install source
  EnergySourceContainer sources = basicSourceHelper.Install (c); //EnergySourceHelper returns a list of EnergySource pointers installed onto a node. Users can use this list to access EnergySource objects to obtain total energy consumption on a node easily
  /* device energy model */
  WifiRadioEnergyModelHelper radioEnergyHelper;
  // configure radio energy model
  radioEnergyHelper.Set ("TxCurrentA", DoubleValue (0.0174));
  // install device model
  DeviceEnergyModelContainer deviceModels = radioEnergyHelper.Install (devices, sources);
  /***************************************************************************/

  /** Internet stack **/
  InternetStackHelper internet;
  internet.Install (networkNodes);

  Ipv4AddressHelper ipv4;
  NS_LOG_INFO ("Assign IP Addresses.");
  ipv4.SetBase ("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer i = ipv4.Assign (devices);

  TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
  Ptr<Socket> recvSink = Socket::CreateSocket (networkNodes.Get (1), tid);  // node 1, receiver
  InetSocketAddress local = InetSocketAddress (Ipv4Address::GetAny (), 80);
  recvSink->Bind (local);
  recvSink->SetRecvCallback (MakeCallback (&ReceivePacket));

  Ptr<Socket> source = Socket::CreateSocket (networkNodes.Get (0), tid);    // node 0, sender
  InetSocketAddress remote = InetSocketAddress (Ipv4Address::GetBroadcast (), 80);
  source->SetAllowBroadcast (true);
  source->Connect (remote);

  /** connect trace sources **/
  /***************************************************************************/
  //all sources are connected to node 1
  // energy source
  /*Ptr<BasicEnergySource> basicSourcePtr = DynamicCast<BasicEnergySource> (sources.Get(1)); 
  basicSourcePtr->TraceConnectWithoutContext ("RemainingEnergy", MakeCallback (&RemainingEnergy));
  // device energy model
  Ptr<DeviceEnergyModel> basicRadioModelPtr =
    basicSourcePtr->FindDeviceEnergyModels ("ns3::WifiRadioEnergyModel").Get (0);
  NS_ASSERT (basicRadioModelPtr != NULL);
  basicRadioModelPtr->TraceConnectWithoutContext ("TotalEnergyConsumption", MakeCallback (&TotalEnergy));*/
  /***************************************************************************/


    
  /** simulation setup **/
  // start traffic
  Simulator::Schedule (Seconds (startTime), &GenerateTraffic, source, PpacketSize,
                       networkNodes.Get (0), numPackets, interPacketInterval);

    AsciiTraceHelper ascii;
      wifiPhy.EnableAsciiAll (ascii.CreateFileStream ("energy-model-simple.tr"));
      wifiPhy.EnablePcap ("energy-model-simple", devices);

  Simulator::Stop (Seconds (50.0));
  Simulator::Run ();

 /*for (DeviceEnergyModelContainer::Iterator iter = deviceModels.Begin (); iter != deviceModels.End (); iter ++)
    {
     double energyConsumed = (*iter)->GetTotalEnergyConsumption ();

     NS_LOG_UNCOND ("End of simulation (" << Simulator::Now ().GetSeconds ()
                     << "s) Total energy consumed by radio = " << energyConsumed << "J");
     NS_ASSERT (energyConsumed <= 0.1);
   }*/
 std::cout << "Energy at node 0: ";
  std::cout << sources.Get(0)->GetRemainingEnergy() << " ";
  std::cout << std::endl;

std::cout << "Energy at node 1: ";
  std::cout << sources.Get(1)->GetRemainingEnergy() << " ";
  std::cout << std::endl;
  Simulator::Destroy ();

  return 0;
}
