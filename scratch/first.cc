/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
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
 */

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/applications-module.h"
#include "ns3/netanim-module.h"

// Default Network Topology
//
//       10.1.1.0
// n0 -------------- n1
//    point-to-point
//
 
using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("FirstScriptExample");

int
main (int argc, char *argv[])
{
  CommandLine cmd (__FILE__);
  cmd.Parse (argc, argv);
  
  //logging results, for research
  Time::SetResolution (Time::NS); //time sampling rate
  LogComponentEnable ("UdpEchoServerApplication", LOG_LEVEL_INFO);

  //how many nodes we are using gets put in container
  NodeContainer nodes;
  nodes.Create (2);

  //What type of channel ie ethernet, point to point.
  PointToPointHelper pointToPoint;
  pointToPoint.SetDeviceAttribute ("DataRate", StringValue ("5Mbps"));
  pointToPoint.SetChannelAttribute ("Delay", StringValue ("2ms"));

  //install p to p channel onto devices(nodes)
  NetDeviceContainer devices;
  devices = pointToPoint.Install (nodes);

  //Use the internet rules.
  InternetStackHelper stack;
  stack.Install (nodes);

  //set the base ip address. IP and subnet mask
  Ipv4AddressHelper address;
  address.SetBase ("10.1.1.0", "255.255.255.0");

  //assign address to devices
  Ipv4InterfaceContainer interfaces = address.Assign (devices);

  //run on port 9
  UdpEchoServerHelper echoServer (9);

  //put the server on the second node
  ApplicationContainer serverApps = echoServer.Install (nodes.Get (1));
  //server will start and stop
  serverApps.Start (Seconds (1.0));
  serverApps.Stop (Seconds (10.0));

  //create client and set attributes set remote address and remote port
  UdpEchoClientHelper echoClient (interfaces.GetAddress (1), 9);
  echoClient.SetAttribute ("MaxPackets", UintegerValue (1));
  echoClient.SetAttribute ("Interval", TimeValue (Seconds (1.0)));
  echoClient.SetAttribute ("PacketSize", UintegerValue (1024));

  //start and stop the client after setting on first node
  ApplicationContainer clientApps = echoClient.Install (nodes.Get (0));
  clientApps.Start (Seconds (2.0));
  clientApps.Stop (Seconds (10.0));

  //Animation
  AnimationInterface anim ("anim1.xml");
  anim.SetConstantPosition(nodes.Get(0), 1.0, 2.0);
  anim.SetConstantPosition(nodes.Get(1), 2.0, 3.0);

  //Run then destroy the Simulation.
  Simulator::Run ();
  Simulator::Destroy ();
  return 0;
}
