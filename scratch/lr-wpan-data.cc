/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2011 The Boeing Company
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
 * Author:  Tom Henderson <thomas.r.henderson@boeing.com>
 */

/*
 * Try to send data end-to-end through a LrWpanMac <-> LrWpanPhy <->
 * SpectrumChannel <-> LrWpanPhy <-> LrWpanMac chain
 *
 * Trace Phy state changes, and Mac DataIndication and DataConfirm events
 * to stdout
 */
#include <ns3/log.h>
#include <ns3/core-module.h>
#include <ns3/lr-wpan-module.h>
#include <ns3/propagation-loss-model.h>
#include <ns3/propagation-delay-model.h>
#include <ns3/simulator.h>
#include <ns3/single-model-spectrum-channel.h>
#include <ns3/constant-position-mobility-model.h>
#include <ns3/packet.h>

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/csma-module.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/applications-module.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/netanim-module.h"

#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include <iostream>

using namespace ns3;

/*static void DataIndication (McpsDataIndicationParams params, Ptr<Packet> p)
{
  NS_LOG_UNCOND ("Received packet of size " << p->GetSize ());
}

static void DataConfirm (McpsDataConfirmParams params)
{
  NS_LOG_UNCOND ("LrWpanMcpsDataConfirmStatus = " << params.m_status);
}

static void StateChangeNotification (std::string context, Time now, LrWpanPhyEnumeration oldState, LrWpanPhyEnumeration newState)
{
  NS_LOG_UNCOND (context << " state change at " << now.As (Time::S)
                         << " from " << LrWpanHelper::LrWpanPhyEnumerationPrinter (oldState)
                         << " to " << LrWpanHelper::LrWpanPhyEnumerationPrinter (newState));
}*/

//How many nodes we want in the simulation
const int NUMBER_OF_NODES = 10;

int main (int argc, char *argv[])
{
  std::srand (time(NULL));

  bool verbose = true;
  bool extended = false;

  CommandLine cmd (__FILE__);
  cmd.AddValue ("verbose", "turn on all log components", verbose);
  cmd.AddValue ("extended", "use extended addressing", extended);
  cmd.Parse (argc, argv);

  //Helper used for configuring and creating our devices/nodes.
  LrWpanHelper lrWpanHelper;

  if (verbose)
  {
	  lrWpanHelper.EnableLogComponents ();
  }

  // Set the shared channel settings
  Ptr<SingleModelSpectrumChannel> channel = CreateObject<SingleModelSpectrumChannel> ();
  Ptr<LogDistancePropagationLossModel> propModel = CreateObject<LogDistancePropagationLossModel> ();
  Ptr<ConstantSpeedPropagationDelayModel> delayModel = CreateObject<ConstantSpeedPropagationDelayModel> ();
  channel->AddPropagationLossModel (propModel);
  channel->SetPropagationDelayModel (delayModel);

  //set it so that all devices use the same channel
  lrWpanHelper.SetChannel(channel);

  //create nodes. Think of Nodes as a generic logical base object.
  NodeContainer zigBeeNodes;
  zigBeeNodes.Create (NUMBER_OF_NODES);

  // create the devices that are going to be 'installed' on the nodes.
  // Devices are installed by the helper.
  NetDeviceContainer zigBeeDevices;
  zigBeeDevices = lrWpanHelper.Install(zigBeeNodes);

  // Enable packet capture for tracing and output to lr-wpan-data.tr"
  lrWpanHelper.EnablePcapAll (std::string ("lr-wpan-data"), true);
  AsciiTraceHelper ascii;
  Ptr<OutputStreamWrapper> stream = ascii.CreateFileStream ("lr-wpan-data.tr");
  lrWpanHelper.EnableAsciiAll (stream);

  //set the mobility model for the devices.
  //Mobility model sets the 'physical' position and velocity of the device
  //x,y,z are location coordinates of device
  int x,y,z;

  for( int i = 0; i < NUMBER_OF_NODES; ++i)
  {
	  Ptr<LrWpanNetDevice> device = DynamicCast<LrWpanNetDevice> (zigBeeDevices.Get(i));
	  Ptr<ConstantPositionMobilityModel> sender0Mobility = CreateObject<ConstantPositionMobilityModel> ();

	  //set location to random value
	  x = rand() % 50;
	  y = rand() % 50;
	  z = rand() % 50;

	  sender0Mobility->SetPosition (Vector (x,y,z));
	  device->GetPhy()->SetMobility (sender0Mobility);
  }


  // The below should trigger two callbacks when end-to-end data is working
  // 1) DataConfirm callback is called
  // 2) DataIndication callback is called with value of 50
  Ptr<Packet> p0 = Create<Packet> (50);  // 50 bytes of dummy data
  McpsDataRequestParams params;
  params.m_dstPanId = 0;
  if (!extended)
    {
      params.m_srcAddrMode = SHORT_ADDR;
      params.m_dstAddrMode = SHORT_ADDR;
      params.m_dstAddr = Mac16Address ("00:02");
    }
  else
    {
      params.m_srcAddrMode = EXT_ADDR;
      params.m_dstAddrMode = EXT_ADDR;
      params.m_dstExtAddr = Mac64Address ("00:00:00:00:00:00:00:02");
    }
  params.m_msduHandle = 0;
  params.m_txOptions = TX_OPTION_ACK;


  Ptr<LrWpanNetDevice> dev1 = DynamicCast<LrWpanNetDevice> (zigBeeDevices.Get(0));

  //NetDevice* x = zigBeeDevices.Get(0);
  //LrWpanNetDevice* y = dynamic_cast<LrWpanNetDevice *>(x);
  dev1->GetMac()->McpsDataRequest(params, p0);
  Simulator::ScheduleWithContext (1, Seconds (0.0),
                                  &LrWpanMac::McpsDataRequest,
								  dev1->GetMac (), params, p0);

  // Send a packet back at time 2 seconds
  Ptr<Packet> p2 = Create<Packet> (60);  // 60 bytes of dummy data
  if (!extended)
    {
      params.m_dstAddr = Mac16Address ("00:01");
    }
  else
    {
      params.m_dstExtAddr = Mac64Address ("00:00:00:00:00:00:00:01");
    }

  Ptr<LrWpanNetDevice> dev2 = DynamicCast<LrWpanNetDevice> (zigBeeDevices.Get(1));
  Simulator::ScheduleWithContext (2, Seconds (2.0),
                                  &LrWpanMac::McpsDataRequest,
								  dev2->GetMac(), params, p2);


  //set up animation
  AnimationInterface anim ("zigbee_simulation.xml");

  int location = 1;
  for ( auto node_iter = zigBeeNodes.Begin(); node_iter != zigBeeNodes.End(); ++node_iter)
  {
	  anim.SetConstantPosition(*node_iter, location, location + 5 );
  	  ++location;
  }


  Simulator::Run ();
  Simulator::Destroy ();
  return 0;
}
