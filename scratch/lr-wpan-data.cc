#include <ns3/log.h>
#include <ns3/core-module.h>
#include <ns3/lr-wpan-module.h>
#include <ns3/propagation-loss-model.h>
#include <ns3/propagation-delay-model.h>
#include <ns3/simulator.h>
#include <ns3/single-model-spectrum-channel.h>
#include <ns3/constant-position-mobility-model.h>
#include <ns3/packet.h>
#include <iostream>
#include <string>
#include <cmath>
#include "ns3/netanim-module.h"
#include "ns3/sixlowpan-module.h"
#include "ns3/applications-module.h"
#include "ns3/config-store-module.h"
#include "ns3/dsdv-helper.h"
#include "ns3/aodv-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/v4ping-helper.h"
#include "ns3/yans-wifi-helper.h"



using namespace ns3;

static void DataIndication (McpsDataIndicationParams params, Ptr<Packet> p)
{
	//size of 48 is sent by aodv routing protocol beacons?
  if(p->GetSize() != 48)
  {
	  NS_LOG_UNCOND ("Received packet of size " << p->GetSize ());
  }
}

static void DataConfirm (McpsDataConfirmParams params)
{
	// 0 = success
	if(params.m_status != 0)
	{
		  NS_LOG_UNCOND ("BAD DATA CONFIRM STATUS: " << params.m_status);
	}
}


//how wide the grid will be (# of nodes = GRID_WIDTH * GRIDWIDTH)
const int GRID_WIDTH = 3;

int main (int argc, char *argv[])
{
  bool verbose = false;
  bool extended = false;

  CommandLine cmd (__FILE__);
  cmd.AddValue ("verbose", "turn on all log components", verbose);
  cmd.AddValue ("extended", "use extended addressing", extended);
  cmd.Parse (argc, argv);

  LrWpanHelper lrWpanHelper;
  if (verbose)
  {
	  lrWpanHelper.EnableLogComponents ();
  }

  //Config::SetDefault  ("ns3::OnOffApplication::PacketSize",StringValue ("64"));
  //Config::SetDefault ("ns3::OnOffApplication::DataRate",  StringValue ("2048bps"));

  // Each device must be attached to the same channel
  Ptr<SingleModelSpectrumChannel> channel = CreateObject<SingleModelSpectrumChannel> ();
  Ptr<LogDistancePropagationLossModel> propModel = CreateObject<LogDistancePropagationLossModel> ();
  Ptr<ConstantSpeedPropagationDelayModel> delayModel = CreateObject<ConstantSpeedPropagationDelayModel> ();
  channel->AddPropagationLossModel (propModel);
  channel->SetPropagationDelayModel (delayModel);
  lrWpanHelper.SetChannel(channel);

  //Create our base nodes
  NodeContainer adHocNodes;
  adHocNodes.Create(GRID_WIDTH * GRID_WIDTH);




  //set up mobility to be static and positioning in a grid
  MobilityHelper mobility;
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");

  mobility.SetPositionAllocator ("ns3::GridPositionAllocator",
                                   "MinX", DoubleValue (0.0),
                                   "MinY", DoubleValue (0.0),
                                   "DeltaX", DoubleValue (100),
                                   "DeltaY", DoubleValue (100),
                                   "GridWidth", UintegerValue (3),
                                   "LayoutType", StringValue ("RowFirst"));

    mobility.Install (adHocNodes);


    AodvHelper aodv;
   // you can configure AODV attributes here using aodv.Set(name, value)

   InternetStackHelper internet;
   internet.SetRoutingHelper (aodv); // has effect on the next Install ()
   internet.Install (adHocNodes);


   //Install Devices on nodes
   NetDeviceContainer adHocDevices = lrWpanHelper.Install(adHocNodes);

   Ipv4AddressHelper address;
   address.SetBase ("10.1.1.0", "255.255.255.0");
   Ipv4InterfaceContainer interfaces = address.Assign (adHocDevices);

  //set up callback
  McpsDataConfirmCallback cb0;
  cb0 = MakeCallback (&DataConfirm);

  McpsDataIndicationCallback cb1;
  cb1 = MakeCallback (&DataIndication);

  McpsDataConfirmCallback cb2;
  cb2 = MakeCallback (&DataConfirm);

  McpsDataIndicationCallback cb3;
  cb3 = MakeCallback (&DataIndication);

  AnimationInterface anim ("PHIL_TEST.xml");


  for(unsigned int i = 0; i < adHocNodes.GetN(); ++i)
  {

	  //set address, this is a bad way to do this
	  Ptr<LrWpanNetDevice> device = DynamicCast<LrWpanNetDevice> (adHocDevices.Get(i));

	  device->GetMac()->SetMcpsDataConfirmCallback (cb0);
	  device->GetMac()->SetMcpsDataIndicationCallback (cb1);
	  device->GetMac()->SetMcpsDataConfirmCallback (cb2);
	  device->GetMac()->SetMcpsDataIndicationCallback (cb3);

  }


  //actual sending of packets
  Ptr<LrWpanNetDevice> dev0 = DynamicCast<LrWpanNetDevice> (adHocDevices.Get(0));
  Ptr<LrWpanNetDevice> dev8 = DynamicCast<LrWpanNetDevice> (adHocDevices.Get(8));



  // Tracing
  lrWpanHelper.EnablePcapAll (std::string ("lr-wpan-data"), true);
  AsciiTraceHelper ascii;
  Ptr<OutputStreamWrapper> stream = ascii.CreateFileStream ("lr-wpan-data.tr");
  lrWpanHelper.EnableAsciiAll (stream);

  // The below should trigger two callbacks when end-to-end data is working
  // 1) DataConfirm callback is called
  // 2) DataIndication callback is called with value of 50
  McpsDataRequestParams params;
  params.m_dstPanId = 0;
  params.m_srcAddrMode = SHORT_ADDR;
  params.m_dstAddrMode = SHORT_ADDR;
  params.m_dstAddr = dev0->GetMac()->GetShortAddress();
  params.m_msduHandle = 0;
  params.m_txOptions = TX_OPTION_ACK;

  // Send a packet back at time 2 seconds
  Ptr<Packet> p2 = Create<Packet> (60);  // 60 bytes of dummy data


  NS_LOG_UNCOND ("PACKET UID: " << p2->GetUid());


  Simulator::ScheduleWithContext (2, Seconds (15.0),
                                  &LrWpanMac::McpsDataRequest,
                                  dev8->GetMac(), params, p2);


  Simulator::Stop (Seconds (30));

  Simulator::Run ();

  Simulator::Destroy ();
  return 0;
}
