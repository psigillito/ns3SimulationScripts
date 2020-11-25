#include <fstream>
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/internet-apps-module.h"
#include "ns3/mobility-module.h"
#include "ns3/spectrum-module.h"
#include "ns3/propagation-module.h"
#include "ns3/sixlowpan-module.h"
#include "ns3/lr-wpan-module.h"
#include "ns3/csma-module.h"
#include "ns3/netanim-module.h"
#include <fstream>
#include <iostream>
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/aodv-module.h"
#include "ns3/olsr-module.h"
#include "ns3/dsdv-module.h"
#include "ns3/dsr-module.h"
#include "ns3/applications-module.h"
#include "ns3/yans-wifi-helper.h"

/*TODO this needs to get cleaned up, I was lazy with the copy-pasta*/
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
#include "ns3/energy-module.h"
#include "ns3/wifi-radio-energy-model-helper.h"



using namespace ns3;


int bytesTotal{0};
int packetsReceived{0};
double TotalTime{200.0};

/// Trace function for remaining energy at node.
/*static void RemainingEnergy (double oldValue, double remainingEnergy)
{
  NS_LOG_UNCOND (Simulator::Now ().GetSeconds ()
                 << "s Current remaining energy = " << remainingEnergy << "J");
}

/// Trace function for total energy consumption at node.
static void TotalEnergy (double oldValue, double totalEnergy)
{
  NS_LOG_UNCOND (Simulator::Now ().GetSeconds ()
                 << "s Total energy consumed by radio = " << totalEnergy << "J");
}
*/

static inline std::string
PrintReceivedPacket (Ptr<Socket> socket, Ptr<Packet> packet, Address senderAddress)
{
  std::ostringstream oss;

  oss << Simulator::Now ().GetSeconds () << " " << socket->GetNode ()->GetId ();

  if (InetSocketAddress::IsMatchingType (senderAddress))
    {
      InetSocketAddress addr = InetSocketAddress::ConvertFrom (senderAddress);
      oss << " received one packet from " << addr.GetIpv4 ();
    }
  else
    {
      oss << " received one packet!";
    }
  return oss.str ();
}


void
ReceivePacket (Ptr<Socket> socket)
{
  Ptr<Packet> packet;
  Address senderAddress;
  while ((packet = socket->RecvFrom (senderAddress)))
    {
      bytesTotal += packet->GetSize ();
      packetsReceived += 1;
      NS_LOG_UNCOND (PrintReceivedPacket (socket, packet, senderAddress));
    }
}


int main (int argc, char** argv)
{
	//convert to int
	auto routing_protocol = std::stoi(argv[1]);

	Packet::EnablePrinting ();

    std::string rate ("2048bps");
    std::string phyMode ("DsssRate11Mbps");
    std::string tr_name ("manet-routing-compare");

    Config::SetDefault  ("ns3::OnOffApplication::PacketSize",StringValue ("64"));
    Config::SetDefault ("ns3::OnOffApplication::DataRate",  StringValue (rate));
    Config::SetDefault ("ns3::WifiRemoteStationManager::NonUnicastMode",StringValue (phyMode));




    //************************ create nodes ******************************************
    int nodes_count = 20;
    NodeContainer nodes;
    nodes.Create(nodes_count);

    //***********************set up net devices / Wifi Mac and Physical ************
    WifiHelper wifi;
    wifi.SetStandard (WIFI_STANDARD_80211b);

    YansWifiPhyHelper wifiPhy =  YansWifiPhyHelper::Default ();
    YansWifiChannelHelper wifiChannel;
    wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
    wifiChannel.AddPropagationLoss ("ns3::FriisPropagationLossModel");
    wifiPhy.SetChannel (wifiChannel.Create ());

    WifiMacHelper wifiMac;
    wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                  "DataMode",StringValue (phyMode),
                                  "ControlMode",StringValue (phyMode));

    wifiPhy.Set ("TxPowerStart",DoubleValue (7.5));
    wifiPhy.Set ("TxPowerEnd", DoubleValue (7.5));

    wifiMac.SetType ("ns3::AdhocWifiMac");
    NetDeviceContainer adhocDevices = wifi.Install (wifiPhy, wifiMac, nodes);

    /*********************** set mobility  *************/

    //set the mobility on our nodes
    MobilityHelper mobility;
    mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
    mobility.SetPositionAllocator ("ns3::GridPositionAllocator",
                                   "MinX", DoubleValue (0.0),
                                   "MinY", DoubleValue (0.0),
                                   "DeltaX", DoubleValue (80),
                                   "DeltaY", DoubleValue (80),
                                   "GridWidth", UintegerValue (10),
                                   "LayoutType", StringValue ("RowFirst"));
    mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
    mobility.Install (nodes);

    /*********************** set up routing **********************/

    InternetStackHelper internet;

	OlsrHelper olsr;
    AodvHelper aodv;
    DsdvHelper dsdv;


    switch( routing_protocol)
    {
    case 1:
        NS_LOG_UNCOND ("AODV ROUTING ENABLED");
        internet.SetRoutingHelper(aodv);
    	break;
    case 2:
        NS_LOG_UNCOND ("OLSR ROUTING ENABLED");
        internet.SetRoutingHelper(olsr);
    	break;
    case 3:
        NS_LOG_UNCOND ("DSDV ROUTING ENABLED");
        internet.SetRoutingHelper(dsdv);
    	break;
    }

    internet.SetRoutingHelper(aodv);
    internet.Install (nodes);

    Ipv4AddressHelper addressAdhoc;
    addressAdhoc.SetBase ("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer adhocInterfaces;
    adhocInterfaces = addressAdhoc.Assign (adhocDevices);

    /***************************** ON/OFF device behaviour **********/
    OnOffHelper onoff1 ("ns3::UdpSocketFactory",Address ());
    onoff1.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1.0]"));
    onoff1.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0.0]"));

    /************************** Energy Model ************************/

    // energy source
 /*   BasicEnergySourceHelper basicSourceHelper;
    // configure energy source
    basicSourceHelper.Set ("BasicEnergySourceInitialEnergyJ", DoubleValue (0.1));
    // install source
    EnergySourceContainer sources = basicSourceHelper.Install (nodes);
    // device energy model
    WifiRadioEnergyModelHelper radioEnergyHelper;
    // configure radio energy model
    radioEnergyHelper.Set ("TxCurrentA", DoubleValue (0.0174));
    // install device model
    DeviceEnergyModelContainer deviceModels = radioEnergyHelper.Install (adhocDevices, sources);
*/
    /** connect trace sources **/
    /***************************************************************************/
    // all sources are connected to node 1
    // energy source
/*    Ptr<BasicEnergySource> basicSourcePtr = DynamicCast<BasicEnergySource> (sources.Get (1));
    basicSourcePtr->TraceConnectWithoutContext ("RemainingEnergy", MakeCallback (&RemainingEnergy));
    // device energy model
    Ptr<DeviceEnergyModel> basicRadioModelPtr =
    basicSourcePtr->FindDeviceEnergyModels ("ns3::WifiRadioEnergyModel").Get (0);
    NS_ASSERT (basicRadioModelPtr != NULL);
    basicRadioModelPtr->TraceConnectWithoutContext ("TotalEnergyConsumption", MakeCallback (&TotalEnergy));
*/

    /***************** Sending Packets *********************/
    //for each node in the list send to individual packet to each other node.
    //this loop completes before simulation starts so packets are not going to be reported received in any particular order.
    TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");

    for (unsigned int i = 0; i < nodes.GetN(); i++)
    {
        for( unsigned int j = 0; j < nodes.GetN(); ++j)
        {
         	Ptr<Socket> sink = Socket::CreateSocket (nodes.Get(i), tid);
			InetSocketAddress local = InetSocketAddress (adhocInterfaces.GetAddress (i), 9/*port number*/);
			sink->Bind (local);
			sink->SetRecvCallback (MakeCallback (ReceivePacket));

			AddressValue remoteAddress (InetSocketAddress (adhocInterfaces.GetAddress (i), 9));
			onoff1.SetAttribute ("Remote", remoteAddress);

			Ptr<UniformRandomVariable> var = CreateObject<UniformRandomVariable> ();

            ApplicationContainer temp = onoff1.Install (nodes.Get(j));
            temp.Start (Seconds (var->GetValue (100.0,101.0)));
            temp.Stop (Seconds (TotalTime));
        }
      }


    NS_LOG_UNCOND ("RUNNNING EXPERIMENT");


    AnimationInterface anim ("PHIL_TEST.xml");

    Simulator::Stop (Seconds (TotalTime));
    Simulator::Run ();

    NS_LOG_UNCOND ("Blargh");

    /*for (DeviceEnergyModelContainer::Iterator iter = deviceModels.Begin (); iter != deviceModels.End (); iter ++)
    {
      double energyConsumed = (*iter)->GetTotalEnergyConsumption ();
      NS_LOG_UNCOND ("End of simulation (" << Simulator::Now ().GetSeconds ()
                     << "s) Total energy consumed by radio = " << energyConsumed << "J");
      NS_ASSERT (energyConsumed <= 0.1);
    }*/


    Simulator::Destroy ();


}
