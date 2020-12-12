//TODO: a lot of these includes are leftovers from experimental changes.
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
#include "ns3/network-module.h"
#include "ns3/aodv-module.h"
#include "ns3/olsr-module.h"
#include "ns3/dsdv-module.h"
#include "ns3/dsr-module.h"
#include "ns3/applications-module.h"
#include "ns3/yans-wifi-helper.h"
#include <ns3/log.h>
#include <ns3/propagation-loss-model.h>
#include <ns3/propagation-delay-model.h>
#include <ns3/simulator.h>
#include <ns3/single-model-spectrum-channel.h>
#include <ns3/constant-position-mobility-model.h>
#include <ns3/packet.h>
#include "ns3/config-store-module.h"
#include "ns3/dsdv-helper.h"
#include "ns3/point-to-point-module.h"
#include "ns3/v4ping-helper.h"
#include "ns3/energy-module.h"
#include "ns3/wifi-radio-energy-model-helper.h"
#include "ns3/stats-module.h" //GNU Plot Helper

#include <string>
#include <cmath>
#include <iostream>
#include <fstream>
#include <iomanip>

using namespace ns3;
using namespace dsr;


int packetsReceived{0};
double TotalTime{301.0}; //5 min simulation extra second is to write values on last second

/// Trace function for remaining energy at node.
void RemainingEnergy(double oldValue, double remainingEnergy)
{
	//Only print if there is a change of at least 0.01
	if( std::fabs(oldValue - remainingEnergy) > 0.01)
	{
	   NS_LOG_UNCOND(Simulator::Now().GetSeconds()
			<< "s Current remaining energy = " << std::setprecision(7) << remainingEnergy << "J");
	}
}

/// Trace function for total energy consumption at node.
void TotalEnergy(double oldValue, double totalEnergy)
{
    NS_LOG_UNCOND(Simulator::Now().GetSeconds()
        << "s Total energy consumed by radio = " << totalEnergy << "J");
}

/// Trace function for the power harvested by the energy harvester.
void HarvestedPower(double oldValue, double harvestedPower)
{
    NS_LOG_UNCOND(Simulator::Now().GetSeconds()
        << "s Current harvested power = " << harvestedPower << " W");
}

/// Trace function for the total energy harvested by the node.
void TotalEnergyHarvested(double oldValue, double TotalEnergyHarvested)
{
    NS_LOG_UNCOND(Simulator::Now().GetSeconds()
        << "s Total energy harvested by harvester = "
        << TotalEnergyHarvested << " J");
}


static inline std::string PrintReceivedPacket (Ptr<Socket> socket, Ptr<Packet> packet, Address senderAddress)
{
  std::ostringstream oss;

  oss << Simulator::Now ().GetSeconds () << " " << socket->GetNode ()->GetId ();

  if (InetSocketAddress::IsMatchingType (senderAddress))
  {
      InetSocketAddress addr = InetSocketAddress::ConvertFrom (senderAddress);
      oss << " received one packet from " << addr.GetIpv4 ();
  }

  return oss.str ();
}


void ReceivePacket (Ptr<Socket> socket)
{
  Ptr<Packet> packet;
  Address senderAddress;
  while ((packet = socket->RecvFrom (senderAddress)))
  {
      packetsReceived += 1;
      NS_LOG_UNCOND (PrintReceivedPacket (socket, packet, senderAddress));
  }
}


//for each node in the list setup applications to be sending to each other so that traffic continues to be generated throughout simulation
void setup_packets_to_be_sent(NodeContainer& nodes, Ipv4InterfaceContainer& adhocInterfaces, OnOffHelper& onoff1 )
{
	TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");

	for (unsigned int i = 0; i < nodes.GetN(); i++)
	{
		Ptr<Socket> sink = Socket::CreateSocket (nodes.Get(i), tid);
		InetSocketAddress local = InetSocketAddress (adhocInterfaces.GetAddress (i), 9);
		sink->Bind (local);
		sink->SetRecvCallback (MakeCallback (ReceivePacket));

		for( unsigned int j = 0; j < nodes.GetN(); ++j)
		{
			AddressValue remoteAddress (InetSocketAddress (adhocInterfaces.GetAddress (i), 9));
			onoff1.SetAttribute ("Remote", remoteAddress);

			Ptr<UniformRandomVariable> var = CreateObject<UniformRandomVariable> ();

			ApplicationContainer temp = onoff1.Install (nodes.Get(j));
			temp.Start (Seconds (var->GetValue (0.0,1.0)));
			temp.Stop (Seconds (TotalTime));
		}
	  }
}

InternetStackHelper setup_internet_stack( int routing_protocol, NodeContainer& nodes)
{
	InternetStackHelper internet;
	OlsrHelper olsr;
    AodvHelper aodv;
    DsdvHelper dsdv;
    DsrHelper dsr;
    DsrMainHelper dsrMain;

    switch( routing_protocol)
        {
        case 1:
            NS_LOG_UNCOND ("AODV ROUTING ENABLED");
            internet.SetRoutingHelper(aodv);
            internet.Install(nodes);
        	break;
        case 2:
            NS_LOG_UNCOND ("OLSR ROUTING ENABLED");
            internet.SetRoutingHelper(olsr);
            internet.Install(nodes);
        	break;
        case 3:
            NS_LOG_UNCOND ("DSDV ROUTING ENABLED");
            internet.SetRoutingHelper(dsdv);
            internet.Install(nodes);
        	break;
        case 4:
            NS_LOG_UNCOND("DSR ROUTING ENABLED");
            internet.Install(nodes);
            dsrMain.Install(dsr, nodes);
            break;
        }

    return internet;
}

NetDeviceContainer setup_net_devices( NodeContainer& nodes, std::string phyMode)
{
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

    return wifi.Install (wifiPhy, wifiMac, nodes);
}

MobilityHelper setup_mobility(int mobility_model=0)
{
    MobilityHelper mobility;

    // Default Static Position Allocator
    mobility.SetPositionAllocator("ns3::GridPositionAllocator",
        "MinX", DoubleValue(0.0),
        "MinY", DoubleValue(0.0),
        "DeltaX", DoubleValue(80),
        "DeltaY", DoubleValue(80),
        "GridWidth", UintegerValue(5), //SETS NUMBER OF NODES IN A ROW
        "LayoutType", StringValue("RowFirst"));

    // Setting Mobility Model
    switch (mobility_model)
    {
    /**Each instance moves with a speed and direction chosen at random
    * with the user - provided random variables until
    * either a fixed distance has been walked or until a fixed amount
    * of time.If we hit one of the boundaries(specified by a rectangle),
    * of the model, we rebound on the boundary with a reflexive angle
    *and speed.This model is often identified as a brownian motion
    * model.
    */
    case 1:
        NS_LOG_UNCOND("USING RANDOM WALK 2D MOBILITY MODEL");
        mobility.SetMobilityModel("ns3::RandomWalk2dMobilityModel",
            "Mode", StringValue("Time"), // Time mode
            "Time", StringValue("2s"), // Time until direction change
            "Speed", StringValue("ns3::ConstantRandomVariable[Constant=1.0]"), // random speed picked at each interval
            "Bounds", StringValue("100|100|100|100")); // walkable bounds
        break;
    /**Each object starts by pausing at time zero for the duration governed
    * by the random variable "Pause".After pausing, the object will pick
    * a new waypoint(via the PositionAllocator) and a new random speed
    * via the random variable "Speed", and will begin moving towards the
    * waypoint at a constant speed.When it reaches the destination,
    * the process starts over(by pausing).
    */
    case 2:
        NS_LOG_UNCOND("USING RANDOM WAYPOINT MOBILITY MODEL");
        mobility.SetMobilityModel("ns3::RandomWaypointMobilityModel",
            // "Position", mobility.GetPosition(), // Position Allocator defines bounds on model
            "Speed", StringValue("ns3::ConstantRandomVariable[Constant=1.0]"), // Random speed picked at each interval
            "Pause", StringValue("ns3::ConstantRandomVariable[Constant=1.0]")); // Random pause time
        break;
    /**
    * The movement of objects is based on random directions: each object
    * pauses for a specific delay, chooses a random direction and speed and
    * then travels in the specific direction until it reaches one of
    * the boundaries of the model. When it reaches the boundary, it pauses,
    * selects a new direction and speed, aso.
    */
    case 3:
        NS_LOG_UNCOND("USING RANDOM DIRECTION 2D MOBILITY MODEL");
        mobility.SetMobilityModel("ns3::RandomDirection2dMobilityModel",
            "Direction", StringValue("ns3::ConstantRandomVariable[Constant=1.0]"), // random direction after wall hit
            "Speed", StringValue("ns3::ConstantRandomVariable[Constant=1.0]"), // random speed to use after wall hit
            "Pause", StringValue("ns3::ConstantRandomVariable[Constant=1.0]"), // random duration to pause at wall time
            "Bounds", StringValue("0|80|0|80")); // walkable bounds
         
    // Default is a static node mode;
    default:
        NS_LOG_UNCOND("USING CONSTANT POSITION MODEL");
        mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    }

    return mobility;
}

//Report energy consumption callback.
//writes out to txt file.
//this function is self scheduling i.e. it reschedules itself for every two seconds.
std::ofstream output_file;
DeviceEnergyModelContainer deviceModels;

void Report_Energy_Consumption()
{
	auto time = Simulator::Now().GetSeconds();
	double energyConsumed = 0;

	for (DeviceEnergyModelContainer::Iterator iter = deviceModels.Begin (); iter != deviceModels.End (); iter++)
	{
		energyConsumed += (*iter)->GetTotalEnergyConsumption ();
	}
	output_file << time << "	" << packetsReceived << "	" << energyConsumed << "\n";

    Simulator::Schedule (Seconds (2.0), &Report_Energy_Consumption);
}


int main (int argc, char** argv)
{
    //LogComponentEnable ("EnergySource", LOG_LEVEL_DEBUG);
    //LogComponentEnable ("BasicEnergySource", LOG_LEVEL_DEBUG);
    //LogComponentEnable ("DeviceEnergyModel", LOG_LEVEL_DEBUG);
    //LogComponentEnable ("WifiRadioEnergyModel", LOG_LEVEL_DEBUG);

    // Energy Harvester variables
   //double harvestingUpdateInterval = 1;  // seconds

    // Starting Energy Source Value
    double basicEnergySourceInitialEnergyJ = 20000; // pprox. 9 volt battery (19440 J )

    // Default Wifi Model Energy Costs
    double transmitCurrent = 0.0174; // Amps
    double recieveCurrent = 0.0197; // Amps

	// first arg is routing protocol 1 = aodv, 2 = olsr, 3 = dsdv, dsr=4
	auto routing_protocol = std::stoi(argv[1]);
	auto nodes_count = std::stoi(argv[2]);

    // third arg is mobility model, default no mobility, 1: Random2Walk, 2: RandomWaypoint, 3: Random Direction 
    auto mobility_model = std::stoi(argv[3]);

	output_file.open ("aodv_25nodes_static_grid.txt");
    NS_LOG_UNCOND ("START TOTAL ENERGY IN EXPERIMENT: " << (nodes_count * basicEnergySourceInitialEnergyJ));
    NS_LOG_UNCOND ("NODES IN EXPERIMENT " + std::string(argv[2]));

	Packet::EnablePrinting ();

    std::string rate ("2048bps");
    std::string phyMode ("DsssRate11Mbps");
    std::string tr_name ("manet-routing-compare");

    Config::SetDefault  ("ns3::OnOffApplication::PacketSize",StringValue ("1000"));
    Config::SetDefault ("ns3::OnOffApplication::DataRate",  StringValue (rate));
    //Config::SetDefault ("ns3::WifiRemoteStationManager::NonUnicastMode",StringValue (phyMode));

    // Create Nodes
    NodeContainer nodes;
    nodes.Create(nodes_count);

    // Create configured Net Devices (wifi and physical layer setup)
    NetDeviceContainer adhocDevices = setup_net_devices(nodes, phyMode);

    // Set Mobility
    MobilityHelper mobility = setup_mobility(mobility_model);
    mobility.Install (nodes);

    // Set Routing and Network Layer
    InternetStackHelper internet = setup_internet_stack(routing_protocol, nodes);

    Ipv4AddressHelper addressAdhoc;
    addressAdhoc.SetBase ("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer adhocInterfaces;
    adhocInterfaces = addressAdhoc.Assign (adhocDevices);

    // ON/OFF device behavior
    OnOffHelper onoff1 ("ns3::UdpSocketFactory",Address ());
    onoff1.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1.0]"));
    onoff1.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0.0]"));

    /************************** Energy Model ************************/
    // Configure and install energy source
    BasicEnergySourceHelper basicSourceHelper;
    basicSourceHelper.Set ("BasicEnergySourceInitialEnergyJ", DoubleValue (basicEnergySourceInitialEnergyJ));
    EnergySourceContainer sources = basicSourceHelper.Install (nodes);

    // configure radio energy model
    WifiRadioEnergyModelHelper radioEnergyHelper;
    
    double voltage = 9.0; // volts

    radioEnergyHelper.Set("TxCurrentA", DoubleValue (transmitCurrent)); // transmission current
    radioEnergyHelper.Set("RxCurrentA", DoubleValue (recieveCurrent)); // receive current

    radioEnergyHelper.SetTxCurrentModel ("ns3::LinearWifiTxCurrentModel",
                                           "Voltage", DoubleValue (voltage) );

    // install device model
    deviceModels = radioEnergyHelper.Install (adhocDevices, sources);

    //Energy Harvester adds power to the device
    //BasicEnergyHarvesterHelper basicHarvesterHelper;

    // configure energy harvester
    //basicHarvesterHelper.Set("PeriodicHarvestedPowerUpdateInterval", TimeValue(Seconds(harvestingUpdateInterval)));
    //basicHarvesterHelper.Set("HarvestablePower", StringValue("ns3::UniformRandomVariable[Min=0.0|Max=0.1]"));

    // install harvester on all energy sources
    //EnergyHarvesterContainer harvesters = basicHarvesterHelper.Install(sources);

    // Connect trace sources
    for (unsigned int i = 0; i < sources.GetN(); i++)
   	{
    	 Ptr<BasicEnergySource> basicSourcePtr = DynamicCast<BasicEnergySource>(sources.Get(i));
    	 basicSourcePtr->TraceConnectWithoutContext("RemainingEnergy", MakeCallback(&RemainingEnergy) );

    	 // device energy model
    	// Ptr<DeviceEnergyModel> basicRadioModelPtr = basicSourcePtr->FindDeviceEnergyModels("ns3::WifiRadioEnergyModel").Get(i);

    	// NS_ASSERT(basicRadioModelPtr);
    	// basicRadioModelPtr->TraceConnectWithoutContext("TotalEnergyConsumption", MakeCallback(&TotalEnergy));

   	     // energy harvester
    	// Ptr<BasicEnergyHarvester> basicHarvesterPtr = DynamicCast<BasicEnergyHarvester>(harvesters.Get(1));
    	// basicHarvesterPtr->TraceConnectWithoutContext("HarvestedPower", MakeCallback(&HarvestedPower));
    	// basicHarvesterPtr->TraceConnectWithoutContext("TotalEnergyHarvested", MakeCallback(&TotalEnergyHarvested));
   	}

    //Setup packets that will be sent during the simulation.
    setup_packets_to_be_sent(nodes, adhocInterfaces, onoff1 );

    //creating animation interface will create xml runnable by netanim
    AnimationInterface anim ("manet_simulation.xml");

    Simulator::Schedule (Seconds (2.0), &Report_Energy_Consumption);

    Simulator::Stop (Seconds (TotalTime));
    Simulator::Run ();

    double total_consumed = 0;
    for (DeviceEnergyModelContainer::Iterator iter = deviceModels.Begin (); iter != deviceModels.End (); iter ++)
        {
          double energyConsumed = (*iter)->GetTotalEnergyConsumption ();
          total_consumed += energyConsumed;
          NS_LOG_UNCOND ("End of simulation (" << Simulator::Now ().GetSeconds ()
                         << "s) Total energy consumed by radio = " << energyConsumed << "J");
        }

    NS_LOG_UNCOND("TOTAL CONSUMED = " << std::setprecision(7) << total_consumed << " J");

    NS_LOG_UNCOND("Packets Received = " << packetsReceived);

    Simulator::Destroy ();
    output_file.close();
}
