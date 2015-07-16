#include <iostream>
#include <fstream>
#include <ostream>
#include <string>
#include <cassert>

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/ipv4-static-routing-helper.h"
#include "ns3/ipv4-list-routing-helper.h"
#include "ns3/netanim-module.h"
#include "ns3/on-off-keying-module-helper.h"
#include "ns3/applications-module.h"
#include "ns3/OOK-error-model.h"
#include "ns3/OOK-error-model-2Interference.h"
#include "ns3/vlc-propagation-loss-model.h"
#include "ns3/packet-sink.h"
#include "ns3/gnuplot.h"
#include "ns3/double.h"
#include "ns3/mobility-module.h"
#include "ns3/VLC-Mobility-Model.h"
#include "ns3/wifi-module.h"
#include <vector>
#include <cmath>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("RoutingTestCase");
static const uint32_t totalTxBytes = 1000;    //Sents 1000 bytes
static uint32_t currentTxBytes = 0;          
static const uint32_t writeSize = 1040;      //Write 1040 bytes
uint8_t data[writeSize];
void StartFlow (Ptr<Socket>, Ipv4Address, uint16_t);  //Method for the socket to start sending
void WriteUntilBufferFull (Ptr<Socket>, uint32_t);    // Method to keep writing untill the Buffer is full


void SendStuff(Ptr<Socket> sock, Ipv4Address dstaddr, uint16_t port); //Old method not used
void BindSock(Ptr<Socket> sock, Ptr<NetDevice> netdev);  //Method used to bind the socket to destination
void srcSocketRecv(Ptr<Socket> Socket); // Call back for when the Source Socket Receives data
void dstSocketRecv(Ptr<Socket> Socket); // Call back for when the Destination Socket Receives data
/*
static void
Trace (Ptr<const Packet> p)
{
  NS_LOG_UNCOND ("Sent Mt");
}
*/
Ptr<PacketSink> sink1;   //Packet sink for first pair of nodes
Ptr<PacketSink> sink2;   //Packet sink for the Second pair of nodes
std::vector<double> Received (1,0); //Vector containing the total data received by MT1 as the last element
std::vector<double> Received2 (1,0); //Vector containing the total data received by MT2 as the last element
std::vector<double> ReceivedT (1,0); // Vector containing the total data received by both MTs
std::vector<double> theTime (1,0); //Vector of every time stamp when data was received by MT1
std::vector<double> theTime2 (1,0);//Vector of every time stamp when data was received by MT2
std::vector<double> theTimeT (1,0);//Vector of every time stamp when data was received by both MTs

static void
RxEnd (Ptr<const Packet> p) //Call back method that is used keeps track of received 
{
  Received.push_back(Received.back() + p->GetSize()); // Pushes the previous data size with the current packet size tot he back of Received
  theTime.push_back(Simulator::Now().GetSeconds()); //Pushes the current time to the back of theTime


   ReceivedT.push_back(ReceivedT.back() + p->GetSize());  // Pushes the previous data size with the current packet size tot he back of ReceivedT
  theTimeT.push_back(Simulator::Now().GetSeconds());//Pushes the current time to the back of theTimeT
//  NS_LOG_UNCOND ("Received : "<< p->GetSize() << " Bytes at " << Simulator::Now ().GetSeconds () <<"s" );
}

static void
TxEnd (Ptr<const Packet> p)
{
 //NS_LOG_UNCOND ("Sent : "<< p->GetSize() << " Bytes at " << Simulator::Now ().GetSeconds () <<"s" );
  Received.push_back(Received.back() + p->GetSize());  // Pushes the previous data size with the current packet size tot he back of Received
  theTime.push_back(Simulator::Now().GetSeconds());//Pushes the current time to the back of theTime
 
     ReceivedT.push_back(ReceivedT.back() + p->GetSize());  // Pushes the previous data size with the current packet size tot he back of ReceivedT
  theTimeT.push_back(Simulator::Now().GetSeconds());//Pushes the current time to the back of theTimeT
   
}

static void
RxEnd2 (Ptr<const Packet> p)
{

  Received2.push_back(Received2.back() + p->GetSize());// Pushes the previous data size with the current packet size tot he back of Received2
  theTime2.push_back(Simulator::Now().GetSeconds());//Pushes the current time to the back of theTime2

     ReceivedT.push_back(ReceivedT.back() + p->GetSize());// Pushes the previous data size with the current packet size tot he back of ReceivedT
  theTimeT.push_back(Simulator::Now().GetSeconds());//Pushes the current time to the back of theTimeT

  //NS_LOG_UNCOND ("Received : "<< p->GetSize() << " Bytes at " << Simulator::Now ().GetSeconds () <<"s" );
}

static void
TxEnd2 (Ptr<const Packet> p)
{
 //NS_LOG_UNCOND ("Sent : "<< p->GetSize() << " Bytes at " << Simulator::Now ().GetSeconds () <<"s" );
  Received2.push_back(Received2.back() + p->GetSize());// Pushes the previous data size with the current packet size tot he back of Received2
  theTime2.push_back(Simulator::Now().GetSeconds());//Pushes the current time to the back of theTime2
 

      ReceivedT.push_back(ReceivedT.back() + p->GetSize());// Pushes the previous data size with the current packet size tot he back of ReceivedT
  theTimeT.push_back(Simulator::Now().GetSeconds());//Pushes the current time to the back of theTimeT
}



int main (int argc, char *argv[])
{
std::ofstream myfile1; 
myfile1.open("BER.dat");  //Opens a file for the simulation to write to



for(double pow1 = 0.0 ; pow1 < 200 ; pow1 += 1.0){ //Loops through 0.0 dbm to 200 dbm for the Main Nodes's Tx Power
  for(double pow = 0.0 ; pow < 200 ; pow+=1.0){  //Loops through 0.0 dbm to 200 dbm for the Interferer's Nodes's Tx Power

Ptr<Node> Ap = CreateObject<Node>();  //Creates a Node for the Server
Ptr<Node> RouterAp = CreateObject<Node>(); //Creates a Node to act as a Router for the Sever and contains the VLC Transmitters
Ptr<Node> relay1 = CreateObject<Node>(); // Creates a Node to act as a Relay and to Contain the VLC Receiver for the Main node
Ptr<Node> Mt1 = CreateObject<Node>();//Creates a Node to be the Client for the Main node
Ptr<Node> relay2 = CreateObject<Node>();// Creates a Node to act as a Relay and to Contain the VLC Receiver for the Interferer node
Ptr<Node> Mt2 = CreateObject<Node>();//Creates a Node to be the Client for the Interferer node


NodeContainer c = NodeContainer(Ap,RouterAp); //Creates a Node Container to contain every node
c.Add(relay1);
c.Add(Mt1);
c.Add(relay2);
c.Add(Mt2);


InternetStackHelper internet;
internet.Install(c); //Installs a InternetStack on all nodes in the Node Container

PointToPointHelper p2p;
p2p.SetDeviceAttribute("DataRate", StringValue("200Mbps"));
p2p.SetChannelAttribute("Delay", StringValue("2ms"));
NetDeviceContainer ndAp_Router = p2p.Install(Ap, RouterAp);  //Sets up a point to point channel and Net devices between the Server and Router
                                                            //Sets the Net device to have a data Rate of 200Mbps and the chennel to have a 2ms Delay
//VLC---------------------------------------------------------
 OOKHelper OOK;
  OOK.SetDeviceAttribute ("DataRate", StringValue ("5Mbps"));
  OOK.SetChannelAttribute ("Delay", StringValue ("2ms"));       //Makes OOK help with a Data rate of 5mbps and delay of 2ms
  
  NetDeviceContainer ndRouterAp_RelayMt1 = OOK.Install(RouterAp, relay1); // Install this link onto the Router and the Main Node's Relay 
  NetDeviceContainer ndRouterAp_RelayMt2 = OOK.Install(RouterAp, relay2); // Intsall another link onto the Router and the Interferer Node's Relay

o
  Ptr<VlcMobilityModel> a = CreateObject<VlcMobilityModel> (); //Creates 2 VLC Mobility Model 
  Ptr<VlcMobilityModel> b = CreateObject<VlcMobilityModel> ();  


  a -> SetPosition (Vector (0.0,0.0,3.0)); //Sets the Transmitter for the Main node to 3.0m in the Z Direction (Up)
  b -> SetPosition (Vector (0.0,0.0,1.0));//Sets the Receiver for the Main to to 1.0m in the Z Direction (Up)
  a ->SetAzimuth(0.0);
  b ->SetAzimuth(0.0);
  a ->SetElevation(180.0); //Flips the Transmitter upside down so that the Transmitter and Reciever are pointed right at eachother
  b ->SetElevation(0.0);

  Ptr<VlcMobilityModel> d = CreateObject<VlcMobilityModel> (); //Creates 2 more VLC Mobility model
  Ptr<VlcMobilityModel> e = CreateObject<VlcMobilityModel> ();  

  d -> SetPosition (Vector (0.0,2.0,3.0)); //Sets the Transmitter for the Interferer Node to 2.0m In the Y Direction (Forward) 
                                            // and 3.0m In the Z Direction (Up)
  e -> SetPosition (Vector (0.0,2.0,1.0)); //Sets the Receiver for the Interferer Node to 2.0m In the Y Direction (Forward)
                                            // and 1.0m In the Z Direction (Up)
  d ->SetAzimuth(0.0);
  e ->SetAzimuth(0.0);
  d ->SetElevation(180.0); //Flips the Transmitter upside down so that the Transmitter and Reciever are pointed right at eachother
  e ->SetElevation(0.0);


  OOK2IntErrorModel *em2 ;
  OOK2IntErrorModel x;
  em2 = &x;//Creates a Pointer to an Error Model for the Main Node

  VLCPropagationLossModel VPLM;
  VPLM.SetTxPower(pow1);    //Sets power of Transmitter
  VPLM.SetLambertianOrder(70);  //Sets the Semi angle of Transmitter
  VPLM.SetFilterGain(1); //Sets filter gain
  VPLM.SetPhotoDetectorArea(1.0e-4);//Sets the Area of the Photo detector in m^2
  VPLM.SetConcentratorGain(70,1.5); //Sets FOV and Refractive Index


  
 
  OOK2IntErrorModel *em3 ;
  OOK2IntErrorModel y;
  em3 = &y; //Creates a Pointer to an Error Model for the Interferer Node

  VLCPropagationLossModel VPLM2;
  VPLM2.SetTxPower(pow); // Sets power of Transmitter
  VPLM2.SetLambertianOrder(70); //Sets Semi angle of Transmitter
  VPLM2.SetFilterGain(1); //Sets filter gain
  VPLM2.SetPhotoDetectorArea(1.0e-4); //Sets the Area of the Photo detector in m^2
  VPLM2.SetConcentratorGain(70,1.5); //Sets the FOV and Refreactive Index


  em2->setNo(380,380,5000,100.0e6, VPLM.GetPhotoDetectorArea(),VPLM.GetRxPower(a,b), VPLM2.GetRxPower(d,b));
    //Sets the Noise power with inputs of (Lower Wave Length, Upper Wave Length, Temp, Noise Bandwith , Photo Detector Area, Power of Main, and Power of Interferer)
  em2->setIntNo( 380,380,5000,100.0e6, VPLM.GetPhotoDetectorArea());
  //Sets the Interferer's Noide with inputs of (Lower wave Length, Upper wave Length, Temp, Noise Bandwidth, and PhotoDetector Area)
  em3->setNo(380,380,5000,100.0e6, VPLM2.GetPhotoDetectorArea(),VPLM2.GetRxPower(d,e),VPLM.GetRxPower(a,e));
 //Sets the Noise power with inputs of (Lower Wave Length, Upper Wave Length, Temp, Noise Bandwith , Photo Detector Area, Power of Main, and Power of Interferer)
em3->setIntNo(380,380,5000,100.0e6, VPLM2.GetPhotoDetectorArea());
  //Sets the Interferer's Noide with inputs of (Lower wave Length, Upper wave Length, Temp, Noise Bandwidth, and PhotoDetector Area)
  ndRouterAp_RelayMt1.Get (1)->SetAttribute ("ReceiveErrorModel", PointerValue (em2)); //Puts this Error model (em2) on the Relay
  ndRouterAp_RelayMt1.Get (0)->SetAttribute ("ReceiveErrorModel", PointerValue (em2)); //Puts this Error model (em2) on the Router

  ndRouterAp_RelayMt2.Get (1)->SetAttribute ("ReceiveErrorModel", PointerValue (em3)); //Puts this Error model (em3) on the Relay
  ndRouterAp_RelayMt2.Get (0)->SetAttribute ("ReceiveErrorModel", PointerValue (em3)); //Puts this Error model (em3) on the Router




//------------------------------------------------------------
//Wifi--------------------------------------------------------
 std::string phyMode ("DsssRate11Mbps"); //Sets the Wifi uplink speed to 11Mbps
  double rss = -80;  // -dBm

NodeContainer cont = NodeContainer(RouterAp, relay1); //Creats a Node Container to contain the nodes that have the wifi uplink
cont.Add(relay2);

  // The below set of helpers will help us to put together the wifi NICs we want
  WifiHelper wifi;
 
  wifi.SetStandard (WIFI_PHY_STANDARD_80211b); //Creates a Wifi helper and sets it to 802.11b
  YansWifiPhyHelper wifiPhy =  YansWifiPhyHelper::Default ();
  // This is one parameter that matters when using FixedRssLossModel
  // set it to zero; otherwise, gain will be added
  wifiPhy.Set ("RxGain", DoubleValue (0) );  
  // ns-3 supports RadioTap and Prism tracing extensions for 802.11b
  wifiPhy.SetPcapDataLinkType (YansWifiPhyHelper::DLT_IEEE802_11_RADIO); 

  YansWifiChannelHelper wifiChannel;
  wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
  // The below FixedRssLossModel will cause the rss to be fixed regardless
  // of the distance between the two stations, and the transmit power
  wifiChannel.AddPropagationLoss ("ns3::FixedRssLossModel","Rss",DoubleValue (rss));
  wifiPhy.SetChannel (wifiChannel.Create ());

  // Add a non-QoS upper mac, and disable rate control
  NqosWifiMacHelper wifiMac = NqosWifiMacHelper::Default ();
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                "DataMode",StringValue (phyMode),
                                "ControlMode",StringValue (phyMode));
  // Set it to adhoc mode
  wifiMac.SetType ("ns3::AdhocWifiMac");
  NetDeviceContainer ndRelayMt_RouterAp = wifi.Install (wifiPhy, wifiMac, cont);

  // Note that with FixedRssLossModel, the positions below are not 
  // used for received signal strength. 
  MobilityHelper mobility;
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  positionAlloc->Add (Vector (0.0, 0.0, 0.0));
  positionAlloc->Add (Vector (5.0, 0.0, 0.0));
  mobility.SetPositionAllocator (positionAlloc);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (cont);  

//NetDeviceContainer ndRelayAp_RelayMt3 = p2p.Install(relayMt,relayAp);
//-------------------------------------------------------------
NetDeviceContainer ndRelay_Mt1 = p2p.Install(relay1, Mt1); //Installs Point to Points Link between Relay and Mt of Main
NetDeviceContainer ndRelay_Mt2 = p2p.Install(relay2, Mt2);//Installs Point to Points Link between Relay and Mt of Interferer



Ipv4AddressHelper ipv4;
ipv4.SetBase("10.1.1.0", "255.255.255.0");
Ipv4InterfaceContainer iAp = ipv4.Assign(ndAp_Router); //Puts the First subnet on The Server and Router

ipv4.SetBase("10.1.2.0", "255.255.255.0");
Ipv4InterfaceContainer iRelayApMt1 = ipv4.Assign(ndRouterAp_RelayMt1); //Puts the Second subnet on the Router and Relay of Main

ipv4.SetBase("10.1.3.0", "255.255.255.0");
Ipv4InterfaceContainer iRelayApMt2 = ipv4.Assign(ndRouterAp_RelayMt2); // Puts the Third subnet on the Router and the Relay of Interfers


ipv4.SetBase("10.1.5.0", "255.255.255.0");
Ipv4InterfaceContainer iwifi = ipv4.Assign(ndRelayMt_RouterAp); //Puts the fifth subnet on the Relays to Router

ipv4.SetBase("10.1.6.0", "255.255.255.0");
Ipv4InterfaceContainer iMt1 = ipv4.Assign(ndRelay_Mt1); //Puts the sixth subnet on the Relay to MT of Main

ipv4.SetBase("10.1.7.0", "255.255.255.0");
Ipv4InterfaceContainer iMt2 = ipv4.Assign(ndRelay_Mt2); //Puts the seventh sub net on the Relay to MT of Interferer



Ptr<Ipv4> ipv4Ap = Ap->GetObject<Ipv4>(); //Makes and Ipv4 object of the Server
Ptr<Ipv4> ipv4RouterAp = RouterAp->GetObject<Ipv4>(); //Make an Ipv4 object of the Router
Ptr<Ipv4> ipv4RelayMt1 = relay1->GetObject<Ipv4>(); //Makes an Ipv4 object of the Main's Relay
Ptr<Ipv4> ipv4RelayMt2 = relay2->GetObject<Ipv4>();//Makes an Ipv4 object of the Interferer's Relay 

Ptr<Ipv4> ipv4Mt1 = Mt1->GetObject<Ipv4>(); //Makes an Ipv4 object of the Main's MT
Ptr<Ipv4> ipv4Mt2 = Mt2->GetObject<Ipv4>(); //Makes and Ipv4 object of the Interferer's MT 


Ipv4StaticRoutingHelper ipv4RoutingHelper;

Ptr<Ipv4StaticRouting> staticRoutingAp = ipv4RoutingHelper.GetStaticRouting(ipv4Ap); //Makes a StaticRouting Object for the Server
Ptr<Ipv4StaticRouting> staticRoutingRouterAp = ipv4RoutingHelper.GetStaticRouting(ipv4RouterAp);//Makes a StaticRouting Object for the Router
Ptr<Ipv4StaticRouting> staticRoutingRelayMt1 = ipv4RoutingHelper.GetStaticRouting(ipv4RelayMt1);//Makes a StaticRouting Object for the Main's Relay
Ptr<Ipv4StaticRouting> staticRoutingRelayMt2 = ipv4RoutingHelper.GetStaticRouting(ipv4RelayMt2);//Makes a StaticRouting Object for the Interferer's Relay

Ptr<Ipv4StaticRouting> staticRoutingMt1 = ipv4RoutingHelper.GetStaticRouting(ipv4Mt1);//Makes a StaticRouting Object for the Main's MT
Ptr<Ipv4StaticRouting> staticRoutingMt2 = ipv4RoutingHelper.GetStaticRouting(ipv4Mt2);//Makes a StaticRouting Object for the Interferer's MT




staticRoutingAp->AddHostRouteTo(Ipv4Address("10.1.6.2"), Ipv4Address("10.1.1.2"), 1,3);  //Creates a Route from Sever to Main's MT
staticRoutingRouterAp->AddHostRouteTo(Ipv4Address("10.1.6.2"), Ipv4Address("10.1.2.2"), 2,2); //Creates a Route from Router to Main's MT
staticRoutingRelayMt1->AddHostRouteTo(Ipv4Address("10.1.6.2"), Ipv4Address("10.1.6.2"), 3,1);//Creates a Route from Relay to Main's MT

staticRoutingAp->AddHostRouteTo(Ipv4Address("10.1.7.2"), Ipv4Address("10.1.1.2"), 1,3); //Creates a Route from Sever to Interferer's MT
staticRoutingRouterAp->AddHostRouteTo(Ipv4Address("10.1.7.2"), Ipv4Address("10.1.3.2"), 3,2);//Creates a Route from Router to Interferer's MT
staticRoutingRelayMt2->AddHostRouteTo(Ipv4Address("10.1.7.2"), Ipv4Address("10.1.7.2"), 3,1);//Creates a Route from Relay to Interferer's MT



staticRoutingMt1->AddHostRouteTo(Ipv4Address("10.1.1.1"), Ipv4Address("10.1.6.1"), 1,3); //Creates Route from Main's MT to Server
staticRoutingRelayMt1->AddHostRouteTo(Ipv4Address("10.1.1.1"), Ipv4Address("10.1.5.1"), 2,2);//Creates Route from Main's Relay to Server
staticRoutingRouterAp->AddHostRouteTo(Ipv4Address("10.1.1.1"), Ipv4Address("10.1.1.1"), 1,1);//Creates Route from Router to Server

staticRoutingMt2->AddHostRouteTo(Ipv4Address("10.1.1.1"), Ipv4Address("10.1.7.1"), 1,3);//Creates Route from Interferer's MT to Server
staticRoutingRelayMt2->AddHostRouteTo(Ipv4Address("10.1.1.1"), Ipv4Address("10.1.5.1"), 2,2);//Creates Route from Interferer's Relay to Server
staticRoutingRouterAp->AddHostRouteTo(Ipv4Address("10.1.1.1"), Ipv4Address("10.1.1.1"), 1,1);//Creates Route from Router to Server

;

 Ptr<Socket> srcSocket1 = Socket::CreateSocket (Ap, TypeId::LookupByName ("ns3::TcpSocketFactory"));
  Ptr<Socket> srcSocket2 = Socket::CreateSocket (Ap, TypeId::LookupByName ("ns3::TcpSocketFactory"));
  Ptr<Socket> srcSocket3 = Socket::CreateSocket (Ap, TypeId::LookupByName ("ns3::TcpSocketFactory"));
  Ptr<Socket> srcSocket4 = Socket::CreateSocket (Ap, TypeId::LookupByName ("ns3::TcpSocketFactory"));
 Ptr<Socket> srcSocket5 = Socket::CreateSocket (Ap, TypeId::LookupByName ("ns3::TcpSocketFactory"));
  Ptr<Socket> srcSocket6 = Socket::CreateSocket (Ap, TypeId::LookupByName ("ns3::TcpSocketFactory")); //Makes 6 sockets on the Server to communicat to all nodes


  uint16_t dstport1 = 12345;
  Ipv4Address dstaddr1 ("10.1.6.2"); //Sets up a Destination port and Address on Main's MT

  PacketSinkHelper sink ("ns3::TcpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), dstport1)); //Sets up sink for TCP and with the Destination Port
  ApplicationContainer apps = sink.Install (Mt1); //Install Packet Sink APP on Main's MT
  sink1 = DynamicCast<PacketSink>(apps.Get(0)); //Sets sink1 to the sink on Main's MT
  apps.Start (Seconds (0.0));
  apps.Stop (Seconds (10.0)); //Starts and Stop the App at 0.0s to 10.0s
 
 uint16_t dstport2 = 12346;
  Ipv4Address dstaddr2 ("10.1.7.2");//Sets up a Destination port and Address on Interferer's MT

  PacketSinkHelper sinka ("ns3::TcpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), dstport2));//Sets up sink for TCP and with the Destination Port
  ApplicationContainer apps2 = sinka.Install (Mt2);//Install Packet Sink APP on Interferer's MT
  sink2 = DynamicCast<PacketSink>(apps2.Get(0)); //Sets sink2 to the sink on Interferer's MT
  apps.Start (Seconds (0.0));
  apps.Stop (Seconds (10.0));//Starts and Stop the App at 0.0s to 10.0s


AsciiTraceHelper ascii;
p2p.EnableAsciiAll(ascii.CreateFileStream ("RoutingTestCase.tr"));
p2p.EnablePcapAll("RoutingTestCase"); //Sets up a PCAP Trace (But we never use it)

LogComponentEnableAll(LOG_PREFIX_TIME);
LogComponentEnable("RoutingTestCase", LOG_LEVEL_INFO); //Sets up a Log (But we never use it)

Ptr<OutputStreamWrapper> stream1 = Create<OutputStreamWrapper> ("Table3", std::ios::out);
ipv4RoutingHelper.PrintRoutingTableAllAt(Seconds(2.0), stream1); //Creates a File names Table3 where the Routing Table is printed to





ndRelay_Mt1.Get (1)->TraceConnectWithoutContext ("PhyRxEnd", MakeCallback (&RxEnd)); //Puts a RX trace on Main's MT

ndRelay_Mt1.Get (1)->TraceConnectWithoutContext ("PhyTxEnd", MakeCallback (&TxEnd));//Puts a TX trace on Main's MT

ndRelay_Mt2.Get (1)->TraceConnectWithoutContext ("PhyRxEnd", MakeCallback (&RxEnd2));//Puts a RX trace on Interferer's MT

ndRelay_Mt2.Get (1)->TraceConnectWithoutContext ("PhyTxEnd", MakeCallback (&TxEnd2));//Puts a TX trace on Interferer's MT


Simulator::Schedule(Seconds(0.1), &StartFlow,srcSocket2, dstaddr1, dstport1); //Starts a flow from Server to Main's MT

Simulator::Schedule(Seconds(0.1), &StartFlow,srcSocket3, dstaddr2, dstport2); //Starts a flow from Server to Interferer's MT

Simulator::Run(); //Runs the Simulation

//double throughput1 = ((Received.back()*8))/ theTime.back();
//double throughput2 = ((Received2.back()*8))/ theTime2.back();
double totalThroughput = ((ReceivedT.back()*8))/ theTimeT.back(); //Calculats Total Throughput of the whole system

std::cout<<"-------------------------"<< std::endl;
//std::cout<<"Received : " << ReceivedT.back() << std::endl;
std::cout<<"Pow1 : " << pow1 << std::endl;
std::cout<<"Pow : " << pow << std::endl;
//std::cout<<"Time : " << theTimeT.back() << std::endl;
std::cout<<"THROUGHPUT : " << totalThroughput << std::endl;
std::cout<<"BER : " << em2->getBER() << std::endl;
  std::cout << "BER 2 : " << em3->getBER() <<std::endl;
std::cout<<"INR : " << em2->getINR() << std::endl;
std::cout<<"SNR : " << em2->getSNR() << std::endl;
myfile1 <<em2->getBER() << " "; //Writes BER to the File


Received.clear();
Received2.clear();
ReceivedT.clear(); //Clears all the Received vectors



Simulator::Destroy(); //Destroies Current Simulation

}
myfile1<<std::endl; //Adds White space to end of file


}


myfile1.close(); //Closes the File we wrote to



return 0;
}

   
void BindSock (Ptr<Socket> sock, Ptr<NetDevice> netdev) //Method used to Bind to sockets
   {
     sock->BindToNetDevice (netdev);
     return;
   }
void StartFlow (Ptr<Socket> localSocket, 
                Ipv4Address servAddress,
                uint16_t servPort) //Method that Send data between sockets
{
  //NS_LOG_INFO ("Starting flow at time " <<  Simulator::Now ().GetSeconds ());
  currentTxBytes = 0;
  localSocket->Bind ();
  localSocket->Connect (InetSocketAddress (servAddress, servPort)); //connect

  // tell the tcp implementation to call WriteUntilBufferFull again
  // if we blocked and new tx buffer space becomes available
  localSocket->SetSendCallback (MakeCallback (&WriteUntilBufferFull));
  WriteUntilBufferFull (localSocket, localSocket->GetTxAvailable ());
}

void WriteUntilBufferFull (Ptr<Socket> localSocket, uint32_t txSpace) //Method that writes until the buffer is filled
{
  while (currentTxBytes < totalTxBytes && localSocket->GetTxAvailable () > 0)
    {
      uint32_t left = totalTxBytes - currentTxBytes;
      uint32_t dataOffset = currentTxBytes % writeSize;
      uint32_t toWrite = writeSize - dataOffset;
      toWrite = std::min (toWrite, left);
      toWrite = std::min (toWrite, localSocket->GetTxAvailable ());
      int amountSent = localSocket->Send (&data[dataOffset], toWrite, 0);
      if(amountSent < 0)
        {
          // we will be called again when new tx space becomes available.
          return;
        }
      currentTxBytes += amountSent;
    }
  localSocket->Close ();
}
