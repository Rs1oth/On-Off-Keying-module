#include <iostream>
#include <fstream>
#include <ostream>
#include <string>
#include <cassert>
#include <sstream>

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/ipv4-static-routing-helper.h"
#include "ns3/ipv4-list-routing-helper.h"
#include "ns3/netanim-module.h"
#include "ns3/on-off-keying-module-helper.h"
#include "ns3/applications-module.h"
#include "ns3/PAM-error-model.h"
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
static const uint32_t totalTxBytes = 100000; //The simulation with send 1000000 bytes in data packets (not including overhead)
static uint32_t currentTxBytes = 0;
static const uint32_t writeSize = 1040; // How big each packet will be, default for TCP is 536 w/out headers
uint8_t data[writeSize];
void StartFlow (Ptr<Socket>, Ipv4Address, uint16_t);//send data
void WriteUntilBufferFull (Ptr<Socket>, uint32_t);


void SendStuff(Ptr<Socket> sock, Ipv4Address dstaddr, uint16_t port);
void BindSock(Ptr<Socket> sock, Ptr<NetDevice> netdev);
void srcSocketRecv(Ptr<Socket> Socket);
void dstSocketRecv(Ptr<Socket> Socket);
/*
static void
Trace (Ptr<const Packet> p)
{
  NS_LOG_UNCOND ("Sent Mt");
}
*/
Ptr<PacketSink> sink1;
std::vector<double> Received (1,0);
std::vector<double> theTime (1,0);

static void
RxEnd (Ptr<const Packet> p) //used for tracing and calculating throughput
{
  if(Received.back() != sink1->GetTotalRx()){
    Received.push_back(sink1->GetTotalRx());
    theTime.push_back(Simulator::Now().GetSeconds());
  }

  //Received.push_back(Received.back() + p->GetSize()); //appends on the received packet to the received data up until that packet and adds that total to the end of the vector
  //theTime.push_back(Simulator::Now().GetSeconds()); // keeps track of the time during simulation that a packet is received

 // NS_LOG_UNCOND ("Received : "<< p->GetSize() << " Bytes at " << Simulator::Now ().GetSeconds () <<"s" );
}

/*static void
TxEnd (Ptr<const Packet> p)//also used as a trace and for calculating throughput
{
 //NS_LOG_UNCOND ("Sent : "<< p->GetSize() << " Bytes at " << Simulator::Now ().GetSeconds () <<"s" );
  Received.push_back(Received.back() + p->GetSize()); //same as for the RxEnd trace
  theTime.push_back(Simulator::Now().GetSeconds());   //
 
}*/

int main (int argc, char *argv[])
{
std::ofstream myfile;
myfile.open("PAMBER.dat");

std::ofstream myfile2;
myfile2.open("PAMSNR.dat");

std::ofstream myfile3;
myfile3.open("PAMGOODPUT.dat");

std::ofstream myfile4;
myfile4.open("GvS.dat");



  for(double dist = 22 ; dist > 0 ; dist-=.01){ //loops the simulation by increasing distance between nodes

//creating each node object
Ptr<Node> wifiAp = CreateObject<Node>();
Ptr<Node> relayAp = CreateObject<Node>();
Ptr<Node> relayMt = CreateObject<Node>();
Ptr<Node> wifiMt = CreateObject<Node>();

//puts all the nodes into one place
NodeContainer c = NodeContainer(wifiAp,relayAp,relayMt,wifiMt);

InternetStackHelper internet; //This helper handles making all the components of the internet stack that will be layered on top on the already exsisting network
internet.Install(c);

//This helper sets up the P2P connections that we will be using
PointToPointHelper p2p;
p2p.SetDeviceAttribute("DataRate", StringValue("5Mbps"));
p2p.SetChannelAttribute("Delay", StringValue("2ms"));
NetDeviceContainer ndAp_Relay = p2p.Install(wifiAp, relayAp);
//VLC---------------------------------------------------------
  Ptr<VlcMobilityModel> a = CreateObject<VlcMobilityModel> ();//These vectors are what represent the nodes moving
  Ptr<VlcMobilityModel> b = CreateObject<VlcMobilityModel> ();//in space

  a -> SetPosition (Vector (0.0,0.0,22.0));
  b -> SetPosition (Vector (0.0,0.0,dist));
  a ->SetAzimuth(0.0);
  b ->SetAzimuth(0.0);
  a ->SetElevation(180.0);
  b ->SetElevation(0.0);

  //Instaniates an Error model to use on the VLC net devices
  PAMErrorModel *em2 ;
  PAMErrorModel x;
  em2 = &x;

  //Sets the initial conditions of the transmitter and receiver in the VLC network
  VLCPropagationLossModel VPLM;
  VPLM.SetTxPower(48.573);
  VPLM.SetLambertianOrder(70);
  VPLM.SetFilterGain(1);
  VPLM.SetPhotoDetectorArea(1.0e-4);
  VPLM.SetConcentratorGain(70,1.5);

  //Also initial conditions, but these are made in the error model since thats where the values are used to calculate BER
  //em2->setRes(0.28);
  em2->setM(8);
  em2->setNo(380,380,5000,100e6,VPLM.GetPhotoDetectorArea(),VPLM.GetRxPower(a,b));
  double sym = log2(em2->getM()) *5;
  std::ostringstream ss;
  ss << sym;
  //std::cout << ss.str() <<std::endl;
OOKHelper OOK; // This helper makes the VLC channel that we are going to use
  OOK.SetDeviceAttribute ("DataRate", StringValue (ss.str() + "Mbps"));
  OOK.SetChannelAttribute ("Delay", StringValue ("2ms"));
  NetDeviceContainer ndRelayAp_RelayMt2 = OOK.Install(relayAp, relayMt);

  ndRelayAp_RelayMt2.Get (1)->SetAttribute ("ReceiveErrorModel", PointerValue (em2)); // putting the error model on the netdevice

//------------------------------------------------------------
//Wifi--------------------------------------------------------
 std::string phyMode ("DsssRate1Mbps");
  double rss = -80;  // -dBm

NodeContainer cont = NodeContainer(relayMt, relayAp);

  // The below set of helpers will help us to put together the wifi NICs we want
  WifiHelper wifi;
 
  wifi.SetStandard (WIFI_PHY_STANDARD_80211b);

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
  NetDeviceContainer ndRelayAp_RelayMt3 = wifi.Install (wifiPhy, wifiMac, cont);

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
NetDeviceContainer ndRelay_Mt = p2p.Install(relayMt, wifiMt);

//The following sets up address bases for out net devices on the nodes so as to identify them on a routing table as we do
Ipv4AddressHelper ipv4;
ipv4.SetBase("10.1.1.0", "255.255.255.0"); //used for the WIFI AP ------- Relay AP      point to point
Ipv4InterfaceContainer iAp = ipv4.Assign(ndAp_Relay);

ipv4.SetBase("10.1.2.0", "255.255.255.0"); // used for the Relay AP --------- Relay MT      VLC 
Ipv4InterfaceContainer iRelayApMt = ipv4.Assign(ndRelayAp_RelayMt2);

ipv4.SetBase("10.1.3.0", "255.255.255.0"); //used for the Relay MT ------- Relay AP       WIFI
Ipv4InterfaceContainer iRelayMtAp = ipv4.Assign(ndRelayAp_RelayMt3);

ipv4.SetBase("10.1.4.0", "255.255.255.0");//used for Relay MT -------- MT       point to point
Ipv4InterfaceContainer iMt = ipv4.Assign(ndRelay_Mt);

//The following sets up each nodes routing table that will be statically added to
Ptr<Ipv4> ipv4Ap = wifiAp->GetObject<Ipv4>();
Ptr<Ipv4> ipv4RelayAp = relayAp->GetObject<Ipv4>();
Ptr<Ipv4> ipv4RelayMt = relayMt->GetObject<Ipv4>();
Ptr<Ipv4> ipv4Mt = wifiMt->GetObject<Ipv4>();

Ipv4StaticRoutingHelper ipv4RoutingHelper;

Ptr<Ipv4StaticRouting> staticRoutingAp = ipv4RoutingHelper.GetStaticRouting(ipv4Ap);

Ptr<Ipv4StaticRouting> staticRoutingRelayAp = ipv4RoutingHelper.GetStaticRouting(ipv4RelayAp);

Ptr<Ipv4StaticRouting> staticRoutingRelayMt = ipv4RoutingHelper.GetStaticRouting(ipv4RelayMt);

Ptr<Ipv4StaticRouting> staticRoutingMt = ipv4RoutingHelper.GetStaticRouting(ipv4Mt);


//The following are the specific routes added to various routing tables and this current scheme is modeing a VLC downlink and a WIFI uplink
staticRoutingAp->AddHostRouteTo(Ipv4Address("10.1.4.2"), Ipv4Address("10.1.1.2"), 1,1);///
staticRoutingRelayAp->AddHostRouteTo(Ipv4Address("10.1.4.2"), Ipv4Address("10.1.2.2"), 2,1);// This block is for sending from WIFI AP to the MT
staticRoutingRelayMt->AddHostRouteTo(Ipv4Address("10.1.4.2"), Ipv4Address("10.1.4.2"), 3,1);//

staticRoutingMt->AddHostRouteTo(Ipv4Address("10.1.1.1"), Ipv4Address("10.1.4.1"), 1,1);///
staticRoutingRelayMt->AddHostRouteTo(Ipv4Address("10.1.1.1"), Ipv4Address("10.1.3.2"), 2,1);// This block is for sending back information from MT to WIFI AP
staticRoutingRelayAp->AddHostRouteTo(Ipv4Address("10.1.1.1"), Ipv4Address("10.1.1.1"), 1,1);//

  //This sets up various sockets on the same node as to allow multiple TCP connections to be made as to pass information through the net devices
  Ptr<Socket> srcSocket1 = Socket::CreateSocket (wifiAp, TypeId::LookupByName ("ns3::TcpSocketFactory"));
  Ptr<Socket> srcSocket2 = Socket::CreateSocket (wifiAp, TypeId::LookupByName ("ns3::TcpSocketFactory"));
  Ptr<Socket> srcSocket3 = Socket::CreateSocket (wifiAp, TypeId::LookupByName ("ns3::TcpSocketFactory"));
  Ptr<Socket> srcSocket4 = Socket::CreateSocket (wifiAp, TypeId::LookupByName ("ns3::TcpSocketFactory"));

  uint16_t dstport = 12345;
  Ipv4Address dstaddr ("10.1.4.2");//destination 

  PacketSinkHelper sink ("ns3::TcpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), dstport)); //setting a sink on a node
  ApplicationContainer apps = sink.Install (wifiMt);
  sink1 = DynamicCast<PacketSink>(apps.Get(0));
  apps.Start (Seconds (0.0));
  apps.Stop (Seconds (10.0));
 
//the following is used for logging and various debugging purposes
AsciiTraceHelper ascii;
p2p.EnableAsciiAll(ascii.CreateFileStream ("RoutingTestCase.tr"));
p2p.EnablePcapAll("RoutingTestCase");

LogComponentEnableAll(LOG_PREFIX_TIME);
LogComponentEnable("RoutingTestCase", LOG_LEVEL_INFO);

Ptr<OutputStreamWrapper> stream1 = Create<OutputStreamWrapper> ("Table3", std::ios::out);
ipv4RoutingHelper.PrintRoutingTableAllAt(Seconds(2.0), stream1);



ndRelay_Mt.Get (1)->TraceConnectWithoutContext ("PhyRxEnd", MakeCallback (&RxEnd));//traces to allow us to see what and when data is sent through the network

//ndRelay_Mt.Get (1)->TraceConnectWithoutContext ("PhyTxEnd", MakeCallback (&TxEnd));//traces to allow us to see what and when data is received through the network

//Simulator schedules
Simulator::Schedule(Seconds(0.1), &StartFlow,srcSocket1, dstaddr, dstport);
Simulator::Run();

double goodput = ((Received.back()*8))/ theTime.back();//goodput calculation
std::cout<<"-------------------------"<< std::endl;
//std::cout<<"Received : " << Received.back() << std::endl;
std::cout<<"Distance : " << dist << std::endl;
//std::cout<<"Time : " << theTime.back() << std::endl;
//std::cout<<"THROUGHPUT : " << throughput << std::endl;
//std::cout<<"SER : " << em2->getSER() << std::endl;



myfile << dist << " " << em2->getSER()*log2(em2->getM()) << std::endl;
myfile2 << dist << " " << em2->getSNR() << std::endl;
myfile3 << dist << " " << goodput << std::endl;
myfile4 << em2->getSNR() << " " << goodput << std::endl;

Received.clear(); // clears the data received vector so as to avoid calculation errors from old and irrelevant values

Simulator::Destroy();


  }


return 0;
}

   
void BindSock (Ptr<Socket> sock, Ptr<NetDevice> netdev)
   {
     sock->BindToNetDevice (netdev);
     return;
   }
void StartFlow (Ptr<Socket> localSocket,
                Ipv4Address servAddress,
                uint16_t servPort)
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

void WriteUntilBufferFull (Ptr<Socket> localSocket, uint32_t txSpace)
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
