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
static const uint32_t totalTxBytes = 10000;
static uint32_t currentTxBytes = 0;
static const uint32_t writeSize = 1040;
uint8_t data[writeSize];
void StartFlow (Ptr<Socket>, Ipv4Address, uint16_t);
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
Ptr<PacketSink> sink2;
std::vector<double> Received (1,0);
std::vector<double> Received2 (1,0);
std::vector<double> ReceivedT (1,0);
std::vector<double> theTime (1,0);
std::vector<double> theTime2 (1,0);
std::vector<double> theTimeT (1,0);

static void
RxEnd (Ptr<const Packet> p)
{
  Received.push_back(Received.back() + p->GetSize());
  theTime.push_back(Simulator::Now().GetSeconds());


   ReceivedT.push_back(ReceivedT.back() + p->GetSize());
  theTimeT.push_back(Simulator::Now().GetSeconds());
//  NS_LOG_UNCOND ("Received : "<< p->GetSize() << " Bytes at " << Simulator::Now ().GetSeconds () <<"s" );
}

static void
TxEnd (Ptr<const Packet> p)
{
 //NS_LOG_UNCOND ("Sent : "<< p->GetSize() << " Bytes at " << Simulator::Now ().GetSeconds () <<"s" );
  Received.push_back(Received.back() + p->GetSize());
  theTime.push_back(Simulator::Now().GetSeconds());
 
     ReceivedT.push_back(ReceivedT.back() + p->GetSize());
  theTimeT.push_back(Simulator::Now().GetSeconds());
   
}

static void
RxEnd2 (Ptr<const Packet> p)
{

  Received2.push_back(Received2.back() + p->GetSize());
  theTime2.push_back(Simulator::Now().GetSeconds());

     ReceivedT.push_back(ReceivedT.back() + p->GetSize());
  theTimeT.push_back(Simulator::Now().GetSeconds());

  //NS_LOG_UNCOND ("Received : "<< p->GetSize() << " Bytes at " << Simulator::Now ().GetSeconds () <<"s" );
}

static void
TxEnd2 (Ptr<const Packet> p)
{
 //NS_LOG_UNCOND ("Sent : "<< p->GetSize() << " Bytes at " << Simulator::Now ().GetSeconds () <<"s" );
  Received2.push_back(Received2.back() + p->GetSize());
  theTime2.push_back(Simulator::Now().GetSeconds());
 

      ReceivedT.push_back(ReceivedT.back() + p->GetSize());
  theTimeT.push_back(Simulator::Now().GetSeconds());
}



int main (int argc, char *argv[])
{
std::ofstream myfile1;
myfile1.open("2IntTestThroughputTotal.dat");

std::ofstream myfile2;
myfile2.open("2IntTestThroughput1.dat");


std::ofstream myfile3;
myfile3.open("2IntTestThroughput2.dat");


  for(double pow = 170 ; pow < 172.0 ; pow+=.001){

Ptr<Node> Ap = CreateObject<Node>();
Ptr<Node> RouterAp = CreateObject<Node>();
Ptr<Node> relay1 = CreateObject<Node>();
Ptr<Node> Mt1 = CreateObject<Node>();
Ptr<Node> relay2 = CreateObject<Node>();
Ptr<Node> Mt2 = CreateObject<Node>();


NodeContainer c = NodeContainer(Ap,RouterAp);
c.Add(relay1);
c.Add(Mt1);
c.Add(relay2);
c.Add(Mt2);


InternetStackHelper internet;
internet.Install(c);

PointToPointHelper p2p;
p2p.SetDeviceAttribute("DataRate", StringValue("200Mbps"));
p2p.SetChannelAttribute("Delay", StringValue("2ms"));
NetDeviceContainer ndAp_Router = p2p.Install(Ap, RouterAp);
//VLC---------------------------------------------------------
 OOKHelper OOK;
  OOK.SetDeviceAttribute ("DataRate", StringValue ("5Mbps"));
  OOK.SetChannelAttribute ("Delay", StringValue ("2ms"));
  
  NetDeviceContainer ndRouterAp_RelayMt1 = OOK.Install(RouterAp, relay1);
  NetDeviceContainer ndRouterAp_RelayMt2 = OOK.Install(RouterAp, relay2);


  Ptr<VlcMobilityModel> a = CreateObject<VlcMobilityModel> ();
  Ptr<VlcMobilityModel> b = CreateObject<VlcMobilityModel> ();  


  a -> SetPosition (Vector (0.0,0.0,3.0));
  b -> SetPosition (Vector (0.0,0.0,1.0));
  a ->SetAzimuth(0.0);
  b ->SetAzimuth(0.0);
  a ->SetElevation(180.0);
  b ->SetElevation(0.0);

  Ptr<VlcMobilityModel> d = CreateObject<VlcMobilityModel> ();
  Ptr<VlcMobilityModel> e = CreateObject<VlcMobilityModel> ();  

  d -> SetPosition (Vector (0.0,2.0,3.0));
  e -> SetPosition (Vector (0.0,2.0,1.0));
  d ->SetAzimuth(0.0);
  e ->SetAzimuth(0.0);
  d ->SetElevation(180.0);
  e ->SetElevation(0.0);


  OOK2IntErrorModel *em2 ;
  OOK2IntErrorModel x;
  em2 = &x;

  VLCPropagationLossModel VPLM;
  VPLM.SetTxPower(48.573);
  VPLM.SetLambertianOrder(70);
  VPLM.SetFilterGain(1);
  VPLM.SetPhotoDetectorArea(1.0e-4);
  VPLM.SetConcentratorGain(70,1.5);


  
 
  OOK2IntErrorModel *em3 ;
  OOK2IntErrorModel y;
  em3 = &y;

  VLCPropagationLossModel VPLM2;
  VPLM2.SetTxPower(pow);
  VPLM2.SetLambertianOrder(70);
  VPLM2.SetFilterGain(1);
  VPLM2.SetPhotoDetectorArea(1.0e-4);
  VPLM2.SetConcentratorGain(70,1.5);


  em2->setNo(380,380,5000,100.0e6, VPLM.GetPhotoDetectorArea(),VPLM.GetRxPower(a,b), VPLM2.GetRxPower(d,b));

  em2->setIntNo( 380,380,5000,100.0e6, VPLM.GetPhotoDetectorArea());

  em3->setNo(380,380,5000,100.0e6, VPLM2.GetPhotoDetectorArea(),VPLM2.GetRxPower(d,e), VPLM.GetRxPower(a,e));

  em3->setIntNo(380,380,5000,100.0e6, VPLM2.GetPhotoDetectorArea());

  ndRouterAp_RelayMt1.Get (1)->SetAttribute ("ReceiveErrorModel", PointerValue (em2));
  ndRouterAp_RelayMt1.Get (0)->SetAttribute ("ReceiveErrorModel", PointerValue (em2));

  ndRouterAp_RelayMt2.Get (1)->SetAttribute ("ReceiveErrorModel", PointerValue (em3));
  ndRouterAp_RelayMt2.Get (0)->SetAttribute ("ReceiveErrorModel", PointerValue (em3));




//------------------------------------------------------------
//Wifi--------------------------------------------------------
 std::string phyMode ("DsssRate11Mbps");
  double rss = -80;  // -dBm

NodeContainer cont = NodeContainer(RouterAp, relay1);
cont.Add(relay2);

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
NetDeviceContainer ndRelay_Mt1 = p2p.Install(relay1, Mt1);
NetDeviceContainer ndRelay_Mt2 = p2p.Install(relay2, Mt2);




Ipv4AddressHelper ipv4;
ipv4.SetBase("10.1.1.0", "255.255.255.0");
Ipv4InterfaceContainer iAp = ipv4.Assign(ndAp_Router);

ipv4.SetBase("10.1.2.0", "255.255.255.0");
Ipv4InterfaceContainer iRelayApMt1 = ipv4.Assign(ndRouterAp_RelayMt1);

ipv4.SetBase("10.1.3.0", "255.255.255.0");
Ipv4InterfaceContainer iRelayApMt2 = ipv4.Assign(ndRouterAp_RelayMt2);


ipv4.SetBase("10.1.5.0", "255.255.255.0");
Ipv4InterfaceContainer iwifi = ipv4.Assign(ndRelayMt_RouterAp);

ipv4.SetBase("10.1.6.0", "255.255.255.0");
Ipv4InterfaceContainer iMt1 = ipv4.Assign(ndRelay_Mt1);

ipv4.SetBase("10.1.7.0", "255.255.255.0");
Ipv4InterfaceContainer iMt2 = ipv4.Assign(ndRelay_Mt2);



Ptr<Ipv4> ipv4Ap = Ap->GetObject<Ipv4>();
Ptr<Ipv4> ipv4RouterAp = RouterAp->GetObject<Ipv4>();
Ptr<Ipv4> ipv4RelayMt1 = relay1->GetObject<Ipv4>();
Ptr<Ipv4> ipv4RelayMt2 = relay2->GetObject<Ipv4>();

Ptr<Ipv4> ipv4Mt1 = Mt1->GetObject<Ipv4>();
Ptr<Ipv4> ipv4Mt2 = Mt2->GetObject<Ipv4>();


Ipv4StaticRoutingHelper ipv4RoutingHelper;

Ptr<Ipv4StaticRouting> staticRoutingAp = ipv4RoutingHelper.GetStaticRouting(ipv4Ap);
Ptr<Ipv4StaticRouting> staticRoutingRouterAp = ipv4RoutingHelper.GetStaticRouting(ipv4RouterAp);
Ptr<Ipv4StaticRouting> staticRoutingRelayMt1 = ipv4RoutingHelper.GetStaticRouting(ipv4RelayMt1);
Ptr<Ipv4StaticRouting> staticRoutingRelayMt2 = ipv4RoutingHelper.GetStaticRouting(ipv4RelayMt2);

Ptr<Ipv4StaticRouting> staticRoutingMt1 = ipv4RoutingHelper.GetStaticRouting(ipv4Mt1);
Ptr<Ipv4StaticRouting> staticRoutingMt2 = ipv4RoutingHelper.GetStaticRouting(ipv4Mt2);




staticRoutingAp->AddHostRouteTo(Ipv4Address("10.1.6.2"), Ipv4Address("10.1.1.2"), 1,3);
staticRoutingRouterAp->AddHostRouteTo(Ipv4Address("10.1.6.2"), Ipv4Address("10.1.2.2"), 2,2);
staticRoutingRelayMt1->AddHostRouteTo(Ipv4Address("10.1.6.2"), Ipv4Address("10.1.6.2"), 3,1);

staticRoutingAp->AddHostRouteTo(Ipv4Address("10.1.7.2"), Ipv4Address("10.1.1.2"), 1,3);
staticRoutingRouterAp->AddHostRouteTo(Ipv4Address("10.1.7.2"), Ipv4Address("10.1.3.2"), 3,2);
staticRoutingRelayMt2->AddHostRouteTo(Ipv4Address("10.1.7.2"), Ipv4Address("10.1.7.2"), 3,1);



staticRoutingMt1->AddHostRouteTo(Ipv4Address("10.1.1.1"), Ipv4Address("10.1.6.1"), 1,3);
staticRoutingRelayMt1->AddHostRouteTo(Ipv4Address("10.1.1.1"), Ipv4Address("10.1.5.1"), 2,2);
staticRoutingRouterAp->AddHostRouteTo(Ipv4Address("10.1.1.1"), Ipv4Address("10.1.1.1"), 1,1);

staticRoutingMt2->AddHostRouteTo(Ipv4Address("10.1.1.1"), Ipv4Address("10.1.7.1"), 1,3);
staticRoutingRelayMt2->AddHostRouteTo(Ipv4Address("10.1.1.1"), Ipv4Address("10.1.5.1"), 2,2);
staticRoutingRouterAp->AddHostRouteTo(Ipv4Address("10.1.1.1"), Ipv4Address("10.1.1.1"), 1,1);

;

 Ptr<Socket> srcSocket1 = Socket::CreateSocket (Ap, TypeId::LookupByName ("ns3::TcpSocketFactory"));
  Ptr<Socket> srcSocket2 = Socket::CreateSocket (Ap, TypeId::LookupByName ("ns3::TcpSocketFactory"));
  Ptr<Socket> srcSocket3 = Socket::CreateSocket (Ap, TypeId::LookupByName ("ns3::TcpSocketFactory"));
  Ptr<Socket> srcSocket4 = Socket::CreateSocket (Ap, TypeId::LookupByName ("ns3::TcpSocketFactory"));
 Ptr<Socket> srcSocket5 = Socket::CreateSocket (Ap, TypeId::LookupByName ("ns3::TcpSocketFactory"));
  Ptr<Socket> srcSocket6 = Socket::CreateSocket (Ap, TypeId::LookupByName ("ns3::TcpSocketFactory"));


  uint16_t dstport1 = 12345;
  Ipv4Address dstaddr1 ("10.1.6.2");

  PacketSinkHelper sink ("ns3::TcpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), dstport1));
  ApplicationContainer apps = sink.Install (Mt1);
  sink1 = DynamicCast<PacketSink>(apps.Get(0));
  apps.Start (Seconds (0.0));
  apps.Stop (Seconds (10.0));
 
 uint16_t dstport2 = 12346;
  Ipv4Address dstaddr2 ("10.1.7.2");

  PacketSinkHelper sinka ("ns3::TcpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), dstport2));
  ApplicationContainer apps2 = sinka.Install (Mt2);
  sink2 = DynamicCast<PacketSink>(apps2.Get(0));
  apps.Start (Seconds (0.0));
  apps.Stop (Seconds (10.0));


AsciiTraceHelper ascii;
p2p.EnableAsciiAll(ascii.CreateFileStream ("RoutingTestCase.tr"));
p2p.EnablePcapAll("RoutingTestCase");

LogComponentEnableAll(LOG_PREFIX_TIME);
LogComponentEnable("RoutingTestCase", LOG_LEVEL_INFO);

Ptr<OutputStreamWrapper> stream1 = Create<OutputStreamWrapper> ("Table3", std::ios::out);
ipv4RoutingHelper.PrintRoutingTableAllAt(Seconds(2.0), stream1);





ndRelay_Mt1.Get (1)->TraceConnectWithoutContext ("PhyRxEnd", MakeCallback (&RxEnd));

ndRelay_Mt1.Get (1)->TraceConnectWithoutContext ("PhyTxEnd", MakeCallback (&TxEnd));

ndRelay_Mt2.Get (1)->TraceConnectWithoutContext ("PhyRxEnd", MakeCallback (&RxEnd2));

ndRelay_Mt2.Get (1)->TraceConnectWithoutContext ("PhyTxEnd", MakeCallback (&TxEnd2));


Simulator::Schedule(Seconds(0.1), &StartFlow,srcSocket2, dstaddr1, dstport1);

Simulator::Schedule(Seconds(0.1), &StartFlow,srcSocket3, dstaddr2, dstport2);

Simulator::Run();

double throughput1 = ((Received.back()*8))/ theTime.back();
double throughput2 = ((Received2.back()*8))/ theTime2.back();
double totalThroughput = ((ReceivedT.back()*8))/ theTimeT.back();

std::cout<<"-------------------------"<< std::endl;
//std::cout<<"Received : " << ReceivedT.back() << std::endl;
std::cout<<"Pow : " << pow << std::endl;
//std::cout<<"Time : " << theTimeT.back() << std::endl;
std::cout<<"THROUGHPUT : " << totalThroughput << std::endl;
std::cout<<"BER : " << em2->getBER() << std::endl;
  std::cout << "BER 2 : " << em3->getBER() <<std::endl;
std::cout<<"INR : " << em2->getINR() << std::endl;
std::cout<<"SNR : " << em2->getSNR() << std::endl;

myfile3 <<pow << " " << throughput2 <<std::endl;
myfile2 <<pow << " " << throughput1 <<std::endl;
myfile1 <<pow << " " << totalThroughput <<std::endl;



Received.clear();
Received2.clear();
ReceivedT.clear();



Simulator::Destroy();

}
 

myfile3.close();
myfile1.close();
myfile2.close();


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
