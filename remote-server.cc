/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*

Author:Reza Poorzare <reza.poorzare@upc.edu>

Supervisor: Anna Calveras<anna.calveras@upc.edu>

*/



#include "ns3/point-to-point-module.h"
#include "ns3/mmwave-helper.h"
#include "ns3/epc-helper.h"
#include "ns3/mmwave-point-to-point-epc-helper.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/applications-module.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/config-store.h"
#include <ns3/buildings-helper.h>
#include <ns3/buildings-module.h>
#include <ns3/packet.h>
#include <ns3/tag.h>
#include <ns3/queue-size.h>
#include "ns3/packet-sink.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/traffic-control-module.h"
/*#include <ns3/lte-helper.h>
#include <ns3/epc-helper.h>
#include <ns3/point-to-point-helper.h>
#include <ns3/lte-module.h>*/

//#include "ns3/gtk-config-store.h"


/**
 * A script to simulate the DOWNLINK TCP data over mmWave links
 * with the mmWave devices and the LTE EPC.
 */
NS_LOG_COMPONENT_DEFINE ("mmWaveTCPExample");


using namespace ns3;
using namespace mmwave;

Ptr<PacketSink> sink;                 
uint64_t lastTotalRx = 0;             


Ptr<RateErrorModel> em;

PointToPointHelper p2ph;


 AsciiTraceHelper asciiTraceHelper1;
Ptr<OutputStreamWrapper> stream5 = asciiTraceHelper1.CreateFileStream ("throughput.txt");



void
CalculateThroughput ()
{
  Time now = Simulator::Now ();                                         

  double cur = (sink->GetTotalRx () - lastTotalRx) * (double) 8 / 1e5;     

  std::cout << now.GetSeconds () << "s: \t" << cur << " Mbit/s" << std::endl;



std::ostream *stream = stream5->GetStream ();
*stream << now.GetSeconds () << "s: \t" << cur << " Mbit/s" << std::endl;





//stream->GetStream () << Simulator::Now ().GetSeconds () << "\t" << oldCwnd << "\t" << newCwnd << std::endl;


  lastTotalRx = sink->GetTotalRx ();
  Simulator::Schedule (MilliSeconds (100), &CalculateThroughput);
  
}



class MyAppTag : public Tag
{
public:
  MyAppTag ()
  {
  }

  MyAppTag (Time sendTs) : m_sendTs (sendTs)
  {
  }

  static TypeId GetTypeId (void)
  {
    static TypeId tid = TypeId ("ns3::MyAppTag")
      .SetParent<Tag> ()
      .AddConstructor<MyAppTag> ();
    return tid;
  }

  virtual TypeId  GetInstanceTypeId (void) const
  {
    return GetTypeId ();
  }

  virtual void  Serialize (TagBuffer i) const
  {
    i.WriteU64 (m_sendTs.GetNanoSeconds ());
  }

  virtual void  Deserialize (TagBuffer i)
  {
    m_sendTs = NanoSeconds (i.ReadU64 ());
  }

  virtual uint32_t  GetSerializedSize () const
  {
    return sizeof (m_sendTs);
  }

  virtual void Print (std::ostream &os) const
  {
    std::cout << m_sendTs;
  }

  Time m_sendTs;
};


class MyApp : public Application
{
public:
  MyApp ();
  virtual ~MyApp ();
  void ChangeDataRate (DataRate rate);
  void Setup (Ptr<Socket> socket, Address address, uint32_t packetSize, uint32_t nPackets, DataRate dataRate);



private:
  virtual void StartApplication (void);
  virtual void StopApplication (void);

  void ScheduleTx (void);
  void SendPacket (void);

  Ptr<Socket>     m_socket;
  Address         m_peer;
  uint32_t        m_packetSize;
  uint32_t        m_nPackets;
  DataRate        m_dataRate;
  EventId         m_sendEvent;
  bool            m_running;
  uint32_t        m_packetsSent;
};

MyApp::MyApp ()
  : m_socket (0),
    m_peer (),
    m_packetSize (0),
    m_nPackets (0),
    m_dataRate (0),
    m_sendEvent (),
    m_running (false),
    m_packetsSent (0)
{
}

MyApp::~MyApp ()
{
  m_socket = 0;
}

void
MyApp::Setup (Ptr<Socket> socket, Address address, uint32_t packetSize, uint32_t nPackets, DataRate dataRate)
{
  m_socket = socket;
  m_peer = address;
  m_packetSize = packetSize;
  m_nPackets = nPackets;
  m_dataRate = dataRate;
}

void
MyApp::ChangeDataRate (DataRate rate)
{
  m_dataRate = rate;
}

void
MyApp::StartApplication (void)
{
  m_running = true;
  m_packetsSent = 0;
  m_socket->Bind ();
  m_socket->Connect (m_peer);
  SendPacket ();
}

void
MyApp::StopApplication (void)
{
  m_running = false;

  if (m_sendEvent.IsRunning ())
    {
      Simulator::Cancel (m_sendEvent);
    }

  if (m_socket)
    {
      m_socket->Close ();
    }
}

void
MyApp::SendPacket (void)
{
  Ptr<Packet> packet = Create<Packet> (m_packetSize);
  MyAppTag tag (Simulator::Now ());

  m_socket->Send (packet);
  if (++m_packetsSent < m_nPackets)
    {
      ScheduleTx ();

    }



}



void
MyApp::ScheduleTx (void)
{
  if (m_running)
    {
      Time tNext (Seconds (m_packetSize * 8 / static_cast<double> (m_dataRate.GetBitRate ())));
      m_sendEvent = Simulator::Schedule (tNext, &MyApp::SendPacket, this);
    }
}





static void
CwndChange (Ptr<OutputStreamWrapper> stream, uint32_t oldCwnd, uint32_t newCwnd)
{
  *stream->GetStream () << Simulator::Now ().GetSeconds () << "\t" << (oldCwnd/1400) << "\t" << (newCwnd/1400) << std::endl;


}











static void
RxDrop (Ptr<PcapFileWrapper> file, Ptr<const Packet> p)
{
  NS_LOG_UNCOND ("RxDrop at " << Simulator::Now ().GetSeconds ());
  file->Write (Simulator::Now (), p);

}









static void
RxDrop1 (Ptr<PcapFileWrapper> file, Ptr<const Packet> p)
{
  NS_LOG_UNCOND ("UE0 RxDrop at " << Simulator::Now ().GetSeconds ());
  file->Write (Simulator::Now (), p);
//the contents of the packet beingdropped to the PCAP file
}




static void
RxDrop2 (Ptr<PcapFileWrapper> file, Ptr<const Packet> p)
{
  NS_LOG_UNCOND ("UE0 RxDrop at " << Simulator::Now ().GetSeconds ());
  file->Write (Simulator::Now (), p);

}






void
DevicePacketsInQueueTrace (uint32_t oldValue, uint32_t newValue)
{
  //std::cout << "DevicePacketsInQueue " <<Simulator::Now ().GetSeconds () << "\t" << oldValue << " to " << newValue << std::endl;
}



void
TcPacketsInQueueTrace (uint32_t oldValue, uint32_t newValue)
{
  //std::cout << "TcPacketsInQueue " << Simulator::Now ().GetSeconds () << "\t" << oldValue << " to " << newValue << std::endl;
}


void
SojournTimeTrace (Time sojournTime)
{
  std::cout << "Sojourn time " << Simulator::Now ().GetSeconds () << "\t" << sojournTime.ToDouble (Time::MS) << "ms" << std::endl;
}




static void
RttChange (Ptr<OutputStreamWrapper> stream, Time oldRtt, Time newRtt)
{
  *stream->GetStream () << Simulator::Now ().GetSeconds () << "\t" << oldRtt.GetSeconds () << "\t" << newRtt.GetSeconds () << std::endl;
}



static void Rx (Ptr<OutputStreamWrapper> stream, Ptr<const Packet> packet, const Address &from)
{
  *stream->GetStream () << Simulator::Now ().GetSeconds () << "\t" << packet->GetSize () << std::endl;
}





static void Sstresh (Ptr<OutputStreamWrapper> stream, uint32_t oldSstresh, uint32_t newSstresh)
{
        *stream->GetStream () << Simulator::Now ().GetSeconds () << "\t" << oldSstresh << "\t" << newSstresh << std::endl;

  NS_LOG_UNCOND ("Slow Start Threshold " << Simulator::Now ().GetSeconds () << "\t" << oldSstresh << "\t" << newSstresh << std::endl);
}





void
ChangeSpeed (Ptr<Node>  n, Vector speed)
{
  n->GetObject<ConstantVelocityMobilityModel> ()->SetVelocity (speed);
}



void
congestionstate (Ptr<OutputStreamWrapper> stream,  TcpSocketState::TcpCongState_t oldvalue, TcpSocketState::TcpCongState_t newvalue)
{
 
  NS_LOG_UNCOND ( Simulator::Now ().GetSeconds () << "\t"  << "The old state is:" <<oldvalue << "\n" << "The new state is: " <<  newvalue  );

//*stream->GetStream () << Simulator::Now ().GetSeconds () << "\t" << oldvalue << "\t" << newvalue << std::endl;;
}







int
main (int argc, char *argv[])
{
//	LogComponentEnable ("MmWaveUePhy", LOG_LEVEL_DEBUG);
//	LogComponentEnable ("MmWaveEnbPhy", LOG_LEVEL_DEBUG);
//	LogComponentEnable ("MmWaveFlexTtiMacScheduler", LOG_LEVEL_DEBUG);
//	LogComponentEnable ("MmWaveFlexTtiMaxWeightMacScheduler", LOG_LEVEL_DEBUG);

  /*
   * scenario 1: 1 building;
   * scenario 2: 3 building;
   * scenario 3: 6 random located small building, simulate tree and human blockage.
   * */
  int scenario = 3;
  double stopTime = 60;
  double simStopTime = 60;
  bool harqEnabled = true;
  bool rlcAmEnabled = true;
  bool tcp = true;

  CommandLine cmd;
//	cmd.AddValue("numEnb", "Number of eNBs", numEnb);
//	cmd.AddValue("numUe", "Number of UEs per eNB", numUe);
  cmd.AddValue ("simTime", "Total duration of the simulation [s])", simStopTime);
//	cmd.AddValue("interPacketInterval", "Inter-packet interval [us])", interPacketInterval);
  cmd.AddValue ("harq", "Enable Hybrid ARQ", harqEnabled);
  cmd.AddValue ("rlcAm", "Enable RLC-AM", rlcAmEnabled);
  cmd.Parse (argc, argv);






//RTO
  Config::SetDefault ("ns3::TcpSocketBase::MinRto", TimeValue (Seconds (1)));

//Slow Start Threshold
  //Config::SetDefault ("ns3::TcpSocket::InitialSlowStartThreshold",  UintegerValue (65500));


  Config::SetDefault ("ns3::TcpSocket::SegmentSize", UintegerValue (1400));

//	Config::SetDefault ("ns3::PointToPointNetDevice::Mtu", UintegerValue (3000));
//	Config::SetDefault ("ns3::VirtualNetDevice::Mtu", UintegerValue (3000));

  Config::SetDefault ("ns3::LteRlcUm::MaxTxBufferSize", UintegerValue (1024 * 1024));
  Config::SetDefault ("ns3::LteRlcUmLowLat::MaxTxBufferSize", UintegerValue (1024 * 1024));
  Config::SetDefault ("ns3::TcpSocket::SndBufSize", UintegerValue (131072 * 50));
  Config::SetDefault ("ns3::TcpSocket::RcvBufSize", UintegerValue (131072 * 50));
  Config::SetDefault ("ns3::MmWavePhyMacCommon::ResourceBlockNum", UintegerValue (1));
  Config::SetDefault ("ns3::MmWavePhyMacCommon::ChunkPerRB", UintegerValue (72));
  Config::SetDefault ("ns3::MmWaveHelper::RlcAmEnabled", BooleanValue (rlcAmEnabled));
  Config::SetDefault ("ns3::MmWaveHelper::HarqEnabled", BooleanValue (harqEnabled));
  Config::SetDefault ("ns3::MmWaveFlexTtiMacScheduler::HarqEnabled", BooleanValue (true));
  Config::SetDefault ("ns3::MmWaveFlexTtiMaxWeightMacScheduler::HarqEnabled", BooleanValue (true));
  Config::SetDefault ("ns3::MmWaveFlexTtiMacScheduler::HarqEnabled", BooleanValue (true));
  Config::SetDefault ("ns3::MmWaveBeamforming::LongTermUpdatePeriod", TimeValue (MilliSeconds (100.0)));
  Config::SetDefault ("ns3::LteRlcAm::PollRetransmitTimer", TimeValue (MilliSeconds (4.0)));
  //Config::SetDefault ("ns3::LteRlcAm::ReorderingTimer", TimeValue (MilliSeconds (2.0)));
  Config::SetDefault ("ns3::LteRlcAm::StatusProhibitTimer", TimeValue (MilliSeconds (1.0)));
  Config::SetDefault ("ns3::LteRlcAm::ReportBufferStatusTimer", TimeValue (MilliSeconds (4.0)));
  Config::SetDefault ("ns3::LteRlcAm::MaxTxBufferSize", UintegerValue (8 * 1024 * 1024));





Config::SetDefault ("ns3::TcpL4Protocol::SocketType", TypeIdValue (TcpNewReno::GetTypeId ()));






  Ptr<MmWaveHelper> mmwaveHelper = CreateObject<MmWaveHelper> ();

  mmwaveHelper->SetAttribute ("PathlossModel", StringValue ("ns3::BuildingsObstaclePropagationLossModel"));
  mmwaveHelper->Initialize ();
  mmwaveHelper->SetHarqEnabled (true);

  Ptr<MmWavePointToPointEpcHelper>  epcHelper = CreateObject<MmWavePointToPointEpcHelper> ();
  mmwaveHelper->SetEpcHelper (epcHelper);

  /*
  Ptr<LteHelper> mmwaveHelper = CreateObject<LteHelper> ();
  Ptr<PointToPointEpcHelper>  epcHelper = CreateObject<PointToPointEpcHelper> ();
  mmwaveHelper->SetEpcHelper (epcHelper);
*/


//The PGW acts as the interface between the LTE network and other packet data networks, such as the Internet or SIP-based IMS networks.

  Ptr<Node> pgw = epcHelper->GetPgwNode ();


   em = CreateObject<RateErrorModel> ();
   em->SetAttribute ("ErrorRate", DoubleValue (0));



  NodeContainer remoteHostContainer;
  remoteHostContainer.Create (1);
  Ptr<Node> remoteHost = remoteHostContainer.Get (0); 
  InternetStackHelper internet;
  internet.Install (remoteHostContainer);

                     

  TrafficControlHelper tchPfifo;
  tchPfifo.SetRootQueueDisc ("ns3::PfifoFastQueueDisc");


  TrafficControlHelper tchCoDel;
  tchCoDel.SetRootQueueDisc ("ns3::CoDelQueueDisc");



  // Create the Internet
 
  p2ph.SetDeviceAttribute ("DataRate", DataRateValue (DataRate ("100Gb/s")));
  p2ph.SetDeviceAttribute ("Mtu", UintegerValue (1500));
  p2ph.SetChannelAttribute ("Delay", TimeValue (Seconds (0.040)));
  NetDeviceContainer internetDevices = p2ph.Install (pgw, remoteHost);
  internetDevices.Get (0)->SetAttribute ("ReceiveErrorModel", PointerValue (em));

  QueueDiscContainer qdiscs=tchPfifo.Install(internetDevices);


  Ptr<QueueDisc> q = qdiscs.Get (1);
  q->TraceConnectWithoutContext ("BytesInQueue", MakeCallback (&TcPacketsInQueueTrace));



  Ipv4AddressHelper ipv4h;
  ipv4h.SetBase ("1.0.0.0", "255.0.0.0");
  Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign (internetDevices);

  Ipv4Address remoteHostAddr;
  remoteHostAddr = internetIpIfaces.GetAddress (1);
  Ipv4StaticRoutingHelper ipv4RoutingHelper;
  Ptr<Ipv4StaticRouting> remoteHostStaticRouting = ipv4RoutingHelper.GetStaticRouting (remoteHost->GetObject<Ipv4> ());
  remoteHostStaticRouting->AddNetworkRouteTo (Ipv4Address ("7.0.0.0"), Ipv4Mask ("255.0.0.0"), 1);



  switch (scenario)
    {
    case 1:
      {

        Ptr < Building > building;
        building = Create<Building> ();
        building->SetBoundaries (Box (10,12,
                                      2, 3,
                                      0.0, 1.5));



        Ptr < Building > building1;
        building1 = Create<Building> ();
        building1->SetBoundaries (Box (20.0,22.0,
                                      6.0, 7,
                                      0.0, 1.5));



        Ptr < Building > building2;
        building2 = Create<Building> ();
        building2->SetBoundaries (Box (15.0,25.0,
                                      12.0, 14,
                                      0.0, 15.0));



        break;
      }
    case 2:
      {



        Ptr < Building > building1;
        building1 = Create<Building> ();
        building1->SetBoundaries (Box (10,12,
                                      2, 3,
                                      0.0, 1.5));





        Ptr < Building > building2;
        building2 = Create<Building> ();
        building2->SetBoundaries (Box (20.0,22.0,
                                      6.0, 7,
                                      0.0, 1.5));





        Ptr < Building > building3;
        building3 = Create<Building> ();
        building3->SetBoundaries (Box (5.0,7.0,
                                      8.0, 9,
                                      0.0, 1.5));






        Ptr < Building > building4;
        building4 = Create<Building> ();
        building4->SetBoundaries (Box (15.0,17.0,
                                       9.0, 10.0,
                                       0.0, 1.5));






        Ptr < Building > building5;
        building5 = Create<Building> ();
        building5->SetBoundaries (Box (13.0,20.0,
                                       6.0, 8.0,
                                       0.0, 3.0));



        Ptr < Building > building6;
        building6 = Create<Building> ();
        building6->SetBoundaries (Box (18.0,25.0,
                                       14.0, 16.0,
                                       0.0, 3.0));


      }
    case 3:
      {
        Ptr < Building > building1;
        building1 = Create<Building> ();
        building1->SetBoundaries (Box (49.0,49.5,
                                       2.2, 2.7,
                                       0.0, 10));


        Ptr < Building > building2;
        building2 = Create<Building> ();
        building2->SetBoundaries (Box (49.0,49.5,
                                       5.6, 6.1,
                                       0.0, 10));



        Ptr < Building > building3;
        building3 = Create<Building> ();
        building3->SetBoundaries (Box (49.0,49.5,
                                       10.0, 10.5,
                                       0.0, 10));



        Ptr < Building > building4;
        building4 = Create<Building> ();
        building4->SetBoundaries (Box (49.0,49.5,
                                       13.8, 14.3,
                                       0.0, 10));


        Ptr < Building > building5;
        building5 = Create<Building> ();
        building5->SetBoundaries (Box (49.0,49.5,
                                       17.6, 18.1,
                                       0.0, 10));




//3.1 The first building
        Ptr < Building > building10;
        building10 = Create<Building> ();
        building10->SetBoundaries (Box (34.0,40.5,
                                       17.8, 23.8 ,
                                       0.0, 30));
       uint16_t number_rooms  = building10->GetNRoomsX();
       uint16_t number_floors = building10->GetNFloors();
       std::cout << "Number of room: " << number_rooms << std::endl;
       std::cout << "Number of floors: " << number_floors << std::endl;



        break;
        break;
      }
    default:
      {
        NS_FATAL_ERROR ("Invalid scenario");
      }
    }


  NodeContainer ueNodes;
  NodeContainer enbNodes;
  enbNodes.Create (1);
  ueNodes.Create (1);

  Ptr<ListPositionAllocator> enbPositionAlloc = CreateObject<ListPositionAllocator> ();
  enbPositionAlloc->Add (Vector (0.0, 0.0, 15.0));
  MobilityHelper enbmobility;
  enbmobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  enbmobility.SetPositionAllocator (enbPositionAlloc);
  enbmobility.Install (enbNodes);
  BuildingsHelper::Install (enbNodes);

  MobilityHelper uemobility;
  uemobility.SetMobilityModel ("ns3::ConstantVelocityMobilityModel");
  uemobility.Install (ueNodes);


  ueNodes.Get (0)->GetObject<MobilityModel> ()->SetPosition (Vector (52, -0.2, 1.5));

  ueNodes.Get (0)->GetObject<ConstantVelocityMobilityModel> ()->SetVelocity (Vector (0, 0, 0));





   Simulator::Schedule (Seconds (1), &ChangeSpeed, ueNodes.Get (0), Vector (0, 1.5, 0));


 Simulator::Schedule (Seconds (5.3), &ChangeSpeed, ueNodes.Get (0), Vector (0, 0, 0));


 Simulator::Schedule (Seconds (9), &ChangeSpeed, ueNodes.Get (0), Vector (0, 1.5, 0));


Simulator::Schedule (Seconds (19), &ChangeSpeed, ueNodes.Get (0), Vector (-1.5, 0, 0));


Simulator::Schedule (Seconds (26), &ChangeSpeed, ueNodes.Get (0), Vector (0, 0, 0));


Simulator::Schedule (Seconds (30), &ChangeSpeed, ueNodes.Get (0), Vector (-1.5, 0, 0));


Simulator::Schedule (Seconds (35), &ChangeSpeed, ueNodes.Get (0), Vector (-30, 0, 0));


Simulator::Schedule (Seconds (60), &ChangeSpeed, ueNodes.Get (0), Vector (0, 0, 0));






  BuildingsHelper::Install (ueNodes);

  // Install LTE Devices to the nodes
  NetDeviceContainer enbDevs = mmwaveHelper->InstallEnbDevice (enbNodes);
  NetDeviceContainer ueDevs = mmwaveHelper->InstallUeDevice (ueNodes);

  // Install the IP stack on the UEs
  // Assign IP address to UEs, and install applications
  internet.Install (ueNodes);
  Ipv4InterfaceContainer ueIpIface;
  ueIpIface = epcHelper->AssignUeIpv4Address (NetDeviceContainer (ueDevs));

  mmwaveHelper->AttachToClosestEnb (ueDevs, enbDevs);
  mmwaveHelper->EnableTraces ();

  // Set the default gateway for the UE
  Ptr<Node> ueNode = ueNodes.Get (0);
  Ptr<Ipv4StaticRouting> ueStaticRouting = ipv4RoutingHelper.GetStaticRouting (ueNode->GetObject<Ipv4> ());
  ueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);



ApplicationContainer sinkApps;

  if (tcp)
    {

      uint16_t sinkPort = 20000;

      Address sinkAddress (InetSocketAddress (ueIpIface.GetAddress (0), sinkPort));

      PacketSinkHelper packetSinkHelper ("ns3::TcpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), sinkPort));

 packetSinkHelper.SetAttribute ("Protocol", TypeIdValue (TcpSocketFactory::GetTypeId ()));
       sinkApps = packetSinkHelper.Install (ueNodes.Get (0));
sink = StaticCast<PacketSink> (sinkApps.Get (0));

      sinkApps.Start (Seconds (0.));


     Simulator::Schedule (Seconds (0), &CalculateThroughput);




      sinkApps.Stop (Seconds (simStopTime));

      Ptr<Socket> ns3TcpSocket = Socket::CreateSocket (remoteHostContainer.Get (0), TcpSocketFactory::GetTypeId ());
      Ptr<MyApp> app = CreateObject<MyApp> ();
      app->Setup (ns3TcpSocket, sinkAddress, 1400, 5000000, DataRate ("1000Mb/s"));

      remoteHostContainer.Get (0)->AddApplication (app);


      AsciiTraceHelper asciiTraceHelper;
      Ptr<OutputStreamWrapper> stream1 = asciiTraceHelper.CreateFileStream ("cwnd.txt");
      ns3TcpSocket->TraceConnectWithoutContext ("CongestionWindow", MakeBoundCallback (&CwndChange, stream1));

      Ptr<OutputStreamWrapper> stream4 = asciiTraceHelper.CreateFileStream ("rtt.txt");
      ns3TcpSocket->TraceConnectWithoutContext ("RTT", MakeBoundCallback (&RttChange, stream4));

      Ptr<OutputStreamWrapper> stream5 = asciiTraceHelper.CreateFileStream ("congestionstate.txt");
      ns3TcpSocket->TraceConnectWithoutContext ("CongState", MakeBoundCallback (&congestionstate, stream5));

      Ptr<OutputStreamWrapper> stream2 = asciiTraceHelper.CreateFileStream ("rx.txt");
      sinkApps.Get (0)->TraceConnectWithoutContext ("Rx",MakeBoundCallback (&Rx, stream2));


 
      Ptr<OutputStreamWrapper> stream35 = asciiTraceHelper.CreateFileStream ("sstresh.txt");
      ns3TcpSocket->TraceConnectWithoutContext("SlowStartThreshold",MakeBoundCallback (&Sstresh, stream35));



  PcapHelper pcapHelper;
  Ptr<PcapFileWrapper> file = pcapHelper.CreateFile ("drop.pcap", std::ios::out, PcapHelper::DLT_PPP);
  internetDevices.Get (0)->TraceConnectWithoutContext ("PhyRxDrop", MakeBoundCallback (&RxDrop, file));

  PcapHelper pcapHelper1;
  Ptr<PcapFileWrapper> file1 = pcapHelper1.CreateFile ("dropue0.pcap", std::ios::out, PcapHelper::DLT_PPP);
  ueNodes.Get (0)->TraceConnectWithoutContext ("PhyRxDrop", MakeBoundCallback (&RxDrop1, file));


  PcapHelper pcapHelper2;
  Ptr<PcapFileWrapper> file2 = pcapHelper1.CreateFile ("serverdrop.pcap", std::ios::out, PcapHelper::DLT_PPP);
  internetDevices.Get (1)->TraceConnectWithoutContext ("PhyRxDrop", MakeBoundCallback (&RxDrop2, file));






      app->SetStartTime (Seconds (0.1));
      app->SetStopTime (Seconds (stopTime));
    }


  else
    {
      // Install and start applications on UEs and remote host
      uint16_t sinkPort = 20000;

      Address sinkAddress (InetSocketAddress (ueIpIface.GetAddress (0), sinkPort));
      PacketSinkHelper packetSinkHelper ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), sinkPort));
      ApplicationContainer sinkApps = packetSinkHelper.Install (ueNodes.Get (0));




    sink = StaticCast<PacketSink> (sinkApps.Get (0));
     


     sinkApps.Start (Seconds (0.));



     Simulator::Schedule (Seconds (0), &CalculateThroughput);




      sinkApps.Stop (Seconds (simStopTime));

      Ptr<Socket> ns3UdpSocket = Socket::CreateSocket (remoteHostContainer.Get (0), UdpSocketFactory::GetTypeId ());
      Ptr<MyApp> app = CreateObject<MyApp> ();
      app->Setup (ns3UdpSocket, sinkAddress, 1400, 5000000, DataRate ("1000Mb/s"));

      remoteHostContainer.Get (0)->AddApplication (app);
      AsciiTraceHelper asciiTraceHelper;
      Ptr<OutputStreamWrapper> stream2 = asciiTraceHelper.CreateFileStream ("mmWave-udp-data-am.txt");
      sinkApps.Get (0)->TraceConnectWithoutContext ("Rx",MakeBoundCallback (&Rx, stream2));

      app->SetStartTime (Seconds (0.1));
      app->SetStopTime (Seconds (stopTime));

    }



  BuildingsHelper::MakeMobilityModelConsistent ();
  Config::Set ("/NodeList/*/DeviceList/*/TxQueue/MaxSize", QueueSizeValue (QueueSize ("1000000p")));




 AsciiTraceHelper ascii;
  MobilityHelper::EnableAsciiAll (ascii.CreateFileStream ("mobility-trace.mob"));

  FlowMonitorHelper flowmon;
  Ptr<FlowMonitor> monitor = flowmon.InstallAll();


  Simulator::Stop (Seconds (simStopTime));
  Simulator::Run ();


  Simulator::Destroy ();


Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier> (flowmon.GetClassifier ());
  std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats ();
  std::cout << std::endl << "*** *Flow monitor statistics ***" << std::endl;
  std::cout << "  *Tx Packets/Bytes:   " << stats[1].txPackets
            << " / " << stats[1].txBytes << std::endl;
  std::cout << "  *Offered Load: " << stats[1].txBytes * 8.0 / (stats[1].timeLastTxPacket.GetSeconds () - stats[1].timeFirstTxPacket.GetSeconds ()) / 1000000 << " *Mbps" << std::endl;
  std::cout << "  *Rx Packets/Bytes:   " << stats[1].rxPackets
            << " / " << stats[1].rxBytes << std::endl;
  uint32_t packetsDroppedByQueueDisc = 0;
  uint64_t bytesDroppedByQueueDisc = 0;
  if (stats[1].packetsDropped.size () > Ipv4FlowProbe::DROP_QUEUE_DISC)
    {
      packetsDroppedByQueueDisc = stats[1].packetsDropped[Ipv4FlowProbe::DROP_QUEUE_DISC];
      bytesDroppedByQueueDisc = stats[1].bytesDropped[Ipv4FlowProbe::DROP_QUEUE_DISC];
    }
  std::cout << "  *Packets/Bytes Dropped by Queue Disc:   " << packetsDroppedByQueueDisc
            << " / " << bytesDroppedByQueueDisc << std::endl;
  uint32_t packetsDroppedByNetDevice = 0;
  uint64_t bytesDroppedByNetDevice = 0;
  if (stats[1].packetsDropped.size () > Ipv4FlowProbe::DROP_QUEUE)
    {
      packetsDroppedByNetDevice = stats[1].packetsDropped[Ipv4FlowProbe::DROP_QUEUE];
      bytesDroppedByNetDevice = stats[1].bytesDropped[Ipv4FlowProbe::DROP_QUEUE];
    }
  std::cout << "  *Packets/Bytes Dropped by NetDevice:   " << packetsDroppedByNetDevice
            << " / " << bytesDroppedByNetDevice << std::endl;
  std::cout << "  *Throughput: " << stats[1].rxBytes * 8.0 / (stats[1].timeLastRxPacket.GetSeconds () - stats[1].timeFirstRxPacket.GetSeconds ()) / 1000000 << " *Mbps" << std::endl;
  std::cout << "  *Mean delay:   " << stats[1].delaySum.GetSeconds () / stats[1].rxPackets << std::endl;
  std::cout << " *Mean jitter:   " << stats[1].jitterSum.GetSeconds () / (stats[1].rxPackets - 1) << std::endl;
  auto dscpVec = classifier->GetDscpCounts (1);
  for (auto p : dscpVec)
    {
      std::cout << "  *DSCP value:   0x" << std::hex << static_cast<uint32_t> (p.first) << std::dec
                << "  *count:   "<< p.second << std::endl;
    }








  std::cout << std::endl << "*** Application statistics ***" << std::endl;
  double thr = 0;
  uint64_t totalPacketsThr = DynamicCast<PacketSink> (sinkApps.Get (0))->GetTotalRx ();
  thr = totalPacketsThr * 8 / (simStopTime * 1000000.0); //Mbit/s
  std::cout << "  Rx Bytes: " << totalPacketsThr << std::endl;
  std::cout << "  Average Goodput For NewReno: " << thr << " Mbit/s" << std::endl;
  std::cout << std::endl << "*** TC Layer statistics ***" << std::endl;
  std::cout << q->GetStats () << std::endl;



  return 0;

}
