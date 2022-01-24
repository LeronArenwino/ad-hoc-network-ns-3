#include "ns3/command-line.h"
#include "ns3/config.h"
#include "ns3/uinteger.h"
#include "ns3/double.h"
#include "ns3/string.h"
#include "ns3/log.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/mobility-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/yans-wifi-channel.h"
#include "ns3/mobility-model.h"
#include "ns3/olsr-helper.h"
#include "ns3/ipv4-static-routing-helper.h"
#include "ns3/ipv4-list-routing-helper.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/opengym-module.h"
#include "ns3/node-list.h"
using namespace ns3;

NS_LOG_COMPONENT_DEFINE("AdhocLab");

// Counters of number of packages received and number of packages sent
int received = 0;
int sent = 0;

// Observation Space
Ptr<OpenGymSpace> MyGetObservationSpace(void)
{
  // X position of the 2-hierarchy nodes
  uint32_t nodeNum = 6;
  float low = 400.0;
  float high = 800.0;
  std::vector<uint32_t> shape = {
      nodeNum,
  };
  std::string dtype = TypeNameGet<uint32_t>();
  Ptr<OpenGymBoxSpace> space = CreateObject<OpenGymBoxSpace>(low, high, shape, dtype);
  NS_LOG_UNCOND("MyGetObservationSpace: " << space);
  return space;
}

// Action Space
Ptr<OpenGymSpace> MyGetActionSpace(void)
{
  // X position of the 2-hierarchy nodes
  uint32_t nodeNum = 6;
  float low = 400.0;
  float high = 800.0;
  std::vector<uint32_t> shape = {
      nodeNum,
  };
  std::string dtype = TypeNameGet<uint32_t>();
  Ptr<OpenGymBoxSpace> space = CreateObject<OpenGymBoxSpace>(low, high, shape, dtype);
  NS_LOG_UNCOND("MyGetActionSpace: " << space);
  return space;
}

// Game over condition
bool MyGetGameOver(void)
{
  // Activate game over condition when the stepcounter reach 20
  bool isGameOver = false;
  static float stepCounter = 0.0;
  stepCounter += 1;
  if (stepCounter == 20)
  {
    isGameOver = true;
  }
  NS_LOG_UNCOND("MyGetGameOver: " << isGameOver);
  return isGameOver;
}

// Get observation of current state from the environment
Ptr<OpenGymDataContainer> MyGetObservation(void)
{
  //Define the base observation space
  uint32_t nodeNum = 6;

  std::vector<uint32_t> shape = {
      nodeNum,
  };
  Ptr<OpenGymBoxContainer<uint32_t>> box = CreateObject<OpenGymBoxContainer<uint32_t>>(shape);
  uint32_t nodeNum2 = NodeList::GetNNodes();
  for (uint32_t i = 0; i < nodeNum2; i++)
  {
    Ptr<Node> node = NodeList::GetNode(i);
    if (node->GetSystemId() == 0)
    {

      //Extract the position from the hierarchy 2 nodes
      Ptr<MobilityModel> cpMob = node->GetObject<MobilityModel>();
      Vector m_position = cpMob->GetPosition();
      box->AddValue(m_position.x);
    }
  }
  NS_LOG_UNCOND("MyGetObservation: " << box);
  return box;
}

// Get reward from the environment
float MyGetReward(void)
{
  static float reward = 0;
  if (sent > 0)
  {
    // Quotient between the number of packages received and the number of packages sent
    reward = ((received * 1.0) / (sent * 1.0));
  }
  NS_LOG_UNCOND(reward);
  NS_LOG_UNCOND(received);
  return reward;
}

// Main function for agent actions execution
bool MyExecuteActions(Ptr<OpenGymDataContainer> action)
{
  NS_LOG_UNCOND("MyExecuteActions: " << action);

  // Get and format actions
  Ptr<OpenGymBoxContainer<uint32_t>> box = DynamicCast<OpenGymBoxContainer<uint32_t>>(action);
  std::vector<uint32_t> actionVector = box->GetData();

  uint32_t nodeNum = NodeList::GetNNodes();
  //Iterate over nodes and check if the nodes are the ones of the second hierarchy
  for (uint32_t i = 0; i < nodeNum; i++)
  {
    Ptr<Node> node = NodeList::GetNode(i);
    if (node->GetSystemId() == 0)
    {

      //Set location of the nodes of the second hierarchy
      Ptr<MobilityModel> cpMob = node->GetObject<MobilityModel>();
      Vector m_position = cpMob->GetPosition();
      m_position.x = actionVector.at(i);
      cpMob->SetPosition(m_position);
    }
  }

  return true;
}

//Traffic generator
static void GenerateTraffic(Ptr<Socket> socket, uint32_t pktSize,
                            uint32_t pktCount, Time pktInterval)
{
  //Increment sent counter
  sent += 1;

  //Send package
  socket->Send(Create<Packet>(pktSize));
}

// Main controller of the simulation
void ScheduleNextStateRead(double envStepTime, Ptr<OpenGymInterface> openGym, Ptr<Socket> source, uint32_t packetSize,
                           uint32_t numPackets, Time interPacketInterval)
{

  //Generate traffic from source node to objective node
  GenerateTraffic(source, packetSize, numPackets, interPacketInterval);
  GenerateTraffic(source, packetSize, numPackets, interPacketInterval);
  GenerateTraffic(source, packetSize, numPackets, interPacketInterval);
  GenerateTraffic(source, packetSize, numPackets, interPacketInterval);
  GenerateTraffic(source, packetSize, numPackets, interPacketInterval);
  GenerateTraffic(source, packetSize, numPackets, interPacketInterval);

  //Schedule next simulation time
  Simulator::Schedule(Seconds(envStepTime), &ScheduleNextStateRead, envStepTime, openGym,
                      source, packetSize, numPackets, interPacketInterval);
  openGym->NotifyCurrentState();
}

// Funcion that is triggered when the objective node receives a package
void ReceivePacket(Ptr<Socket> socket)
{
  while (socket->Recv())
  {
    NS_LOG_UNCOND("Received one packet!");
    received += 1;
  }
}

int main(int argc, char *argv[])
{
  std::string phyMode("DsssRate1Mbps");
  double distance = 5;        // m
  uint32_t packetSize = 1000; // bytes
  uint32_t numPackets = 10;
  uint32_t numNodes = 4;    // by default, 5x2
  uint32_t sinkNode = 0;    // objective node of cluster 1
  uint32_t sourceNode = 3;  //source node  of cluster 3
  double envStepTime = 0.6; //seconds, ns3gym env step time interval
  double interval = 0.1;    // seconds

  CommandLine cmd(__FILE__);
  cmd.AddValue("distance", "distance (m)", distance);

  cmd.Parse(argc, argv);
  // Convert to time object
  Time interPacketInterval = Seconds(interval);

  // Fix non-unicast data rate to be the same as that of unicast
  Config::SetDefault("ns3::WifiRemoteStationManager::NonUnicastMode",
                     StringValue(phyMode));

  //Cluster and node creation
  NodeContainer hierar2Nodes;
  hierar2Nodes.Create(6, 0);

  NodeContainer c1;
  c1.Add(hierar2Nodes.Get(0));
  c1.Add(hierar2Nodes.Get(1));
  c1.Create(numNodes, 1);

  NodeContainer c2;
  c2.Add(hierar2Nodes.Get(2));
  c2.Add(hierar2Nodes.Get(3));
  c2.Create(numNodes, 2);

  NodeContainer c3;
  c3.Add(hierar2Nodes.Get(4));
  c3.Add(hierar2Nodes.Get(5));
  c3.Create(numNodes, 3);

  // The below set of helpers will help us to put together the wifi NICs we want
  WifiHelper wifi;

  YansWifiPhyHelper wifiPhy = YansWifiPhyHelper();
  wifiPhy.SetPcapDataLinkType(YansWifiPhyHelper::DLT_IEEE802_11);

  // Wifi power setted for the simulation
  wifiPhy.Set("RxGain", DoubleValue(0));
  wifiPhy.Set("TxPowerStart", DoubleValue(33));
  wifiPhy.Set("TxPowerEnd", DoubleValue(33));
  wifiPhy.Set("TxGain", DoubleValue(0));
  wifiPhy.Set("RxSensitivity", DoubleValue(-64));

  YansWifiChannelHelper wifiChannel;
  wifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
  wifiChannel.AddPropagationLoss("ns3::TwoRayGroundPropagationLossModel",
                                 "SystemLoss", DoubleValue(1),
                                 "HeightAboveZ", DoubleValue(1.5));

  //Wifi channel creation
  wifiPhy.SetChannel(wifiChannel.Create());
  WifiMacHelper wifiMac;
  wifi.SetStandard(WIFI_PHY_STANDARD_80211b);
  wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                               "DataMode", StringValue(phyMode),
                               "ControlMode", StringValue(phyMode));
  // Set it to adhoc mode
  wifiMac.SetType("ns3::AdhocWifiMac");
  NetDeviceContainer devices1 = wifi.Install(wifiPhy, wifiMac, c1);
  NetDeviceContainer devices2 = wifi.Install(wifiPhy, wifiMac, c2);
  NetDeviceContainer devices3 = wifi.Install(wifiPhy, wifiMac, c3);
  NetDeviceContainer devicesHierar2 = wifi.Install(wifiPhy, wifiMac, hierar2Nodes);

  // Mobility models creation
  MobilityHelper mobility;
  mobility.SetPositionAllocator("ns3::GridPositionAllocator",
                                "MinX", DoubleValue(500.0),
                                "MinY", DoubleValue(205.0),
                                "DeltaX", DoubleValue(distance),
                                "DeltaY", DoubleValue(distance),
                                "GridWidth", UintegerValue(5),
                                "LayoutType", StringValue("RowFirst"));
  mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  MobilityHelper mobility2;
  mobility2.SetPositionAllocator("ns3::GridPositionAllocator",
                                 "MinX", DoubleValue(650.0),
                                 "MinY", DoubleValue(-205.0),
                                 "DeltaX", DoubleValue(distance),
                                 "DeltaY", DoubleValue(distance),
                                 "GridWidth", UintegerValue(5),
                                 "LayoutType", StringValue("RowFirst"));
  mobility2.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  MobilityHelper mobility3;
  mobility3.SetPositionAllocator("ns3::GridPositionAllocator",
                                 "MinX", DoubleValue(750.0),
                                 "MinY", DoubleValue(205.0),
                                 "DeltaX", DoubleValue(distance),
                                 "DeltaY", DoubleValue(distance),
                                 "GridWidth", UintegerValue(5),
                                 "LayoutType", StringValue("RowFirst"));
  mobility3.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  MobilityHelper mobilityHierar2;
  mobilityHierar2.SetPositionAllocator("ns3::GridPositionAllocator",
                                       "MinX", DoubleValue(600.0),
                                       "MinY", DoubleValue(0),
                                       "DeltaX", DoubleValue((1) + 3 * distance),
                                       "DeltaY", DoubleValue(distance),
                                       "GridWidth", UintegerValue(6),
                                       "LayoutType", StringValue("RowFirst"));
  mobility3.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  // Mobility models installation
  mobility.Install(c1);
  mobility2.Install(c2);
  mobility3.Install(c3);
  mobilityHierar2.Install(hierar2Nodes);

  // Enable OLSR
  OlsrHelper olsr;
  OlsrHelper olsr2;
  Ipv4StaticRoutingHelper staticRouting;

  Ipv4ListRoutingHelper list;
  list.Add(staticRouting, 0);
  list.Add(olsr, 30);
  InternetStackHelper internet;

  // Internet installation
  internet.SetRoutingHelper(list);
  internet.Install(c1);
  internet.Install(c2);
  internet.Install(c3);

  // IP assignation
  Ipv4AddressHelper ipv4;
  NS_LOG_INFO("Assign IP Addresses.");
  ipv4.SetBase("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer i1 = ipv4.Assign(devices1);
  Ipv4InterfaceContainer i4 = ipv4.Assign(devicesHierar2);
  Ipv4InterfaceContainer i2 = ipv4.Assign(devices2);
  Ipv4InterfaceContainer i3 = ipv4.Assign(devices3);

  // Socket definition and package source and objective preparation
  TypeId tid = TypeId::LookupByName("ns3::UdpSocketFactory");
  Ptr<Socket> recvSink = Socket::CreateSocket(c1.Get(sinkNode), tid);
  InetSocketAddress local = InetSocketAddress(Ipv4Address::GetAny(), 80);
  recvSink->Bind(local);
  recvSink->SetRecvCallback(MakeCallback(&ReceivePacket));

  Ptr<Socket> source = Socket::CreateSocket(c3.Get(sourceNode), tid);
  InetSocketAddress remote = InetSocketAddress(i1.GetAddress(sinkNode, 0), 80);
  source->Connect(remote);

  //OPEN Gym connection
  uint32_t openGymPort = 5555;
  Ptr<OpenGymInterface> openGym = CreateObject<OpenGymInterface>(openGymPort);
  openGym->SetGetActionSpaceCb(MakeCallback(&MyGetActionSpace));
  openGym->SetGetObservationSpaceCb(MakeCallback(&MyGetObservationSpace));
  openGym->SetGetGameOverCb(MakeCallback(&MyGetGameOver));
  openGym->SetGetObservationCb(MakeCallback(&MyGetObservation));
  openGym->SetGetRewardCb(MakeCallback(&MyGetReward));
  openGym->SetExecuteActionsCb(MakeCallback(&MyExecuteActions));
  Simulator::Schedule(Seconds(0.0), &ScheduleNextStateRead, envStepTime, openGym, source, packetSize, numPackets, interPacketInterval);

  // Output what we are doing
  NS_LOG_UNCOND("Testing from node " << sourceNode << " to " << sinkNode << " with grid distance " << distance);

  //Simulation run
  Simulator::Run();
  Simulator::Stop(Seconds(100));
  Simulator::Destroy();

  return 0;
}
