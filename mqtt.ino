
//#include <Arduino.h>
#include "painlessMesh.h"
#include "PubSubClient.h"


// PainlessMesh credentials ( name, password and port ): You should change these
#define   MESH_PREFIX     "123"
#define   MESH_PASSWORD   "123"
#define   MESH_PORT       5555

// WiFi credentials: should match your access point!

#define   STATION_SSID     "123"
#define   STATION_PASSWORD "123"

#define   HOSTNAME         "123"

Scheduler userScheduler;   // to control your personal task

painlessMesh  mesh;
WiFiClient wifiClient;

// Prototypes
void receivedCallback( const uint32_t &from, const String &msg );
void mqttCallback(char* topic, byte* payload, unsigned int length);

IPAddress getlocalIP();
IPAddress myIP(0,0,0,0);

// hivemq pubblic broker address and port
char mqttBroker[]  = "www.coolerd.net";
#define MQTTPORT 1883

// topic's suffix: everyone can publish/subscribe to this public broker,
// you have to change the following 2 defines
#define PUBPLISHSUFFIX             "gym/data"
#define SUBSCRIBESUFFIX            "gym/data"

#define PUBPLISHFROMGATEWAYSUFFIX  PUBPLISHSUFFIX "gateway"

#define CHECKCONNDELTA 60     // check interval ( seconds ) for mqtt connection

PubSubClient mqttClient;



bool calc_delay = false;
SimpleList<uint32_t> nodes;
uint32_t nsent=0;
char buff[512];
uint32_t nexttime=0;
uint8_t  initialized=0;


//Task taskSendMessage( TASK_SECOND * 1 , TASK_FOREVER, &sendMessage );



// messages received from the mqtt broker
  void mqttCallback(char* topic, uint8_t* payload, unsigned int length) 
  {
  char* cleanPayload = (char*)malloc(length+1);
  payload[length] = '\0';
  memcpy(cleanPayload, payload, length+1);
  String msg = String(cleanPayload);
  free(cleanPayload);

  Serial.printf("mc t:%s  p:%s\n", topic, payload);
  
  String targetStr = String(topic).substring(strlen(SUBSCRIBESUFFIX));
  if(targetStr == "gateway")
    {
    if(msg == "getNodes")
      {
      auto nodes = mesh.getNodeList(true);
      String str;mqttClient
      for (auto &&id : nodes)
        str += String(id) + String(" ");
      mqttClient.publish(PUBPLISHFROMGATEWAYSUFFIX, str.c_str());
      }
    }
  else if(targetStr == "broadcast") 
    {
    mesh.sendBroadcast(msg);
    }
  else
    {
    uint32_t target = strtoul(targetStr.c_str(), NULL, 10);
    if(mesh.isConnected(target))
      {
      mesh.sendSingle(target, msg);
      }
    else
      {
      mqttClient.publish(PUBPLISHFROMGATEWAYSUFFIX, "Client not connected!");
      }
    }
  }





// Needed for painless library

// messages received from painless mesh network

  void receivedCallback( const uint32_t &from, const String &msg ) 
  {
  Serial.printf("bridge: Received from %u msg=%s\n", from, msg.c_str());
  String topic = PUBPLISHSUFFIX + String(from);
  mqttClient.publish(topic.c_str(), msg.c_str());
  }




  void newConnectionCallback(uint32_t nodeId) 
  {
  Serial.printf("--> Start: New Connection, nodeId = %u\n", nodeId);
  Serial.printf("--> Start: New Connection, %s\n", mesh.subConnectionJson(true).c_str());
  }




  void changedConnectionCallback() 
  {
  Serial.printf("Changed connections\n");

  nodes = mesh.getNodeList();
  Serial.printf("Num nodes: %d\n", nodes.size());
  Serial.printf("Connection list:");
  SimpleList<uint32_t>::iterator node = nodes.begin();
  while (node != nodes.end()) 
    {
    Serial.printf(" %u", *node);
    node++;
    }
  Serial.println();
  calc_delay = true;

  sprintf(buff,"Nodes:%d",nodes.size());
  tft.drawString(buff, 0, 32);
  }




void nodeTimeAdjustedCallback(int32_t offset) 
  {
  Serial.printf("Adjusted time %u Offset = %d\n", mesh.getNodeTime(),offset);
  }




void onNodeDelayReceived(uint32_t nodeId, int32_t delay)
  {
  Serial.printf("Delay from node:%u delay = %d\n", nodeId,delay);
  }




void reconnect()
{
  //byte mac[6];
  char MAC[9];
  int i;
  
  //WiFi.macAddress(mac);
  //sprintf(MAC,"%02X",mac[2],mac[3],mac[4],mac[5]);
  sprintf(MAC, "%08X",(uint32_t)ESP.getEfuseMac());  // generate unique addresss.

  serial.print("MAC:",MAC);
  // Loop until we're reconnected
    while (!mqttClient.connected()) 
    {
      Serial.println("Attempting MQTT connection...");    
      if(mqttClient.connect(/*MQTT_CLIENT_NAME*/MAC)) 
      {
      Serial.println("Connected");  
      mqttClient.publish(PUBPLISHFROMGATEWAYSUFFIX,"Ready!");
      mqttClient.subscribe(SUBSCRIBESUFFIX "#");
      } 
    else
      {
      Serial.print("Failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 2 seconds");
      // Wait 2 seconds before retrying
      delay(2000);
      mesh.update();
      mqttClient.loop();
      }
    }
}






  IPAddress getlocalIP() 
  {
  return IPAddress(mesh.getStationIP());
  }



  void setup() 
  {
  Serial.begin(115200);

  //mesh.setDebugMsgTypes( ERROR | MESH_STATUS | CONNECTION | SYNC | COMMUNICATION | MSG_TYPES | REMOTE ); // all types on except GENERAL
  //mesh.setDebugMsgTypes( ERROR | MESH_STATUS | CONNECTION | SYNC | COMMUNICATION | GENERAL | MSG_TYPES | REMOTE ); // all types on
  mesh.setDebugMsgTypes( ERROR | STARTUP | CONNECTION );  // set before init() so that you can see startup messages

  // Channel set to 1. Make sure to use the same channel for your mesh and for you other
  // network (STATION_SSID)
  mesh.init( MESH_PREFIX, MESH_PASSWORD, MESH_PORT, WIFI_AP_STA );
  mesh.onReceive(&receivedCallback);
  mesh.onNewConnection(&newConnectionCallback);
  mesh.onChangedConnections(&changedConnectionCallback);
  mesh.onNodeTimeAdjusted(&nodeTimeAdjustedCallback);

  mesh.stationManual(STATION_SSID, STATION_PASSWORD);
  mesh.setHostname(HOSTNAME);

  // Bridge node, should (in most cases) be a root node. See [the wiki](https://gitlab.com/painlessMesh/painlessMesh/wikis/Possible-challenges-in-mesh-formation) for some background
  mesh.setRoot(true);
  // This node and all other nodes should ideally know the mesh contains a root, so call this on all nodes
  mesh.setContainsRoot(true);

  
  mesh.initOTAReceive("bridge");

  sprintf(buff,"Id:%d",mesh.getNodeId());
  
  mqttClient.setServer(mqttBroker, MQTTPORT);
  mqttClient.setCallback(mqttCallback);  
  mqttClient.setClient(wifiClient);
  }






  void loop() 
  {
  mesh.update();
  mqttClient.loop();

    if(myIP != getlocalIP())
    {
    myIP = getlocalIP();
    Serial.println("My IP is " + myIP.toString());
    initialized = 1;
    }
  if ( ( millis() >= nexttime ) && ( initialized ) )
    {
    nexttime=millis()+CHECKCONNDELTA*1000;
    if (!mqttClient.connected()) 
      {reconnect();}
    }
  }
