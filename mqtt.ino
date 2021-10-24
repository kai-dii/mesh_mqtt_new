#include <Arduino.h>
#include <painlessMesh.h>
#include <PubSubClient.h>
#include <WiFiClient.h>

#define   MESH_PREFIX     "gym_nfu"
#define   MESH_PASSWORD   "gym_nfu123456"
#define   MESH_PORT       5555
#define SENDTOPIC "gym/data"


#define   STATION_SSID     "YOUR_WIFI_SSID"
#define   STATION_PASSWORD "YOUR_WIFI_PASSWORD"

#define CHECKCONNDELTA 60   // check interval ( seconds ) for mqtt connection

const char *mqttUser="MQTT_USER";
const char *mqttPassword="MQTT_PASSWORD";
const int mqttPort=1883;
const char *mqttServer="MQTT_HOST";


bool calc_delay = false;
SimpleList<uint32_t> nodes;
uint32_t nsent=0;
char buff[512];
uint32_t nexttime=0;
uint8_t  initialized=0;

void publshMQTT(String topic,String msg);
void reconnectMQTT();
#define HOSTNAME "MQTT_Bridge"

// Prototypes
void receivedCallback( const uint32_t &from, const String &msg );
void mqttCallback(char* topic, byte* payload, unsigned int length);
void nodeTimeAdjustedCallback(int32_t offset) ;
void onNodeDelayReceived(uint32_t nodeId, int32_t delay);
IPAddress getlocalIP();

IPAddress myIP(0,0,0,0);
IPAddress mqttBroker(61, 58, 248, 108);
Scheduler userScheduler;
painlessMesh  mesh;
WiFiClient wifiClient;
PubSubClient mqttClient;


void nodeTimeAdjustedCallback(int32_t offset) 
  {
  Serial.printf("Adjusted time %u Offset = %d\n", mesh.getNodeTime(),offset);
  }




void onNodeDelayReceived(uint32_t nodeId, int32_t delay)
  {
  Serial.printf("Delay from node:%u delay = %d\n", nodeId,delay);
  }
void receivedCallback( const uint32_t &from, const String &msg ) {
  Serial.printf("bridge: Received from %u msg=%s\n", from, msg.c_str());
  
   /*String payload = "{";
  payload += "\"MAC\"\"" + String(from);
  payload += "\",\"X\"\""+String(boardsStruct[myData.id-1].x);
  payload += "\",\"Y\"\""+String(boardsStruct[myData.id-1].y);
  payload += "\"}";*/
  String payload=msg.c_str();
  SentToBroker(payload);
}

void mqttCallback(char* topic, uint8_t* payload, unsigned int length) {
  char* cleanPayload = (char*)malloc(length+1);
  memcpy(cleanPayload, payload, length);
  cleanPayload[length] = '\0';
  String msg = String(cleanPayload);
  free(cleanPayload);

  String targetStr = String(topic).substring(16);

  if(targetStr == "gateway")
  {
    if(msg == "getNodes")
    {
      auto nodes = mesh.getNodeList(true);
      String str;
      for (auto &&id : nodes)
        str += String(id) + String(" ");
      mqttClient.publish("gym/data", str.c_str());
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
     mqttClient.publish("gym/data", "Client not connected!");
    }
  }
}

IPAddress getlocalIP() {
  return IPAddress(mesh.getStationIP());
}
void SentToBroker(const String &msg){
 
  Serial.println(msg);
  publshMQTT(SENDTOPIC,msg);
  digitalWrite(2,HIGH);
  delay(1000);
  digitalWrite(2,LOW);
  delay(1000);
    
  }


void publshMQTT(String topic,String msg)
{
  Serial.println("publish");
  if(!mqttClient.connected()){
    reconnectMQTT();
  }
  mqttClient.publish(topic.c_str(),msg.c_str());
}
void reconnectMQTT()
{
  char MAC[9];
  int i;

  //WiFi.macAddress(mac);
  //sprintf(MAC,"%02X",mac[2],mac[3],mac[4],mac[5]);
  //sprintf(MAC, "%08X",(uint32_t)ESP.getEfuseMac());  // generate unique addresss.
  // Loop until we're reconnected
  while (!mqttClient.connected()) 
  {
    Serial.println("Attempting MQTT connection...");    
    // Attemp to connect
    if (mqttClient.connect("MESH_MQTT",mqttUser,mqttPassword)) 
    {
      Serial.println("Connected");  
      mqttClient.publish("gym/data","Ready!");
      
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
 }
  void newConnectionCallback(uint32_t nodeId) 
  {
  Serial.printf("--> Start: New Connection, nodeId = %u\n", nodeId);
  Serial.printf("--> Start: New Connection, %s\n", mesh.subConnectionJson(true).c_str());
  }


void setup() {
  Serial.begin(115200);

  mesh.setDebugMsgTypes( ERROR | STARTUP | CONNECTION );  // set before init() so that you can see startup messages

  // Channel set to 6. Make sure to use the same channel for your mesh and for you other
  // network (STATION_SSID)
  mesh.init( MESH_PREFIX, MESH_PASSWORD, MESH_PORT, WIFI_AP_STA );
  mesh.onReceive(&receivedCallback);
  mesh.onNewConnection(&newConnectionCallback);
  mesh.onChangedConnections(&changedConnectionCallback);
  mesh.onNodeTimeAdjusted(&nodeTimeAdjustedCallback);

  mesh.stationManual(STATION_SSID, STATION_PASSWORD);
  mesh.setHostname(HOSTNAME);
  mesh.setRoot(true);
  mesh.setContainsRoot(true);
  mesh.initOTAReceive("bridge");

  sprintf(buff,"Id:%d",mesh.getNodeId());

  mqttClient.setServer(mqttBroker, mqttPort);
  mqttClient.setCallback(mqttCallback);  
  mqttClient.setClient(wifiClient);
}

void loop() {
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
      reconnectMQTT();
    }
}
