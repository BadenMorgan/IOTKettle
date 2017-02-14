
#include <ESP8266WiFi.h>
#include <AsyncMqttClient.h>
#include <ESP8266HTTPClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include "Settings.h"

extern "C" {
#include "user_interface.h"
}

//timer objects and variables
os_timer_t myTimer;

bool tickOccured;


#ifdef _MQTT_
AsyncMqttClient mqttClient;
#endif
// Data wire is plugged into pin 2 on the Arduino
#define ONE_WIRE_BUS 14

// Setup a oneWire instance to communicate with any OneWire devices
// (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

#define TEMPERATURE_PRECISION 12

int numberOfDevices; // Number of temperature devices found

DeviceAddress tempDeviceAddress; // We'll use this variable to store a found device address

WiFiUDP ntpUDP;

uint32_t CustomMillis = 0;

// You can specify the time server pool and the offset (in seconds, can be
// changed later with setTimeOffset() ). Additionaly you can specify the
// update interval (in milliseconds, can be changed using setUpdateInterval() ).
NTPClient timeClient(ntpUDP, "ntp2.is.co.za");

//variables
uint32_t stamp = 0;
uint32_t stamp1 = 0;
uint32_t stamp2 = 0;
float temperature = 0;
byte hours = 0;
byte minutes = 0;
byte seconds = 0;

#ifdef _MQTT_
void onMqttConnect(bool sessionPresent) {
  #ifdef _DEBUG_
  Serial.println("** Connected to the broker **");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
  #endif
  uint16_t packetIdSub = mqttClient.subscribe("test/lol", 2);
  #ifdef _DEBUG_
  Serial.print("Subscribing at QoS 2, packetId: ");
  Serial.println(packetIdSub);
  #endif
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  #ifdef _DEBUG_
  Serial.println("** Disconnected from the broker **");
  Serial.println("Reconnecting to MQTT...");
  #endif
  mqttClient.connect();
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  #ifdef _DEBUG_
  Serial.println("** Subscribe acknowledged **");
  Serial.print("  packetId: ");
  Serial.println(packetId);
  Serial.print("  qos: ");
  Serial.println(qos);
  #endif
}

void onMqttUnsubscribe(uint16_t packetId) {
  #ifdef _DEBUG_
  Serial.println("** Unsubscribe acknowledged **");
  Serial.print("  packetId: ");
  Serial.println(packetId);
  #endif
}

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  #ifdef _DEBUG_
  Serial.println("** Publish received **");
  Serial.print("  topic: ");
  Serial.println(topic);
  Serial.print("  qos: ");
  Serial.println(properties.qos);
  Serial.print("  dup: ");
  Serial.println(properties.dup);
  Serial.print("  retain: ");
  Serial.println(properties.retain);
  Serial.print("  len: ");
  Serial.println(len);
  Serial.print("  index: ");
  Serial.println(index);
  Serial.print("  total: ");
  Serial.println(total);
  Serial.print("  payload[0]: ");
  Serial.println(payload[0]);
  #endif
  if(topic == '' && payload[0] == '1'){
    Serial.print("A");
    digitalWrite(12,HIGH);
  }else {
    Serial.print("B");
    digitalWrite(12,LOW);
  }
}

void onMqttPublish(uint16_t packetId) {
  #ifdef _DEBUG_
  Serial.println("** Publish acknowledged **");
  Serial.print("  packetId: ");
  Serial.println(packetId);
  #endif
}
#endif

#ifdef _DEBUG_
void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    if (deviceAddress[i] < 16) Serial.print("0");
  #ifdef _DEBUG_
    Serial.print(deviceAddress[i], HEX);
  #endif
  }
}
#endif

// start of timerCallback
void timerCallback(void *pArg) {
  CustomMillis++;
} // End of timerCallback

void user_init(void) {
  os_timer_setfn(&myTimer, timerCallback, NULL);
  os_timer_arm(&myTimer, 1, true);
} // End of user_init

//request the temperature every 1 second
void TempRequest() {
  if (CustomMillis - stamp1 >= 1000) {
    temperature = sensors.getTempCByIndex(0);
    sensors.requestTemperatures();
    stamp1 = CustomMillis;
  }
}

//update the time every 10 seconds
void UpdateTime(){
  if (CustomMillis - stamp2 >= 1000) {
  #ifdef _DEBUG_
    Serial.println("updating time");
  #endif
    uint32_t tempstamp = CustomMillis;
    timeClient.forceUpdate();
  #ifdef _DEBUG_
    Serial.print("time to update: ");
    Serial.println(CustomMillis - tempstamp);
  #endif
    hours = timeClient.getHours();
    minutes = timeClient.getMinutes();
    seconds = timeClient.getSeconds();
    stamp2 = CustomMillis;
  }
}

#ifdef _DEBUG_
void outputVal(){
  if (CustomMillis - stamp >= 1000) {
    Serial.print("Temperature: ");
    Serial.println(temperature);
    Serial.print("Time: ");
    Serial.print(hours);
    Serial.print(":");
    Serial.print(minutes);
    Serial.print(":");
    Serial.println(seconds);
    stamp = CustomMillis;
  }
}
#endif

void setup() {
 #ifdef _DEBUG_
  Serial.begin(115200);
  Serial.println("hello world");
  Serial.println();
  Serial.println();
#endif
  WifiSetup();
  MQTTPreSetup();
  MQTTsetup();

  pinMode(12, OUTPUT);
  pinMode(13, OUTPUT);

  sensors.begin();
  sensors.setWaitForConversion(FALSE);

#ifdef _DEBUG_
  Serial.print("number of devices: ");
#endif

  numberOfDevices = sensors.getDeviceCount();
#ifdef _DEBUG_
  Serial.println(numberOfDevices);
#endif

  for(int i=0;i<numberOfDevices; i++)
  {
    // Search the wire for address
    if(sensors.getAddress(tempDeviceAddress, i))
  	{
#ifdef _DEBUG_
  		Serial.print("Found device ");
  		Serial.print(i, DEC);
  		Serial.print(" with address: ");
  		printAddress(tempDeviceAddress);
  		Serial.println();

  		Serial.print("Setting resolution to ");
  		Serial.println(TEMPERATURE_PRECISION, DEC);
#endif
  		// set the resolution to TEMPERATURE_PRECISION bit (Each Dallas/Maxim device is capable of several different resolutions)
  		sensors.setResolution(tempDeviceAddress, TEMPERATURE_PRECISION);
#ifdef _DEBUG_
  		Serial.print("Resolution actually set to: ");
  		Serial.print(sensors.getResolution(tempDeviceAddress), DEC);
  		Serial.println();
#endif
  	}
#ifdef _DEBUG_
    else{
  		Serial.print("Found ghost device at ");
  		Serial.print(i, DEC);
  		Serial.print(" but could not detect address. Check power and cabling");
  	}
    #endif
  }

  timeClient.begin();

  user_init();
}

void loop() {
  TempRequest();
  UpdateTime();
#ifdef _DEBUG_
  outputVal();
#endif
  delay(0);
}

//wifi setup
void WifiSetup(){
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
#ifdef _DEBUG_
  Serial.print("Connecting to Wi-Fi");
#endif
  WiFi.begin("MorganOnly", "1234m0rg@n2015");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
#ifdef _DEBUG_
    Serial.print(".");
#endif
  }
#ifdef _DEBUG_
  Serial.println(" OK");
#endif
}

//mqtt setup
void  MQTTPreSetup(){
#ifdef _MQTT_
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.onPublish(onMqttPublish);
#endif
}

void MQTTsetup(){
#ifdef _MQTT_
  mqttClient.setServer("m21.cloudmqtt.com", 11502);
  mqttClient.setKeepAlive(5).setCleanSession(false).setWill("topic/online", 2, true, "no").setCredentials("zhodqirz", "eeSp74dXYRpS").setClientId("myDevice");
  #ifdef _DEBUG_
  Serial.println("Connecting to MQTT...");
  #endif
  mqttClient.connect();
#endif
}
