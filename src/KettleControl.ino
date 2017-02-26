
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <AsyncMqttClient.h>
#include <ESP8266mDNS.h>
#include <ESP8266HTTPClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <ESP8266HTTPUpdateServer.h>
#include <ESP8266WebServer.h>
#include <ArduinoOTA.h>
#include <ArduinoJson.h> // required for settings file to make it readable
#include <ESPmanager.h>
#include <EEPROM.h>

#include "Settings.h"

extern "C" {
#include "user_interface.h"
}

//timer objects and variables
os_timer_t myTimer;

bool tickOccured;

ESP8266WebServer HTTP(80);

const char * defaultSSID = "MorganOnly";
const char * defaultPSK = "1234m0rg@n2015";
ESPmanager settings(HTTP, SPIFFS, "ESPManager", defaultSSID , defaultPSK);




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
uint32_t stamp3 = 0;
uint32_t stamp4 = 0;
uint32_t stamp5 = 0;
float temperature = 0;
byte hours = 0;
byte minutes = 0;
byte seconds = 0;
byte TempLimit = 100;
byte TempLimitMem = 100;
byte RelayState = 0;
byte Onhours = 6;
byte Onminutes = 1;
byte Onseconds = 0;
byte keep = 0;
byte CommandRec = 0;
byte mqttconnected = 0;

void setup() {
  pinMode(12, OUTPUT);
  digitalWrite(12,LOW);
  pinMode(13, INPUT);
  beginEEPROM();

 #ifdef _DEBUG_
  Serial.begin(115200);
  Serial.println("hello world");
  Serial.println();
  Serial.println();
#endif

  SPIFFS.begin();
  settings.begin();
  HTTP.begin();

  //WifiSetup();
  MQTTPreSetup();
  MQTTsetup();



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
  		//Serial.print(i, DEC);
  		Serial.print(" with address: ");
  		printAddress(tempDeviceAddress);
  		Serial.println(" ");

  		Serial.print("Setting resolution to ");
  		Serial.println(TEMPERATURE_PRECISION, DEC);
#endif
  		// set the resolution to TEMPERATURE_PRECISION bit (Each Dallas/Maxim device is capable of several different resolutions)
  		sensors.setResolution(tempDeviceAddress, TEMPERATURE_PRECISION);
#ifdef _DEBUG_
  		Serial.print("Resolution actually set to: ");
  		Serial.print(sensors.getResolution(tempDeviceAddress), DEC);
  		Serial.println(" ");
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
  timeClient.setTimeOffset(7200);

  user_init();

  stamp = millis();
  stamp1 = CustomMillis;
  stamp2 = stamp2 + 250;
  stamp3 = stamp2 + 500;

}

void loop() {
  uint32_t temp = RelayState;
  HTTP.handleClient();
	settings.handle();
  TempRequest();
  UpdateTime();
  if(mqttconnected) PublishMessage();
  TurnOffKettle();
  KeepTemp();
  TurnOnKettle();
#ifdef _DEBUG_
  //outputVal();
#endif
  delay(0);
  if(temp != RelayState){
    stamp4 = CustomMillis;
  }
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



#ifdef _MQTT_
void PublishMessage(){
  if(CustomMillis - stamp3 >= 1000){
    byte first2 = temperature;
    byte second2 = (byte) ((temperature - first2) * 100);
    char mess[13] = {3,CommandRec,hours,minutes,seconds,first2,second2,RelayState,TempLimit,keep,Onhours,Onminutes,Onseconds};
    String Stringmess = "";
    for(int i = 0; i < 13 ; i ++ ){
      Stringmess.concat(mess[i]);
    }
    CommandRec = 0;
    uint16_t packetIdPub2 = mqttClient.publish("d/kettle0", 2, true, &Stringmess[0],13);
    stamp3 = CustomMillis;
  #ifdef _DEBUG_
    Serial.print("Publishing at QoS 2, packetId: ");
    Serial.println(packetIdPub2);
  #endif
  }
}

void onMqttConnect(bool sessionPresent) {
  mqttconnected = 1;
  #ifdef _DEBUG_
  Serial.println("** Connected to the broker **");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
  #endif
  uint16_t packetIdSub = mqttClient.subscribe("s/kettle0", 2);
  #ifdef _DEBUG_
  Serial.print("Subscribing at QoS 2, packetId: ");
  Serial.println(packetIdSub);
  #endif
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  mqttconnected = 0;
  ESP.restart();
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
  byte CheckTopic = 1;
  String MatchTopic = "s/kettle0";
  for(int i = 0 ; i < strlen(topic) ; i++){
    if(topic[i] != MatchTopic[i]) CheckTopic = 0;
  }
  if(CheckTopic && payload[0] == 1){
    CommandRec = 1;
    RelaySwitch(12,keep);
  }else if(CheckTopic && payload[0] == 0) {
    CommandRec = 1;
    if(keep == 1) keep = 0;
    digitalWrite(12,LOW);
    RelayState |= 1;
    RelayState ^= 1;
  }else if(CheckTopic && payload[0] == 2){
    CommandRec = 1;
    RelaySwitch(12,1);
  }else if(CheckTopic && payload[0] == 3){
    CommandRec = 1;
    keep = payload[1];
    EEPROM.write(3,keep);
    EEPROM.commit();
  }else if(CheckTopic && payload[0] == 4){
    CommandRec = 1;
    Onhours = payload[1];
    Onminutes = payload[2];
    Onseconds = payload[3];
    EEPROM.write(0, Onhours);
    EEPROM.write(1, Onminutes);
    EEPROM.write(2, Onseconds);
    EEPROM.commit();
  }else if(CheckTopic && payload[0] == 5){
    CommandRec = 1;
    TempLimit = payload[1];
    TempLimitMem = TempLimit;
    EEPROM.write(4, TempLimit);
    EEPROM.commit();
  }else if(CheckTopic && payload[0] == 9){
    CommandRec = 1;
    ESP.restart();
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

void TurnOffKettle(){
  if(((CustomMillis - stamp4 >= 300000) && (RelayState & 1)) || (temperature >= TempLimit)){
    digitalWrite(12,LOW);
    TempLimit = TempLimitMem;
    RelayState |= 1;
    RelayState ^= 1;
  }
}

void TurnOnKettle(){
  if(hours == Onhours && minutes == Onminutes && seconds == Onseconds){
    TempLimitMem = TempLimit;
    TempLimit = 100;
    if(keep == 2) RelaySwitch(12,0);
    else RelaySwitch(12,1);
  }
}

void KeepTemp(){
  if(keep == 1 && temperature < TempLimit - 10 && (RelayState & 1) == 0 && (CustomMillis - stamp5 <= 1800000)){
    digitalWrite(12,HIGH);
    RelayState |= 1;
    stamp4 = CustomMillis;
  }else if(CustomMillis - stamp5 >= 1800000){
    if(keep != 0){
      digitalWrite(12,LOW);
      keep = 0;
      TempLimit = TempLimitMem;
      RelayState |= 1;
      RelayState ^= 1;
    }
  }
}

void RelaySwitch(byte pin, byte keeper){
  digitalWrite(pin,HIGH);
  RelayState |= 1;
  keep = keeper;
  stamp4 = CustomMillis;
  if(keeper == 1){
    stamp5 = CustomMillis;
  }
}

void beginEEPROM(){
  EEPROM.begin(10);
  Onhours = EEPROM.read(0);
  Onminutes = EEPROM.read(1);
  Onseconds = EEPROM.read(2);
  keep = EEPROM.read(3);
  TempLimit = EEPROM.read(4);
}
