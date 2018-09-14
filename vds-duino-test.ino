/*
 *  read a ds18b20 sensor and publish the value with MQTT
 *
 * references:
 *   https://techtutorialsx.com/2017/04/09/esp8266-connecting-to-mqtt-broker/
 *   https://create.arduino.cc/projecthub/TheGadgetBoy/ds18b20-digital-temperature-sensor-and-arduino-9cc806
 *   https://github.com/arduino-libraries/NTPClient/blob/master/examples/Basic/Basic.ino
 *
*/

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

#include <OneWire.h>
#include <DallasTemperature.h>

#include <NTPClient.h>
#include <WiFiUDP.h>

#include <Time.h>          // https://github.com/PaulStoffregen/Time
#include <TimeLib.h>       // https://github.com/PaulStoffregen/Time
//#include <Timezone.h>      // https://github.com/JChristensen/Timezone

#include "wifi.h"     // #define ESSID and PSK in this, see wifi.h.example for syntax

/*
 * Start editing here
 *
*/

const char* mqttServer = "192.168.1.24"; // hostname 'mqtt' locally
const int mqttPort = 1883;
//const char* mqttUser = "none";       // alter the setup_mqtt( ) routine below if
//const char* mqttPass = "none;        // you password-protect your MQTT broker

#define PROGRAM_NAME "vds-duino-test"  // so I can tell what is loaded on the nodeMCU long after the fact.
                                       // Based on a great idea from reddit user /u/blimpway in /r/esp8266

#define ONE_WIRE_BUS 4                 // nodemcu pin D2

#define LED D0                         // Led in NodeMCU at pin GPIO16 (D0).

// Define NTP properties
#define NTP_OFFSET   60 * 60            // In seconds
#define NTP_INTERVAL 60 * 1000          // In miliseconds
#define NTP_ADDRESS  "ca.pool.ntp.org"  // change this to whatever pool is closest (see ntp.org)

/*
 * Stop editing here
 *
*/

OneWire oneWire(ONE_WIRE_BUS); 
DallasTemperature sensors(&oneWire);

WiFiClient espClient;
PubSubClient client(espClient);

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, NTP_ADDRESS, NTP_OFFSET, NTP_INTERVAL);

// this will be ala "esp/obs/12345678"
String topicPrefix     = "esp/obs/";
String chipid          = String(ESP.getChipId()).c_str();
String together        = topicPrefix + chipid;
const char * mqttTopic = together.c_str();

/*
 * setup the wifi connection
 *
*/
  
void setup_wifi() {
  delay(1000);
  Serial.print("Connecting to WiFi..");
  WiFi.begin(ESSID,PSK);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
//    blinkPattern(5000,100);
//    blinkPattern(5000,100);
//    blinkPattern(5000,100);
  }
  Serial.println("Connected to the WiFi network");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

/*
 * set up the MQTT connection
 *
*/

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived in topic: ");
  Serial.println(topic);
  Serial.print("Message:");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
  Serial.println("-----------------------");
}

void setup_mqtt() { 
  client.setServer(mqttServer,mqttPort);         // add mqttUser,mqttPass if you password protect your broker
  client.setCallback(callback);
  while (!client.connected()) {
    Serial.println("Connecting to MQTT...");
    if (client.connect("ESP8266Client" )) {
      Serial.println("connected");
      client.subscribe(mqttTopic);
    } else {
      Serial.print("failed with state ");
      Serial.print(client.state());
      delay(2000);
    }
  } 
}

/*
 * read the 'first' ds18b20 and return its value
 *
*/

float read_ds18b20() {  
 sensors.requestTemperatures(); 
 float temp0 = sensors.getTempFByIndex(0);
 return temp0;
}

/*
 * the overall setup routine
 *
*/
 
void setup() {
  Serial.begin(115200);
  Serial.print("program name = ");
  Serial.println(PROGRAM_NAME);
  Serial.print("chip id = ");
  Serial.println(chipid);
  
  pinMode(LED, OUTPUT);    // LED pin as output.
  
  setup_wifi();
  setup_mqtt();
  sensors.begin();

//  timeClient.begin();
//  unsigned long epochTime =  timeClient.getEpochTime();
//
//  // convert received time stamp to time_t object
//  time_t local, utc;
//  utc = epochTime;
//  // Then convert the UTC UNIX timestamp to local time
//  TimeChangeRule usPDT = {"PDT", Second, Sun, Mar, 2, -480};  //UTC - 8 hours - change this as needed
//  TimeChangeRule usPST = {"PST", First, Sun, Nov, 2, -560};   //UTC - 9 hours - change this as needed
//  Timezone usPacific(usPDT, usPST);
//  local = usPacific.toLocal(utc);
//
//  Serial.print("utc="); Serial.println(utc);
//  Serial.print("local="); Serial.println(local);
//  Serial.println(timeClient.getFormattedTime());
//  Serial.println(timeClient.getEpochTime());

  Serial.println("done setup...");
}

/*
 * the overall loop
*/

void loop() {

  StaticJsonBuffer<200> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();

  root["espID"]     = chipid.toInt(); 
  root["timestamp"] = now();              // secs since bootup
  
  float degF = read_ds18b20();
  if ((degF > -20) && (degF < 140)) {
    root["degF"] = degF;
  } else {
    Serial.print("error detecting degF: ");
    Serial.println(degF);
    delay(5000);
    return;
  }

  char JSONmessageBuffer[200];
  root.printTo(JSONmessageBuffer, sizeof(JSONmessageBuffer));
  
  // MQTT will drop you occasionally due to session timeout
  // so be sure to re-establish as needed
  if (!client.connected()) {
    Serial.println("mqtt needs to reconnect...");
    Serial.println(JSONmessageBuffer);   // so we see the payload including timestamp
    blinkPattern(100,100);
    blinkPattern(100,100);
    blinkPattern(100,100);
    blinkPattern(500,100);
    blinkPattern(500,100);
    blinkPattern(500,100);
    blinkPattern(100,100);
    blinkPattern(100,100);
    blinkPattern(100,100);
    setup_mqtt();
    return;
  }

  // to do: the MQTT topic should be defined at the top of this program
  if (client.publish(mqttTopic, JSONmessageBuffer) == true) {
        //Serial.print("Success sending message to ");
        //Serial.println(topic);
        blinkPattern(100,100);
        blinkPattern(100,100);
        blinkPattern(100,100);
        blinkPattern(100,100);
        blinkPattern(100,100);
    } else {
        Serial.println("Error sending mqtt message");
        Serial.println(JSONmessageBuffer); // so we see the payload including timestamp
        blinkPattern(1000,1000);
        blinkPattern(1000,1000);
        blinkPattern(1000,1000);
        blinkPattern(1000,1000);
         // belt and suspenders just in case wifi ever drops
        if (WiFi.status() != WL_CONNECTED) {
           Serial.println("wifi needs to reconnect in loop()...");
           ESP.restart();
        }  
        setup_mqtt();
        return;
    }

  // to do: the delay should be defined at the top of this program
  delay(5000);

}

// LOW=on, HIGH=OFF
void blinkPattern(int onTime,int offTime) {
  digitalWrite(LED,LOW);  delay(onTime);
  digitalWrite(LED,HIGH); delay(offTime);
}


