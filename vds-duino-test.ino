/*
 *  read a ds18b20 sensor and publish the value with MQTT, with status to the onboard LED
 *
 * references:
 *   https://techtutorialsx.com/2017/04/09/esp8266-connecting-to-mqtt-broker/
 *   https://create.arduino.cc/projecthub/TheGadgetBoy/ds18b20-digital-temperature-sensor-and-arduino-9cc806
 *   https://github.com/arduino-libraries/NTPClient/blob/master/examples/Basic/Basic.ino
 *
 * this is tested on a nodeMCU esp8266
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

#include "wifi.h"          // #define ESSID and PSK in this, see wifi.h.example for syntax

//-----------------------------------------------------
//
// Start editing here
//
//

#define PROGRAM_NAME "vds-duino-test"    // so I can tell what is loaded on the nodeMCU long after the fact.
#define PROGRAM_VER  "2"                 // Based on a great idea from reddit user /u/blimpway in /r/esp8266

const int DELAY_MS = 15000;               // how often to publish in ms

const char* mqttServer = "192.168.1.24"; // FQDN works too, but this saves DNS lookups
const int mqttPort = 1883;
//const char* mqttUser = "none";         // also alter the setup_mqtt( ) routine below if
//const char* mqttPass = "none;          // you password-protect your MQTT broker

#define ONE_WIRE_BUS 4                   // nodemcu pin D2 the ds18b20 data pin is connect to

#define LED D0                           // status LED we blink - NodeMCU pin GPIO16 (D0).

//
// Stop editing here
//
//-----------------------------------------------------

OneWire oneWire(ONE_WIRE_BUS); 
DallasTemperature sensors(&oneWire);

WiFiClient espClient;
PubSubClient client(espClient);

WiFiUDP ntpUDP;

// ultimate MQTT topic will be ala "esp/obs/12345678"
String topicPrefix     = "esp/obs/";
String chipid          = String(ESP.getChipId()).c_str();
String together        = topicPrefix + chipid;
const char * mqttTopic = together.c_str();

/*
 * set up the wifi connection
 *
*/
  
void setup_wifi() {
  delay(1000);
  Serial.print("Connecting to WiFi..");
  WiFi.begin(ESSID,PSK);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("Connected to the WiFi network");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

/*
 * set up the MQTT connection
 *
*/

// this really doesn't do anything since we don't listen for topics
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
  client.setServer(mqttServer,mqttPort);                      // for an open broker
  // client.setServer(mqttServer,mqttPort,mqttUser,mqttPass)  // if you password protect your broker
  client.setCallback(callback);
  while (!client.connected()) {
    Serial.println("Connecting to MQTT...");
    if (client.connect("ESP8266Client" )) {
      Serial.println("connected");
      client.subscribe(mqttTopic);
    } else {
      Serial.print("failed with state ");
      Serial.println(client.state());
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
 float temp0 = sensors.getTempFByIndex(0);   // index 0 = first ds18b20 on the data bus
 return temp0;
}

/*
 * the overall setup routine
 *
*/
 
void setup() {

  Serial.begin(115200);
  delay(5000);
  Serial.print("program name = ");
  Serial.print(PROGRAM_NAME);
  Serial.print(", ver = ");
  Serial.println(PROGRAM_VER);
  Serial.print("chip id = ");
  Serial.println(chipid);
  
  pinMode(LED, OUTPUT);    // LED pin as output.
  
  setup_wifi();
  setup_mqtt();
  sensors.begin();

  Serial.println("done setup...");
}

/*
 * the overall loop
*/

void loop() {

  StaticJsonBuffer<200> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();

  root["espID"]     = chipid.toInt(); 
  root["timestamp"] = now();          // this is really 'uptime' since card reset
  
  float degF = read_ds18b20();
  if ((degF > -20) && (degF < 140)) {
    root["degF"] = degF;
  } else {
    Serial.print("error detecting degF: ");
    Serial.println(degF);
    delay(5000);
    return;                // hopefully this prevents publishing no value for degF
  }

  char JSONmessageBuffer[200];
  root.printTo(JSONmessageBuffer, sizeof(JSONmessageBuffer));
  
  // MQTT will drop you occasionally due to session timeout
  // so be sure to re-establish as needed
  if (!client.connected()) {
    Serial.println("mqtt needs to reconnect...");
    Serial.println(JSONmessageBuffer);   // so we see the payload including timestamp
    //blinkPattern(500,100,3);
    //blinkPattern(100,100,3);
    setup_mqtt();
    return;                     // hopefully we just re-loop() after MQTT comes up
  }

  // do the actual MQTT publishing
  //  - if it fails, report to Serial output
  //  - if the card thinks wifi is disconnected, just restart the card
  //        (symptoms are system responds to pings, but mqtt never reconnects)

  if (client.publish(mqttTopic, JSONmessageBuffer) == true) {
        blinkPattern(100,100,2);   // a little positive feedback via onboard LED
    } else {
        Serial.println("Error sending mqtt message");
        Serial.println(JSONmessageBuffer); // so we see the payload including timestamp
        blinkPattern(1000,1000,2);
        if (WiFi.status() != WL_CONNECTED) {
           Serial.println("restarting - wifi needs to reconnect in loop()...");
           ESP.restart();
        }
        setup_mqtt();
        return;
    }

  // to do: the delay should be defined at the top of this program
  delay(DELAY_MS);

}

// LOW=on, HIGH=OFF
void blinkPattern(int onTime,int offTime, int count) {
  for (int i=0; i<count; i++) {
    digitalWrite(LED,LOW);  delay(onTime);
    digitalWrite(LED,HIGH); delay(offTime);
  }
}


