/*
 *  read a ds18b20 sensor and publish the value with MQTT
 *
 * references:
 *   https://techtutorialsx.com/2017/04/09/esp8266-connecting-to-mqtt-broker/
 *   https://create.arduino.cc/projecthub/TheGadgetBoy/ds18b20-digital-temperature-sensor-and-arduino-9cc806
 *
*/

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

#include <OneWire.h>
#include <DallasTemperature.h>

#include "wifi.h"     // #define ESSID and PSK in this, see wifi.h.example for syntax

/*
 * Start editing here
 *
*/

const char* mqttServer = "mqtt";
const int mqttPort = 1883;
//const char* mqttUser = "none";       // alter the setup_mqtt( ) routine below if
//const char* mqttPass = "none;        // you password-protect your MQTT broker

#define PROGRAM_NAME "vds-duino-test"  // so I can tell what is loaded on the nodeMCU long after the fact.
                                       // Based on a great idea from reddit user /u/blimpway in /r/esp8266

#define ONE_WIRE_BUS 4                 // nodemcu pin D2

/*
 * Stop editing here
 *
*/

OneWire oneWire(ONE_WIRE_BUS); 
DallasTemperature sensors(&oneWire);

WiFiClient espClient;
PubSubClient client(espClient);

String chipid = String(ESP.getChipId()).c_str();   // we use this later for debugging

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
  }
  Serial.println("Connected to the WiFi network");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

/*
 * setup the MQTT connection
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
      client.subscribe("esp/test");
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

  root["espID"] = chipid.toInt();
  
  float degF = read_ds18b20();
  if ((degF > -20) && (degF < 140)) {
    root["degF"] = degF;
  } else {
    Serial.print("error detecting degF: ");
    Serial.println(degF);
  }

  char JSONmessageBuffer[200];
  root.printTo(JSONmessageBuffer, sizeof(JSONmessageBuffer));
  Serial.println(JSONmessageBuffer);

  // MQTT will drop you occasionally due to session timeout
  // so be sure to re-establish as needed
  if (!client.connected()) setup_mqtt();

  // to do: the MQTT topic should be defined at the top of this program
  if (client.publish("esp/test", JSONmessageBuffer) == true) {
        Serial.println("Success sending message");
    } else {
        Serial.println("Error sending mqtt message");
    }

  /*
   * we've had no issues with wifi disconnecting but if we did,
   * than a similar check-then-reconnect approach ala the following
   * (untested) would be needed
   *
   * if (WiFi.status() != WL_CONNECTED) WiFi.begin(wifi_ssid, wifi_pass);
   *
  */

  // to do: the delay should be defined at the top of this program
  delay(5000);
}


