/*
 *  GitHub link => https://github.com/vinceskahan/nodeMCU-ds18b20-to-mqtt
 *
 *  read a ds18b20 sensor and publish the value with MQTT, with status to the onboard LED and OLED display
 *
 *  the wifi/mqtt features may be disabled by grounding nodeMCU pin D6 (GPIO12)
 *
 * references:
 *   https://techtutorialsx.com/2017/04/09/esp8266-connecting-to-mqtt-broker/
 *   https://create.arduino.cc/projecthub/TheGadgetBoy/ds18b20-digital-temperature-sensor-and-arduino-9cc806
 *   https://github.com/arduino-libraries/NTPClient/blob/master/examples/Basic/Basic.ino
 *
 * this is tested on a nodeMCU esp8266
 *  * this is a merge of:
 *    - the adafruit ssd1306_128x32_i2c.ino example sketch from their library
 *    - my local tweaks, modifications, and crimes against arduino best practices
 *
 * all bugs are mine, please don't bother the nice folks at Adafruit who wrote the code this is based on...
 *
 * there are undoubtedly copyright notices that I failed to cut+paste into this, but suffice it to say
 * that the code I learned from (mentioned above) is much appreciated
 *
 * ----------------------------------------------
 *
 * Wiring Setup:
 *
 *   ds18b20:
 *     GRD to ground rail on breadboard
 *     VCC to power rail on breadboard
 *     DAT to GPIO2 (nodeMCU pin D4)
 *     4.7k ohm resistor between power and data
 *
 *   OLED:
 *    SDA to GPIO4 (nodeMCU pin D2)
 *    SCL to GPIO5 (nodeMCU pin D1)
 *    GRD to ground rail on breadboard
 *    VCC to power rail on breadboard
 *
 *   nodeMCU:
 *    D1 (nodeMCU GPIO5) to OLED SDA
 *    D2 (nodeMCU GPIO4) to OLED SCL
 *    D4 to ds18b20 data pin
 *    3V3  to power rail on breadboard
 *    GND  to ground rail on breadboard
 *
 *   optional wifi disable jumper:
 *    D6 (nodeMCU GPIO12) to ground
 *
 * ----------------------------------------------
 *
 * Software used here:
 *     Arduino IDE 1.8.5 on a Macbook Air
 *        at least one driver for the Air to be able to load the nodeMCU
 *        over usb-to-serial port but which one has been list to the sands of time...
 *     Adafruit SSD1306 library version 1.1.2
 *     Adafruit GFX library version 1.2.7
 *
 * Installed but probably not needed for the code below....
 *     DHT sensor library by Adafruit version 1.3.0
 *     ESP8266 Weather Station by ThingPulse version 1.3.2
 *     ESP8266 and ESP32 Oled Driver for SSD1306 by Daniel Eichhorn et.al. version 3.2.7
 *
*/

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

#include <OneWire.h>
#include <DallasTemperature.h>

//#include <NTPClient.h>

#include <WiFiUDP.h>

#include <Time.h>          // https://github.com/PaulStoffregen/Time
#include <TimeLib.h>       // https://github.com/PaulStoffregen/Time

#include "wifi.h"          // #define ESSID and PSK in this, see wifi.h.example for syntax

//--- for OLED display ---
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "DHT.h"

/*
 * for the particular mono OLED display from the Weather Station kit
 * we need to fake the OLED_RESET setting below.  Some smart fellow
 * on internet used LED_BUILTIN to make this work, which worked for me
 * so I used it here too.  Unfortunately I lost the page/author of the
 * actual smarts (sorry) so I can't credit them with the wizardry.
 *
 * unfortunately this will blink the onboard bright-blue LED every time
 * the screen refreshes, but that does serve as a heartbeat indicator.....
 *
*/

#define OLED_RESET LED_BUILTIN
Adafruit_SSD1306 display(OLED_RESET);


//-----------------------------------------------------
//
// Start editing here
//
//

#define PROGRAM_NAME "vds-duino-test"    // so I can tell what is loaded on the nodeMCU long after the fact.
#define PROGRAM_VER  "6"                 // Based on a great idea from reddit user /u/blimpway in /r/esp8266

const int DELAY_MS = 15000;              // how often to publish in ms

const char* mqttServer = "192.168.1.24"; // FQDN works too, but this saves DNS lookups
const int mqttPort = 1883;
//const char* mqttUser = "none";         // also alter the setup_mqtt( ) routine below if
//const char* mqttPass = "none;          // you password-protect your MQTT broker

#define ONE_WIRE_BUS 2                   // nodemcu pin D4 the ds18b20 data pin is connect to

#define LED D0                           // status LED we blink - NodeMCU pin GPIO16 (D0)

#define WIFI_ENABLE_PIN D6               // open=enable wifi, ground=disable - NodeMCU pin GPIO12 (D6)

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

// similarly lets do a unique client id based on chip id
String clientPrefix         = "esp-";
String clientTogether       = clientPrefix + chipid;
const char * mqttClientName = clientTogether.c_str();

// so we can reference this later
int wifiEnabled = 1;

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
    if (client.connect(mqttClientName)) {
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

void printProgramVersion() {
  Serial.print("program name = ");
  Serial.print(PROGRAM_NAME);
  Serial.print(", ver = ");
  Serial.println(PROGRAM_VER);
  Serial.print("chip id = ");
  Serial.println(chipid);
  Serial.print("mqtt clientname = ");
  Serial.println(mqttClientName);

  display.setTextSize(0);
  display.setTextColor(WHITE);
  display.setCursor(10,1);
  display.println(PROGRAM_NAME);
  display.setCursor(10,10);
  display.println(PROGRAM_VER);
  display.setCursor(10,20);
  display.println(mqttClientName);
  display.display();
  delay(5000);

}

/*
 * the overall setup routine
 *
*/

void setup() {

  Serial.begin(115200);
  delay(5000);
  printProgramVersion();

  pinMode(WIFI_ENABLE_PIN, INPUT_PULLUP);

  // pin grounded reads 0, pin open reads 1
  wifiEnabled = digitalRead(WIFI_ENABLE_PIN);
  Serial.print("wifi enabled: ");
  Serial.println(wifiEnabled);

  pinMode(LED, OUTPUT);    // LED pin as output.
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);

  delay(2000);
  display.display();  // show Adafruit splash screen initially

  delay(2000);
  display.clearDisplay();
  display.display();

  if (wifiEnabled) {

    // setup wifi
    display.setTextSize(0);
    display.setTextColor(WHITE);
    display.setCursor(10,1);
    display.println("connecting to wifi");
    display.display();

    delay(2000);

    setup_wifi();
    display.setCursor(10,10);
    display.println("success!");
    display.setCursor(10,20);
    display.println(WiFi.localIP());
    display.display();

    delay(2000);
    display.clearDisplay();
    display.display();

    // setup mqtt
    display.setTextSize(0);
    display.setTextColor(WHITE);
    display.setCursor(10,1);
    display.println("connecting to mqtt");
    display.display();

    setup_mqtt();

    delay(2000);
    display.setCursor(10,10);
    display.println("success!");
    display.display();

  }

  delay(2000);
  display.clearDisplay();
  display.display();

  // setup sensors
  display.setTextSize(0);
  display.setTextColor(WHITE);
  display.setCursor(10,1);
  display.println("setting up sensors");
  display.display();

  sensors.begin();

  delay(2000);
  display.clearDisplay();
  display.display();

  display.setTextSize(0);
  display.setTextColor(WHITE);
  display.setCursor(10,1);
  display.println("done setup");
  display.display();

  delay(2000);
  display.clearDisplay();
  display.display();
  printProgramVersion();

  Serial.println("done setup...");
  delay(2000);
  display.clearDisplay();
  display.display();

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
    delay(2000);
    display.clearDisplay();
    display.setTextSize(0);
    display.setTextColor(WHITE);
    display.setCursor(25,10);
    display.setTextSize(2);
    display.print(degF);
    display.println(" F");
    display.display();

  } else {
    Serial.print("error detecting degF: ");
    Serial.println(degF);

   delay(2000);
   display.clearDisplay();
   display.display();

   display.setTextSize(0);
   display.setTextColor(WHITE);
   display.setCursor(25,5);
   display.println("error: bad degF");
   display.setCursor(45,20);
   display.println(degF);
   display.display();

   delay(5000);
   return;                // hopefully this prevents publishing no value for degF
  }

  if (wifiEnabled) {
          char JSONmessageBuffer[200];
          root.printTo(JSONmessageBuffer, sizeof(JSONmessageBuffer));

          // MQTT will drop you occasionally due to session timeout
          // so be sure to re-establish as needed
          if (!client.connected()) {
            Serial.println("mqtt needs to reconnect...");
            Serial.println(JSONmessageBuffer);   // so we see the payload including timestamp
            setup_mqtt();
            return;               // hopefully we just re-loop() after MQTT comes up
          }

          // do the actual MQTT publishing
          //  - if it fails, report to Serial output
          //  - if the card thinks wifi is disconnected, just restart the card
          //        (symptoms are system responds to pings, but mqtt never reconnects)

          if (client.publish(mqttTopic, JSONmessageBuffer) == true) {
                blinkPattern(100,100,1);   // a little positive feedback via onboard LED
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

  }

  delay(DELAY_MS);

}

// LOW=on, HIGH=OFF
void blinkPattern(int onTime,int offTime, int count) {
  for (int i=0; i<count; i++) {
    digitalWrite(LED,LOW);  delay(onTime);
    digitalWrite(LED,HIGH); delay(offTime);
  }
}


