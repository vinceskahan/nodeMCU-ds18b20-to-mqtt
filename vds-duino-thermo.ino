/*
 *  GitHub link => https://github.com/vinceskahan/nodeMCU-ds18b20-to-mqtt
 *
 *  read a ds18b20 sensor and publish the value with MQTT, with status to the onboard LED and OLED display
 *
 *  the wifi/mqtt features may be disabled by grounding nodeMCU pin D6 (GPIO12)
 *
 * references:
 *   https://startingelectronics.org/tutorials/arduino/modules/OLED-128x64-I2C-display/
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

#include <ArduinoJson.h>

#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>

#include <ESP8266WiFi.h>         // for wifi
#include <PubSubClient.h>        // for MQTT pub/sub
#include <OneWire.h>             // for one-wire sensors
#include <DallasTemperature.h>   // for the ds18b20 specifically
#include <ArduinoJson.h>         // to use JSON - this is coded to v6 of this library
#include <Time.h>                // for now() - library is https://github.com/PaulStoffregen/Time
#include <TimeLib.h>             // for now() - library is https://github.com/PaulStoffregen/Time

#include "wifi.h"                // #define ESSID and PSK in this, see wifi.h.example for syntax

//------ start editing here ------------

// OLED display TWI address
#define OLED_ADDR   0x3C                  // you will not likely need to edit this

// ground this pin to disable wifi+mqtt
#define WIFI_ENABLE_PIN D6                // NodeMCU pin GPIO12 (D6)

// status link we blink in rare cases
#define LED D0                            // status LED we blink - NodeMCU pin GPIO16 (D0)

// which program and version are we
#define PROGRAM_NAME "vds-duino-thermo"   // so I can tell what is loaded on the nodeMCU long after the fact.
#define PROGRAM_VER  "v1.0"               // Based on a great idea from reddit user /u/blimpway in /r/esp8266

// pin the ds18b20 is connected to
#define ONE_WIRE_BUS 2                    // nodemcu pin D4 = pin 2

// how often to display and (optionally) publish in ms
const int DELAY_MS = 60000;

// MQTT settings
const char* mqttServer = "192.168.1.152"; // FQDN works too, but this saves DNS lookups
const int mqttPort = 1883;                // standard MQTT broker listens on this port

//const char* mqttUser = "none";          // uncomment these two if you password-protect MQTT broker 
//const char* mqttPass = "none;           // also alter the setup_mqtt( ) routine below to match

//------ stop editing here ------------

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

WiFiClient espClient;
PubSubClient client(espClient);

// ultimate MQTT topic will be ala "esp/obs/12345678"
String topicPrefix     = "esp/obs/";
String chipid          = String(ESP.getChipId()).c_str();
String together        = topicPrefix + chipid;
const char * mqttTopic = together.c_str();

// similarly lets do a unique client id based on chip id
String clientPrefix         = "esp-";
String clientTogether       = clientPrefix + chipid;
const char * mqttClientName = clientTogether.c_str();

// initialize variable so that we can reference it later
int wifiEnabled = 1;

// initialize the display using no reset pin, and verify the lib matches
Adafruit_SSD1306 display(-1);
#if (SSD1306_LCDHEIGHT != 64)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif

//---------------------------------------------------------
// this just writes 1 2 3 to the screen in different fonts
// changing after a couple seconds, mainly to exercise the screen
//---------------------------------------------------------
void display_test() {

 delay(2000);
 display.clearDisplay();
 display.display();

 display.setTextSize(1);
 display.setTextColor(WHITE);
 display.setCursor(27,30);
 display.print("size 1");
 display.display();
 
 delay(2000);
 display.clearDisplay();
 display.display();

 display.setTextSize(2);
 display.setTextColor(WHITE);
 display.setCursor(27,30);
 display.print("2");
 display.display();

 delay(2000);
 display.clearDisplay();
 display.display();

 display.setTextSize(3);
 display.setTextColor(WHITE);
 display.setCursor(27,30);
 display.print("3");
 display.display();
 
}

//---------------------------------------------------------
// set up wifi
//---------------------------------------------------------
void setup_wifi_connection() {
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

//---------------------------------------------------------
// a little debugging to the console
//---------------------------------------------------------
void serial_printProgramVersion() {
  Serial.print("program name = ");
  Serial.print(PROGRAM_NAME);
  Serial.print(", ver = ");
  Serial.println(PROGRAM_VER);
  Serial.print("chip id = ");
  Serial.println(chipid);
  Serial.print("mqtt clientname = ");
  Serial.println(mqttClientName);
  delay(1000);
}

//---------------------------------------------------------
// similarly, some debugging to the display
//    the display has to be set up before calling this of course
//---------------------------------------------------------
void display_printProgramVersion() {
  display.clearDisplay();
  display.display();
  
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

//---------------------------------------------------------
// for setup_mqtt, but this really doesn't do anything since we don't listen for topics
//---------------------------------------------------------
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

//---------------------------------------------------------
// set up mqtt - needs editing if you password protect the broker
//               (TO DO - could be made smarter of course)
//---------------------------------------------------------
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

//---------------------------------------------------------
// read the sensor and return its value
//---------------------------------------------------------
float read_ds18b20() {
 sensors.requestTemperatures();
 float temp0 = sensors.getTempFByIndex(0);   // index 0 = first ds18b20 on the data bus
 return temp0;
}

//---------------------------------------------------------
// utility routine to blink a light in a configurable way
//
// times are in ms so (1000,2000,3) would be a on-1-sec,off-2-sec sequence 3x
// LOW=on, HIGH=OFF
//---------------------------------------------------------
void blinkPattern(int onTime,int offTime, int count) {
  for (int i=0; i<count; i++) {
    digitalWrite(LED,LOW);  delay(onTime);
    digitalWrite(LED,HIGH); delay(offTime);
  }
}

//---------------------------------------------------------
// things we do in setup only if wifi is enabled
//---------------------------------------------------------
void setup_wifi() {
  // do the actual wifi setup
  display.clearDisplay();
  display.display();
  display.setTextSize(0);
  display.setTextColor(WHITE);
  display.setCursor(10,1);
  display.println("connecting to wifi");
  display.display();
  delay(2000);
  setup_wifi_connection();
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

  delay(2000);
  display.clearDisplay();
  display.display();
}

//----------------------------------------------------------------------------
//  usual arduino setup routine
//----------------------------------------------------------------------------

void setup() {

  // a little serial feedback
  Serial.begin(115200);
  delay(2000);                                      // give it time to settle
  serial_printProgramVersion();
  
  // initialize and clear display
  Serial.println("setting up display");
  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  delay(2000);                                     // give it time to settle

  // show Adafruit splash screen then clear
  display.display();                               
  delay(3000);
  display.clearDisplay();
  display.display();
   
  // display a pixel in each corner of the screen
  display.drawPixel(0, 0, WHITE);
  display.drawPixel(127, 0, WHITE);
  display.drawPixel(0, 63, WHITE);
  display.drawPixel(127, 63, WHITE);
  display.display();
  delay(1000);

  // lets see the program info on the screen too
  display_printProgramVersion();

  // check to see if we should enable wifi
  pinMode(WIFI_ENABLE_PIN, INPUT_PULLUP);
  wifiEnabled = digitalRead(WIFI_ENABLE_PIN); // pin grounded reads 0, pin open reads 1
  Serial.print("wifi enabled: ");
  Serial.println(wifiEnabled);

  // setup wifi - this will also set up mqtt once wifi is connected
  if (wifiEnabled) {
   setup_wifi();
  }

}

//---------------------------------------------------------------------------------
// the usual loop() routine here
//---------------------------------------------------------------------------------
//
// fyi - setCursor(col,row) where this is a 128x64 OLED
//                 25,10 is about 1/3 down top-to-bottom
//                 25,25 is about centered both ways

void loop() {
  // put your main code here, to run repeatedly:
  // display_test();
  // delay(3000);
  // return;

  StaticJsonDocument<200> root;       // to hold the values we optionally publish
  root["espID"]     = chipid.toInt();
  root["timestamp"] = now();          // this is really 'uptime' since card reset

  // read and display the temperature
  float degF = read_ds18b20();
  if ((degF > -20) && (degF < 140)) {
    root["degF"] = degF;
    delay(100);
    display.clearDisplay();
    display.setTextSize(0);
    display.setTextColor(WHITE);
    display.setCursor(25,25);
    display.setTextSize(2);
    display.print(degF);
    display.println(" F");
    
    display.display();

  } else {
    Serial.print("error detecting degF: ");
    Serial.println(degF);

    delay(100);
    display.clearDisplay();
    display.display();

    display.setTextSize(0);
    display.setTextColor(WHITE);
    display.setCursor(25,5);            // toward the top of the screen, centered left-right
    display.println("error: bad degF");
    display.setCursor(45,20);
    display.println(degF);
    display.display();

    delay(5000);
    return;                // hopefully this prevents publishing no value for degF
  }

  // and print to mqtt if we have wifi-enabled
  if (wifiEnabled) {
          char JSONmessageBuffer[200];
          serializeJson(root,JSONmessageBuffer);   // root holds the JSON-formatted payload
          
          // MQTT will drop you occasionally due to session timeout
          // so be sure to re-establish as needed
          if (!client.connected()) {
            Serial.println("");
            Serial.println("mqtt needs to reconnect...");
            setup_mqtt();
            return;               // hopefully we just re-loop() after MQTT comes up
          }

          // do the actual MQTT publishing
          //  - if it fails, report to Serial output
          //  - if the card thinks wifi is disconnected, just restart the card
          //        (symptoms are system responds to pings, but mqtt never reconnects)

          if (client.publish(mqttTopic, JSONmessageBuffer) == true) {
                Serial.println(JSONmessageBuffer);   // so we see the payload including timestamp
                blinkPattern(1,1,1);   // a little positive feedback via onboard LED
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
  
  delay(DELAY_MS);  // over 20000 or so and your MQTT will likely get disconnected, but we self-heal

}

//------ that's all folks -----------------------------------

