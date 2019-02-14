/*
    This file is part of RoombaEsp.

    RoombaEsp is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    RoombaEsp is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with RoombaEsp.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <Arduino.h>
#include <ESP8266WiFi.h>

#include <DNSServer.h>
#include <ESP8266mDNS.h>

#include <ESP8266WebServer.h>   // Include the WebServer library

#include <ArduinoJson.h>
#include <PubSubClient.h>

#include <WiFiUdp.h>
#include <ArduinoOTA.h>

#include <roomba.h>

#include "config.h"

WiFiClient wifiClient;

bool OTAStarted = false;

// roomba timers
int lastStateMsgTime = 0;
int lastWakeupTime = 0;

PubSubClient mqttClient(wifiClient);
const char *commandTopic PROGMEM = MQTT_COMMAND_TOPIC;
const char *statusTopic PROGMEM = MQTT_STATE_TOPIC;

// mqtt timer
int lastConnectTime = 0;

Roomba *roomba = NULL;

Roomba::RoombaSensor sensors[6] = {
    Roomba::Distance, // PID 19, 2 bytes, mm, signed
    Roomba::ChargingState, // PID 21, 1 byte
    Roomba::Voltage, // PID 22, 2 bytes, mV, unsigned
    Roomba::Current, // PID 23, 2 bytes, mA, signed
    Roomba::BatteryCharge, // PID 25, 2 bytes, mAh, unsigned
    Roomba::BatteryCapacity // PID 26, 2 bytes, mAh, unsigned
};

// Remote debugging over telnet. Just run:
// `telnet roomba.local` OR `nc roomba.local 23`
#include <RemoteDebug.h>
#define LOG(msg, ...) Debug.printf(msg, ##__VA_ARGS__);
RemoteDebug Debug;

// web server -------------------------------------------------------------------------------------

// Create a webserver object that listens for HTTP request on port 80
ESP8266WebServer WebServer(80);

// function prototypes for HTTP handlers
void handleRoot();
void handleNotFound();

// mqtt -------------------------------------------------------------------------------------------

void mqttCallback(char *topic, byte *payload, unsigned int length)
{
    LOG("Received mqtt callback for topic %s\n", topic);
    if (strcmp(commandTopic, topic) == 0) {
        // turn payload into a null terminated string
        char *cmd = (char *)malloc(length + 1);
        memcpy(cmd, payload, length);
        cmd[length] = 0;

        if (roomba != NULL) {
            if (!roomba->performMQTTCommand(cmd)) {
                LOG("Unknown command %s\n", cmd);
            }
        }

        free(cmd);
    }
}

void mqttSetup()
{
    mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
    mqttClient.setCallback(mqttCallback);
}

void mqttReconnect()
{
    // Attempt to connect
    if (mqttClient.connect(HOSTNAME, MQTT_USER, MQTT_PASS)) {
        LOG("MQTT connected\n");
        mqttClient.subscribe(commandTopic);
    }
}

void mqttSendStatus() {
    if (!mqttClient.connected()) {
        LOG("MQTT Disconnected, not sending status\n");
        return;
    }

    if (roomba == NULL) {
        LOG("Roomba is not ready, not sending status\n");
        return;
    }

    LOG("Reporting packet Distance:%dmm ChargingState:%d Voltage:%dmV Current:%dmA Charge:%dmAh Capacity:%dmAh\n",
        roomba->roombaState.distance,
        roomba->roombaState.chargingState,
        roomba->roombaState.voltage,
        roomba->roombaState.current,
        roomba->roombaState.charge,
        roomba->roombaState.capacity);

    StaticJsonBuffer<200> jsonBuffer;
    JsonObject& root = jsonBuffer.createObject();
    if (roomba->roombaState.capacity != 0) {
        root["battery_level"] = (roomba->roombaState.charge * 100) / roomba->roombaState.capacity;
    }
    root["cleaning"] = roomba->roombaState.cleaning;
    root["docked"] = roomba->roombaState.docked;
    root["charging"] = roomba->roombaState.chargingState == Roomba::ChargeStateReconditioningCharging
        || roomba->roombaState.chargingState == Roomba::ChargeStateFullCharging
        || roomba->roombaState.chargingState == Roomba::ChargeStateTrickleCharging;
    root["voltage"] = roomba->roombaState.voltage;
    root["current"] = roomba->roombaState.current;
    root["charge"] = roomba->roombaState.charge;
    String jsonStr;
    root.printTo(jsonStr);
    mqttClient.publish(statusTopic, jsonStr.c_str());
}

// ESP Sleep if necesary (battery low) ----------------------------------------------------------------

float readADC(int samples) {
  // Basic code to read from the ADC
  int adc = 0;
  for (int i = 0; i < samples; i++) {
    delay(1);
    adc += analogRead(A0);
  }
  adc = adc / samples;
  float mV = adc * ADC_VOLTAGE_DIVIDER;
  LOG("ADC for %d is %.1fmV with %d samples\n", adc, mV, samples);
  return mV;
}

void sleepIfNecessary() {
#ifdef ENABLE_ADC_SLEEP
  // Check the battery, if it's too low, sleep the ESP (so we don't murder the battery)
  float mV = readADC(10);
  // According to this post, you want to stop using NiMH batteries at about 0.9V per cell
  // https://electronics.stackexchange.com/a/35879 For a 12 cell battery like is in the Roomba,
  // That's 10.8 volts.
  if (mV < 10800) {
    // Fire off a quick message with our most recent state, if MQTT is connected
    LOG("Battery voltage is low (%.1fV). Sleeping for 10 minutes\n", mV / 1000);
    if (mqttClient.connected()) {
      StaticJsonBuffer<200> jsonBuffer;
      JsonObject& root = jsonBuffer.createObject();
      root["battery_level"] = 0;
      root["cleaning"] = false;
      root["docked"] = false;
      root["charging"] = false;
      root["voltage"] = mV / 1000;
      root["charge"] = 0;
      String jsonStr;
      root.printTo(jsonStr);
      mqttClient.publish(statusTopic, jsonStr.c_str(), true);
    }
    delay(200);

    // Sleep for 10 minutes
    ESP.deepSleep(600e6);
  }
#endif
}

// Wifi and OTA -----------------------------------------------------------------------------------

void wifiSetup()
{
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    while (WiFi.waitForConnectResult() != WL_CONNECTED) {
        Serial.println("Connection Failed! Rebooting...");
        delay(1000);
        ESP.restart();
    }

    Serial.println("");
    Serial.print("Connected to ");
    Serial.println(WIFI_SSID);
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());

    // Set host name
    String hostname(HOSTNAME);
    hostname.concat(".local");
    WiFi.hostname(hostname);

    // Start the mDNS responder for roomba.local
    if (MDNS.begin(HOSTNAME)) {
        Serial.print("* MDNS responder started. Hostname -> ");
        Serial.println(HOSTNAME);
    }

    MDNS.addService("telnet", "tcp", 23);
}

void onOTAStart() {
    roomba->pause();
    OTAStarted = true;
}

void OTASetup()
{
    ArduinoOTA.setHostname(HOSTNAME);
    ArduinoOTA.setPassword(OTA_PASSWORD);
    ArduinoOTA.begin();
    ArduinoOTA.onStart(onOTAStart);
}

// Main program (setup and main loop) -------------------------------------------------------------

void setup() {
    // High-impedence on the BRC_PIN
    pinMode(BRC_PIN, INPUT);

    Serial.begin(115200, SERIAL_8N1);

    // Start telnet debug
    Debug.begin(HOSTNAME);
    // Enable the reset command
    Debug.setResetCmdEnabled(true);
    // All messages are send to serial too, and can be seen in the  serial monitor
    Debug.setSerialEnabled(true);

    roomba = new Roomba();

    roomba->start();
    delay(100);

    // Reset stream sensor values
    roomba->stream({}, 0);
    delay(100);

    // Request sensor stream
    roomba->stream((const uint8_t *)sensors, sizeof(sensors));

    // Sleep immediately if ENABLE_ADC_SLEEP and the battery is low
    sleepIfNecessary();

    wifiSetup();

    OTASetup();

    mqttSetup();

    // Start web server
    // Call the 'handleRoot' function when a client requests URI "/"
    WebServer.on("/", handleRoot);
    // When a client requests an unknown URI (i.e. something other than "/"), call function "handleNotFound"
    WebServer.onNotFound(handleNotFound);
    // Actually start the server
    WebServer.begin();
}

void loop() {

    // Important callbacks that _must_ happen every cycle
    // ArduinoOTA.handle();
    
    Debug.handle();

    // Give a time for ESP8266
    yield();

    // Skip all other logic if we're running an OTA update
    if (OTAStarted) {
        return;
    }

    long now = millis();

    // If MQTT is not connected to the broker and enough time has passed, try to reconnect
    if (!mqttClient.connected() && (now - lastConnectTime) > 5000) {
        LOG("MQTT not connected. Reconnecting...\n");
        lastConnectTime = now;
        mqttReconnect();
    }

    // Wakeup the roomba at fixed intervals
    if (now - lastWakeupTime > 50000) {
        lastWakeupTime = now;
        if (!roomba->isCleaning()) {
            if (roomba->isDocked()) {
                roomba->wakeOnDock();
            } else {
                // roomba->wakeOffDock();
                roomba->wakeup();
            }
        } else {
            roomba->wakeup();
        }
    }

    // Report the status over mqtt at fixed intervals
    if (now - lastStateMsgTime > 10000) {
        lastStateMsgTime = now;
        if (now - roomba->getTimestamp() > 30000 || roomba->isSent()) {
            LOG("Roomba state already sent (%.1fs old)\n", (now - roomba->getTimestamp()) / 1000.0);
            LOG("Request stream\n");
            roomba->stream((const uint8_t *)sensors, sizeof(sensors));
        } else {
            mqttSendStatus();
            roomba->roombaState.sent = true;
        }
        sleepIfNecessary();
    }

    roomba->readSensorPacket();

    mqttClient.loop();

    // Listen for HTTP requests from clients
    WebServer.handleClient(); 
}

// See https://tttapa.github.io/ESP8266/Chap10%20-%20Simple%20Web%20Server.html

static void WebGetArg(const char* arg, char* out, size_t max)
{
  String s = WebServer.arg(arg);
  strlcpy(out, s.c_str(), max);
}

void handleRoot() {
    String s =  "<html>"
    "<head>"
    "<title>ROOMBA-ESP8266</title>"
    "<style>div,fieldset,input,select{padding:5px;font-size:1em;}input{width:100%;box-sizing:border-box;-webkit-box-sizing:border-box;-moz-box-sizing:border-box;}"
    "select{width:100%;}textarea{resize:none;width:98%;height:318px;padding:5px;overflow:auto;}body{text-align:center;font-family:verdana;}td{padding:0px;}"
    "button{border:0;border-radius:0.3rem;background-color:#1fa3ec;color:#fff;line-height:2.4rem;font-size:1.2rem;width:100%;-webkit-transition-duration:0.4s;transition-duration:0.4s;cursor:pointer;}"
    "button:hover{background-color:#0e70a4;}.bred{background-color:#d43535;}.bred:hover{background-color:#931f1f;}.bgrn{background-color:#47c266;}"
    ".bgrn:hover{background-color:#5aaf6f;}a{text-decoration:none;}.p{float:left;text-align:left;}.q{float:right;text-align:right;}</style>"
    "</head>"
    "<body>"
    "<div style='text-align:left;display:inline-block;min-width:340px;'><div style='text-align:center;'>"
    "<h1>" D_PROGRAMNAME "</h1>"
    "<form action='wi' method='get'><button>Configure WiFi</button></form><br>"
    "<form action='/' method='post'><button>Turn On</button><input name='CM' type='hidden' value='turn_on'></form><br>"
    "<form action='/' method='post'><button>Turn Off</button><input name='CM' type='hidden' value='turn_off'></form><br>"
    "<form action='/' method='post'><button>Pause / Continue</button><input name='CM' type='hidden' value='start_pause'></form><br>"
    "<form action='/' method='post'><button>Stop</button><input name='CM' type='hidden' value='stop'></form><br>"
    "<form action='/' method='post'><button>Clean Spot</button><input name='CM' type='hidden' value='clean_spot'></form><br>"
    "<form action='/' method='post'><button>Locate</button><input name='CM' type='hidden' value='locate'></form><br>"
    "<form action='/' method='post'><button>Return to Base</button><input name='CM' type='hidden' value='return_to_base'></form><br>"
    "<div style='text-align:right;font-size:11px;'><hr/><a href='http://" HOSTNAME ".local' target='_blank' style='color:#aaa;'>" D_PROGRAMNAME " by " D_AUTHOR "</a></div>"
    "</div>"
    "</div></div>"
    "</body>"
    "</html>";

    if (WebServer.hasArg("CM")) {
        char cmd[100];
        WebGetArg("CM", cmd, sizeof(cmd));
        roomba->performMQTTCommand(cmd);
    }

    WebServer.send(200, "text/html", s);
}

void handleNotFound(){
    // Send HTTP status 404 (Not Found) when there's no handler for the URI in the request
    WebServer.send(404, "text/html", "<h1>I find your lack of faith disturbing...</h1>");
}
