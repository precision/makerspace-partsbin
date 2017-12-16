/*
 *  Copyright 2017 Google Inc. All Rights Reserved.
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 */

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_BME280.h>

#include "user_config.h"

// ESP12 has the LED on GPIO2
#define BUILTIN_LED 2

WiFiClient espClient;
PubSubClient mqttclient(espClient);

int sensorValue;
float voltage;
float temperature;
float pressure;
float humidity;
unsigned long boot_timer;

StaticJsonBuffer<512> jsonBuffer;
JsonObject& state = jsonBuffer.createObject();
JsonObject& wifi = state.createNestedObject("wifi");
StaticJsonBuffer<256> sensorBuffer;
JsonObject& sensor = jsonBuffer.createObject();

boolean readBMP180() {
  Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);

  if(!bmp.begin()) {
    Serial.println("Could not find a valid BMP180 sensor.");
    return false;
  }
  delay(100);

  sensors_event_t event;
  bmp.getEvent(&event);
  if (event.pressure) {
    bmp.getTemperature(&temperature);
    pressure = event.pressure;
  }

  return true;
}

boolean readBMP280() {
  Adafruit_BME280 bme;

  if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BME280 sensor.");
    return false;
  }
  delay(100);

  temperature = bme.readTemperature();
  pressure = bme.readPressure() / 100.0F;
  humidity = bme.readHumidity();
  return true;
}

void setup() {
  // toggle the mosfets on before anything else
  pinMode(MOSFET_PIN, OUTPUT);
  digitalWrite(MOSFET_PIN, HIGH);

  pinMode(A0, INPUT);
  pinMode(BUILTIN_LED, OUTPUT);

  boot_timer = millis();
  digitalWrite(BUILTIN_LED, LOW); // LED on

  Serial.begin(115200);

  // Calculating battery voltage before bringing up Wifi
  sensorValue = analogRead(A0);
  voltage = sensorValue * (VOLTAGE_DIVIDER / 1023.0);  // correct for new black boards
  state["voltage"] = round(voltage*100)/100.00;
  state["adc"] = sensorValue;
  state["reset_reason"] = ESP.getResetReason();

  // Connect to WiFi
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.config(IP, GATEWAY, SUBNET);
  WiFi.begin(SSID, PASSWORD, CHANNEL, BSSID);
  Serial.println("");
  Serial.print("Connecting");
  int connect_counter = 0;
  while (WiFi.status() != WL_CONNECTED) {
    connect_counter += 1;
    if (connect_counter > 200) {
      // bail if we can't get a wifi signal in 5 seconds
      Serial.println("Unable to Connect to WiFi");
      break;
    }
    delay(25);
    Serial.print(".");
  }
  Serial.println("");

  if (WiFi.status() == WL_CONNECTED) {
    // Store the WiFi in the JSON
    wifi["ip"] = WiFi.localIP().toString();
    wifi["ssid"] = String(SSID);
    wifi["rssi"] = WiFi.RSSI();
    wifi["bssid"] = WiFi.BSSIDstr();
    wifi["mac"] = WiFi.macAddress();

    Serial.print("Connected to: ");
    Serial.println(SSID);
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    Serial.println("");

    unsigned long wifi_connect = millis() - boot_timer;
    Serial.printf("Wifi Connected in: %d milliseconds\r\n", wifi_connect);
  }

  digitalWrite(BUILTIN_LED, HIGH); // LED off
}


void loop() {
  char tmpBuf[512];

#ifndef BUTTON_MODE
  if (!readBMP180()) {
    readBMP280();
  }
  sensor["temperature"] = temperature;
  sensor["pressure"] = pressure;
  sensor["humidity"] = humidity;
#endif

  // Only do the mqtt dance if we're connected to Wifi
  if (WiFi.status() == WL_CONNECTED) {
    // Send to MQTT
    mqttclient.setServer(MQTT_SERVER_IP, 1883);
    while (!mqttclient.connected()) {
      Serial.print("Attempting MQTT connection...");
      if (mqttclient.connect(String(ESP.getChipId()).c_str(), MQTT_USER, MQTT_PASSWORD)) {
        Serial.println(" connected.");
#ifdef BUTTON_MODE
        // toggle the button
        mqttclient.publish(MQTT_TOGGLE_TOPIC, String("TOGGLE").c_str(), true);
#else
        // publish the sensor info
        memset(tmpBuf, 0, sizeof(tmpBuf));
        sensor.printTo(tmpBuf, sizeof(tmpBuf));
        mqttclient.publish(MQTT_SENSOR_TOPIC, tmpBuf, true);
        sensor.prettyPrintTo(Serial);
#endif
        state["uptime"] = millis() - boot_timer;
        memset(tmpBuf, 0, sizeof(tmpBuf));
        state.printTo(tmpBuf, sizeof(tmpBuf));
        state.prettyPrintTo(Serial);

        mqttclient.publish(MQTT_STATE_TOPIC, tmpBuf, true);
      } else {
        Serial.print(" failed, Error value=");
        Serial.println(mqttclient.state());
        break;
      }
    }
    mqttclient.disconnect();
  }

  Serial.println();
  delay(50); // give serial some time to drain

  // Turn off the mosfets
  Serial.println("Mosfet Off");
  digitalWrite(MOSFET_PIN, LOW);

  // night night, in button mode this only holds us here until the caps discharge and power us off
  Serial.printf("Deep sleeping for %d seconds\n\n", SLEEP_SECONDS);
  ESP.deepSleep(SLEEP_SECONDS * 1000000);
}
