#ifndef MYCOMMS_H
#define MYCOMMS_H

#include <WebServer.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <AsyncUDP.h>
#include <WiFiUdp.h>
#include <time.h>
#include <Arduino.h>
#include <WiFi.h>

//***************  Determine the time ***************//
const char* id_token = getenv("ID_TOKEN");
const char* ntpServer = "au.pool.ntp.org";
const char* ntpServer1 = "1.au.pool.ntp.org";
const char* ntpServer2 = "2.au.pool.ntp.org";
const long gmtOffset_sec = 3600 * 11;  // GMT+11 for AEDT
const int daylightOffset_sec = 0;  // No additional offset needed during daylight saving
char timeStringBuff[50];

bool getCurrentTimeStamp() {
  struct tm timeInfo;
  bool getTimer = false;
  for (int i = 0; i < 5; i++) {
    if (getLocalTime(&timeInfo)) {
      getTimer = true;
      break;
    } else {
      Serial.println("Failed to obtain time");
      configTime(gmtOffset_sec, daylightOffset_sec, ntpServer, ntpServer1, ntpServer2);
    }
  }
  strftime(timeStringBuff, sizeof(timeStringBuff), "%A, %B %d %Y %H:%M:%S", &timeInfo);
  return getTimer;
}

//***************  Generate JSON Payload ****************//
StaticJsonDocument<250> jsonDocument;
char buffer[250];

inline void create_json(uint32_t stepsAmount) {
  // Clear and fill json
  jsonDocument.clear();
  jsonDocument["steps"] = stepsAmount;
  jsonDocument["dog_id"] = id_token;
  jsonDocument["time"] = timeStringBuff;

  serializeJson(jsonDocument, buffer);
}

unsigned long lastTime = 0;
unsigned long timerDelay = 10000;



///***************  Send data to server ****************//

bool send_data_to_server(uint32_t stepsAmount) {
    bool ret = false;
    create_json(stepsAmount);
    HTTPClient http;
    WiFiClient client;
    http.begin(getenv("SERVER_NAME"));
    http.addHeader("Content-Type", "application/json");
    int responseCode = http.POST(buffer);
    http.end();
    ret = true;
    return ret;
}

/* INTEGRATE THE FOLLOWING CODE

unsigned long lastTime = 0;
unsigned long timerDelay = 10000;
HTTPClient http;
WiFiClient client;

inline bool send_data_to_server() {
  bool ret = false;
  if ((millis() - lastTime) >= timerDelay) {
    // Check WiFi connection status
    if (WiFi.status() == WL_CONNECTED && getCurrentTimeStamp()) {
      http.addHeader("Content-Type", "application/json");
      int current_steps = step_count;
      create_json(current_steps);
      Serial.print("Post pedometer: ");
      Serial.println(current_steps);
      int httpResponseCode = http.POST(buffer);

      Serial.print("HTTP Response code: ");
      Serial.println(http.errorToString(httpResponseCode));
      Serial.println(httpResponseCode);
      Serial.println(buffer);

      if (httpResponseCode == 201) {
        Serial.println("Post successful");
        step_count = step_count - current_steps;
        ret = true;
      } else {
        ret = false;
      }
    } else {
      Serial.println("WiFi Disconnected");
      ret = false;
    }
    lastTime = millis();
  }
  return ret;
}

inline void setup_wifi() {
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to the WiFi network");
  Serial.println("IP Address: ");
  Serial.println(WiFi.localIP());

  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer, ntpServer1, ntpServer2);
  getCurrentTimeStamp();

  //  Your Domain name with URL path or IP address with path
  // http.begin(serverName, root_ca);
  http.begin(serverName);
}*/

#endif