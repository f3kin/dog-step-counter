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


///***************  Send data to server ****************//

// Add time delay functionality
unsigned long lastTime = 0;
unsigned long timerDelay = 10000;

bool send_data_to_server(uint32_t stepsAmount) {
    bool ret = false;
    

    if ((millis() - lastTime) >= timerDelay) {
      if (WiFi.status() == WL_CONNECTED && getCurrentTimeStamp()) {
        // Setup HTTP client
        HTTPClient http;
        WiFiClient client;
        Serial.println("Server URL test");
        Serial.println(serverUrl);
        http.begin(serverUrl);
        http.addHeader("Content-Type", "application/json");

        // Create JSON payload and send
        create_json(stepsAmount);
        int responseCode = http.POST(buffer);
        http.end();
        Serial.print("HTTP Response code: ");
        Serial.println(responseCode);

        // Check the Post was successful
        if (responseCode == 200) {
          Serial.println("Post successful");
          ret = true;
        }
        else{
          Serial.println("Post failed");
        } 
      }
    }
    return ret;
}


#endif