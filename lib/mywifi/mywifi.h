#ifndef MYWIFI_H
#define MYWIFI_H

#include <Arduino.h>
#include <WiFi.h>


const char* ssid = getenv("SSID");
const char* password = getenv("PASSWORD");

void wifi_setup() {
  Serial.println("WiFi Connection Test");
  
  WiFi.mode(WIFI_STA);  // Set WiFi to station mode
  WiFi.begin(ssid, password);
  
  Serial.println("Connecting to WiFi...");
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nConnected to WiFi");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    Serial.print("Signal Strength (RSSI): ");
    Serial.println(WiFi.RSSI());
  } else {
    Serial.println("\nFailed to connect to WiFi");
    Serial.print("WiFi status: ");
    Serial.println(WiFi.status());
  }
}

void wifi_loop() {
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi still connected");
  } else {
    Serial.println("WiFi disconnected");
  }
  delay(5000);  // Check every 5 seconds
}

#endif