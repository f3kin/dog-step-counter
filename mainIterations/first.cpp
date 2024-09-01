#include <Arduino.h>


void setup(void) {

  Serial.begin(115200);
  delay(100); 
  Serial.println("Test");
  // Try to initialize!
  Serial.println("MPU6050 Found!");

}


void loop() {
}