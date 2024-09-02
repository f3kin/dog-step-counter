#include <Wire.h>
#include <MPU6050.h>
#include "esp_timer.h"
#include "communication.h"

MPU6050 mpu;
void setup(void) {

  Serial.begin(115200);
  while (!Serial) {
    delay(10); // will pause Zero, Leonardo, etc until serial console opens
  }

  Wire.begin(21, 22);

  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("Failed to connect to MPU6050");
    while (1) {
      delay(10);
    }
  }
  
  Serial.println("MPU6050 Connected!");


}


void loop() {
  // if (mpu.getMotionInterruptStatus()) {
  //step_counter();
  // Serial.println("Calling step Counter");
  // }

}