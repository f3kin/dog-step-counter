#include "mpu6050Uitls.h"
#include "esp_timer.h"
#include "communication.h"

float accel_bias[3] = {0, 0, 0};
float gyro_bias[3] = {0, 0, 0};

float quart[4] = {1.0f, 0.0f, 0.0f, 0.0f};
float delta_t = 0.0f;
#define AVG_BUFF_SIZE 20

sensors_event_t a, g, temp;
uint64_t now_time = 0, prev_time = 0;
int8_t range = 0;
float accel_g_x, accel_g_y, accel_g_z;
float accel_int_x, accel_int_y, accel_int_z;
float gyro_ds_x, gyro_ds_y, gyro_ds_z, accel_res, gyro_res, temp_c;
int accel_x_avg_buff[AVG_BUFF_SIZE];
int accel_y_avg_buff[AVG_BUFF_SIZE];
int accel_z_avg_buff[AVG_BUFF_SIZE];
int accel_x_avg_buff_count = 0;
int accel_y_avg_buff_count = 0;
int accel_z_avg_buff_count = 0;
int accel_x_avg, accel_y_avg, accel_z_avg;
int min_reg_accel_x = 0, min_reg_accel_y = 0, min_reg_accel_z = 0;
int max_reg_accel_x = 0, max_reg_accel_y = 0, max_reg_accel_z = 0;
int min_curr_accel_x, min_curr_accel_y, min_curr_accel_z;
int max_curr_accel_x, max_curr_accel_y, max_curr_accel_z;
int dy_thres_accel_x = 0, dy_thres_accel_y = 0, dy_thres_accel_z = 0;
int dy_chan_accel_x, dy_chan_accel_y, dy_chan_accel_z;
int sample_new = 0, sample_old = 0;
int step_size = 200;
int active_axis = 0, interval = 500000;
int step_changed = 0;


bool ledReadOn = true;
bool bluetoothConnected = true;

#define RESISTOR_R1 17800
#define RESISTOR_R2 9640

float mapping(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void setup(void) {

  Serial.begin(115200);
  while (!Serial) {
    delay(10); // will pause Zero, Leonardo, etc until serial console opens
  }


  // Try to initialize!
  if (!mpu.begin()) {
    while (1) {
      Serial.println("Failed to find MPU6050 chip");
      delay(1000);
    }
  }
  Serial.println("MPU6050 Found!");

  Serial.println("MPU6050 calibrating...");
  mpu6050_calibrate(accel_bias, gyro_bias);
  Serial.println("MPU6050 calibrate finished!");
  Serial.print("Accel Bias :");
  Serial.print(accel_bias[0]);
  Serial.print(",");
  Serial.print(accel_bias[1]);
  Serial.print(",");
  Serial.print(accel_bias[2]);
  Serial.print(", ");
  Serial.print("Gyro Bias :");
  Serial.print(gyro_bias[0]);
  Serial.print(",");
  Serial.print(gyro_bias[1]);
  Serial.print(",");
  Serial.print(gyro_bias[2]);
  Serial.println("");

  // setup motion detection
  mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
  mpu.setMotionDetectionThreshold(1);
  mpu.setMotionDetectionDuration(1);
  mpu.setInterruptPinLatch(true); // Keep it latched.  Will turn off when reinitialized.
  mpu.setInterruptPinPolarity(true);
  mpu.setMotionInterrupt(true);

  delay(100);
}

void loop() {
  // if (mpu.getMotionInterruptStatus()) {
  step_counter();
  // Serial.println("Calling step Counter");
  // }

}