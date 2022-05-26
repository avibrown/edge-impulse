// Basic demo for accelerometer readings from Adafruit MPU6050

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

#define FREQUENCY_HZ        100
#define INTERVAL_MS         (1000 / (FREQUENCY_HZ + 1))

static unsigned long last_interval_ms = 0;


void setup(void) {
  Serial.begin(115200);
  delay(10);
    
  if (!mpu.begin()) {
      delay(10);
    }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    break;
  case MPU6050_RANGE_4_G:
    break;
  case MPU6050_RANGE_8_G:
    break;
  case MPU6050_RANGE_16_G:
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    break;
  case MPU6050_BAND_184_HZ:
    break;
  case MPU6050_BAND_94_HZ:
    break;
  case MPU6050_BAND_44_HZ:
    break;
  case MPU6050_BAND_21_HZ:
    break;
  case MPU6050_BAND_10_HZ:
    break;
  case MPU6050_BAND_5_HZ:
    break;
  }
  
  delay(100);
}

void loop() {

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  /* Print out the values */

    if (millis() > last_interval_ms + INTERVAL_MS) {
        last_interval_ms = millis();

    Serial.print(a.acceleration.x); Serial.print(",");
    Serial.print(a.acceleration.y); Serial.print(",");
    Serial.println(a.acceleration.z);
    }
  



}
