#ifndef ICM20948_IMU_H
#define ICM20948_IMU_H

#include "IMU.h"
#include <Adafruit_ICM20948.h>
#include <MadgwickAHRS.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

class ICM20948_IMU : public IMU {
private:
  struct Config {
    // static constexpr float ACCEL_SCALE = 9.81f / 16384.0f;  // 2G range: 16384 LSB/g
    // static constexpr float GYRO_SCALE = 250.0f / 32768.0f;  // 250 DPS range: 32768 LSB/dps
    // static constexpr float MAG_SCALE = 0.00015f;            // 0.15 µT/LSB
    static constexpr float ACCEL_SCALE = 1.0f;
    static constexpr float GYRO_SCALE = 1.0f;
    static constexpr float MAG_SCALE = 1.0f;
  };

  Adafruit_ICM20948 icm;
  bool debugEventTime = false;

public:
  ICM20948_IMU(float rate = 1000.0f) : IMU(rate) {}

  void setDebugEventTime(bool enable) {
    debugEventTime = enable;
  }

  bool initialize() override {
    name = "ICM20948";
    vTaskDelay(pdMS_TO_TICKS(100));

    if (!icm.begin_I2C()) return false;
    vTaskDelay(pdMS_TO_TICKS(100));

    filter.begin(samplingRate);
    filter6.begin(samplingRate);

    // Set up accelerometer
    icm.setAccelRange(ICM20948_ACCEL_RANGE_2_G);
    vTaskDelay(pdMS_TO_TICKS(50));

    // Set up gyroscope
    icm.setGyroRange(ICM20948_GYRO_RANGE_250_DPS);
    vTaskDelay(pdMS_TO_TICKS(50));

    // Set up magnetometer
    icm.setMagDataRate(AK09916_MAG_DATARATE_100_HZ);
    vTaskDelay(pdMS_TO_TICKS(50));

    return true;
  }

  bool read() override {
    bool success = false;

    sensors_event_t accel;
    uint32_t startTime = micros();
    if (icm.getAccelerometerSensor()->getEvent(&accel)) {
      uint32_t endTime = micros();
      if (debugEventTime) {
        Serial.printf("[ICM20948] Accel getEvent time: %lu us\n", endTime - startTime);
      }
      data.accelX = accel.acceleration.x * Config::ACCEL_SCALE;
      data.accelY = accel.acceleration.y * Config::ACCEL_SCALE;
      data.accelZ = accel.acceleration.z * Config::ACCEL_SCALE;
      accelUpdated = true;
      success = true;
    }
    

    // Read gyroscope data
    sensors_event_t gyro;
    startTime = micros();
    if (icm.getGyroSensor()->getEvent(&gyro)) {
      uint32_t endTime = micros();
      if (debugEventTime) {
        Serial.printf("[ICM20948] Gyro getEvent time: %lu us\n", endTime - startTime);
      }
      data.gyroX = gyro.gyro.x * Config::GYRO_SCALE;
      data.gyroY = gyro.gyro.y * Config::GYRO_SCALE;
      data.gyroZ = gyro.gyro.z * Config::GYRO_SCALE;
      gyroUpdated = true;
      success = true;
    }

    // Read magnetometer data
    sensors_event_t mag;
    startTime = micros();
    if (icm.getMagnetometerSensor()->getEvent(&mag)) {
      uint32_t endTime = micros();
      if (debugEventTime) {
        Serial.printf("[ICM20948] Mag getEvent time: %lu us\n", endTime - startTime);
      }
      data.magX = mag.magnetic.x * Config::MAG_SCALE;
      data.magY = mag.magnetic.y * Config::MAG_SCALE;
      data.magZ = mag.magnetic.z * Config::MAG_SCALE;
      magUpdated = true;
      success = true;
    }
    return success;
  }
};

#endif
