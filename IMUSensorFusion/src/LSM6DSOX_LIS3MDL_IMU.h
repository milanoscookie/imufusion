#ifndef LSM6DSOX_LIS3MDL_IMU_H
#define LSM6DSOX_LIS3MDL_IMU_H

#include "IMU.h"
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_LIS3MDL.h>
#include <MadgwickAHRS.h>

class LSM6DSOX_LIS3MDL_IMU : public IMU {
private:
  struct Config {
    // static constexpr float ACCEL_SCALE = 9.81f / 16384.0f;  // 2G range: 16384 LSB/g
    // static constexpr float GYRO_SCALE = 250.0f / 32768.0f;  // 250 DPS range: 32768 LSB/dps
    // static constexpr float MAG_SCALE = 0.0001f;             // 4 Gauss range: 0.0001 gauss/LSB

    static constexpr float ACCEL_SCALE = 1.0f;
    static constexpr float GYRO_SCALE = 1.0f;
    static constexpr float MAG_SCALE = 1.0f;
  };

  Adafruit_LSM6DSOX imu;
  Adafruit_LIS3MDL mag;

public:
  LSM6DSOX_LIS3MDL_IMU(float rate = 833.0f) : IMU(rate) {}

  bool initialize() override {
    name = "LSM6DSOX_LIS3MDL";

    bool success = true;
    if (!imu.begin_I2C())
    {
      Serial.println("Failed to initialize LSM6DSOX");
      success= false;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
    if (!mag.begin_I2C(0x1E))
    {
      Serial.println("Failed to initialize LIS3MDL");
      success = false;
      vTaskDelay(pdMS_TO_TICKS(100));
      if(mag.begin_I2C()) {
        Serial.println("JK");
        success = true;
      }
    }

    if(!success) return false;

    filter.begin(samplingRate);
    filter6.begin(samplingRate);

    imu.enableI2CMasterPullups(true); 

    // Set up accelerometer
    imu.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
    imu.setAccelDataRate(LSM6DS_RATE_833_HZ);

    // Set up gyroscope
    imu.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
    imu.setGyroDataRate(LSM6DS_RATE_833_HZ);

    // Set up magnetometer
    mag.setPerformanceMode(LIS3MDL_HIGHMODE);
    mag.setOperationMode(LIS3MDL_CONTINUOUSMODE);
    mag.setDataRate(LIS3MDL_DATARATE_1000_HZ);
    mag.setRange(LIS3MDL_RANGE_4_GAUSS);

    return true;
  }

  bool read() override {
    bool success = false;

    // Read accelerometer data
    sensors_event_t accel;
    uint32_t startTime = micros();
    if (imu.getAccelerometerSensor()->getEvent(&accel)) {
      uint32_t endTime = micros();
      if (debugEventTime) {
        Serial.printf("[LSM6DSOX] Accel getEvent time: %lu us\n", endTime - startTime);
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
    if (imu.getGyroSensor()->getEvent(&gyro)) {
      uint32_t endTime = micros();
      if (debugEventTime) {
        Serial.printf("[LSM6DSOX] Gyro getEvent time: %lu us\n", endTime - startTime);
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
    if (this->mag.getEvent(&mag)) {
      uint32_t endTime = micros();
      if (debugEventTime) {
        Serial.printf("[LSM6DSOX] Mag getEvent time: %lu us\n", endTime - startTime);
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
