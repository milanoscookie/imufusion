#ifndef ISM330DHCX_LIS3MDL_IMU_H
#define ISM330DHCX_LIS3MDL_IMU_H

#include "IMU.h"
#include <Adafruit_ISM330DHCX.h>
#include <Adafruit_LIS3MDL.h>
#include <MadgwickAHRS.h>

class ISM330DHCX_LIS3MDL_IMU : public IMU {
private:
  struct Config {
    // static constexpr float ACCEL_SCALE = 9.81f / 16384.0f;  // 2G range: 16384 LSB/g
    // static constexpr float GYRO_SCALE = 250.0f / 32768.0f;  // 250 DPS range: 32768 LSB/dps
    // static constexpr float MAG_SCALE = 0.0001f;             // 4 Gauss range: 0.0001 gauss/LSB
    static constexpr float ACCEL_SCALE = 1.0f;
    static constexpr float GYRO_SCALE = 1.0f;
    static constexpr float MAG_SCALE = 1.0f;
  };

  Adafruit_ISM330DHCX ism;
  Adafruit_LIS3MDL mag;

public:
  ISM330DHCX_LIS3MDL_IMU(float rate = 833.0f) : IMU(rate) {}

  bool initialize() override {
    name = "ISM330DHCX_LIS3MDL";
    if (!ism.begin_I2C())
    {
      Serial.println("Failed to initialize LSM6DSOX");
      return false;
    }
    if (!mag.begin_I2C())
    {
      Serial.println("Failed to initialize LIS3MDL");
      return false;
    }

    filter.begin(samplingRate);
    filter6.begin(samplingRate);

    // Set up accelerometer
    ism.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
    ism.setAccelDataRate(LSM6DS_RATE_833_HZ);

    // Set up gyroscope
    ism.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
    ism.setGyroDataRate(LSM6DS_RATE_833_HZ);

    // Set up magnetometer
    mag.setPerformanceMode(LIS3MDL_HIGHMODE);
    mag.setOperationMode(LIS3MDL_CONTINUOUSMODE);
    mag.setDataRate(LIS3MDL_DATARATE_1000_HZ);
    mag.setRange(LIS3MDL_RANGE_4_GAUSS);

    return true;
  }

  bool read() override {
    bool success = true;

    // Read accelerometer data
    sensors_event_t accel;
    uint32_t startTime = micros();
    if (ism.getAccelerometerSensor()->getEvent(&accel)) {
      uint32_t endTime = micros();
      if (debugEventTime) {
        Serial.printf("[ISM330DHCX] Accel getEvent time: %lu us\n", endTime - startTime);
      }
      data.accelX = accel.acceleration.x * Config::ACCEL_SCALE;
      data.accelY = accel.acceleration.y * Config::ACCEL_SCALE;
      data.accelZ = accel.acceleration.z * Config::ACCEL_SCALE;
      accelUpdated = true;
    } else {
      success = false;
    }

    // Read gyroscope data
    sensors_event_t gyro;
    startTime = micros();
    if (ism.getGyroSensor()->getEvent(&gyro)) {
      uint32_t endTime = micros();
      if (debugEventTime) {
        Serial.printf("[ISM330DHCX] Gyro getEvent time: %lu us\n", endTime - startTime);
      }
      data.gyroX = gyro.gyro.x * Config::GYRO_SCALE;
      data.gyroY = gyro.gyro.y * Config::GYRO_SCALE;
      data.gyroZ = gyro.gyro.z * Config::GYRO_SCALE;
      gyroUpdated = true;
    } else {
      success = false;
    }

    // Read magnetometer data
    sensors_event_t mag;
    startTime = micros();
    if (this->mag.getEvent(&mag)) {
      uint32_t endTime = micros();
      if (debugEventTime) {
        Serial.printf("[ISM330DHCX] Mag getEvent time: %lu us\n", endTime - startTime);
      }
      data.magX = mag.magnetic.x * Config::MAG_SCALE;
      data.magY = mag.magnetic.y * Config::MAG_SCALE;
      data.magZ = mag.magnetic.z * Config::MAG_SCALE;
      magUpdated = true;
    } else {
      success = false;
    }

    return success;
  }
};

#endif
