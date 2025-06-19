#ifndef IMU_H
#define IMU_H

#include <Adafruit_Sensor.h>
#include <MadgwickAHRS.h>

class IMU {
protected:
  Madgwick filter;
  Madgwick filter6;
  float samplingRate; // Hz
  sensors_event_t accel, gyro, temp, magnetic;
  String name = "Generic";
  bool accelUpdated = false;
  bool gyroUpdated = false;
  bool magUpdated = false;

public:
  IMU(float rate = 833.0f) : samplingRate(rate) {}
  virtual ~IMU() {}
  bool debugEventTime = false;
  struct SensorData
  {
      float accelX, accelY, accelZ; // in g-force
      float gyroX, gyroY, gyroZ; // in deg/s^2
      float magX, magY, magZ; // in µT
      float yaw, pitch, roll; // in deg
      float yaw6, pitch6, roll6; // in deg
  };

  SensorData data;

  virtual bool initialize() = 0;
  virtual bool read() = 0;
  virtual void filterData() {
    // If either accel or gyro was updated, update both filters
    if (accelUpdated || gyroUpdated) {
      filter.update(
        data.gyroX, data.gyroY, data.gyroZ,
        data.accelX, data.accelY, data.accelZ,
        data.magX, data.magY, data.magZ
      );

      filter6.updateIMU(
        data.gyroX, data.gyroY, data.gyroZ,
        data.accelX, data.accelY, data.accelZ
      );
    }
    // If only magnetometer was updated, update only the 9-axis filter
    else if (magUpdated) {
      filter.update(
        data.gyroX, data.gyroY, data.gyroZ,
        data.accelX, data.accelY, data.accelZ,
        data.magX, data.magY, data.magZ
      );
    }

    data.yaw = filter.getYaw();
    data.pitch = filter.getPitch();
    data.roll = filter.getRoll();
    data.yaw6 = filter6.getYaw();
    data.pitch6 = filter6.getPitch();
    data.roll6 = filter6.getRoll();

    // Reset update flags
    accelUpdated = false;
    gyroUpdated = false;
    magUpdated = false;
  }
  virtual void getSensorData(SensorData* outData) {
    if (outData != nullptr) {
      *outData = data;
    }
  }

  virtual void printData() {
    Serial.printf("=========== %s\n ============\n", name);
    Serial.printf("Accel:\t%f\t%f\t%f\n", data.accelX, data.accelY, data.accelZ);
    Serial.printf("Gyro:\t%f\t%f\t%f\n", data.gyroX, data.gyroY, data.gyroZ);
    Serial.printf("Mag:\t%f\t%f\t%f\n", data.magX, data.magY, data.magZ);
    Serial.printf("Angles:\t%f\t%f\t%f\n", data.roll, data.pitch, data.yaw);
    Serial.printf("Angles6:\t%f\t%f\t%f\n", data.roll6, data.pitch6, data.yaw6);
  }
};

#endif
