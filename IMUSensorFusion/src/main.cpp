#include <Arduino.h>
#include <Wire.h>
#include "LSM6DSOX_LIS3MDL_IMU.h"
#include "ICM20948_IMU.h"
#include "ISM330DHCX_LIS3MDL_IMU.h"
#include "SensorWebsocketClient.h"
#include "WiFiManager.h"

const char* ssid = "Device-Northwestern";

// Create IMU instances
LSM6DSOX_LIS3MDL_IMU imu1;
ICM20948_IMU imu2;
// ISM330DHCX_LIS3MDL_IMU imu3;

SensorWebSocketServer wsServer1(8080);
SensorWebSocketServer wsServer2(8081);
// SensorWebSocketServer wsServer3(8082);

WiFiManager wifiManager(ssid);

// Track sensor initialization status
bool imu1Initialized = false;
bool imu2Initialized = false;
// bool imu3Initialized = false;

// Add these at the top with other global variables
struct IMUStats {
    uint32_t lastUpdateTime;
    uint32_t updateCount;
    float averageFrequency;
    uint32_t minInterval;
    uint32_t maxInterval;
};

IMUStats imu1Stats = {0, 0, 0.0f, UINT32_MAX, 0};
IMUStats imu2Stats = {0, 0, 0.0f, UINT32_MAX, 0};

void printIMUStats(const char* name, IMUStats& stats) {
    if (stats.updateCount > 0) {
        Serial.printf("\n=== %s Statistics ===\n", name);
        Serial.printf("Update Count: %lu\n", stats.updateCount);
        Serial.printf("Average Frequency: %.2f Hz\n", stats.averageFrequency);
        Serial.printf("Min Interval: %lu ms\n", stats.minInterval);
        Serial.printf("Max Interval: %lu ms\n", stats.maxInterval);
    }
}

void updateIMUStats(IMUStats& stats) {
    uint32_t currentTime = millis();
    if (stats.lastUpdateTime > 0) {
        uint32_t interval = currentTime - stats.lastUpdateTime;
        stats.minInterval = min(stats.minInterval, interval);
        stats.maxInterval = max(stats.maxInterval, interval);
        stats.averageFrequency = 1000.0f / ((float)interval);
    }
    stats.lastUpdateTime = currentTime;
    stats.updateCount++;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("\n=== ESP32 IMU System Starting ===");
  Serial.println("Initializing I2C...");
  Wire.begin();
  Wire.setClock(4000000);  // Set I2C to 4MHz for faster communication
  delay(100);

  // Configure WiFi for low latency
  WiFi.setSleep(false);  // Disable WiFi sleep mode
  esp_wifi_set_ps(WIFI_PS_NONE);  // Disable power saving

  Serial.println("\n=== Connecting to WiFi ===");
  Serial.print("SSID: ");
  Serial.println(ssid);
  
  if (!wifiManager.connect()) {
    Serial.println("Failed to connect to WiFi. Retrying in 5 seconds...");
    delay(5000);
    if (!wifiManager.connect()) {
      Serial.println("Failed to connect to WiFi after retry. Please check:");
      Serial.println("1. SSID is correct");
      Serial.println("2. ESP32 is in range of the WiFi network");
      Serial.println("3. Network allows ESP32 connections");
      Serial.println("4. No firewall blocking the connection");
      Serial.println("\nContinuing without WiFi...");
    }
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n=== Network Information ===");
    Serial.printf("IP Address: %s\n", WiFi.localIP().toString().c_str());
    Serial.printf("Subnet Mask: %s\n", WiFi.subnetMask().toString().c_str());
    Serial.printf("Gateway IP: %s\n", WiFi.gatewayIP().toString().c_str());
    Serial.printf("DNS IP: %s\n", WiFi.dnsIP().toString().c_str());
    Serial.printf("Signal Strength (RSSI): %d dBm\n", WiFi.RSSI());
  }

  Serial.println("\n=== Initializing IMUs ===");
  // Initialize LSM6DSOX + LIS3MDL
  imu1Initialized = imu1.initialize();
  if (imu1Initialized) {
    Serial.println("[1] LSM6DSOX + LIS3MDL initialized successfully.");
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("Starting Webserver for IMU 1");
      wsServer1.begin();
      delay(100);
    }
  } else {
    Serial.println("[1] Failed to initialize LSM6DSOX + LIS3MDL.");
  }

  // Initialize ICM20948
  imu2Initialized = imu2.initialize();
  if (imu2Initialized) {
    Serial.println("[2] ICM20948 initialized successfully.");
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("Starting Webserver for IMU 2");
      wsServer2.begin();
      delay(100);
    }
  } else {
    Serial.println("[2] Failed to initialize ICM20948.");
  }

  Serial.println("\n=== Initialization Complete ===");
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWebSocket URLs:");
    Serial.printf("IMU 1: ws://%s:8080/ws\n", WiFi.localIP().toString().c_str());
    Serial.printf("IMU 2: ws://%s:8081/ws\n", WiFi.localIP().toString().c_str());
    Serial.println("\nTest the connection by visiting:");
    Serial.printf("http://%s:8080/status\n", WiFi.localIP().toString().c_str());
    Serial.printf("http://%s:8081/status\n", WiFi.localIP().toString().c_str());
  }
}

void loop() {
  static uint32_t lastStatsPrint = 0;
  static uint32_t lastUpdate = 0;
  uint32_t currentTime = millis();

  // Update WebSocket servers
  wsServer1.update();
  wsServer2.update();

  // Read sensors every 1ms
  if (currentTime - lastUpdate >= 1) {
    lastUpdate = currentTime;

    // --- IMU 1: LSM6DSOX + LIS3MDL ---
    if (imu1Initialized) {
      if (imu1.read()) {
        imu1.filterData();
        if (WiFi.status() == WL_CONNECTED) {
          wsServer1.broadcastSensorData(imu1.data);
        }
        updateIMUStats(imu1Stats);
      } else {
        Serial.println("[1] Failed to read from LSM6DSOX + LIS3MDL");
        imu1Initialized = false;
      }
    }

    // --- IMU 2: ICM20948 ---
    if (imu2Initialized) {
      if (imu2.read()) {
        imu2.filterData();
        if (WiFi.status() == WL_CONNECTED) {
          wsServer2.broadcastSensorData(imu2.data);
        }
        updateIMUStats(imu2Stats);
      } else {
        Serial.println("[2] Failed to read from ICM20948");
        imu2Initialized = false;
      }
    }
  }

  // Print statistics every 15 seconds
  if (currentTime - lastStatsPrint >= 15000) {
    printIMUStats("LSM6DSOX + LIS3MDL", imu1Stats);
    printIMUStats("ICM20948", imu2Stats);
    lastStatsPrint = currentTime;
  }

  // Small delay to prevent watchdog timer issues
  delay(1);
}