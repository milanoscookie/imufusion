#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_log.h>

class WiFiManager {
public:
    WiFiManager(const char* ssid) : ssid(ssid) {}

    void readMacAddress() {
        uint8_t baseMac[6];
        esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, baseMac);
        if (ret == ESP_OK) {
            Serial.printf("MAC Address: %02x:%02x:%02x:%02x:%02x:%02x\n",
                          baseMac[0], baseMac[1], baseMac[2],
                          baseMac[3], baseMac[4], baseMac[5]);
        } else {
            Serial.println("Failed to read MAC address");
        }
    }

    bool connect() {
        Serial.println("Initializing WiFi...");
        
        // Clean up any existing WiFi state
        WiFi.disconnect(true);
        WiFi.mode(WIFI_OFF);
        vTaskDelay(pdMS_TO_TICKS(1000));
        
        // Initialize WiFi in station mode
        WiFi.mode(WIFI_STA);
        vTaskDelay(pdMS_TO_TICKS(100));
        
        // Set WiFi power save mode to NONE
        WiFi.setSleep(false);
        
        // Print MAC address before connecting
        Serial.print("ESP32 MAC Address: ");
        readMacAddress();
        
        Serial.print("Connecting to SSID: ");
        Serial.println(ssid);

        // Start connection attempt
        WiFi.begin(ssid);

        // String homeSSID = "SuperAwesomeDreamHouseWiFi";
        // String homePswd = "smellyBBQ2024!";
        // WiFi.begin(homeSSID, homePswd);
        
        int retries = 20;
        while (WiFi.status() != WL_CONNECTED && retries > 0) {
            vTaskDelay(pdMS_TO_TICKS(500));
            Serial.print(".");
            retries--;
        }
        Serial.println();

        if (WiFi.status() == WL_CONNECTED) {
            Serial.println("\nWiFi Connected Successfully!");
            Serial.print("IP Address: ");
            Serial.println(WiFi.localIP());
            Serial.print("Subnet Mask: ");
            Serial.println(WiFi.subnetMask());
            Serial.print("Gateway IP: ");
            Serial.println(WiFi.gatewayIP());
            Serial.print("DNS IP: ");
            Serial.println(WiFi.dnsIP());
            Serial.print("Signal Strength (RSSI): ");
            Serial.print(WiFi.RSSI());
            Serial.println(" dBm");
            return true;
        } else {
            Serial.println("\nWiFi Connection Failed!");
            Serial.print("Status code: ");
            Serial.println(WiFi.status());
            switch (WiFi.status()) {
                case WL_IDLE_STATUS:
                    Serial.println("WiFi is in idle state");
                    break;
                case WL_NO_SSID_AVAIL:
                    Serial.println("SSID not found");
                    break;
                case WL_CONNECT_FAILED:
                    Serial.println("Connection failed");
                    break;
                case WL_CONNECTION_LOST:
                    Serial.println("Connection lost");
                    break;
                case WL_DISCONNECTED:
                    Serial.println("Disconnected");
                    break;
                default:
                    Serial.println("Unknown error");
            }
            
            // Clean up on failure
            WiFi.disconnect(true);
            WiFi.mode(WIFI_OFF);
            return false;
        }
    }

    ~WiFiManager() {
        // Clean up on destruction
        WiFi.disconnect(true);
        WiFi.mode(WIFI_OFF);
    }

private:
    const char* ssid;
};

#endif 