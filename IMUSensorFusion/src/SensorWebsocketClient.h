#ifndef SENSOR_WEBSOCKET_CLIENT_H
#define SENSOR_WEBSOCKET_CLIENT_H
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include "IMU.h"

// WebSocket server class
class SensorWebSocketServer {
public:
    SensorWebSocketServer(uint16_t port = 8080)
        : server(port), ws("/ws"), port(port), lastHeartbeat(0), lastDataSent(0) {
        Serial.printf("Creating WebSocket server on port %d\n", port);
    }
    bool started = false;

    void begin() {
        started = true;
        Serial.printf("Starting WebSocket server on port %d\n", port);
        
        // Attach WebSocket handler with improved error handling
        ws.onEvent([this](AsyncWebSocket* server, AsyncWebSocketClient* client,
                          AwsEventType type, void* arg, uint8_t* data, size_t len) {
            switch (type) {
                case WS_EVT_CONNECT:
                    Serial.printf("Client %u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
                    // Set client-specific settings
                    client->text("{\"type\":\"connected\"}");
                    sendHeartbeat(client);
                    break;
                case WS_EVT_DISCONNECT:
                    Serial.printf("Client %u disconnected\n", client->id());
                    break;
                case WS_EVT_ERROR:
                    Serial.printf("WebSocket error from client %u: %s\n", client->id(), (char*)data);
                    break;
                case WS_EVT_PONG:
                    // Only log pong if there was an issue
                    if (client->queueIsFull()) {
                        Serial.printf("Received pong from client %u after queue was full\n", client->id());
                    }
                    break;
                case WS_EVT_DATA:
                    // Handle incoming data
                    handleWebSocketMessage(client, data, len);
                    break;
            }
        });

        server.addHandler(&ws);

        // Simple root endpoint
        server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
            request->send(200, "text/plain", "ESP32 WebSocket server is running");
        });

        // Add a status endpoint with more detailed information
        server.on("/status", HTTP_GET, [this](AsyncWebServerRequest* request) {
            JsonDocument doc;  // Status info needs about 200 bytes
            doc["status"] = "running";
            doc["clients"] = ws.count();
            doc["port"] = port;
            doc["uptime"] = millis() / 1000;
            doc["last_heartbeat"] = lastHeartbeat;
            doc["last_data_sent"] = lastDataSent;
            
            String response;
            serializeJson(doc, response);
            request->send(200, "application/json", response);
        });

        server.begin();
        Serial.printf("HTTP/WebSocket server started on port %d\n", port);
    }

    void update() {
        // Send heartbeat to all clients every 5 seconds
        uint32_t currentTime = millis();
        if (currentTime - lastHeartbeat >= 5000) {
            for (auto& client : ws.getClients()) {
                if (client.status() == WS_CONNECTED) {
                    // If queue is full, try to process some messages first
                    if (client.queueIsFull()) {
                        Serial.printf("Client %u queue is full, processing messages...\n", client.id());
                        // Give some time for messages to be processed
                        delay(1);
                    }
                    sendHeartbeat(&client);
                }
            }
            lastHeartbeat = currentTime;
        }
    }

    bool queueIsFull() {
        // Check if any client's queue is full
        for (auto& client : ws.getClients()) {
            if (client.queueIsFull()) {
                return true;
            }
        }
        return false;
    }

    uint32_t id() {
        return port;  // Use port as identifier
    }

    void broadcastSensorData(const IMU::SensorData& data) {
        if (ws.count() == 0) {
            return;  // No clients connected
        }

        // Rate limit data sending to prevent queue overflow
        uint32_t currentTime = millis();
        if (currentTime - lastDataSent < 10) {  // Minimum 10ms between sends
            return;
        }
        lastDataSent = currentTime;

        // Use a JSON document with optimized size
        JsonDocument doc;  // Sensor data needs about 350 bytes
        
        // Add timestamp to data
        doc["t"] = currentTime;
        
        // Optimize JSON structure for faster serialization
        JsonObject accel = doc["a"].to<JsonObject>();
        accel["x"] = data.accelX;
        accel["y"] = data.accelY;
        accel["z"] = data.accelZ;

        JsonObject gyro = doc["g"].to<JsonObject>();
        gyro["x"] = data.gyroX;
        gyro["y"] = data.gyroY;
        gyro["z"] = data.gyroZ;

        JsonObject mag = doc["m"].to<JsonObject>();
        mag["x"] = data.magX;
        mag["y"] = data.magY;
        mag["z"] = data.magZ;

        JsonObject angles = doc["e"].to<JsonObject>();
        angles["y"] = data.yaw;
        angles["p"] = data.pitch;
        angles["r"] = data.roll;

        JsonObject angles6 = doc["e6"].to<JsonObject>();
        angles6["y"] = data.yaw6;
        angles6["p"] = data.pitch6;
        angles6["r"] = data.roll6;

        String jsonStr;
        serializeJson(doc, jsonStr);

        // Check each client's queue before sending
        for (auto& client : ws.getClients()) {
            if (client.status() == WS_CONNECTED) {
                if (client.queueIsFull()) {
                    // If queue is full, try to process some messages first
                    Serial.printf("Client %u queue is full, processing messages...\n", client.id());
                    // Give more time for messages to be processed
                    delay(2);
                    // If still full, skip this client
                    if (client.queueIsFull()) {
                        continue;
                    }
                }
                // Try to send the message
                client.text(jsonStr);
            }
        }
    }

private:
    AsyncWebServer server;
    AsyncWebSocket ws;
    uint16_t port;
    uint32_t lastHeartbeat;
    uint32_t lastDataSent;  // Track last data send time

    void sendHeartbeat(AsyncWebSocketClient* client) {
        // If queue is full, try to process some messages first
        if (client->queueIsFull()) {
            Serial.printf("Client %u queue is full, processing messages...\n", client->id());
            // Give more time for messages to be processed
            delay(2);
            // If still full, skip this heartbeat
            if (client->queueIsFull()) {
                return;
            }
        }

        JsonDocument doc;  // Heartbeat message needs about 100 bytes
        doc["type"] = "heartbeat";
        doc["timestamp"] = millis();
        
        String jsonStr;
        serializeJson(doc, jsonStr);
        client->text(jsonStr);
    }

    void handleWebSocketMessage(AsyncWebSocketClient* client, uint8_t* data, size_t len) {
        // Handle incoming messages from clients
        String message = String((char*)data);
        JsonDocument doc;  // Client messages are small, about 100 bytes
        DeserializationError error = deserializeJson(doc, message);

        if (error) {
            Serial.printf("Failed to parse message from client %u: %s\n", client->id(), error.c_str());
            return;
        }

        // Handle different message types
        if (doc["type"].is<const char*>()) {
            String type = doc["type"];
            if (type == "heartbeat") {
                // If queue is full, try to process some messages first
                if (client->queueIsFull()) {
                    Serial.printf("Client %u queue is full, processing messages...\n", client->id());
                    // Give more time for messages to be processed
                    delay(2);
                    // If still full, skip this response
                    if (client->queueIsFull()) {
                        return;
                    }
                }
                // Client acknowledged heartbeat
                JsonDocument response;  // Response is very small, about 50 bytes
                response["type"] = "heartbeat_ack";
                String jsonStr;
                serializeJson(response, jsonStr);
                client->text(jsonStr);
            }
        }
    }
};
#endif