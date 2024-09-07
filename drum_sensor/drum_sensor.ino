/**
 * @file drum_sensor.ino
 * @author Austin Esquirell (ajesquirell@yahoo.com)
 * @brief Firmware for Traffic Light Bass Drum Sensor Module
 * @version 0.1
 * @date 2024-09-07
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "ESP32_NOW.h"
#include "WiFi.h"
#include <esp_mac.h>  // For the MAC2STR and MACSTR macros
#include "../broadcast_peer.h"
#include "../definitions.h"

// GPIO definitions
#define INPUT_SENSOR D0

bool triggered = false;

ESP_NOW_Broadcast_Peer broadcast_peer;
broadcast_payload_t broadcast_payload;

class ESP_NOW_Station_Peer : public ESP_NOW_Peer {
public:
  ESP_NOW_Station_Peer(const uint8_t *mac_addr) : ESP_NOW_Peer(mac_addr, ESPNOW_WIFI_CHANNEL, WIFI_IF_STA, NULL) {}

  ~ESP_NOW_Station_Peer() {
    remove();
  }

  bool addPeer() {
    if (!add()) {
      log_e("Failed to register the station peer " MACSTR, MAC2STR(addr()));
      return false;
    }
    return true;
  }

  bool sendPulse() {
    uint8_t dummy = 0;
    if (!send(&dummy, 1)) {
      log_e("Failed to send pulse message to station");
      return false;
    }
    return true;
  }

  void onReceive(const uint8_t *data, size_t len, bool broadcast) {
    log_v("Received a message from station " MACSTR " (%s)\n", MAC2STR(addr()), broadcast ? "broadcast" : "unicast");

    if (!broadcast) {
      log_v("Received non-broadcast message from station. Ignoring...");
      return;
    }
    
    // Known peer asking for connection. Broadcast back so it can connect to this mac addr.
    if (len == sizeof(broadcast_payload_t) && ((broadcast_payload_t*)data)->to == DEVICE_TYPE_DRUM_SENSOR) {
      broadcast_peer.sendMessage(broadcast_payload);
    }
  }
};

ESP_NOW_Station_Peer station_peer(NULL);

// Callback called when an unknown peer sends a message
void on_new_connection(const esp_now_recv_info_t *info, const uint8_t *data, int len, void *arg) {
  if (memcmp(info->des_addr, ESP_NOW.BROADCAST_ADDR, 6) != 0) {
    log_v("Received a unicast message from unknown peer " MACSTR, MAC2STR(info->src_addr));
    log_v("Igorning the message");
    return;
  }

  log_v("Unknown peer " MACSTR " sent a broadcast message\n", MAC2STR(info->src_addr));

  if (len != sizeof(broadcast_payload_t)) {
    log_e("Invalid broadcast payload len from unknown peer. Ignoring.");
    return;
  }

  broadcast_payload_t* payload = (broadcast_payload_t*)data;

  if (payload->from != DEVICE_TYPE_STATION) {
    log_v("Peer is not station. Ignoring...");
    return;
  }

  log_v("Registering the peer as station");
  station_peer = ESP_NOW_Station_Peer(info->src_addr);
  if (station_peer.addPeer()) {
    digitalWrite(LED_BUILTIN, HIGH); // OFF
  }
}

unsigned long now = 0;
unsigned long last = 0;
void IRAM_ATTR isr() {
  now = millis();
  if (now - last > 150) {
    triggered = true;
    last = now;
  }
}

void setup() {
  // Initialize the Wi-Fi module
  WiFi.enableLongRange(true);
  WiFi.mode(WIFI_STA);
  WiFi.setChannel(ESPNOW_WIFI_CHANNEL);
  while (!WiFi.STA.started()) {
    delay(100);
  }

  log_i("ESP-NOW Traffic Light Bass Drum Sensor\n");
  log_i("Wi-Fi parameters:\n");
  log_i("  Mode: STA\n");
  log_i("  MAC Address: " MACSTR "\n", MAC2STR(WiFi.macAddress()));
  log_i("  Channel: %d\n", ESPNOW_WIFI_CHANNEL);

  // Initialize the ESP-NOW protocol
  if (!ESP_NOW.begin()) {
    log_e("Failed to initialize ESP-NOW");
    log_e("Rebooting...");
    delay(200);
    ESP.restart();
  }

  ESP_NOW.onNewPeer(on_new_connection, NULL);

  // Register the broadcast peer
  if (!broadcast_peer.addPeer()) {
    log_e("Rebooting...");
    delay(200);
    ESP.restart();
  }

  // GPIO
  pinMode(INPUT_SENSOR, INPUT_PULLUP);
  attachInterrupt(INPUT_SENSOR, isr, FALLING);

  // Built-in LED used as connection status indicator
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  broadcast_payload.from = DEVICE_TYPE_DRUM_SENSOR;
  broadcast_payload.to = DEVICE_TYPE_STATION;
}

void loop() {
  // Broadcast myself to station until we connect
  if (!station_peer) {
    broadcast_peer.sendMessage(broadcast_payload);
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
    return;
  }

  if (triggered) {
    station_peer.sendPulse();
    triggered = false;
  }

  delay(50);
}

