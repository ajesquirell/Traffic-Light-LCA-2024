/**
 * @file station.ino
 * @author Austin Esquirell (ajesquirell@yahoo.com)
 * @brief Firmware for Traffic Light Main Station
 * @version 0.2
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
#define OUT_RED D2
#define OUT_YLW D1
#define OUT_GRN D0

uint8_t curr_outputs = 0;
uint8_t prev_outputs = 0;

TrafficSignalMode curr_mode = MODE_REMOTE;
TrafficSignalMode prev_mode = MODE_REMOTE;

ESP_NOW_Broadcast_Peer broadcast_peer;
broadcast_payload_t broadcast_payload;

class ESP_NOW_Remote_Peer : public ESP_NOW_Peer {
public:
  ESP_NOW_Remote_Peer(const uint8_t *mac_addr) : ESP_NOW_Peer(mac_addr, ESPNOW_WIFI_CHANNEL, WIFI_IF_STA, NULL) {}

  ~ESP_NOW_Remote_Peer() {
    remove();
  }

  bool addPeer() {
    if (!add()) {
      log_e("Failed to register the remote peer " MACSTR, MAC2STR(addr()));
      return false;
    }
    return true;
  }

  bool sendState() {
    main_station_payload_t payload;
    payload.mode = curr_mode;
    payload.outputs = curr_outputs;

    if (!send((uint8_t *)&payload, sizeof(payload))) {
      log_e("Failed to send message to remote");
      return false;
    }
    return true;
  }

  void onReceive(const uint8_t *data, size_t len, bool broadcast) {
    log_v("Received a message from remote " MACSTR " (%s)\n", MAC2STR(addr()), broadcast ? "broadcast" : "unicast");
    if (broadcast) {
      // Known peer asking for connection. Broadcast back so it can connect to this mac addr.
      broadcast_payload.to = DEVICE_TYPE_REMOTE;
      broadcast_peer.sendMessage(broadcast_payload);
      sendState();
      return;
    }
    if (len < sizeof(remote_payload_t)) {
      log_e("Invalid message size received from remote");
      return;
    }

    remote_payload_t* payload = (remote_payload_t*)data;

    if (payload->button_code & BUTTON_CODE_BLU) {
      curr_mode = curr_mode == MODE_REMOTE ? MODE_DRUM_SENSOR : MODE_REMOTE;
    }

    if (curr_mode == MODE_REMOTE) {
      payload->button_code & BUTTON_CODE_RED ? curr_outputs |= STATION_OUTPUT_RED : curr_outputs &= ~STATION_OUTPUT_RED;
      payload->button_code & BUTTON_CODE_YLW ? curr_outputs |= STATION_OUTPUT_YELLOW : curr_outputs &= ~STATION_OUTPUT_YELLOW;
      payload->button_code & BUTTON_CODE_GRN ? curr_outputs |= STATION_OUTPUT_GREEN : curr_outputs &= ~STATION_OUTPUT_GREEN;
    }
  }
};

class ESP_NOW_Drum_Peer : public ESP_NOW_Peer {
public:
  ESP_NOW_Drum_Peer(const uint8_t *mac_addr) : ESP_NOW_Peer(mac_addr, ESPNOW_WIFI_CHANNEL, WIFI_IF_STA, NULL) {}

  ~ESP_NOW_Drum_Peer() {
    remove();
  }

  bool addPeer() {
    if (!add()) {
      log_e("Failed to register the bass drum sensor peer " MACSTR, MAC2STR(addr()));
      return false;
    }
    return true;
  }

  bool sendMessage(const main_station_payload_t& payload) {
    if (!send((uint8_t *)&payload, sizeof(payload))) {
      log_e("Failed to send message to bass drum sensor");
      return false;
    }
    return true;
  }

  void onReceive(const uint8_t *data, size_t len, bool broadcast) {
    log_v("Received a message from bass drum sensor " MACSTR " (%s)\n", MAC2STR(addr()), broadcast ? "broadcast" : "unicast");
    if (broadcast) {
      // Known peer asking for connection. Broadcast back so it can connect to this mac addr.
      broadcast_payload.to = DEVICE_TYPE_DRUM_SENSOR;
      broadcast_peer.sendMessage(broadcast_payload);
      return;
    }

    if (curr_mode != MODE_DRUM_SENSOR) {
      return;
    }
    
    if (curr_outputs & STATION_OUTPUT_RED) {
      curr_outputs = STATION_OUTPUT_YELLOW;
    } else if (curr_outputs & STATION_OUTPUT_YELLOW) {
      curr_outputs = STATION_OUTPUT_GREEN;
    } else {
      curr_outputs = STATION_OUTPUT_RED;
    }
  }
};

ESP_NOW_Remote_Peer remote_peer(NULL);
ESP_NOW_Drum_Peer drum_peer(NULL);

// Callback when an unknown peer sends a message
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

  switch (payload->from) {
    case DEVICE_TYPE_REMOTE: {
      log_i("Device type is REMOTE. Registering peer...");
      remote_peer = ESP_NOW_Remote_Peer(info->src_addr);
      remote_peer.addPeer();
      remote_peer.sendState();
      break;
    }
    case DEVICE_TYPE_DRUM_SENSOR: {
      log_i("Device type is BASS DRUM SENSOR. Registering peer...");
      drum_peer = ESP_NOW_Drum_Peer(info->src_addr);
      drum_peer.addPeer();
      break;
    }
  }

  if (remote_peer && drum_peer) {
    digitalWrite(LED_BUILTIN, HIGH); // OFF
  }
}

void setup() {
  // GPIO - Set first so that light outputs turn off as soon as possible
  pinMode(OUT_RED, OUTPUT);
  pinMode(OUT_YLW, OUTPUT);
  pinMode(OUT_GRN, OUTPUT);
  digitalWrite(OUT_RED, LOW);
  digitalWrite(OUT_YLW, LOW);
  digitalWrite(OUT_GRN, LOW);

  // Built-in LED used as connection status indicator
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  // Initialize the Wi-Fi module
  WiFi.enableLongRange(true);
  WiFi.mode(WIFI_STA);
  WiFi.setChannel(ESPNOW_WIFI_CHANNEL);
  while (!WiFi.STA.started()) {
    delay(100);
  }

  log_i("ESP-NOW Traffic Light Main Station\n");
  log_i("Wi-Fi parameters:\n");
  log_i("  Mode: STA\n");
  log_i("  MAC Address: " MACSTR "\n", MAC2STR(WiFi.macAddress()));
  log_i("  Channel: %d\n", ESPNOW_WIFI_CHANNEL);

  // Initialize the ESP-NOW protocol
  if (!ESP_NOW.begin()) {
    log_e("Failed to initialize ESP-NOW");
    log_e("Rebooting in 200 ms...");
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

  broadcast_payload.from = DEVICE_TYPE_STATION;
}

unsigned long broadcast_timer = 0;

void loop() {
  // Broadcast myself to network while peers are unknown
  if (millis() - broadcast_timer > 100) {
    bool no_connection = false;
    if (!remote_peer) {
      broadcast_payload.to = DEVICE_TYPE_REMOTE;
      broadcast_peer.sendMessage(broadcast_payload);
      no_connection = true;
    }
    if (!drum_peer) {
      broadcast_payload.to = DEVICE_TYPE_DRUM_SENSOR;
      broadcast_peer.sendMessage(broadcast_payload);
      no_connection = true;
    }
    if(no_connection) {
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
    broadcast_timer = millis();
  }

  digitalWrite(OUT_RED, curr_outputs & STATION_OUTPUT_RED);
  digitalWrite(OUT_YLW, curr_outputs & STATION_OUTPUT_YELLOW);
  digitalWrite(OUT_GRN, curr_outputs & STATION_OUTPUT_GREEN);

  if (curr_outputs != prev_outputs || curr_mode != prev_mode) {
    remote_peer.sendState();
  }

  prev_outputs = curr_outputs;
  prev_mode = curr_mode;

  delay(1);
}
