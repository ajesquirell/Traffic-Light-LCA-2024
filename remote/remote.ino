/**
 * @file remote.ino
 * @author Austin Esquirell (ajesquirell@yahoo.com)
 * @brief Firmware for Traffic Light Remote
 * @version 0.3
 * @date 2024-09-18
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
#define N_BUTTONS 6

#define BTN_RED D2
#define BTN_YLW D1
#define BTN_GRN D0
#define BTN_BLU D7
#define BTN_WHT D8
#define BTN_BLK D9

#define LED_RED D6
#define LED_YLW D5
#define LED_GRN D4
#define LED_BLU D3

uint8_t curr_buttons = 0;
uint8_t prev_buttons = 0;

const uint8_t buttons[N_BUTTONS] = {BTN_RED, BTN_YLW, BTN_GRN, BTN_BLU , BTN_WHT, BTN_BLK};
const ButtonCode button_code_map[N_BUTTONS] = {BUTTON_CODE_RED, BUTTON_CODE_YLW, BUTTON_CODE_GRN, BUTTON_CODE_BLU, BUTTON_CODE_WHT, BUTTON_CODE_BLK};

ESP_NOW_Broadcast_Peer broadcast_peer;

// Heartbeat and Deep Sleep
unsigned long last_seen_time = 0;
int heartbeat_cnt = 0;
const int heartbeat_interval = 30 * 1000; // 30 secs
const int heartbeat_timeout_ms = 1 * 60000; // 1 min
const int deep_sleep_timeout_ms = 5 * 60000; // 5 mins

bool send_connection_request(bool is_ack = false) {
  broadcast_payload_t payload;
  payload.from = DEVICE_TYPE_REMOTE;
  payload.to = DEVICE_TYPE_STATION;
  payload.is_ack = is_ack;
  return broadcast_peer.sendMessage(payload);
}

void enter_deep_sleep() {
  log_w("Not connected to station. Going into deep sleep...");
  delay(500);
  esp_deep_sleep_start();
}

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

  bool sendButtons() {
    remote_payload_t payload;
    payload.button_code = curr_buttons;
    if (!send((uint8_t *)&payload, sizeof(payload))) {
      log_e("Failed to send message to station");
      return false;
    }
    return true;
  }

  bool sendHeartbeat() {
    remote_payload_t payload;
    payload.button_code = BUTTON_CODE_HEARTBEAT;
    if (!send((uint8_t *)&payload, sizeof(payload))) {
      log_e("Failed to send heartbeat to station");
      return false;
    }
    return true;
  }

  virtual void onReceive(const uint8_t *data, size_t len, bool broadcast) override {
    log_v("Received %d bytes from station " MACSTR " (%s)\n", len, MAC2STR(addr()), broadcast ? "broadcast" : "unicast");

    last_seen_time = millis();

    if (broadcast) {
      // Known peer asking for connection. Broadcast back so it can connect to this mac addr.
      if (len == sizeof(broadcast_payload_t) && ((broadcast_payload_t*)data)->to == DEVICE_TYPE_REMOTE && !((broadcast_payload_t*)data)->is_ack) {
        send_connection_request(true);
      }
      return;
    }

    if (len < sizeof(main_station_payload_t)) {
      log_e("Invalid message size received from station");
      return;
    }

    main_station_payload_t* payload = (main_station_payload_t*)data;

    digitalWrite(LED_RED, payload->outputs & STATION_OUTPUT_RED);
    digitalWrite(LED_YLW, payload->outputs & STATION_OUTPUT_YELLOW);
    digitalWrite(LED_GRN, payload->outputs & STATION_OUTPUT_GREEN);

    digitalWrite(LED_BLU, payload->mode == MODE_DRUM_SENSOR);
  }

  virtual void onSent(bool success) override {
    log_v("Message transmission to peer " MACSTR " %s", MAC2STR(addr()), success ? "successful" : "failed");
    if (success) {
      last_seen_time = millis();
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
  station_peer.addPeer();
}

void setup() {
  // Initialize the Wi-Fi module
  WiFi.enableLongRange(true);
  WiFi.mode(WIFI_STA);
  WiFi.setChannel(ESPNOW_WIFI_CHANNEL);
  while (!WiFi.STA.started()) {
    delay(100);
  }

  WiFi.setSleep(false);
  WiFi.setTxPower(WIFI_POWER_21dBm);

  log_i("ESP-NOW Traffic Light Remote");
  log_i("Wi-Fi parameters:");
  log_i("  Mode: STA");
  log_i("  MAC Address: " MACSTR, MAC2STR(WiFi.macAddress()));
  log_i("  Channel: %d", ESPNOW_WIFI_CHANNEL);

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
  pinMode(BTN_RED, INPUT_PULLUP);
  pinMode(BTN_YLW, INPUT_PULLUP);
  pinMode(BTN_GRN, INPUT_PULLUP);
  pinMode(BTN_BLU, INPUT_PULLUP);
  pinMode(BTN_WHT, INPUT_PULLUP);
  pinMode(BTN_BLK, INPUT_PULLUP);

  pinMode(LED_RED, OUTPUT);
  pinMode(LED_YLW, OUTPUT);
  pinMode(LED_GRN, OUTPUT);
  pinMode(LED_BLU, OUTPUT);

  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_YLW, LOW);
  digitalWrite(LED_GRN, LOW);
  digitalWrite(LED_BLU, LOW);

  // Built-in LED used as connection status indicator
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  // Deep Sleep (only RED, YELLOW, and GREEN are RTC pins)
  esp_deep_sleep_enable_gpio_wakeup(BIT(BTN_RED) | BIT(BTN_YLW) | BIT(BTN_GRN), ESP_GPIO_WAKEUP_GPIO_LOW);

  int connect_cnt = 0;

  // Broadcast myself to station until we connect
  while (!station_peer && connect_cnt < 50) {
    send_connection_request();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    ++connect_cnt;
    delay(200);
  }

  if (!station_peer) {
    enter_deep_sleep();
  }

  digitalWrite(LED_BUILTIN, HIGH); // OFF
}

void loop() {
  unsigned long delta_seen = millis() - last_seen_time;

  if (delta_seen > heartbeat_timeout_ms) {
    if (delta_seen > heartbeat_timeout_ms + (heartbeat_interval * heartbeat_cnt)) {
      station_peer.sendHeartbeat();
      ++heartbeat_cnt;
    }
  } else {
    heartbeat_cnt = 0;
  }

  // Check if connected to station
  if (delta_seen > deep_sleep_timeout_ms) {
    enter_deep_sleep();
  }

  for (int i = 0; i < N_BUTTONS; i++) {
    if (digitalRead(buttons[i]) == LOW) {
      curr_buttons |= button_code_map[i];
    } else {
      curr_buttons &= ~button_code_map[i];
    }
  }

  if (curr_buttons != prev_buttons && curr_buttons) {
    station_peer.sendButtons();
  }

  prev_buttons = curr_buttons;

  delay(1);
}
