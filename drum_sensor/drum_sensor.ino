/**
 * @file drum_sensor.ino
 * @author Austin Esquirell (ajesquirell@yahoo.com)
 * @brief Firmware for Traffic Light Bass Drum Sensor Module
 * @version 0.2
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
#define INPUT_SENSOR D0
#define LED_AUX D7

ESP_NOW_Broadcast_Peer broadcast_peer;

// Trigger interrupt
// Originally implemented using a binary semaphore approach, but RTOS recommends using direct notifications instead as it is faster and more memory efficient:
// https://www.freertos.org/Documentation/02-Kernel/02-Kernel-features/03-Direct-to-task-notifications/02-As-binary-semaphore

// Task to be notified by trigger
static TaskHandle_t taskHandle = NULL;
unsigned long now = 0, last = 0;
void IRAM_ATTR onTrigger() {
  if (taskHandle == NULL) return;
  now = millis();
  if (now - last > 150) {
    // Notify loop
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(taskHandle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

    last = now;
  }
}

bool send_connection_request(bool is_ack = false) {
  broadcast_payload_t payload;
  payload.from = DEVICE_TYPE_DRUM_SENSOR;
  payload.to = DEVICE_TYPE_STATION;
  payload.is_ack = is_ack;
  return broadcast_peer.sendMessage(payload);
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

  bool sendPulse() {
    uint8_t dummy = 0;
    if (!send(&dummy, 1)) {
      log_e("Failed to send pulse message to station");
      return false;
    }
    return true;
  }

  virtual void onReceive(const uint8_t *data, size_t len, bool broadcast) {
    log_v("Received %d bytes from station " MACSTR " (%s)\n", len, MAC2STR(addr()), broadcast ? "broadcast" : "unicast");

    if (!broadcast) {
      log_v("Received non-broadcast message from station. Ignoring...");
      return;
    }
    
    // Known peer asking for connection. Broadcast back so it can connect to this mac addr.
    if (len == sizeof(broadcast_payload_t) && ((broadcast_payload_t*)data)->to == DEVICE_TYPE_DRUM_SENSOR && !((broadcast_payload_t*)data)->is_ack) {
      send_connection_request(true);
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

  log_i("ESP-NOW Traffic Light Bass Drum Sensor");
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
  pinMode(INPUT_SENSOR, INPUT_PULLUP);
  attachInterrupt(INPUT_SENSOR, onTrigger, FALLING);

  // Built-in LED used as connection status indicator
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  pinMode(LED_AUX, OUTPUT);
  digitalWrite(LED_AUX, LOW);

  // Handle of curent task, so we can be informed when triggered
  taskHandle = xTaskGetCurrentTaskHandle();

  // Broadcast myself to station until we connect
  while (!station_peer) {
    send_connection_request();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(200);
  }

  digitalWrite(LED_BUILTIN, LOW); // Keep on once connected to show power state
}

void loop() {
  // pdTRUE: acting like binary semaphore
  // portMAX_DELAY will block until ready for less CPU and battery usage
  if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY)) {
    station_peer.sendPulse();
    digitalWrite(LED_AUX, HIGH);
    delay(50);
    digitalWrite(LED_AUX, LOW);
  }
}

