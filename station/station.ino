/**
 * @file station.ino
 * @author Austin Esquirell (ajesquirell@yahoo.com)
 * @brief Firmware for Traffic Light Main Station
 * @version 0.1
 * @date 2024-08-23
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "WiFi.h"
#include <esp_now.h>
#include <esp_mac.h>  // For the MAC2STR and MACSTR macros

#include "../definitions.h"

#define ESPNOW_WIFI_CHANNEL 6

// GPIO definitions
#define OUT_RED D2
#define OUT_YLW D1
#define OUT_GRN D0

uint8_t curr_outputs = 0;
uint8_t prev_outputs = 0;

payload_t tmpp;

bool led_tmp = false;

void OnDataRecv(const esp_now_recv_info_t * mac, const uint8_t *incomingData, int len) 
{
  digitalWrite(LED_BUILTIN, led_tmp);
  led_tmp = !led_tmp;

  if (len == sizeof(tmpp)) {
    payload_t* d = (payload_t*)incomingData;

    if (d->type == DEVICE_TYPE_REMOTE) {
      d->button_code & BUTTON_CODE_RED ? curr_outputs |= TRAFFIC_SIGNAL_RED : curr_outputs &= ~TRAFFIC_SIGNAL_RED;
      d->button_code & BUTTON_CODE_YLW ? curr_outputs |= TRAFFIC_SIGNAL_YELLOW : curr_outputs &= ~TRAFFIC_SIGNAL_YELLOW;
      d->button_code & BUTTON_CODE_GRN ? curr_outputs |= TRAFFIC_SIGNAL_GREEN : curr_outputs &= ~TRAFFIC_SIGNAL_GREEN;
    }
  }
}

void setup() {
  // GPIO - Set first so that light outputs turn off as soon as possible

  // TODO PWM - LEDC stuff
  pinMode(OUT_RED, OUTPUT);
  pinMode(OUT_YLW, OUTPUT);
  pinMode(OUT_GRN, OUTPUT);
  digitalWrite(OUT_RED, LOW);
  digitalWrite(OUT_YLW, LOW);
  digitalWrite(OUT_GRN, LOW);  

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  Serial.begin(115200);
  // while (!Serial) {
  //   delay(10);
  // }

  // Initialize the Wi-Fi module
  WiFi.mode(WIFI_STA);
  WiFi.setChannel(ESPNOW_WIFI_CHANNEL);
  while (!WiFi.STA.started()) {
    delay(100);
  }

  Serial.println("ESP-NOW Traffic Light");
  Serial.println("Wi-Fi parameters:");
  Serial.println("  Mode: STA");
  Serial.println("  MAC Address: " + WiFi.macAddress());
  Serial.printf("  Channel: %d\n", ESPNOW_WIFI_CHANNEL);

  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
   
  // Register callback function
  esp_now_register_recv_cb(OnDataRecv);
}



void broadcast(const payload_t &message)
{
  uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  esp_now_peer_info_t peerInfo = {};
  memcpy(&peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 6;
  if (!esp_now_is_peer_exist(broadcastAddress))
  {
    esp_now_add_peer(&peerInfo);
  }

  esp_err_t result = esp_now_send(broadcastAddress, (const uint8_t *)&message, sizeof(message));

  if (result == ESP_OK)
  {
    Serial.println("Broadcast message success");
  }
  else if (result == ESP_ERR_ESPNOW_NOT_INIT)
  {
    Serial.println("ESP-NOW not Init.");
  }
  else if (result == ESP_ERR_ESPNOW_ARG)
  {
    Serial.println("Invalid Argument");
  }
  else if (result == ESP_ERR_ESPNOW_INTERNAL)
  {
    Serial.println("Internal Error");
  }
  else if (result == ESP_ERR_ESPNOW_NO_MEM)
  {
    Serial.println("ESP_ERR_ESPNOW_NO_MEM");
  }
  else if (result == ESP_ERR_ESPNOW_NOT_FOUND)
  {
    Serial.println("Peer not found.");
  }
  else
  {
    Serial.println("Unknown error");
  }
}

void loop() {
  if (curr_outputs & TrafficSignalColor::TRAFFIC_SIGNAL_RED) {
    digitalWrite(OUT_RED, HIGH);
  } else {
    digitalWrite(OUT_RED, LOW);
  }

  if (curr_outputs & TrafficSignalColor::TRAFFIC_SIGNAL_YELLOW) {
    digitalWrite(OUT_YLW, HIGH);
  } else {
    digitalWrite(OUT_YLW, LOW);
  }

  if (curr_outputs & TrafficSignalColor::TRAFFIC_SIGNAL_GREEN) {
    digitalWrite(OUT_GRN, HIGH);
  } else {
    digitalWrite(OUT_GRN, LOW);
  }

  if (curr_outputs != prev_outputs) {
    payload_t p;
    p.type = DEVICE_TYPE_STATION;
    p.outputs = curr_outputs;
    broadcast(p);
  }

  prev_outputs = curr_outputs;

  delay(1);
}
