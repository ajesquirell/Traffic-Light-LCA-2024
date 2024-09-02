/**
 * @file remote.ino
 * @author Austin Esquirell (ajesquirell@yahoo.com)
 * @brief Firmware for Traffic Light Remote
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
#define N_BUTTONS 6
#define N_LEDS 4

#define BTN_RED D0
#define BTN_YLW D1
#define BTN_GRN D2
#define BTN_BLU D7
#define BTN_WHT D8
#define BTN_BLK D9

#define LED_RED D3
#define LED_YLW D4
#define LED_GRN D5
#define LED_BLU D6

payload_t tmpp;

uint8_t buttons[N_BUTTONS] = {BTN_RED, BTN_YLW, BTN_GRN, BTN_BLU , BTN_WHT, BTN_BLK};
ButtonCode button_code_map[N_BUTTONS] = {BUTTON_CODE_RED, BUTTON_CODE_YLW, BUTTON_CODE_GRN, BUTTON_CODE_BLU, BUTTON_CODE_WHT, BUTTON_CODE_BLK};

bool led_blink_state = false;

void OnDataRecv(const esp_now_recv_info_t * mac, const uint8_t *incomingData, int len) 
{
  if (len == sizeof(tmpp)) {
    memcpy(&tmpp, incomingData, sizeof(tmpp));
    if (tmpp.type == DEVICE_TYPE_STATION) {
      // TODO leds
    }
  }
}

void setup() {
  // Initialize the Wi-Fi module
  WiFi.mode(WIFI_STA);
  WiFi.setChannel(ESPNOW_WIFI_CHANNEL);
  while (!WiFi.STA.started()) {
    delay(100);
  }

  Serial.begin(115200);

  Serial.println("ESP-NOW Traffic Light Remote");
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

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
}



void broadcast(const payload_t &message)
{
  uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  esp_now_peer_info_t peerInfo = {};
  peerInfo.channel = 6;
  memcpy(&peerInfo.peer_addr, broadcastAddress, 6);
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

uint8_t curr_buttons = 0;
uint8_t prev_buttons = 0;

void loop() {  
  for (int i = 0; i < N_BUTTONS; i++) {
    if (digitalRead(buttons[i]) == LOW) {
      curr_buttons |= button_code_map[i];
    } else {
      curr_buttons &= ~button_code_map[i];
    }
  }

  if (curr_buttons && curr_buttons != prev_buttons) {
    payload_t p;
    p.type = DEVICE_TYPE_REMOTE;
    p.button_code = curr_buttons;
    broadcast(p);
    digitalWrite(LED_BUILTIN, !led_blink_state);
    led_blink_state = !led_blink_state;
  }

  prev_buttons = curr_buttons;

  delay(1);
}
