#pragma once

#include "ESP32_NOW.h"
#include "definitions.h"

class ESP_NOW_Broadcast_Peer : public ESP_NOW_Peer {
public:
  ESP_NOW_Broadcast_Peer() : ESP_NOW_Peer(ESP_NOW.BROADCAST_ADDR, ESPNOW_WIFI_CHANNEL, WIFI_IF_STA, NULL) {}

  ~ESP_NOW_Broadcast_Peer() {
    remove();
  }

  bool addPeer() {
    if (!add()) {
      log_e("Failed to register broadcast peer");
      return false;
    }
    return true;
  }

  bool sendMessage(const broadcast_payload_t& data) {
    if (!send((uint8_t *)&data, sizeof(data))) {
      log_e("Failed to broadcast message");
      return false;
    }
    return true;
  }
};