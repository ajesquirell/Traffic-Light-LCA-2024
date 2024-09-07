#pragma once

#define ESPNOW_WIFI_CHANNEL 6

enum DeviceType
{
    DEVICE_TYPE_STATION,
    DEVICE_TYPE_REMOTE,
    DEVICE_TYPE_DRUM_SENSOR
};

enum ButtonCode
{
    BUTTON_CODE_RED = 0x01,
    BUTTON_CODE_YLW = 0x02,
    BUTTON_CODE_GRN = 0x04,
    BUTTON_CODE_BLU = 0x08,
    BUTTON_CODE_WHT = 0x10,
    BUTTON_CODE_BLK = 0x20
};

enum TrafficSignalColor
{
    STATION_OUTPUT_RED = 0x01,
    STATION_OUTPUT_YELLOW = 0x02,
    STATION_OUTPUT_GREEN = 0x04,
};

enum TrafficSignalMode
{
    MODE_REMOTE,
    MODE_DRUM_SENSOR
};

typedef struct
{
    DeviceType from;
    DeviceType to;
} __attribute__((packed)) broadcast_payload_t;

typedef struct
{
    uint8_t button_code;
} __attribute__((packed)) remote_payload_t;

typedef struct
{
    TrafficSignalMode mode;
    uint8_t outputs;
} __attribute__((packed)) main_station_payload_t;
