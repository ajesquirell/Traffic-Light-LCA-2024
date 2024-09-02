#pragma once

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
    TRAFFIC_SIGNAL_RED = 0x01,
    TRAFFIC_SIGNAL_YELLOW = 0x02,
    TRAFFIC_SIGNAL_GREEN = 0x04,
};

typedef struct
{
    DeviceType type;
    unsigned char outputs;
    unsigned char button_code;
} __attribute__((packed)) payload_t;
