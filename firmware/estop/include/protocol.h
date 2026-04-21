#pragma once
#include <stdint.h>

// ESP-NOW channel — must match on both station and receiver.
static const uint8_t ESTOP_ESPNOW_CHANNEL = 1;

// Station → Receiver at ~20 Hz.
struct __attribute__((packed)) EStopMsg {
    uint8_t  estop_active;  // 1 = STOP, 0 = clear
    uint8_t  nc_raw;        // raw NC pin reading (diagnostics)
    uint8_t  no_raw;        // raw NO pin reading (diagnostics)
    uint32_t seq;           // rolling counter
};
