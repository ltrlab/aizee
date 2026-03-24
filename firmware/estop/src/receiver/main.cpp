// E-Stop Receiver firmware
// Hardware: ESP-WROOM-32, connected to Jetson Orin Nano via USB
//
// Receives EStopMsg over ESP-NOW from the station and prints JSON lines
// to Serial for the Jetson bridge script to read.
//
// Only forwards actual received state — signal loss is NOT treated as e-stop.

#include <Arduino.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include "protocol.h"

// ── Station MAC — printed by station firmware on first boot ─────────────────
static const uint8_t STATION_MAC[6] = {0xB4, 0x3A, 0x45, 0xBE, 0x0F, 0xEC};

static const uint32_t LOOP_MS = 50;  // 20 Hz output rate

// ── Received state (updated from ISR context) ──────────────────────────────
static volatile bool     g_got_msg     = false;
static volatile uint32_t g_last_rx_ms  = 0;
static volatile EStopMsg g_last_msg    = {};

// ── Forwarded state (main loop) ────────────────────────────────────────────
static int8_t g_prev_estop = -1;  // -1 = no message received yet

static void onReceive(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
    if (len != sizeof(EStopMsg)) return;
    memcpy((void*)&g_last_msg, data, sizeof(EStopMsg));
    g_last_rx_ms = millis();
    g_got_msg = true;
}

void setup() {
    Serial.begin(115200);
    Serial.println("[ESTOP-RX] E-Stop Receiver");

    WiFi.mode(WIFI_STA);
    esp_wifi_set_channel(ESTOP_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
    Serial.printf("[ESTOP-RX] Receiver MAC: %s\n", WiFi.macAddress().c_str());
    Serial.println("[ESTOP-RX] ^^ Give this MAC to station firmware ^^");

    if (esp_now_init() != ESP_OK) {
        Serial.println("[ESTOP-RX] ESP-NOW init failed — halting");
        while (true) delay(1000);
    }
    esp_now_register_recv_cb(onReceive);

    // Register station as peer (not strictly required for RX-only, but good practice)
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, STATION_MAC, 6);
    peer.channel = ESTOP_ESPNOW_CHANNEL;
    peer.encrypt = false;
    esp_now_add_peer(&peer);

    Serial.println("[ESTOP-RX] Waiting for station...");
}

void loop() {
    uint32_t tick_start = millis();

    if (g_got_msg) {
        g_got_msg = false;

        // Copy volatile state
        EStopMsg msg;
        memcpy(&msg, (void*)&g_last_msg, sizeof(msg));
        uint32_t age_ms = millis() - g_last_rx_ms;

        // Only emit a JSON command line on state *change* (or first message)
        if (msg.estop_active != g_prev_estop) {
            Serial.printf("{\"estop\":%s}\n",
                          msg.estop_active ? "true" : "false");
            g_prev_estop = msg.estop_active;
        }

        // Always emit a diagnostic line (prefixed so bridge can ignore it)
        Serial.printf("# nc=%d no=%d seq=%u age=%u\n",
                      msg.nc_raw, msg.no_raw, msg.seq, age_ms);
    }

    uint32_t elapsed = millis() - tick_start;
    if (elapsed < LOOP_MS) delay(LOOP_MS - elapsed);
}
