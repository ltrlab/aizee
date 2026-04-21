// E-Stop Station firmware
// Hardware: AtomS3R (ESP32-S3) + TailBat
//
// Reads e-stop button contact on G8, broadcasts state over ESP-NOW.
//
// Wiring:
//   G8  → e-stop contact terminal
//   GND → e-stop common terminal
//
// Logic: G8 LOW (pressed) = E-STOP, G8 HIGH (released) = SAFE
// Wire break floats HIGH = safe (not fail-safe on wire break,
// but acceptable since signal loss is not treated as e-stop either).

#include <Arduino.h>
#include <M5Unified.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include "protocol.h"

// ── Pin assignment ──────────────────────────────────────────────────────────
#define PIN_ESTOP  GPIO_NUM_8

// ── Receiver MAC ────────────────────────────────────────────────────────────
static uint8_t RECEIVER_MAC[6] = {0x24, 0xDC, 0xC3, 0x99, 0x0E, 0x20};

static const uint32_t LOOP_MS = 50;  // 20 Hz

static uint32_t g_seq = 0;
static bool     g_last_send_ok = true;

// ── Display ─────────────────────────────────────────────────────────────────
static LGFX_Sprite canvas(&M5.Display);

static void onSend(const uint8_t*, esp_now_send_status_t status) {
    g_last_send_ok = (status == ESP_NOW_SEND_SUCCESS);
}

void setup() {
    M5.begin();
    Serial.begin(115200);
    Serial.println("[ESTOP-STA] E-Stop Station");

    pinMode(PIN_ESTOP, INPUT_PULLUP);

    // Display
    int w = M5.Display.width();
    int h = M5.Display.height();
    canvas.createSprite(w, h);
    canvas.setTextDatum(middle_center);

    // ESP-NOW
    WiFi.mode(WIFI_STA);
    esp_wifi_set_channel(ESTOP_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
    Serial.printf("[ESTOP-STA] Station MAC: %s\n", WiFi.macAddress().c_str());

    if (esp_now_init() != ESP_OK) {
        Serial.println("[ESTOP-STA] ESP-NOW init failed — halting");
        while (true) delay(1000);
    }
    esp_now_register_send_cb(onSend);

    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, RECEIVER_MAC, 6);
    peer.channel = ESTOP_ESPNOW_CHANNEL;
    peer.encrypt = false;
    esp_now_add_peer(&peer);

    Serial.println("[ESTOP-STA] Ready. ESTOP=G8 (LOW=stop, HIGH=safe)");
}

void loop() {
    M5.update();
    uint32_t tick_start = millis();

    // LOW = pressed = e-stop active
    uint8_t pin = digitalRead(PIN_ESTOP);
    bool estop = (pin == LOW);

    // ── Send over ESP-NOW ───────────────────────────────────────────────────
    EStopMsg msg = {};
    msg.estop_active = estop ? 1 : 0;
    msg.nc_raw = pin;
    msg.no_raw = 0;
    msg.seq = g_seq++;

    esp_now_send(RECEIVER_MAC, (const uint8_t*)&msg, sizeof(msg));

    // ── Update display ──────────────────────────────────────────────────────
    int w = canvas.width();
    int h = canvas.height();

    if (estop) {
        canvas.fillSprite(TFT_RED);
        canvas.setTextColor(TFT_WHITE);
        canvas.setTextSize(2);
        canvas.drawString("STOP", w / 2, h / 3);
    } else {
        canvas.fillSprite(TFT_DARKGREEN);
        canvas.setTextColor(TFT_WHITE);
        canvas.setTextSize(2);
        canvas.drawString("SAFE", w / 2, h / 3);
    }

    canvas.setTextSize(1);
    canvas.drawString(g_last_send_ok ? "TX OK" : "TX FAIL", w / 2, h - 8);
    canvas.pushSprite(0, 0);

    // ── Serial debug ────────────────────────────────────────────────────────
    Serial.printf("[ESTOP-STA] estop=%d pin=%d seq=%u tx=%s\n",
                  msg.estop_active, pin, msg.seq,
                  g_last_send_ok ? "ok" : "fail");

    uint32_t elapsed = millis() - tick_start;
    if (elapsed < LOOP_MS) delay(LOOP_MS - elapsed);
}
