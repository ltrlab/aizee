// ============================================================================
// Vertical Gantry Actuator (Minerva `lift` joint) — firmware
// ============================================================================
// Board : Elegoo Uno R3 (ATmega328P, 16 MHz)
// CAN   : Inland/Seeed-style MCP2515 shield (16 MHz crystal -> 1 Mbps OK)
// Enc   : AS5600 magnetic encoder on I2C (A4=SDA, A5=SCL), multi-turn tracked
// Motor : IKEA desk-leg 12 VDC column (non-backdrivable) via DROK L298 H-bridge
//
// It emulates a ROBSTRIDE motor on the AIZEE CAN chain (see include/protocol.h)
// so rust/motor_control drives it as an ordinary joint. We run our own position
// PID in encoder counts and ignore the MIT Kp/Kd/torque fields.
//
// Position pipeline:
//   AS5600 multi-turn counts  --(LIFT_RAD_PER_COUNT)-->  pseudo-radians (±4π)
//   which is what we report/accept over CAN. The meters<->rad mapping for the
//   policy lives in the Minerva command layer (a linear axis needs it anyway).
//
// Boot behaviour: STALL-HOME to the bottom hard stop (sets absolute zero), THEN
// start answering CAN. The AS5600 is single-turn absolute, so turn-count — and
// thus absolute height — is lost on every power cycle and must be re-homed.
// Enable the arm on the Jetson AFTER the leg's boot-home LED settles.
// ============================================================================

#include <Arduino.h>
#include <SPI.h>
#include <mcp_can.h>
#include <Wire.h>
#include "AS5600.h"
#include "protocol.h"

// ---------------------------------------------------------------------------
// Node identity — must match the `can_id` given to this joint in the Minerva
// hardware yaml (as `type: ROBSTRIDE02`). Pick an id unused by other motors.
// ---------------------------------------------------------------------------
static const uint8_t LIFT_CAN_ID = 0x11;   // 17 — Minerva joint index 16 (+1)

// ---------------------------------------------------------------------------
// Pins  (Uno + MCP2515 shield leaves A4/A5 for I2C and D3-D8 mostly free)
// ---------------------------------------------------------------------------
static const uint8_t PIN_CAN_CS = 9;   // Seeed-style shields = D9; some = D10
static const uint8_t PIN_ENA    = 3;   // L298 ENA — PWM speed (must be a PWM pin)
static const uint8_t PIN_IN1    = 4;   // L298 IN1 — direction
static const uint8_t PIN_IN2    = 7;   // L298 IN2 — direction

// ---------------------------------------------------------------------------
// Calibration & tuning  (CALIBRATE marks values to measure on the bench)
// ---------------------------------------------------------------------------
// CALIBRATE: pick so the full mechanical stroke maps to < 4π (12.566) rad with
// margin. Reported pos(rad) = counts * LIFT_RAD_PER_COUNT (4096 counts/rev).
static const float LIFT_RAD_PER_COUNT = 6.0e-5f;
// CALIBRATE: encoder counts at the TOP hard stop — firmware safety backstop.
static const long  LIFT_MAX_COUNTS    = 180000L;

static const uint8_t LIFT_INVERT = 0;  // set 1 if "up" drive DECREASES counts

// Position PID (P + light I). Output is PWM counts (0-255).
static const float KP_PWM_PER_COUNT = 0.02f;
static const int   DEADBAND_COUNTS  = 40;    // within this, coast (leg self-holds)
static const int   MIN_PWM          = 55;    // overcome stiction
static const int   MAX_PWM          = 220;   // cap speed (desk legs are slow)

// Stall homing
static const int          HOMING_PWM        = 110;   // drive toward bottom
static const long         STALL_EPS_COUNTS  = 8;     // "not moving" threshold
static const unsigned long STALL_WINDOW_MS  = 350;   // motionless this long = stop
static const unsigned long HOMING_TIMEOUT_MS = 30000;

// Command watchdog — mirror motor_control's 0.5 s watchdog_timeout.
static const unsigned long WATCHDOG_MS = 500;
// Feedback heartbeat: reply per control frame, plus this floor so telemetry
// stays fresh if control frames are sparse.
static const unsigned long FEEDBACK_MS = 20;

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------
MCP_CAN CAN(PIN_CAN_CS);
AS5600  encoder;

static bool  homed        = false;
static bool  enabled      = false;      // Robstride Run/Reset mirror
static long  cur_counts   = 0;          // AS5600 cumulative (0 = bottom stop)
static long  target_counts = 0;
static unsigned long last_cmd_ms      = 0;
static unsigned long last_feedback_ms = 0;

// ---------------------------------------------------------------------------
// Motor output — DROK L298 (ENA=PWM, IN1/IN2=direction). Positive pwm must
// raise the leg (increase counts); LIFT_INVERT flips it. Coast at pwm==0: the
// leg is non-backdrivable, so a coasting column holds its height with no current.
// ---------------------------------------------------------------------------
static void driveMotor(int pwm) {
    if (LIFT_INVERT) pwm = -pwm;
    if (pwm > 255) pwm = 255;
    if (pwm < -255) pwm = -255;

    if (pwm > 0) {            // up
        digitalWrite(PIN_IN1, HIGH);
        digitalWrite(PIN_IN2, LOW);
        analogWrite(PIN_ENA, pwm);
    } else if (pwm < 0) {     // down
        digitalWrite(PIN_IN1, LOW);
        digitalWrite(PIN_IN2, HIGH);
        analogWrite(PIN_ENA, -pwm);
    } else {                  // coast (holds mechanically)
        digitalWrite(PIN_IN1, LOW);
        digitalWrite(PIN_IN2, LOW);
        analogWrite(PIN_ENA, 0);
    }
}

// ---------------------------------------------------------------------------
// CAN feedback — emulate a Robstride telemetry frame at our current position.
// ---------------------------------------------------------------------------
static void sendFeedback(uint8_t mode) {
    uint16_t pos = rs_encode_pos((float)cur_counts * LIFT_RAD_PER_COUNT);
    uint8_t buf[8];
    buf[0] = pos >> 8;             buf[1] = pos & 0xFF;             // position
    buf[2] = RS_SIGNED_ZERO >> 8;  buf[3] = RS_SIGNED_ZERO & 0xFF;  // velocity ~0
    buf[4] = RS_SIGNED_ZERO >> 8;  buf[5] = RS_SIGNED_ZERO & 0xFF;  // torque   ~0
    uint16_t temp = 250;                                            // 25.0°C ×10
    buf[6] = temp >> 8;            buf[7] = temp & 0xFF;
    CAN.sendMsgBuf(rs_feedback_arb(LIFT_CAN_ID, mode, 0), 1 /*ext*/, 8, buf);
    last_feedback_ms = millis();
}

// ---------------------------------------------------------------------------
// Stall homing: drive down until the encoder stops changing, then zero.
// ---------------------------------------------------------------------------
static void homeStallToBottom() {
    homed = false;
    long last_seen   = encoder.getCumulativePosition();
    unsigned long t_still = millis();
    unsigned long t_start = millis();

    driveMotor(-HOMING_PWM);   // toward the bottom hard stop
    while (millis() - t_start < HOMING_TIMEOUT_MS) {
        long now = encoder.getCumulativePosition();
        if (labs(now - last_seen) > STALL_EPS_COUNTS) {
            last_seen = now;
            t_still = millis();          // still moving — reset the still-timer
        } else if (millis() - t_still > STALL_WINDOW_MS) {
            break;                        // motionless long enough → hard stop
        }
        delay(5);
    }
    driveMotor(0);
    delay(150);                           // let it settle against the stop
    encoder.resetCumulativePosition(0);   // this position is absolute zero
    cur_counts = 0;
    target_counts = 0;
    homed = true;
}

// ---------------------------------------------------------------------------
// Handle one received CAN frame if it's addressed to us.
// ---------------------------------------------------------------------------
static void handleCanFrame(uint32_t arb, uint8_t len, const uint8_t* data) {
    if (rs_cmd_motor(arb) != LIFT_CAN_ID) return;   // not ours

    switch (rs_msg_type(arb)) {
        case RS_MSG_ENABLE:
            enabled = true;
            last_cmd_ms = millis();
            sendFeedback(RS_MODE_RUN);   // the enable poll waits for this
            break;

        case RS_MSG_DISABLE:
            enabled = false;
            driveMotor(0);
            sendFeedback(RS_MODE_RESET);
            break;

        case RS_MSG_CONTROL: {
            last_cmd_ms = millis();
            if (len >= 2) {
                uint16_t pos_raw = ((uint16_t)data[0] << 8) | data[1];
                float pos_rad = rs_decode_pos(pos_raw);
                long t = (long)(pos_rad / LIFT_RAD_PER_COUNT);
                if (t < 0) t = 0;                          // 0 = bottom
                if (t > LIFT_MAX_COUNTS) t = LIFT_MAX_COUNTS;
                target_counts = t;
            }
            sendFeedback(enabled ? RS_MODE_RUN : RS_MODE_RESET);  // reply per control
            break;
        }

        case RS_MSG_ZEROPOS:
            // motor_control's mech_zero: re-home to re-establish absolute zero.
            driveMotor(0);
            homeStallToBottom();
            sendFeedback(enabled ? RS_MODE_RUN : RS_MODE_RESET);
            break;

        default:
            // ReadParam/WriteParam/SaveConfig/VBUS queries — not needed for the
            // enable handshake; ignore. (VBUS round-robin is fire-and-forget.)
            break;
    }
}

static void serviceCan() {
    uint32_t rxId;
    uint8_t  ext, len, buf[8];
    while (CAN.checkReceive() == CAN_MSGAVAIL) {
        if (CAN.readMsgBuf(&rxId, &ext, &len, buf) != CAN_OK) break;
        if (!ext) continue;                 // Robstride uses extended ids only
        handleCanFrame(rxId & 0x1FFFFFFF, len, buf);
    }
}

// ---------------------------------------------------------------------------
// Position control step (P-controller; leg is slow so P is plenty).
// ---------------------------------------------------------------------------
static void runControl() {
    long err = target_counts - cur_counts;
    if (labs(err) <= DEADBAND_COUNTS) { driveMotor(0); return; }

    int pwm = (int)(KP_PWM_PER_COUNT * (float)err);
    if (pwm > 0 && pwm < MIN_PWM)  pwm = MIN_PWM;
    if (pwm < 0 && pwm > -MIN_PWM) pwm = -MIN_PWM;
    if (pwm >  MAX_PWM) pwm =  MAX_PWM;
    if (pwm < -MAX_PWM) pwm = -MAX_PWM;
    driveMotor(pwm);
}

void setup() {
    Serial.begin(115200);

    pinMode(PIN_ENA, OUTPUT);
    pinMode(PIN_IN1, OUTPUT);
    pinMode(PIN_IN2, OUTPUT);
    driveMotor(0);

    Wire.begin();
    encoder.begin();
    if (!encoder.isConnected()) {
        Serial.println(F("[LIFT] AS5600 not found — check I2C wiring/magnet"));
    }

    // 1 Mbps to match the AIZEE CAN chain; 16 MHz crystal on the shield.
    while (CAN.begin(MCP_ANY, CAN_1000KBPS, MCP_16MHZ) != CAN_OK) {
        Serial.println(F("[LIFT] MCP2515 init failed — retrying"));
        delay(500);
    }
    CAN.setMode(MCP_NORMAL);
    Serial.print(F("[LIFT] CAN up, id=0x"));
    Serial.println(LIFT_CAN_ID, HEX);

    homeStallToBottom();   // establish absolute zero before answering enable
    Serial.println(F("[LIFT] homed — ready"));
    last_cmd_ms = millis();
}

void loop() {
    cur_counts = encoder.getCumulativePosition();  // call often: tracks wraps

    serviceCan();

    // Command watchdog: on dropout, stop at the current height and coast (the
    // non-backdrivable column holds). Stay "enabled" so we resume cleanly.
    if (enabled && (millis() - last_cmd_ms > WATCHDOG_MS)) {
        target_counts = cur_counts;
    }

    if (enabled && homed) runControl();
    else                  driveMotor(0);

    if (enabled && (millis() - last_feedback_ms > FEEDBACK_MS)) {
        sendFeedback(RS_MODE_RUN);
    }
}
