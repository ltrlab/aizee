#pragma once
#include <stdint.h>
#include <math.h>

// ============================================================================
// ROBSTRIDE emulation — wire protocol
// ============================================================================
// The Vertical Gantry Actuator (Minerva's `lift` joint) is an IKEA desk-leg
// column: a 12 VDC, non-backdrivable motor with an AS5600 magnetic encoder,
// driven by a DROK L298 H-bridge from an Arduino Uno + MCP2515 CAN shield.
//
// Rather than add a new actuator type to the (Robstride-only, safety-critical)
// Rust motor_control, this firmware *impersonates a ROBSTRIDE motor* on the CAN
// chain. motor_control then drives the lift as an ordinary joint — same enable
// handshake, control frames, telemetry, watchdog and e-stop — with no Rust
// changes. The leg simply runs its own position PID and ignores the MIT
// Kp/Kd/torque fields it can't honor.
//
// Frame formats mirror rust/motor_control/src/robstride.rs exactly:
//   - Extended 29-bit CAN IDs.
//   - Command arb id: (msg_type << 24) | (... << 8) | motor_id   (motor_id low byte)
//   - Feedback arb id: (2 << 24) | (mode << 22) | (err << 16) | (motor_id << 8) | host_id
//   - Feedback data: pos_u16 | vel_u16 | torque_u16 | temp_u16   (all big-endian)
// ============================================================================

// Host controller id Robstride replies address (robstride.rs: HOST_CAN_ID).
static const uint8_t RS_HOST_ID = 0xAA;

// Message types (robstride.rs: MotorMsg). Only the ones we act on are listed.
enum RsMsg : uint8_t {
    RS_MSG_CONTROL  = 1,   // host -> motor: MIT position/vel/kp/kd/torque
    RS_MSG_FEEDBACK = 2,   // motor -> host: position/vel/torque/temp + mode/err
    RS_MSG_ENABLE   = 3,   // host -> motor: enter Run mode
    RS_MSG_DISABLE  = 4,   // host -> motor: back to Reset
    RS_MSG_ZEROPOS  = 6,   // host -> motor: set mechanical zero (we re-home)
};

// Feedback "mode" bits (robstride.rs: MotorMode). motor_control treats a
// Run->Reset transition on an active motor as a hardware fault, so while
// enabled we must consistently report RUN.
enum RsMode : uint8_t {
    RS_MODE_RESET = 0,
    RS_MODE_RUN   = 2,
};

// MIT position full-scale is ±4π for every Robstride model (robstride.rs
// parse_feedback / build_control_frame both hardcode 4π), so position encoding
// is model-independent.
static const float RS_POS_MAX = 4.0f * (float)M_PI;   // 12.566371

// Encode a position (rad, clamped to ±4π) into the 16-bit feedback field.
static inline uint16_t rs_encode_pos(float pos_rad) {
    if (pos_rad >  RS_POS_MAX) pos_rad =  RS_POS_MAX;
    if (pos_rad < -RS_POS_MAX) pos_rad = -RS_POS_MAX;
    float f = (pos_rad + RS_POS_MAX) / (2.0f * RS_POS_MAX);  // 0..1
    return (uint16_t)(f * 65535.0f + 0.5f);
}

// Decode a commanded position (16-bit control field) back to rad. We use the
// same symmetric scale for encode and decode so a commanded target round-trips
// cleanly through our own feedback (motor_control's control-side scale differs
// by <1 LSB, which is negligible here).
static inline float rs_decode_pos(uint16_t raw) {
    return ((float)raw / 65535.0f) * (2.0f * RS_POS_MAX) - RS_POS_MAX;
}

// Midpoint code = "zero" for the signed velocity/torque fields.
static const uint16_t RS_SIGNED_ZERO = 0x7FFF;

// Build the extended feedback arbitration id for this node.
static inline uint32_t rs_feedback_arb(uint8_t motor_id, uint8_t mode, uint8_t err_bits) {
    return ((uint32_t)RS_MSG_FEEDBACK << 24)
         | ((uint32_t)(mode & 0x03) << 22)
         | ((uint32_t)(err_bits & 0x1F) << 16)
         | ((uint32_t)motor_id << 8)
         | (uint32_t)RS_HOST_ID;
}

// Extract the message type and addressed motor id from a received command frame
// (command arb ids carry motor_id in the low byte — robstride.rs build_arb_id
// and build_control_frame both do `| motor_id`).
static inline uint8_t rs_msg_type(uint32_t arb) { return (uint8_t)((arb >> 24) & 0x1F); }
static inline uint8_t rs_cmd_motor(uint32_t arb) { return (uint8_t)(arb & 0xFF); }
