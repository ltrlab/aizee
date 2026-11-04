#pragma once

#include <stdint.h>

// The lift's USB serial link, 1 Mbaud, 8N1. The host asks and the board answers; the board never speaks first.
//
// Frame: A5 5A | type | length | payload | CRC-16
// The CRC is CRC-16/CCITT-FALSE over type, length and payload. Multi-byte fields are little-endian.
// A frame with a bad CRC gets no reply.
//
// Every command's payload starts with a u16 sequence number, which its reply echoes:
//   IDENTIFY  seq                     -> IDENTITY
//   STATE     seq                     -> STATE
//   ENABLE    seq                     -> STATE    Takes the setpoint to where the column is. Moves nothing.
//   DISABLE   seq                     -> STATE    Coasts now. Never refused.
//   MOVE      seq, i32 target, i32 cps -> STATE   Counts above the bottom switch, and the speed the target moves at.
//   HOME      seq                     -> STATE    Finds the bottom switch and zeroes on it. Only ever on request.
//   CLEAR     seq                     -> STATE    Clears a latched fault, only while disabled.
//
// IDENTITY: seq, "LIFT", u16 protocol, u16 firmware, u16 config CRC, i32 highest, i32 lowest, i32 max cps, u8 measured
// STATE: seq, u32 board ms, i32 counts, i32 cps, i16 mA, u16 mV, i16 pwm, u16 flags, u8 mode, u8 fault, u8 reject,
//        u8 duty %, u8 link errors, u8 late ticks
//
// The two counters saturate at 255. STATE's reject says whether this frame's command was refused.
//
// A MOVE outside lowest..highest is refused and does not count as hearing from the host. Homing stops inside the
// deadband of lowest, so possibly just below it: clamp a "stay here" target into the range, or it times out.
namespace protocol {

const uint8_t sync0 = 0xA5;
const uint8_t sync1 = 0x5A;
const uint16_t version = 1;
const uint16_t firmwareVersion = 1;
const uint8_t maxPayload = 32;
const uint8_t frameOverhead = 6;
const uint8_t identityLength = 25;
const uint8_t stateLength = 26;
const uint8_t maxReply = frameOverhead + identityLength;

enum class Command : uint8_t {
    identify = 0x01,
    state = 0x02,
    enable = 0x03,
    disable = 0x04,
    move = 0x05,
    home = 0x06,
    clear = 0x07,
};

enum class Reply : uint8_t {
    identity = 0x81,
    state = 0x82,
};

const uint16_t flagHomed = 1u << 0;          // Counts are measured from the bottom switch.
const uint16_t flagEncoderOk = 1u << 1;      // The AS5600 answers and sees its magnet.
const uint16_t flagMagnetMarginal = 1u << 2; // The AS5600 calls the magnet too weak or too strong.
const uint16_t flagSensorOk = 1u << 3;       // The INA219 answers.
const uint16_t flagAtTop = 1u << 4;          // Top switch open, or its wire broken.
const uint16_t flagAtBottom = 1u << 5;       // Bottom switch open, or its wire broken.
const uint16_t flagMeasured = 1u << 6;       // config.h holds a measured column.
const uint16_t flagAtRest = 1u << 7;         // Nothing is being asked of the motor.

} // namespace protocol

enum class Mode : uint8_t {
    disabled = 0,
    enabled = 1,
    homing = 2,
    jog = 3, // lift_bench only.
};

// Every fault coasts the motor at once and latches until CLEAR.
enum class Fault : uint8_t {
    none = 0,
    overcurrent = 1,
    stall = 2,          // Driven hard enough to turn, and not turning.
    wrongDirection = 3, // Driven one way, moving the other: a swapped motor or encoder.
    encoder = 4,        // The AS5600 stopped answering, or its angle jumped. Homing is lost.
    magnet = 5,         // The AS5600 lost its magnet. Homing is lost.
    currentSensor = 6,  // The INA219 stopped answering.
    limitTop = 7,
    limitBottom = 8,
    switchWiring = 9, // Both switches open at once: most likely a broken ground.
    undervoltage = 10,
    noSupply = 11,          // Nothing on the 12 V line: the kill switch, or the supply.
    duty = 12,              // More running than the motor's rating allows.
    commandTimeout = 13,    // The host went quiet.
    homeTimeout = 14,       // Homing took longer than the whole travel should.
    uncommandedMotion = 15, // Moving with the motor off. The column cannot back-drive, so this is a stuck bridge or
                            // a mechanical failure.
};

enum class Reject : uint8_t {
    none = 0,
    badCommand = 1,
    unmeasured = 2, // config.h still has zeros in it.
    faulted = 3,    // A fault is latched. CLEAR first.
    notHomed = 4,
    notEnabled = 5,
    busy = 6,       // Homing, or already enabled.
    outOfRange = 7, // The target or speed is outside what config.h allows.
    sensors = 8,    // The encoder or the INA219 is not answering.
    noSupply = 9,   // The 12 V line is low.
    switches = 10,  // A limit switch forbids it, or both are open.
    cooling = 11,   // The motor is still over its duty allowance.
};
