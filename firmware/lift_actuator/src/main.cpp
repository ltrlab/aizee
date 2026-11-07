// Minerva's lift: USB serial to the receiver, a 1 kHz tick to the motor. lib/lift_core holds everything but the wires.
#ifndef LIFT_BENCH

#include <Arduino.h>

#include "board.h"
#include "config.h"
#include "lift.h"
#include "link.h"

namespace {

Lift lift(liftConfig);
FrameParser parser;

// A limit per pass, so a burst of frames cannot hold up the tick.
const uint8_t maxBytesPerLoop = 32;

} // namespace

void setup() {
    Serial.begin(1000000);
    board::begin(liftConfig);
}

void loop() {
    for (uint8_t i = 0; i < maxBytesPerLoop && Serial.available() > 0; i++) {
        if (parser.push((uint8_t)Serial.read())) {
            uint8_t reply[protocol::maxReply];
            Serial.write(reply, handleFrame(parser, lift, reply));
        }
    }

    bool late = false;
    if (board::tickDue(late)) {
        if (late) lift.countLateTick();
        board::drive(lift.tick(board::read()));
        board::feedWatchdog();
    }
}

#endif
