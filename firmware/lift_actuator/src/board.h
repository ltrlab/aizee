#pragma once

#include <stdint.h>

#include "guards.h"
#include "lift_config.h"

// The Uno: pins, PWM, I2C and the millisecond tick. README.md has the wiring.
namespace board {

// Outputs off first, then the buses. Leaves the watchdog running: a tick that never comes resets the board.
void begin(const LiftConfig &config);

// True once per millisecond. `late` says this tick came more than a whole tick behind.
bool tickDue(bool &late);

// Reads what this tick's slot holds: the angle always, the shunt on odd ticks, the bus and the magnet on two of the
// even ones, so a tick never makes more than two I2C reads.
LiftInputs read();

// Signed PWM, up is positive. Zero coasts.
void drive(int16_t pwm);

// Call after every tick.
void feedWatchdog();

} // namespace board
