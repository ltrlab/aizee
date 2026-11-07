#pragma once

#include <stdint.h>

#include "lift_config.h"

// Turns a target into PWM once per millisecond. Up is positive.
//
// A setpoint chases the target at no more than the speed cap, under a fixed acceleration, and stops on it without
// overshooting. The motor follows the setpoint on feed-forward (stiction plus a share of full speed) and a position
// term. At rest it coasts inside a deadband, because the column holds itself. The output rises over at least 150 ms,
// drops at once, and waits 50 ms before reversing.
class Controller {
  public:
    explicit Controller(const LiftConfig &config);

    // Setpoint to `position`, at rest, output off. The wait before a reversal still applies.
    void start(int32_t position);

    // The target and the speed it is moving at. Between calls the target carries on at that speed.
    void setGoal(int32_t target, int32_t velocityCps, int32_t speedCap);

    int16_t tick(int32_t position);

    // The soft start and the wait before a reversal, for a PWM from elsewhere. tick() does this itself.
    int16_t shape(int16_t wanted);

    // Nothing asked of the motor: the setpoint has stopped and the column is inside the deadband.
    bool atRest() const { return velocity_ == 0 && settled_; }

  private:
    void stepTarget();
    void stepSetpoint(int32_t position);
    int16_t law(int32_t position);

    const LiftConfig &config_;
    int32_t accelPerTick_;
    int32_t cap_ = 0;
    int32_t maxLag_ = 0;
    int32_t target_ = 0;
    int32_t targetRemainder_ = 0;
    int32_t targetVelocity_ = 0;
    int32_t setpoint_ = 0;
    int32_t setpointRemainder_ = 0;
    int32_t velocity_ = 0;
    int32_t magnitudeQ8_ = 0; // Output size, 8 fractional bits, so the rise can be slower than 1 PWM per tick.
    int8_t direction_ = 0;
    uint8_t dwellMs_ = 0;
    bool settled_ = true;
};
