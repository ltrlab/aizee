#pragma once

#include <stdint.h>

#include "encoder.h"
#include "lift_config.h"
#include "protocol.h"

enum class Reading : uint8_t {
    none,   // Not read this tick.
    good,
    failed, // The chip did not answer.
};

// One millisecond's readings. The board reads the angle every tick, the shunt every 2 ms, and the bus voltage and the
// magnet every 20 ms.
struct LiftInputs {
    Reading angle = Reading::none;
    uint16_t rawAngle = 0;
    Reading magnet = Reading::none;
    bool magnetPresent = false;
    bool magnetMarginal = false;
    Reading shunt = Reading::none;
    int16_t shuntRaw = 0; // INA219 shunt register: 10 uV per count, pinned at +-320 mV.
    Reading bus = Reading::none;
    uint16_t busMillivolts = 0;
    bool topOpen = true;
    bool bottomOpen = true;
};

// Every trip except the command watchdog and the homing timeout. Each one has its own timer here; what happens next
// is up to the caller.
class Guards {
  public:
    explicit Guards(const LiftConfig &config) : config_(config) {}

    // `pwm` is what the motor got last tick. `active` means enabled, homing or jogging; most failures only latch when
    // something depends on them. A jog runs without the stall check, so a PWM too low to move can be tried.
    Fault update(const LiftInputs &in, const Encoder &encoder, int16_t pwm, bool active, bool jogging);

    // A switch open in the direction this PWM would travel.
    Fault limitFault(int16_t pwm) const;

    // Timers restart. The duty allowance does not: it is about the motor, not about the fault.
    void clear();

    bool atTop() const { return topOpenTicks_ >= switchTicks; }
    bool atBottom() const { return bottomOpenTicks_ >= switchTicks; }
    bool sensorOk() const {
        return shuntSeen_ && busSeen_ && shuntFailures_ < failureLimit && busFailures_ < failureLimit;
    }
    bool supplyOk() const { return busSeen_ && lowSamples_ == 0; }
    bool magnetLost() const { return magnetAbsent_ >= magnetLimit; }
    bool magnetMarginal() const { return magnetMarginal_; }
    int16_t milliamps() const { return milliamps_; }
    uint16_t millivolts() const { return millivolts_; }
    uint8_t dutyPercent() const;

  private:
    static const uint8_t failureLimit = 3;
    static const uint8_t magnetLimit = 3;
    static const uint8_t switchTicks = 2;

    Fault checkCurrent(const LiftInputs &in, bool active);
    Fault checkSupply(const LiftInputs &in, bool active);
    Fault checkEncoder(const LiftInputs &in, const Encoder &encoder, bool active);
    Fault checkSwitches(const LiftInputs &in, bool active);
    Fault checkStall(int32_t position, int16_t pwm, bool jogging);
    Fault checkWrongDirection(int32_t position, int16_t pwm);
    Fault checkRest(int32_t position, int16_t pwm);
    Fault checkDuty(int16_t pwm);
    bool hasEffort(int16_t pwm) const;

    const LiftConfig &config_;
    int16_t shuntHistory_[4] = {};
    uint8_t shuntIndex_ = 0;
    uint8_t shuntFailures_ = 0;
    uint8_t busFailures_ = 0;
    uint8_t lowSamples_ = 0;
    uint8_t magnetAbsent_ = 0;
    uint8_t topOpenTicks_ = 0;
    uint8_t bottomOpenTicks_ = 0;
    uint8_t overcurrentMs_ = 0;
    bool shuntSeen_ = false;
    bool busSeen_ = false;
    bool magnetMarginal_ = false;
    int16_t milliamps_ = 0;
    uint16_t millivolts_ = 0;
    int16_t stallMs_ = 0;
    int32_t stallAnchor_ = 0;
    int8_t travelDirection_ = 0;
    int32_t furthest_ = 0;
    uint8_t wrongDirectionMs_ = 0;
    int16_t restMs_ = 0;
    int32_t restAnchor_ = 0;
    uint32_t duty_ = 0;
};
