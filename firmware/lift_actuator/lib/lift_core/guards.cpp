#include "guards.h"

namespace {

const int16_t shuntPinned = 31900; // The register tops out at 32000.
const uint8_t shuntPeriodMs = 2;
const uint8_t overcurrentMs = 20;
const uint8_t lowSampleLimit = 3; // 60 ms, at one bus reading per 20 ms.
const int16_t stallMs = 300;
const uint8_t wrongDirectionMs = 100;
const int16_t restSettleMs = 300;
// A leaky bucket for 2 minutes in 20. Running adds 9 per tick and resting takes 1, so 2 minutes of running fills it
// and 18 minutes of rest empties it.
const uint32_t dutyFull = 120000UL * 9;

int32_t magnitude(int32_t value) { return value < 0 ? -value : value; }

} // namespace

Fault Guards::update(const LiftInputs &in, const Encoder &encoder, int16_t pwm, bool active, bool jogging) {
    const int32_t position = encoder.counts();
    Fault fault = checkSwitches(in, active);
    const Fault current = checkCurrent(in, active);
    const Fault supply = checkSupply(in, active);
    const Fault encoderFault = checkEncoder(in, encoder, active);
    const Fault stall = checkStall(position, pwm, jogging);
    const Fault wrongDirection = checkWrongDirection(position, pwm);
    const Fault rest = checkRest(position, pwm);
    const Fault duty = checkDuty(pwm);

    if (fault == Fault::none) fault = current;
    if (fault == Fault::none) fault = encoderFault;
    if (fault == Fault::none) fault = supply;
    if (fault == Fault::none) fault = wrongDirection;
    if (fault == Fault::none) fault = stall;
    if (fault == Fault::none) fault = rest;
    if (fault == Fault::none) fault = duty;
    return fault;
}

Fault Guards::limitFault(int16_t pwm) const {
    if (pwm > 0 && atTop()) return Fault::limitTop;
    if (pwm < 0 && atBottom()) return Fault::limitBottom;
    return Fault::none;
}

void Guards::clear() {
    overcurrentMs_ = 0;
    stallMs_ = 0;
    wrongDirectionMs_ = 0;
    travelDirection_ = 0;
    restMs_ = 0;
}

uint8_t Guards::dutyPercent() const {
    const uint32_t percent = duty_ / (dutyFull / 100);
    return percent > 100 ? 100 : (uint8_t)percent;
}

Fault Guards::checkCurrent(const LiftInputs &in, bool active) {
    if (in.shunt == Reading::none) return Fault::none;
    if (in.shunt == Reading::failed) {
        if (shuntFailures_ < failureLimit) shuntFailures_++;
        return active && shuntFailures_ >= failureLimit ? Fault::currentSensor : Fault::none;
    }
    shuntFailures_ = 0;
    shuntSeen_ = true;

    shuntHistory_[shuntIndex_] = in.shuntRaw;
    shuntIndex_ = (shuntIndex_ + 1) % 4;
    const int32_t sum = (int32_t)shuntHistory_[0] + shuntHistory_[1] + shuntHistory_[2] + shuntHistory_[3];
    // 10 uV per count, over 4 samples: mA = sum * 10 / 4 / mOhm.
    const int32_t milliamps = sum * 5 / 2 / config_.shuntMilliohms;
    milliamps_ = milliamps > 32767 ? 32767 : (milliamps < -32767 ? -32767 : (int16_t)milliamps);

    if (magnitude(in.shuntRaw) >= shuntPinned) return Fault::overcurrent;
    if (magnitude(milliamps) > config_.tripMilliamps) {
        if (overcurrentMs_ < overcurrentMs) overcurrentMs_ += shuntPeriodMs;
        if (overcurrentMs_ >= overcurrentMs) return Fault::overcurrent;
    } else {
        overcurrentMs_ = 0;
    }
    return Fault::none;
}

Fault Guards::checkSupply(const LiftInputs &in, bool active) {
    if (in.bus == Reading::none) return Fault::none;
    if (in.bus == Reading::failed) {
        if (busFailures_ < failureLimit) busFailures_++;
        return active && busFailures_ >= failureLimit ? Fault::currentSensor : Fault::none;
    }
    busFailures_ = 0;
    busSeen_ = true;
    millivolts_ = in.busMillivolts;

    const bool low = in.busMillivolts < (uint16_t)config_.minMillivolts;
    if (!low)
        lowSamples_ = 0;
    else if (lowSamples_ < lowSampleLimit)
        lowSamples_++;

    if (!active || lowSamples_ < lowSampleLimit) return Fault::none;
    return in.busMillivolts < (uint16_t)config_.noSupplyMillivolts ? Fault::noSupply : Fault::undervoltage;
}

Fault Guards::checkEncoder(const LiftInputs &in, const Encoder &encoder, bool active) {
    if (in.magnet == Reading::good) {
        magnetMarginal_ = in.magnetMarginal;
        if (in.magnetPresent)
            magnetAbsent_ = 0;
        else if (magnetAbsent_ < magnetLimit)
            magnetAbsent_++;
    }
    if (encoder.jumped()) return Fault::encoder;
    if (!active) return Fault::none;
    if (encoder.lost()) return Fault::encoder;
    if (magnetLost()) return Fault::magnet;
    return Fault::none;
}

Fault Guards::checkSwitches(const LiftInputs &in, bool active) {
    if (!in.topOpen)
        topOpenTicks_ = 0;
    else if (topOpenTicks_ < switchTicks)
        topOpenTicks_++;
    if (!in.bottomOpen)
        bottomOpenTicks_ = 0;
    else if (bottomOpenTicks_ < switchTicks)
        bottomOpenTicks_++;
    return active && atTop() && atBottom() ? Fault::switchWiring : Fault::none;
}

bool Guards::hasEffort(int16_t pwm) const {
    if (pwm == 0) return false;
    return pwm > 0 ? pwm >= config_.stictionPwmUp : -pwm >= config_.stictionPwmDown;
}

Fault Guards::checkStall(int32_t position, int16_t pwm, bool jogging) {
    if (jogging || !hasEffort(pwm)) {
        stallMs_ = 0;
        stallAnchor_ = position;
        return Fault::none;
    }
    if (magnitude(position - stallAnchor_) >= config_.stallMinCounts) {
        stallAnchor_ = position;
        stallMs_ = 0;
        return Fault::none;
    }
    return ++stallMs_ >= stallMs ? Fault::stall : Fault::none;
}

Fault Guards::checkWrongDirection(int32_t position, int16_t pwm) {
    const int8_t direction = pwm > 0 ? 1 : (pwm < 0 ? -1 : 0);
    if (direction != travelDirection_) {
        travelDirection_ = direction;
        furthest_ = position;
        wrongDirectionMs_ = 0;
    }
    if (direction == 0) return Fault::none;

    const int32_t progress = direction > 0 ? position - furthest_ : furthest_ - position;
    if (progress > 0) furthest_ = position;
    if (progress > -(int32_t)config_.wrongDirectionCounts) {
        wrongDirectionMs_ = 0;
        return Fault::none;
    }
    return ++wrongDirectionMs_ >= wrongDirectionMs ? Fault::wrongDirection : Fault::none;
}

Fault Guards::checkRest(int32_t position, int16_t pwm) {
    if (pwm != 0) {
        restMs_ = 0;
        return Fault::none;
    }
    if (restMs_ < restSettleMs) {
        restMs_++;
        restAnchor_ = position;
        return Fault::none;
    }
    return magnitude(position - restAnchor_) > config_.restMotionCounts ? Fault::uncommandedMotion : Fault::none;
}

Fault Guards::checkDuty(int16_t pwm) {
    if (hasEffort(pwm)) {
        if (duty_ < dutyFull) duty_ += 9;
    } else if (duty_ > 0) {
        duty_--;
    }
    return duty_ >= dutyFull ? Fault::duty : Fault::none;
}
