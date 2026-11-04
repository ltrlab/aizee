#include "control.h"

namespace {

const int16_t fullPwm = 255;
const uint8_t reverseDwellMs = 50;
// 255 << 8 over 150 ticks, rounded down so the rise takes at least 150 ms.
const int32_t risePerTickQ8 = 435;
// The position term's error is clamped here so the product stays inside 32 bits.
const int32_t largestError = 1L << 20;

int32_t magnitude(int32_t value) { return value < 0 ? -value : value; }

// Adds velocity * 1 ms to a position, keeping the part too small to count yet. 1049 / 2^20 is within 0.04 % of
// 1 / 1000, and it spares this path a 32-bit division, the slowest thing in the tick.
void advance(int32_t &position, int32_t &remainder, int32_t velocityCps) {
    remainder += velocityCps * 1049;
    position += remainder >> 20;
    remainder &= (1L << 20) - 1;
}

} // namespace

Controller::Controller(const LiftConfig &config) : config_(config) {
    accelPerTick_ = config.maxSpeedCps / config.rampMs;
    if (accelPerTick_ < 1) accelPerTick_ = 1;
}

void Controller::start(int32_t position) {
    if (magnitudeQ8_ > 0) dwellMs_ = reverseDwellMs;
    magnitudeQ8_ = 0;
    target_ = position;
    targetRemainder_ = 0;
    targetVelocity_ = 0;
    setpoint_ = position;
    setpointRemainder_ = 0;
    velocity_ = 0;
    settled_ = true;
}

void Controller::setGoal(int32_t target, int32_t velocityCps, int32_t speedCap) {
    target_ = target;
    targetRemainder_ = 0;
    targetVelocity_ = velocityCps;
    cap_ = speedCap;
    maxLag_ = speedCap / 4;
    const int32_t leastLag = (int32_t)config_.deadbandCounts * 4;
    if (maxLag_ < leastLag) maxLag_ = leastLag;
}

int16_t Controller::tick(int32_t position) {
    stepTarget();
    stepSetpoint(position);
    return shape(law(position));
}

void Controller::stepTarget() {
    if (targetVelocity_ == 0) return;
    advance(target_, targetRemainder_, targetVelocity_);
    if (target_ < config_.clearanceCounts) target_ = config_.clearanceCounts;
    if (target_ > config_.travelCounts) target_ = config_.travelCounts;
}

void Controller::stepSetpoint(int32_t position) {
    // All of this is seen from the target, which may be moving: stop relative to it, then match its speed.
    const int32_t error = target_ - setpoint_;
    const int32_t closing = velocity_ - targetVelocity_;
    const int32_t closingSpeed = magnitude(closing);
    const int32_t stopCounts = closingSpeed / accelPerTick_ * closingSpeed / 2000;
    const bool towardTarget = closing == 0 || (closing > 0) == (error > 0);

    int32_t wanted = targetVelocity_;
    if (error != 0 && towardTarget && magnitude(error) > stopCounts) wanted = error > 0 ? cap_ : -cap_;

    if (velocity_ < wanted) {
        velocity_ += accelPerTick_;
        if (velocity_ > wanted) velocity_ = wanted;
    } else if (velocity_ > wanted) {
        velocity_ -= accelPerTick_;
        if (velocity_ < wanted) velocity_ = wanted;
    }

    advance(setpoint_, setpointRemainder_, velocity_);

    // Reaching or crossing the target slowly enough to stop on it: stop on it, rather than wobble around it.
    const int32_t after = target_ - setpoint_;
    const bool reached = after == 0 || (after > 0) != (error > 0);
    if (reached && stopCounts <= 2) {
        setpoint_ = target_;
        setpointRemainder_ = 0;
        velocity_ = targetVelocity_;
    }

    // A setpoint far ahead of a column that cannot keep up would only be a surprise later.
    if (setpoint_ - position > maxLag_) setpoint_ = position + maxLag_;
    if (position - setpoint_ > maxLag_) setpoint_ = position - maxLag_;
}

int16_t Controller::law(int32_t position) {
    int32_t error = setpoint_ - position;
    if (error > largestError) error = largestError;
    if (error < -largestError) error = -largestError;
    const int32_t size = magnitude(error);

    if (velocity_ == 0) {
        if (settled_ && size <= 2 * (int32_t)config_.deadbandCounts) return 0;
        settled_ = size <= config_.deadbandCounts;
        if (settled_) return 0;
    } else {
        settled_ = false;
    }

    const bool moving = velocity_ != 0;
    const bool up = moving ? velocity_ > 0 : error > 0;
    const int32_t stiction = up ? config_.stictionPwmUp : config_.stictionPwmDown;
    const int32_t fullSpeed = up ? config_.fullSpeedUpCps : config_.fullSpeedDownCps;

    int32_t effort = (up ? error : -error) * config_.kpPwmPerKilocount / 1000;
    if (moving) {
        effort += (fullPwm - stiction) * magnitude(velocity_) / fullSpeed;
        // Ahead of a moving setpoint: wait for it instead of reversing.
        if (effort <= 0) return 0;
    } else if (effort < 1) {
        effort = 1; // Outside the deadband at rest, stiction alone has to be enough.
    }

    effort += stiction;
    if (effort > fullPwm) effort = fullPwm;
    return up ? (int16_t)effort : (int16_t)-effort;
}

int16_t Controller::shape(int16_t wanted) {
    if (dwellMs_ > 0) dwellMs_--;

    const int8_t wantedDirection = wanted > 0 ? 1 : (wanted < 0 ? -1 : 0);
    if (wantedDirection == 0) {
        magnitudeQ8_ = 0;
        return 0;
    }
    if (direction_ != 0 && wantedDirection != direction_) {
        if (magnitudeQ8_ > 0) {
            magnitudeQ8_ = 0;
            dwellMs_ = reverseDwellMs;
            return 0;
        }
        if (dwellMs_ > 0) return 0;
    }

    direction_ = wantedDirection;
    const int32_t wantedQ8 = (int32_t)(wanted < 0 ? -wanted : wanted) << 8;
    magnitudeQ8_ += risePerTickQ8;
    if (magnitudeQ8_ > wantedQ8) magnitudeQ8_ = wantedQ8;
    dwellMs_ = reverseDwellMs;

    const int16_t output = (int16_t)(magnitudeQ8_ >> 8);
    return direction_ > 0 ? output : (int16_t)-output;
}
