#include "lift.h"

namespace {

const int16_t commandTimeoutMs = 100;
const uint8_t coolingPercent = 75; // Past this share of the duty allowance, nothing new starts.
const uint32_t leastHomeLimitMs = 30000;

} // namespace

Lift::Lift(const LiftConfig &config)
    : config_(config), measured_(liftConfigMeasured(config)), encoder_(config.encoderInverted), guards_(config),
      controller_(config) {}

int16_t Lift::tick(const LiftInputs &in) {
    boardMs_++;
    encoder_.update(in.angle == Reading::good, in.rawAngle);

    const bool active = mode_ != Mode::disabled;
    Fault fault = guards_.update(in, encoder_, output_, active, mode_ == Mode::jog);
    if (!encoderOk()) homed_ = false;

    if (fault == Fault::none && (mode_ == Mode::enabled || mode_ == Mode::homing)) {
        if (++commandAgeMs_ > commandTimeoutMs) fault = Fault::commandTimeout;
    }
    if (fault != Fault::none) return trip(fault);

    const int32_t position = encoder_.counts();
    int16_t pwm = 0;
    switch (mode_) {
    case Mode::enabled:
        pwm = controller_.tick(position);
        break;
    case Mode::homing:
        if (++homeMs_ > homeLimitMs_) return trip(Fault::homeTimeout);
        pwm = tickHoming(position);
        break;
#ifdef LIFT_BENCH
    case Mode::jog:
        if (jogMs_ == 0) {
            mode_ = Mode::disabled;
            controller_.start(position);
        } else {
            jogMs_--;
            pwm = controller_.shape(jogPwm_);
        }
        break;
#endif
    default:
        break;
    }

    // Moving away from an open switch is the way out, so only the direction of travel counts.
    fault = guards_.limitFault(pwm);
    if (fault != Fault::none) return trip(fault);

    output_ = pwm;
    return pwm;
}

Reject Lift::enable() {
    if (mode_ == Mode::enabled) return Reject::none;
    if (mode_ != Mode::disabled) return Reject::busy;
    const Reject reject = readyToMove();
    if (reject != Reject::none) return reject;
    if (!homed_) return Reject::notHomed;
    controller_.start(encoder_.counts());
    commandAgeMs_ = 0;
    mode_ = Mode::enabled;
    return Reject::none;
}

Reject Lift::disable() {
    mode_ = Mode::disabled;
    controller_.start(encoder_.counts());
    output_ = 0;
    return Reject::none;
}

Reject Lift::move(int32_t target, int32_t velocityCps) {
    if (mode_ != Mode::enabled) return Reject::notEnabled;
    if (target < config_.clearanceCounts || target > config_.travelCounts) return Reject::outOfRange;
    if (velocityCps > config_.maxSpeedCps || velocityCps < -config_.maxSpeedCps) return Reject::outOfRange;
    controller_.setGoal(target, velocityCps, config_.maxSpeedCps);
    commandAgeMs_ = 0;
    return Reject::none;
}

Reject Lift::home() {
    if (mode_ != Mode::disabled) return Reject::busy;
    const Reject reject = readyToMove();
    if (reject != Reject::none) return reject;

    homed_ = false;
    homeMs_ = 0;
    // The whole travel at the homing speed, half as long again, and the slow approach. Never under 30 s.
    homeLimitMs_ = ((uint32_t)(config_.travelCounts / config_.homeSpeedCps) + 1) * 1500UL + 10000UL;
    if (homeLimitMs_ < leastHomeLimitMs) homeLimitMs_ = leastHomeLimitMs;
    commandAgeMs_ = 0;
    mode_ = Mode::homing;
    beginHomePhase(guards_.atBottom() ? HomePhase::backOff : HomePhase::seek, encoder_.counts());
    return Reject::none;
}

Reject Lift::clear() {
    if (mode_ != Mode::disabled) return Reject::busy;
    fault_ = Fault::none;
    encoder_.clearJump();
    guards_.clear();
    return Reject::none;
}

void Lift::heard() {
    if (mode_ == Mode::homing) commandAgeMs_ = 0;
}

void Lift::countLateTick() {
    if (lateTicks_ < 255) lateTicks_++;
}

Reject Lift::readyToMove() const {
    if (!measured_) return Reject::unmeasured;
    if (fault_ != Fault::none) return Reject::faulted;
    if (!encoderOk() || !guards_.sensorOk()) return Reject::sensors;
    if (!guards_.supplyOk()) return Reject::noSupply;
    if (guards_.atTop() && guards_.atBottom()) return Reject::switches;
    if (guards_.dutyPercent() >= coolingPercent) return Reject::cooling;
    return Reject::none;
}

int16_t Lift::trip(Fault fault) {
    if (fault_ == Fault::none) fault_ = fault;
    mode_ = Mode::disabled;
    controller_.start(encoder_.counts());
    output_ = 0;
    return 0;
}

void Lift::beginHomePhase(HomePhase phase, int32_t position) {
    homePhase_ = phase;
    controller_.start(position);
    // Further than the column can go: the phase ends on the switch, not on the target.
    const int32_t beyond = config_.travelCounts * 2;
    switch (phase) {
    case HomePhase::seek:
        controller_.setGoal(position - beyond, 0, config_.homeSpeedCps);
        break;
    case HomePhase::backOff:
        switchClosedAgain_ = false;
        controller_.setGoal(position + beyond, 0, config_.homeSpeedCps);
        break;
    case HomePhase::approach:
        controller_.setGoal(position - beyond, 0, config_.homeSpeedCps / 2);
        break;
    case HomePhase::leave:
        controller_.setGoal(config_.clearanceCounts, 0, config_.homeSpeedCps / 2);
        break;
    }
}

int16_t Lift::tickHoming(int32_t position) {
    switch (homePhase_) {
    case HomePhase::seek:
        if (guards_.atBottom()) beginHomePhase(HomePhase::backOff, position);
        break;
    case HomePhase::backOff:
        if (!guards_.atBottom() && !switchClosedAgain_) {
            switchClosedAgain_ = true;
            backOffStart_ = position;
        }
        if (switchClosedAgain_ && position - backOffStart_ >= config_.clearanceCounts) {
            beginHomePhase(HomePhase::approach, position);
        }
        break;
    case HomePhase::approach:
        if (guards_.atBottom()) {
            encoder_.setZeroHere();
            beginHomePhase(HomePhase::leave, 0);
            return controller_.tick(0);
        }
        break;
    case HomePhase::leave:
        if (controller_.atRest() && !guards_.atBottom()) {
            homed_ = true;
            mode_ = Mode::disabled;
            controller_.start(position);
            return 0;
        }
        break;
    }
    return controller_.tick(position);
}

#ifdef LIFT_BENCH
Reject Lift::jog(int16_t pwm, uint16_t durationMs) {
    if (mode_ != Mode::disabled) return Reject::busy;
    if (fault_ != Fault::none) return Reject::faulted;
    if (!encoderOk() || !guards_.sensorOk()) return Reject::sensors;
    if (!guards_.supplyOk()) return Reject::noSupply;
    if (guards_.limitFault(pwm) != Fault::none) return Reject::switches;
    if (guards_.dutyPercent() >= coolingPercent) return Reject::cooling;
    if (pwm == 0 || pwm > 255 || pwm < -255 || durationMs > 2000) return Reject::outOfRange;
    jogPwm_ = pwm;
    jogMs_ = durationMs;
    mode_ = Mode::jog;
    return Reject::none;
}
#endif
