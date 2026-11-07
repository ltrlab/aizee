#pragma once

#include <stdint.h>

#include "control.h"
#include "encoder.h"
#include "guards.h"
#include "lift_config.h"
#include "protocol.h"

// The lift's modes, and the only path to the motor. It starts disabled and unhomed, nothing moves until the host asks,
// and every way out of a mode coasts.
class Lift {
  public:
    explicit Lift(const LiftConfig &config);

    // One millisecond: the readings go in, the signed PWM comes out. Up is positive.
    int16_t tick(const LiftInputs &in);

    Reject enable();
    Reject disable();
    Reject move(int32_t target, int32_t velocityCps);
    Reject home();
    Reject clear();

    // Any good frame from the host. While homing it keeps the watchdog fed; while enabled only MOVE does.
    void heard();

    void countLateTick();

    Mode mode() const { return mode_; }
    Fault fault() const { return fault_; }
    bool homed() const { return homed_; }
    bool encoderOk() const { return !encoder_.lost() && !encoder_.jumped() && !guards_.magnetLost(); }
    bool measured() const { return measured_; }
    bool atRest() const { return mode_ == Mode::disabled || (mode_ == Mode::enabled && controller_.atRest()); }
    int32_t counts() const { return encoder_.counts(); }
    int32_t velocityCps() const { return encoder_.velocityCps(); }
    int16_t pwm() const { return output_; }
    uint32_t boardMs() const { return boardMs_; }
    uint8_t lateTicks() const { return lateTicks_; }
    const Guards &guards() const { return guards_; }
    const LiftConfig &config() const { return config_; }

#ifdef LIFT_BENCH
    // Open loop, at most 2 s, under every guard but the stall check.
    Reject jog(int16_t pwm, uint16_t durationMs);
#endif

  private:
    enum class HomePhase : uint8_t {
        seek,     // Down to the bottom switch.
        backOff,  // Up until it has closed, plus the clearance.
        approach, // Down again at half speed. Its edge is zero.
        leave,    // Up to the lowest allowed position.
    };

    Reject readyToMove() const;
    int16_t trip(Fault fault);
    int16_t tickHoming(int32_t position);
    void beginHomePhase(HomePhase phase, int32_t position);

    const LiftConfig &config_;
    const bool measured_;
    Encoder encoder_;
    Guards guards_;
    Controller controller_;
    Mode mode_ = Mode::disabled;
    Fault fault_ = Fault::none;
    HomePhase homePhase_ = HomePhase::seek;
    bool homed_ = false;
    bool switchClosedAgain_ = false;
    int16_t output_ = 0;
    int16_t commandAgeMs_ = 0;
    uint8_t lateTicks_ = 0;
    int32_t backOffStart_ = 0;
    uint32_t homeMs_ = 0;
    uint32_t homeLimitMs_ = 0;
    uint32_t boardMs_ = 0;

#ifdef LIFT_BENCH
    int16_t jogPwm_ = 0;
    uint16_t jogMs_ = 0;
#endif
};
