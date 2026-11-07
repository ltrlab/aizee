#pragma once

#include <avr/interrupt.h>
#include <avr/sleep.h>

#include "lift.h"

// A column in the same shape as the real one will be: non-backdrivable, stiction, a hard stop at each end and an NC
// switch before each stop. The numbers are invented and only for these tests; nothing real has been measured yet.
const LiftConfig testConfig = {
    /* travelCounts */ 40000,
    /* clearanceCounts */ 400,
    /* maxSpeedCps */ 4000,
    /* homeSpeedCps */ 2000,
    /* fullSpeedUpCps */ 5000,
    /* fullSpeedDownCps */ 6000,
    /* stictionPwmUp */ 90,
    /* stictionPwmDown */ 60,
    /* encoderInverted */ false,
    /* motorInverted */ false,
    /* kpPwmPerKilocount */ 200,
    /* deadbandCounts */ 40,
    /* rampMs */ 300,
    /* shuntMilliohms */ 100,
    /* tripMilliamps */ 3000,
    /* minMillivolts */ 10500,
    /* noSupplyMillivolts */ 3000,
    /* stallMinCounts */ 16,
    /* wrongDirectionCounts */ 32,
    /* restMotionCounts */ 64,
};

class Plant {
  public:
    float position = 3000; // Where the Uno wakes up: above the bottom switch, and it does not know how far.
    float speed = 0;
    float bottomSwitch = 1000; // Plant counts. The lift's zero should land here.
    float bottomStop = 800;
    float topSwitch = 42000;
    float topStop = 42200;
    float stiffness = 0.05f; // Share of the speed error closed per millisecond.
    float upStiction = 90;
    float downStiction = 60;
    float upFull = 5000;
    float downFull = 6000;
    bool motorReversed = false;
    bool noisySwitchGround = false;

    // What the test can break.
    uint16_t busMillivolts = 12000;
    int16_t extraMilliamps = 0;
    bool angleAnswers = true;
    bool magnetPresent = true;
    bool shuntAnswers = true;
    bool busAnswers = true;
    bool topStuckClosed = false;
    bool bottomStuckClosed = false;
    bool topBroken = false;
    bool bottomBroken = false;
    bool jammed = false;
    float driftPerMs = 0;

    void step(int16_t pwm) {
        const float size = pwm < 0 ? -pwm : pwm;
        float wanted = 0;
        if (pwm > 0 && size >= upStiction) wanted = (size - upStiction) / (255 - upStiction) * upFull;
        if (pwm < 0 && size >= downStiction) wanted = -(size - downStiction) / (255 - downStiction) * downFull;
        if (motorReversed) wanted = -wanted;
        speed += (wanted - speed) * stiffness;
        if (jammed) speed = 0;
        position += speed * 0.001f + driftPerMs;
        if (position <= bottomStop) {
            position = bottomStop;
            speed = 0;
        }
        if (position >= topStop) {
            position = topStop;
            speed = 0;
        }
        lastPwmSize_ = size;
    }

    // The same schedule the board reads on.
    LiftInputs read() {
        slot_ = (slot_ + 1) % 20;
        LiftInputs in;
        in.angle = angleAnswers ? Reading::good : Reading::failed;
        in.rawAngle = (uint16_t)((int32_t)position & 4095);
        if (slot_ % 2 == 1) {
            in.shunt = shuntAnswers ? Reading::good : Reading::failed;
            const int32_t milliamps = (int32_t)(lastPwmSize_ * 3.0f) + extraMilliamps;
            in.shuntRaw = (int16_t)min(milliamps * testConfig.shuntMilliohms / 10, 32000L);
        } else if (slot_ == 0) {
            in.bus = busAnswers ? Reading::good : Reading::failed;
            in.busMillivolts = busMillivolts;
        } else if (slot_ == 10) {
            in.magnet = Reading::good;
            in.magnetPresent = magnetPresent;
        }
        in.topOpen = topBroken || (!topStuckClosed && position >= topSwitch);
        in.bottomOpen = bottomBroken || (!bottomStuckClosed && position <= bottomSwitch);
        return in;
    }

  private:
    uint8_t slot_ = 0;
    float lastPwmSize_ = 0;
};

// simavr quits when the chip sleeps with interrupts off. Without this a finished suite spins in loop() for ever, and
// the test run never ends.
inline void stopSimulator() {
    Serial.flush();
    cli();
    sleep_mode();
}

// One millisecond of lift and plant together.
inline int16_t run(Lift &lift, Plant &plant) {
    const int16_t pwm = lift.tick(plant.read());
    plant.step(pwm);
    return pwm;
}

inline void runFor(Lift &lift, Plant &plant, uint32_t ms) {
    for (uint32_t i = 0; i < ms; i++) run(lift, plant);
}

// Long enough for the first bus and magnet readings to arrive.
inline void wake(Lift &lift, Plant &plant) { runFor(lift, plant, 30); }

// Homes with a host that keeps answering. True if it came back homed.
inline bool homeNow(Lift &lift, Plant &plant) {
    wake(lift, plant);
    if (lift.home() != Reject::none) return false;
    for (uint32_t i = 0; i < 200000UL && lift.mode() == Mode::homing; i++) {
        lift.heard();
        run(lift, plant);
    }
    return lift.homed();
}

// Follows a target with a host commanding every 10 ms, until the column has come to rest or the time runs out.
inline void moveTo(Lift &lift, Plant &plant, int32_t target, uint32_t limitMs) {
    for (uint32_t i = 0; i < limitMs; i++) {
        if (i % 10 == 0) lift.move(target, 0);
        run(lift, plant);
        if (i > 20 && lift.atRest()) break;
    }
}
