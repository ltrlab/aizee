#pragma once

#include <stdint.h>

// What the firmware needs to know about one column. include/config.h holds the real one.
struct LiftConfig {
    // Measured with lift_bench (README.md, "Bring-up"). A zero here refuses HOME and ENABLE.
    int32_t travelCounts;     // From the bottom switch's edge to the highest allowed position.
    int32_t clearanceCounts;  // How far above the bottom switch's edge the column rests. Also the lowest allowed.
    int32_t maxSpeedCps;      // The speed cap, no more than either full speed.
    int32_t homeSpeedCps;     // The first approach to the bottom switch. The second runs at half of it.
    int32_t fullSpeedUpCps;   // Steady speed at PWM 255, going up.
    int32_t fullSpeedDownCps; // Steady speed at PWM 255, going down.
    int16_t stictionPwmUp;    // The lowest PWM that keeps the column moving up.
    int16_t stictionPwmDown;  // The lowest PWM that keeps the column moving down.

    // Wiring, found in the first jogs. Up is positive PWM and rising counts.
    bool encoderInverted; // The raw angle falls going up.
    bool motorInverted;   // IN1 high drives down.

    // Control.
    int16_t kpPwmPerKilocount; // PWM per 1000 counts of position error.
    int16_t deadbandCounts;    // Within this of the setpoint at rest the motor coasts. Twice this restarts it.
    int16_t rampMs;            // How long the setpoint takes to reach the speed cap.

    // Guards.
    int16_t shuntMilliohms;
    int16_t tripMilliamps;      // Supply current, averaged over 8 ms, held for 20 ms.
    int16_t minMillivolts;      // Below this, held for 60 ms, is undervoltage.
    int16_t noSupplyMillivolts; // Below this it is no supply instead.
    int16_t stallMinCounts;     // Driven, the column must move this far every 300 ms.
    int16_t wrongDirectionCounts;
    int16_t restMotionCounts; // Coasting, moving this far means something is wrong.
};

inline bool liftConfigMeasured(const LiftConfig &c) {
    return c.travelCounts > 0 && c.clearanceCounts > 0 && c.clearanceCounts < c.travelCounts && c.maxSpeedCps > 0 &&
           c.homeSpeedCps > 0 && c.homeSpeedCps <= c.maxSpeedCps && c.fullSpeedUpCps >= c.maxSpeedCps &&
           c.fullSpeedDownCps >= c.maxSpeedCps && c.stictionPwmUp > 0 && c.stictionPwmUp < 255 &&
           c.stictionPwmDown > 0 && c.stictionPwmDown < 255 && c.rampMs > 0 && c.shuntMilliohms > 0;
}
