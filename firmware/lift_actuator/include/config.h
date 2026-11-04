#pragma once

#include "lift_config.h"

// This column. The zeros have not been measured yet, and they keep HOME and ENABLE refused until they are:
// README.md, "Bring-up", says how lift_bench measures each one.
const LiftConfig liftConfig = {
    // UNMEASURED.
    /* travelCounts */ 0,
    /* clearanceCounts */ 0,
    /* maxSpeedCps */ 0,
    /* homeSpeedCps */ 0,
    /* fullSpeedUpCps */ 0,
    /* fullSpeedDownCps */ 0,
    /* stictionPwmUp */ 0,
    /* stictionPwmDown */ 0,

    // Found in the first jogs.
    /* encoderInverted */ false,
    /* motorInverted */ false,

    // Starting points, tuned once it moves closed loop.
    /* kpPwmPerKilocount */ 20,
    /* deadbandCounts */ 40,
    /* rampMs */ 300,

    // The INA219's 0.1 ohm shunt reads to 3.2 A, so the trip sits just under that until the column's current has been
    // measured. Lower it to about 1.5x the running current then.
    /* shuntMilliohms */ 100,
    /* tripMilliamps */ 3000,
    /* minMillivolts */ 10500,
    /* noSupplyMillivolts */ 3000,

    // Well above the AS5600's noise of a few counts.
    /* stallMinCounts */ 16,
    /* wrongDirectionCounts */ 32,
    /* restMotionCounts */ 64,
};
