#include <Arduino.h>
#include <unity.h>

#include "../plant.h"

void setUp() {}
void tearDown() {}

namespace {

int32_t plantCounts(const Plant &plant) { return (int32_t)plant.position - (int32_t)plant.bottomSwitch; }
int32_t distance(int32_t a, int32_t b) { return a > b ? a - b : b - a; }

} // namespace

void encoderFollowsTheAngleAcrossTheWrap() {
    Encoder encoder(false);
    encoder.update(true, 4090);
    encoder.update(true, 5); // +11 across the wrap.
    TEST_ASSERT_EQUAL_INT32(11, encoder.counts());
    encoder.update(true, 4094); // -7 back across it.
    TEST_ASSERT_EQUAL_INT32(4, encoder.counts());
}

void aReversedEncoderCountsTheOtherWay() {
    Encoder encoder(true);
    encoder.update(true, 100);
    encoder.update(true, 110);
    TEST_ASSERT_EQUAL_INT32(-10, encoder.counts());
}

void encoderMeasuresSpeedOverSixteenMilliseconds() {
    Encoder encoder(false);
    uint16_t angle = 0;
    for (uint8_t i = 0; i < 40; i++) {
        angle = (angle + 8) & 4095;
        encoder.update(true, angle);
    }
    TEST_ASSERT_EQUAL_INT32(8000, encoder.velocityCps());
}

void aZeroMovesTheCountsButNotTheSpeed() {
    Encoder encoder(false);
    uint16_t angle = 0;
    for (uint8_t i = 0; i < 40; i++) {
        angle = (angle + 8) & 4095;
        encoder.update(true, angle);
    }
    encoder.setZeroHere();
    TEST_ASSERT_EQUAL_INT32(0, encoder.counts());
    TEST_ASSERT_EQUAL_INT32(8000, encoder.velocityCps());
}

void nothingMovesAtPowerUp() {
    Lift lift(testConfig);
    Plant plant;
    for (uint16_t i = 0; i < 5000; i++) TEST_ASSERT_EQUAL_INT16(0, run(lift, plant));
    TEST_ASSERT_FALSE(lift.homed());
    TEST_ASSERT_TRUE(lift.mode() == Mode::disabled);
    TEST_ASSERT_TRUE(lift.fault() == Fault::none);
}

void enableIsRefusedUntilHomed() {
    Lift lift(testConfig);
    Plant plant;
    wake(lift, plant);
    TEST_ASSERT_TRUE(lift.enable() == Reject::notHomed);
}

void anUnmeasuredColumnRefusesHomeAndEnable() {
    LiftConfig unmeasured = testConfig;
    unmeasured.travelCounts = 0;
    Lift lift(unmeasured);
    Plant plant;
    wake(lift, plant);
    TEST_ASSERT_FALSE(lift.measured());
    TEST_ASSERT_TRUE(lift.home() == Reject::unmeasured);
    TEST_ASSERT_TRUE(lift.enable() == Reject::unmeasured);
}

void homingZeroesOnTheBottomSwitchAndStopsClearOfIt() {
    Lift lift(testConfig);
    Plant plant;
    TEST_ASSERT_TRUE(homeNow(lift, plant));
    TEST_ASSERT_TRUE(lift.mode() == Mode::disabled);
    TEST_ASSERT_TRUE(lift.fault() == Fault::none);

    // Zero is the switch's edge: the plant and the lift agree, to within the slow approach's debounce.
    TEST_ASSERT_INT32_WITHIN(12, plantCounts(plant), lift.counts());
    TEST_ASSERT_INT32_WITHIN(2 * testConfig.deadbandCounts, testConfig.clearanceCounts, lift.counts());
    TEST_ASSERT_FALSE(lift.guards().atBottom());

    // And the column stays there.
    for (uint16_t i = 0; i < 1000; i++) TEST_ASSERT_EQUAL_INT16(0, run(lift, plant));
}

void homingStartingOnTheSwitchBacksOffFirst() {
    Lift lift(testConfig);
    Plant plant;
    plant.position = 950; // Below the switch's edge.
    TEST_ASSERT_TRUE(homeNow(lift, plant));
    TEST_ASSERT_INT32_WITHIN(12, plantCounts(plant), lift.counts());
}

void homingStopsWhenTheHostGoesQuiet() {
    Lift lift(testConfig);
    Plant plant;
    wake(lift, plant);
    TEST_ASSERT_TRUE(lift.home() == Reject::none);
    for (uint16_t i = 0; i < 90; i++) {
        lift.heard();
        run(lift, plant);
    }
    for (uint16_t i = 0; i < 101; i++) run(lift, plant);
    TEST_ASSERT_TRUE(lift.fault() == Fault::commandTimeout);
    TEST_ASSERT_TRUE(lift.mode() == Mode::disabled);
    TEST_ASSERT_EQUAL_INT16(0, run(lift, plant));
    TEST_ASSERT_FALSE(lift.homed());
}

void homingGivesUpIfTheSwitchNeverOpens() {
    Lift lift(testConfig);
    Plant plant;
    plant.bottomStuckClosed = true;
    plant.bottomStop = -100000000.0f; // Nothing stops it either, so only the clock can.
    TEST_ASSERT_FALSE(homeNow(lift, plant));
    TEST_ASSERT_TRUE(lift.fault() == Fault::homeTimeout);
}

void homingStallsOnTheHardStopIfTheSwitchNeverOpens() {
    Lift lift(testConfig);
    Plant plant;
    plant.bottomStuckClosed = true;
    TEST_ASSERT_FALSE(homeNow(lift, plant));
    TEST_ASSERT_TRUE(lift.fault() == Fault::stall);
}

void enableMovesNothing() {
    Lift lift(testConfig);
    Plant plant;
    TEST_ASSERT_TRUE(homeNow(lift, plant));
    TEST_ASSERT_TRUE(lift.enable() == Reject::none);
    for (uint8_t i = 0; i < 90; i++) TEST_ASSERT_EQUAL_INT16(0, run(lift, plant));
    TEST_ASSERT_TRUE(lift.atRest());
}

void aMoveArrivesUnderItsCapsAndThenCoasts() {
    Lift lift(testConfig);
    Plant plant;
    TEST_ASSERT_TRUE(homeNow(lift, plant));
    TEST_ASSERT_TRUE(lift.enable() == Reject::none);

    const int32_t target = 20000;
    int16_t last = 0;
    int16_t biggestRise = 0;
    int32_t fastest = 0;
    for (uint32_t i = 0; i < 30000 && !(i > 20 && lift.atRest()); i++) {
        if (i % 10 == 0) TEST_ASSERT_TRUE(lift.move(target, 0) == Reject::none);
        const int16_t pwm = run(lift, plant);
        TEST_ASSERT_TRUE(pwm >= 0); // A move this long never needs to reverse.
        if (pwm - last > biggestRise) biggestRise = pwm - last;
        last = pwm;
        if (lift.velocityCps() > fastest) fastest = lift.velocityCps();
    }

    TEST_ASSERT_TRUE(lift.fault() == Fault::none);
    TEST_ASSERT_INT32_WITHIN(2 * testConfig.deadbandCounts, target, lift.counts());
    TEST_ASSERT_LESS_OR_EQUAL_INT16(2, biggestRise); // Soft start: at least 150 ms from 0 to 255.
    TEST_ASSERT_LESS_OR_EQUAL_INT32(testConfig.maxSpeedCps * 11 / 10, fastest);
    TEST_ASSERT_GREATER_THAN_INT32(testConfig.maxSpeedCps / 2, fastest);

    // It stays put without hunting, for as long as the host keeps asking.
    for (uint16_t i = 0; i < 3000; i++) {
        if (i % 10 == 0) lift.move(target, 0);
        TEST_ASSERT_EQUAL_INT16(0, run(lift, plant));
    }
    TEST_ASSERT_TRUE(lift.fault() == Fault::none);
}

void softStartTakesAtLeast150Milliseconds() {
    Controller controller(testConfig);
    controller.start(0);
    uint16_t ms = 0;
    int16_t output = 0;
    while (output < 255 && ms < 1000) {
        output = controller.shape(255);
        if (output < 255) ms++;
    }
    TEST_ASSERT_GREATER_OR_EQUAL_UINT16(149, ms);
    TEST_ASSERT_LESS_OR_EQUAL_UINT16(160, ms);
    TEST_ASSERT_EQUAL_INT16(0, controller.shape(0)); // Dropping is immediate.
}

void aReversalWaitsWithTheMotorOff() {
    Lift lift(testConfig);
    Plant plant;
    TEST_ASSERT_TRUE(homeNow(lift, plant));
    TEST_ASSERT_TRUE(lift.enable() == Reject::none);
    moveTo(lift, plant, 20000, 30000);

    // Up at speed, then a target far below.
    uint16_t zeroRun = 0;
    int8_t direction = 0;
    uint16_t shortestGap = 65535;
    for (uint16_t i = 0; i < 6000; i++) {
        if (i % 10 == 0) lift.move(i < 1500 ? 30000 : 5000, 0);
        const int16_t pwm = run(lift, plant);
        if (pwm == 0) {
            zeroRun++;
        } else {
            const int8_t now = pwm > 0 ? 1 : -1;
            if (direction != 0 && now != direction && zeroRun < shortestGap) shortestGap = zeroRun;
            direction = now;
            zeroRun = 0;
        }
    }
    TEST_ASSERT_TRUE(lift.fault() == Fault::none);
    TEST_ASSERT_NOT_EQUAL(65535, shortestGap); // It did reverse.
    TEST_ASSERT_GREATER_OR_EQUAL_UINT16(49, shortestGap);
}

void aMovingTargetIsFollowedWithoutStopping() {
    Lift lift(testConfig);
    Plant plant;
    TEST_ASSERT_TRUE(homeNow(lift, plant));
    TEST_ASSERT_TRUE(lift.enable() == Reject::none);

    moveTo(lift, plant, 5000, 30000);

    // A host walking the target up at 2000 counts/s, a sample every 10 ms, with its speed.
    const int32_t speed = 2000;
    const int32_t start = lift.counts();
    uint16_t zeros = 0;
    int32_t worstLag = 0;
    for (uint16_t i = 0; i < 5000; i++) {
        const int32_t target = start + (int32_t)(i / 10) * 20;
        if (i % 10 == 0) TEST_ASSERT_EQUAL_UINT8(0, (uint8_t)lift.move(target, speed));
        const int16_t pwm = run(lift, plant);
        if (i > 1000) {
            if (pwm == 0) zeros++;
            if (distance(target, lift.counts()) > worstLag) worstLag = distance(target, lift.counts());
        }
    }
    TEST_ASSERT_TRUE(lift.fault() == Fault::none);
    TEST_ASSERT_EQUAL_UINT16(0, zeros);
    TEST_ASSERT_LESS_OR_EQUAL_INT32(300, worstLag); // 150 ms at this speed.
}

void movesOutsideTheAllowedRangeAreRefused() {
    Lift lift(testConfig);
    Plant plant;
    TEST_ASSERT_TRUE(homeNow(lift, plant));
    TEST_ASSERT_TRUE(lift.enable() == Reject::none);
    TEST_ASSERT_TRUE(lift.move(testConfig.clearanceCounts - 1, 0) == Reject::outOfRange);
    TEST_ASSERT_TRUE(lift.move(testConfig.travelCounts + 1, 0) == Reject::outOfRange);
    TEST_ASSERT_TRUE(lift.move(5000, testConfig.maxSpeedCps + 1) == Reject::outOfRange);
    TEST_ASSERT_TRUE(lift.move(testConfig.clearanceCounts, 0) == Reject::none);
}

void aRefusedMoveDoesNotFeedTheWatchdog() {
    Lift lift(testConfig);
    Plant plant;
    TEST_ASSERT_TRUE(homeNow(lift, plant));
    TEST_ASSERT_TRUE(lift.enable() == Reject::none);
    for (uint8_t i = 0; i < 101; i++) {
        lift.move(testConfig.travelCounts + 1, 0);
        run(lift, plant);
    }
    TEST_ASSERT_TRUE(lift.fault() == Fault::commandTimeout);
}

void disableCoastsAtOnceFromAnywhere() {
    Lift lift(testConfig);
    Plant plant;
    TEST_ASSERT_TRUE(homeNow(lift, plant));
    TEST_ASSERT_TRUE(lift.enable() == Reject::none);
    for (uint16_t i = 0; i < 1000; i++) {
        if (i % 10 == 0) lift.move(30000, 0);
        run(lift, plant);
    }
    TEST_ASSERT_NOT_EQUAL(0, lift.pwm());
    TEST_ASSERT_TRUE(lift.disable() == Reject::none);
    TEST_ASSERT_EQUAL_INT16(0, lift.pwm());
    TEST_ASSERT_EQUAL_INT16(0, run(lift, plant));
    TEST_ASSERT_TRUE(lift.homed());
}

void aTickFitsTheMillisecond() {
    Lift lift(testConfig);
    Plant plant;
    TEST_ASSERT_TRUE(homeNow(lift, plant));
    TEST_ASSERT_TRUE(lift.enable() == Reject::none);
    uint32_t worst = 0;
    for (uint16_t i = 0; i < 2000; i++) {
        if (i % 10 == 0) lift.move(1000 + (int32_t)i * 2, 2000); // Moving: the target and the setpoint both advance.
        const LiftInputs in = plant.read();
        const uint32_t started = micros();
        const int16_t pwm = lift.tick(in);
        const uint32_t took = micros() - started;
        if (took > worst) worst = took;
        plant.step(pwm);
    }
    // Of the 1000 us, two I2C reads at 400 kHz should take about 300 and the pins about 25 (calculated, not measured).
    // This limit leaves room for a serial frame in the same millisecond. On the board, late= counts any tick missed
    // outright.
    char message[32];
    snprintf(message, sizeof(message), "worst tick %lu us", (unsigned long)worst);
    TEST_MESSAGE(message);
    TEST_ASSERT_LESS_THAN_UINT32(500, worst);
}

void setup() {
    UNITY_BEGIN();
    RUN_TEST(encoderFollowsTheAngleAcrossTheWrap);
    RUN_TEST(aReversedEncoderCountsTheOtherWay);
    RUN_TEST(encoderMeasuresSpeedOverSixteenMilliseconds);
    RUN_TEST(aZeroMovesTheCountsButNotTheSpeed);
    RUN_TEST(nothingMovesAtPowerUp);
    RUN_TEST(enableIsRefusedUntilHomed);
    RUN_TEST(anUnmeasuredColumnRefusesHomeAndEnable);
    RUN_TEST(homingZeroesOnTheBottomSwitchAndStopsClearOfIt);
    RUN_TEST(homingStartingOnTheSwitchBacksOffFirst);
    RUN_TEST(homingStopsWhenTheHostGoesQuiet);
    RUN_TEST(homingGivesUpIfTheSwitchNeverOpens);
    RUN_TEST(homingStallsOnTheHardStopIfTheSwitchNeverOpens);
    RUN_TEST(enableMovesNothing);
    RUN_TEST(aMoveArrivesUnderItsCapsAndThenCoasts);
    RUN_TEST(softStartTakesAtLeast150Milliseconds);
    RUN_TEST(aReversalWaitsWithTheMotorOff);
    RUN_TEST(aMovingTargetIsFollowedWithoutStopping);
    RUN_TEST(movesOutsideTheAllowedRangeAreRefused);
    RUN_TEST(aRefusedMoveDoesNotFeedTheWatchdog);
    RUN_TEST(disableCoastsAtOnceFromAnywhere);
    RUN_TEST(aTickFitsTheMillisecond);
    UNITY_END();
    stopSimulator();
}

void loop() {}
