#include <Arduino.h>
#include <unity.h>

#include "../plant.h"

void setUp() {}
void tearDown() {}

namespace {

// A homed, enabled lift moving up at speed, the way most trips find it.
void runningUp(Lift &lift, Plant &plant) {
    TEST_ASSERT_TRUE(homeNow(lift, plant));
    TEST_ASSERT_TRUE(lift.enable() == Reject::none);
    for (uint16_t i = 0; i < 1000; i++) {
        if (i % 10 == 0) lift.move(testConfig.travelCounts, 0);
        run(lift, plant);
    }
    TEST_ASSERT_TRUE(lift.pwm() > 0);
    TEST_ASSERT_TRUE(lift.fault() == Fault::none);
}

// Keeps commanding until something trips. Returns the milliseconds it took.
uint16_t timeToTrip(Lift &lift, Plant &plant, uint16_t limitMs) {
    for (uint16_t i = 0; i < limitMs; i++) {
        lift.move(testConfig.travelCounts, 0);
        run(lift, plant);
        if (lift.fault() != Fault::none) return i + 1;
    }
    return limitMs;
}

void assertTripped(Lift &lift, Plant &plant, Fault fault) {
    TEST_ASSERT_EQUAL_UINT8((uint8_t)fault, (uint8_t)lift.fault());
    TEST_ASSERT_TRUE(lift.mode() == Mode::disabled);
    TEST_ASSERT_EQUAL_INT16(0, lift.pwm());
    TEST_ASSERT_EQUAL_INT16(0, run(lift, plant));
}

} // namespace

void sustainedOvercurrentTripsInTwentyMilliseconds() {
    Lift lift(testConfig);
    Plant plant;
    runningUp(lift, plant);
    plant.extraMilliamps = 2430; // About 3.1 A in all: over the trip, under the shunt's ceiling.
    const uint16_t ms = timeToTrip(lift, plant, 200);
    assertTripped(lift, plant, Fault::overcurrent);
    TEST_ASSERT_INT_WITHIN(6, 28, ms); // The 8 ms average has to climb past the trip first.
}

void aBriefOvercurrentDoesNotTrip() {
    Lift lift(testConfig);
    Plant plant;
    runningUp(lift, plant);
    plant.extraMilliamps = 2430;
    timeToTrip(lift, plant, 12);
    plant.extraMilliamps = 0;
    timeToTrip(lift, plant, 500);
    TEST_ASSERT_TRUE(lift.fault() == Fault::none);
}

void aPinnedShuntTripsOnTheNextReading() {
    Lift lift(testConfig);
    Plant plant;
    runningUp(lift, plant);
    plant.extraMilliamps = 3300;
    const uint16_t ms = timeToTrip(lift, plant, 200);
    assertTripped(lift, plant, Fault::overcurrent);
    TEST_ASSERT_LESS_OR_EQUAL_UINT16(2, ms);
}

void aJamTripsAsAStallInThreeHundredMilliseconds() {
    Lift lift(testConfig);
    Plant plant;
    runningUp(lift, plant);
    plant.jammed = true;
    const uint16_t ms = timeToTrip(lift, plant, 1000);
    assertTripped(lift, plant, Fault::stall);
    TEST_ASSERT_INT_WITHIN(20, 300, ms);
}

void aSwappedMotorTripsAsWrongDirection() {
    Lift lift(testConfig);
    Plant plant;
    TEST_ASSERT_TRUE(homeNow(lift, plant));
    TEST_ASSERT_TRUE(lift.enable() == Reject::none);
    plant.motorReversed = true;
    const uint16_t ms = timeToTrip(lift, plant, 2000);
    assertTripped(lift, plant, Fault::wrongDirection);
    TEST_ASSERT_LESS_OR_EQUAL_UINT16(800, ms);
}

void threeFailedAngleReadsTripAndLoseHoming() {
    Lift lift(testConfig);
    Plant plant;
    runningUp(lift, plant);
    plant.angleAnswers = false;
    const uint16_t ms = timeToTrip(lift, plant, 200);
    assertTripped(lift, plant, Fault::encoder);
    TEST_ASSERT_EQUAL_UINT16(3, ms);
    TEST_ASSERT_FALSE(lift.homed());
}

void twoFailedAngleReadsAreForgiven() {
    Lift lift(testConfig);
    Plant plant;
    runningUp(lift, plant);
    plant.angleAnswers = false;
    timeToTrip(lift, plant, 2);
    plant.angleAnswers = true;
    timeToTrip(lift, plant, 200);
    TEST_ASSERT_TRUE(lift.fault() == Fault::none);
    TEST_ASSERT_TRUE(lift.homed());
}

void anImpossibleJumpTripsEvenWhileDisabled() {
    Lift lift(testConfig);
    Plant plant;
    TEST_ASSERT_TRUE(homeNow(lift, plant));
    plant.position += 1500;
    run(lift, plant);
    assertTripped(lift, plant, Fault::encoder);
    TEST_ASSERT_FALSE(lift.homed());
    TEST_ASSERT_TRUE(lift.clear() == Reject::none);
    runFor(lift, plant, 10);
    TEST_ASSERT_TRUE(lift.fault() == Fault::none);
}

void aLostMagnetTripsAndLosesHoming() {
    Lift lift(testConfig);
    Plant plant;
    runningUp(lift, plant);
    plant.magnetPresent = false;
    const uint16_t ms = timeToTrip(lift, plant, 200);
    assertTripped(lift, plant, Fault::magnet);
    TEST_ASSERT_LESS_OR_EQUAL_UINT16(60, ms);
    TEST_ASSERT_FALSE(lift.homed());
}

void aSilentIna219TripsWhileMoving() {
    Lift lift(testConfig);
    Plant plant;
    runningUp(lift, plant);
    plant.shuntAnswers = false;
    const uint16_t ms = timeToTrip(lift, plant, 200);
    assertTripped(lift, plant, Fault::currentSensor);
    TEST_ASSERT_LESS_OR_EQUAL_UINT16(6, ms);
    TEST_ASSERT_TRUE(lift.homed());
}

void aSilentIna219RefusesToStartButDoesNotLatchAtRest() {
    Lift lift(testConfig);
    Plant plant;
    plant.shuntAnswers = false;
    runFor(lift, plant, 100);
    TEST_ASSERT_TRUE(lift.fault() == Fault::none);
    TEST_ASSERT_TRUE(lift.home() == Reject::sensors);
}

void theTopSwitchStopsUpButNotDown() {
    Lift lift(testConfig);
    Plant plant;
    runningUp(lift, plant);
    plant.topBroken = true;
    const uint16_t ms = timeToTrip(lift, plant, 200);
    assertTripped(lift, plant, Fault::limitTop);
    TEST_ASSERT_LESS_OR_EQUAL_UINT16(2, ms);
    TEST_ASSERT_TRUE(lift.homed());

    // Cleared, it may only leave.
    TEST_ASSERT_TRUE(lift.clear() == Reject::none);
    TEST_ASSERT_TRUE(lift.enable() == Reject::none);
    for (uint16_t i = 0; i < 500; i++) {
        lift.move(testConfig.clearanceCounts, 0);
        run(lift, plant);
    }
    TEST_ASSERT_TRUE(lift.fault() == Fault::none);
    TEST_ASSERT_TRUE(lift.pwm() < 0);
}

void theBottomSwitchTripsOutsideHoming() {
    Lift lift(testConfig);
    Plant plant;
    TEST_ASSERT_TRUE(homeNow(lift, plant));
    TEST_ASSERT_TRUE(lift.enable() == Reject::none);
    moveTo(lift, plant, 8000, 20000);
    for (uint16_t i = 0; i < 300; i++) {
        lift.move(testConfig.clearanceCounts, 0);
        run(lift, plant);
    }
    TEST_ASSERT_TRUE(lift.pwm() < 0);
    plant.bottomBroken = true;
    for (uint8_t i = 0; i < 4; i++) {
        lift.move(testConfig.clearanceCounts, 0);
        run(lift, plant);
    }
    assertTripped(lift, plant, Fault::limitBottom);
}

void bothSwitchesOpenStopsEverything() {
    Lift lift(testConfig);
    Plant plant;
    runningUp(lift, plant);
    plant.topBroken = true;
    plant.bottomBroken = true;
    timeToTrip(lift, plant, 20);
    assertTripped(lift, plant, Fault::switchWiring);
    TEST_ASSERT_TRUE(lift.clear() == Reject::none);
    run(lift, plant);
    TEST_ASSERT_TRUE(lift.enable() == Reject::switches);
    TEST_ASSERT_TRUE(lift.home() == Reject::switches);
}

void undervoltageTripsInSixtyMilliseconds() {
    Lift lift(testConfig);
    Plant plant;
    runningUp(lift, plant);
    plant.busMillivolts = 10000;
    const uint16_t ms = timeToTrip(lift, plant, 200);
    assertTripped(lift, plant, Fault::undervoltage);
    TEST_ASSERT_LESS_OR_EQUAL_UINT16(60, ms);
}

void theKillSwitchReadsAsNoSupply() {
    Lift lift(testConfig);
    Plant plant;
    runningUp(lift, plant);
    plant.busMillivolts = 200;
    timeToTrip(lift, plant, 200);
    assertTripped(lift, plant, Fault::noSupply);
    TEST_ASSERT_TRUE(lift.clear() == Reject::none);
    TEST_ASSERT_TRUE(lift.enable() == Reject::noSupply);
}

void aLowSupplyAtRestRefusesWithoutLatching() {
    Lift lift(testConfig);
    Plant plant;
    plant.busMillivolts = 200;
    runFor(lift, plant, 200);
    TEST_ASSERT_TRUE(lift.fault() == Fault::none);
    TEST_ASSERT_TRUE(lift.home() == Reject::noSupply);
}

void theDutyAllowanceTripsAfterTwoMinutesOfRunning() {
    Guards guards(testConfig);
    Encoder encoder(false);
    LiftInputs in;
    in.topOpen = false;
    in.bottomOpen = false;
    uint16_t angle = 0;
    uint32_t ms = 0;
    Fault fault = Fault::none;
    while (fault == Fault::none && ms < 130000UL) {
        angle = (angle + 2) & 4095;
        encoder.update(true, angle);
        fault = guards.update(in, encoder, 200, true, false);
        ms++;
    }
    TEST_ASSERT_EQUAL_UINT8((uint8_t)Fault::duty, (uint8_t)fault);
    TEST_ASSERT_UINT32_WITHIN(50, 120000UL, ms);
    TEST_ASSERT_EQUAL_UINT8(100, guards.dutyPercent());
}

void theDutyAllowanceDrainsAtANinthOfTheRate() {
    Guards guards(testConfig);
    Encoder encoder(false);
    LiftInputs in;
    in.topOpen = false;
    in.bottomOpen = false;
    uint16_t angle = 0;
    for (uint16_t ms = 0; ms < 12000; ms++) {
        angle = (angle + 2) & 4095;
        encoder.update(true, angle);
        guards.update(in, encoder, 200, true, false);
    }
    TEST_ASSERT_EQUAL_UINT8(10, guards.dutyPercent());
    // Resting takes it back at a ninth of the rate it filled.
    for (uint16_t ms = 0; ms < 10800; ms++) {
        encoder.update(true, angle);
        guards.update(in, encoder, 0, false, false);
    }
    TEST_ASSERT_EQUAL_UINT8(9, guards.dutyPercent());
}

void aHostGoingQuietStopsTheColumnInAHundredMilliseconds() {
    Lift lift(testConfig);
    Plant plant;
    runningUp(lift, plant);
    uint16_t ms = 0;
    while (lift.fault() == Fault::none && ms < 500) {
        run(lift, plant);
        ms++;
    }
    assertTripped(lift, plant, Fault::commandTimeout);
    TEST_ASSERT_INT_WITHIN(10, 100, ms);
    TEST_ASSERT_TRUE(lift.homed());
}

void motionWithTheMotorOffLatches() {
    Lift lift(testConfig);
    Plant plant;
    TEST_ASSERT_TRUE(homeNow(lift, plant));
    runFor(lift, plant, 400);
    TEST_ASSERT_TRUE(lift.fault() == Fault::none);
    plant.driftPerMs = 1;
    runFor(lift, plant, 100);
    TEST_ASSERT_TRUE(lift.fault() == Fault::uncommandedMotion);
    TEST_ASSERT_TRUE(lift.enable() == Reject::faulted);
}

void aFaultStaysUntilClearedWhileDisabled() {
    Lift lift(testConfig);
    Plant plant;
    runningUp(lift, plant);
    plant.jammed = true;
    timeToTrip(lift, plant, 1000);
    TEST_ASSERT_TRUE(lift.fault() == Fault::stall);
    plant.jammed = false;

    runFor(lift, plant, 3000);
    TEST_ASSERT_TRUE(lift.fault() == Fault::stall);
    TEST_ASSERT_TRUE(lift.enable() == Reject::faulted);
    TEST_ASSERT_TRUE(lift.home() == Reject::faulted);

    TEST_ASSERT_TRUE(lift.clear() == Reject::none);
    TEST_ASSERT_TRUE(lift.fault() == Fault::none);
    TEST_ASSERT_TRUE(lift.enable() == Reject::none);
    TEST_ASSERT_TRUE(lift.clear() == Reject::busy);
}

void theFirstFaultIsTheOneReported() {
    Lift lift(testConfig);
    Plant plant;
    runningUp(lift, plant);
    plant.jammed = true;
    timeToTrip(lift, plant, 1000);
    plant.jammed = false;
    plant.extraMilliamps = 3300;
    runFor(lift, plant, 20);
    TEST_ASSERT_TRUE(lift.fault() == Fault::stall);
}

void aJogRunsItsTimeAndEnds() {
    Lift lift(testConfig);
    Plant plant;
    wake(lift, plant);
    TEST_ASSERT_TRUE(lift.jog(200, 500) == Reject::none);
    uint16_t driven = 0;
    for (uint16_t i = 0; i < 1000; i++) {
        if (run(lift, plant) != 0) driven++;
    }
    TEST_ASSERT_INT_WITHIN(5, 500, driven);
    TEST_ASSERT_TRUE(lift.mode() == Mode::disabled);
    TEST_ASSERT_TRUE(lift.fault() == Fault::none);
}

void aJogIsBoundedAndGuarded() {
    Lift lift(testConfig);
    Plant plant;
    wake(lift, plant);
    TEST_ASSERT_TRUE(lift.jog(200, 2001) == Reject::outOfRange);
    TEST_ASSERT_TRUE(lift.jog(256, 100) == Reject::outOfRange);
    TEST_ASSERT_TRUE(lift.jog(0, 100) == Reject::outOfRange);

    // Too little PWM to move is allowed to sit, not to stall.
    TEST_ASSERT_TRUE(lift.jog(-50, 2000) == Reject::none);
    runFor(lift, plant, 1000);
    TEST_ASSERT_TRUE(lift.fault() == Fault::none);
    lift.disable();

    plant.position = 900; // Inside the bottom switch.
    runFor(lift, plant, 5);
    lift.clear();
    TEST_ASSERT_TRUE(lift.jog(-200, 100) == Reject::switches);
}

void anUnmeasuredColumnCanStillJog() {
    LiftConfig unmeasured = testConfig;
    unmeasured.travelCounts = 0;
    unmeasured.maxSpeedCps = 0;
    unmeasured.stictionPwmUp = 0;
    Lift lift(unmeasured);
    Plant plant;
    wake(lift, plant);
    TEST_ASSERT_TRUE(lift.jog(200, 100) == Reject::none);
    runFor(lift, plant, 200);
    TEST_ASSERT_TRUE(lift.fault() == Fault::none);
}

void setup() {
    UNITY_BEGIN();
    RUN_TEST(sustainedOvercurrentTripsInTwentyMilliseconds);
    RUN_TEST(aBriefOvercurrentDoesNotTrip);
    RUN_TEST(aPinnedShuntTripsOnTheNextReading);
    RUN_TEST(aJamTripsAsAStallInThreeHundredMilliseconds);
    RUN_TEST(aSwappedMotorTripsAsWrongDirection);
    RUN_TEST(threeFailedAngleReadsTripAndLoseHoming);
    RUN_TEST(twoFailedAngleReadsAreForgiven);
    RUN_TEST(anImpossibleJumpTripsEvenWhileDisabled);
    RUN_TEST(aLostMagnetTripsAndLosesHoming);
    RUN_TEST(aSilentIna219TripsWhileMoving);
    RUN_TEST(aSilentIna219RefusesToStartButDoesNotLatchAtRest);
    RUN_TEST(theTopSwitchStopsUpButNotDown);
    RUN_TEST(theBottomSwitchTripsOutsideHoming);
    RUN_TEST(bothSwitchesOpenStopsEverything);
    RUN_TEST(undervoltageTripsInSixtyMilliseconds);
    RUN_TEST(theKillSwitchReadsAsNoSupply);
    RUN_TEST(aLowSupplyAtRestRefusesWithoutLatching);
    RUN_TEST(theDutyAllowanceTripsAfterTwoMinutesOfRunning);
    RUN_TEST(theDutyAllowanceDrainsAtANinthOfTheRate);
    RUN_TEST(aHostGoingQuietStopsTheColumnInAHundredMilliseconds);
    RUN_TEST(motionWithTheMotorOffLatches);
    RUN_TEST(aFaultStaysUntilClearedWhileDisabled);
    RUN_TEST(theFirstFaultIsTheOneReported);
    RUN_TEST(aJogRunsItsTimeAndEnds);
    RUN_TEST(aJogIsBoundedAndGuarded);
    RUN_TEST(anUnmeasuredColumnCanStillJog);
    UNITY_END();
    stopSimulator();
}

void loop() {}
