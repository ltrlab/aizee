// The bring-up console: one-letter commands from a serial monitor, on the same guards and control as the real build.
// It measures what include/config.h needs. Any key stops whatever is moving. README.md has the order to do things in.
#ifdef LIFT_BENCH

#include <Arduino.h>
#include <stdlib.h>

#include "board.h"
#include "config.h"
#include "lift.h"

namespace {

Lift lift(liftConfig);

enum class Task : uint8_t { none, jog, home, move };
Task task = Task::none;

char line[24];
uint8_t lineLength = 0;
char lastChar = 0;
int32_t moveTarget = 0;
uint32_t moveMs = 0;
uint32_t lastPrintMs = 0;
bool watching = false;

// What a jog found. The first 300 ms are the soft start, so the speed comes from the rest.
const uint16_t jogSettleMs = 300;
const uint32_t moveLimitMs = 60000;
int16_t jogPwm = 0;
uint32_t jogMs = 0;
int32_t jogStart = 0;
int32_t jogSettled = 0;
int32_t jogCurrentSum = 0;
int16_t jogCurrentPeak = 0;
uint16_t jogLowestMillivolts = 0;

const __FlashStringHelper *faultName(Fault fault) {
    switch (fault) {
    case Fault::none: return F("none");
    case Fault::overcurrent: return F("overcurrent");
    case Fault::stall: return F("stall");
    case Fault::wrongDirection: return F("wrong direction: swap encoderInverted or motorInverted");
    case Fault::encoder: return F("encoder");
    case Fault::magnet: return F("magnet");
    case Fault::currentSensor: return F("current sensor");
    case Fault::limitTop: return F("top switch");
    case Fault::limitBottom: return F("bottom switch");
    case Fault::switchWiring: return F("both switches open");
    case Fault::undervoltage: return F("undervoltage");
    case Fault::noSupply: return F("no supply");
    case Fault::duty: return F("duty");
    case Fault::commandTimeout: return F("command timeout");
    case Fault::homeTimeout: return F("home timeout");
    case Fault::uncommandedMotion: return F("moved with the motor off");
    }
    return F("?");
}

const __FlashStringHelper *rejectName(Reject reject) {
    switch (reject) {
    case Reject::none: return F("ok");
    case Reject::badCommand: return F("bad command");
    case Reject::unmeasured: return F("config.h still has zeros");
    case Reject::faulted: return F("fault latched, c clears it");
    case Reject::notHomed: return F("not homed, h first");
    case Reject::notEnabled: return F("not enabled");
    case Reject::busy: return F("busy");
    case Reject::outOfRange: return F("out of range");
    case Reject::sensors: return F("encoder or INA219 not answering");
    case Reject::noSupply: return F("12 V low");
    case Reject::switches: return F("limit switch");
    case Reject::cooling: return F("motor cooling");
    }
    return F("?");
}

const __FlashStringHelper *modeName(Mode mode) {
    switch (mode) {
    case Mode::disabled: return F("disabled");
    case Mode::enabled: return F("enabled");
    case Mode::homing: return F("homing");
    case Mode::jog: return F("jog");
    }
    return F("?");
}

void printState() {
    const Guards &guards = lift.guards();
    Serial.print(modeName(lift.mode()));
    Serial.print(F(" counts="));
    Serial.print(lift.counts());
    Serial.print(F(" cps="));
    Serial.print(lift.velocityCps());
    Serial.print(F(" pwm="));
    Serial.print(lift.pwm());
    Serial.print(F(" mA="));
    Serial.print(guards.milliamps());
    Serial.print(F(" mV="));
    Serial.print(guards.millivolts());
    Serial.print(F(" top="));
    Serial.print(guards.atTop() ? F("open") : F("closed"));
    Serial.print(F(" bottom="));
    Serial.print(guards.atBottom() ? F("open") : F("closed"));
    Serial.print(F(" encoder="));
    Serial.print(lift.encoderOk() ? (guards.magnetMarginal() ? F("marginal") : F("ok")) : F("LOST"));
    Serial.print(F(" ina219="));
    Serial.print(guards.sensorOk() ? F("ok") : F("LOST"));
    Serial.print(F(" homed="));
    Serial.print(lift.homed() ? 1 : 0);
    Serial.print(F(" duty="));
    Serial.print(guards.dutyPercent());
    Serial.print(F("% late="));
    Serial.print(lift.lateTicks());
    Serial.print(F(" fault="));
    Serial.println(faultName(lift.fault()));
}

void printCommands() {
    Serial.println(F("u <pwm> [ms]   up, open loop (default 1000 ms, at most 2000)"));
    Serial.println(F("d <pwm> [ms]   down, open loop"));
    Serial.println(F("h              home on the bottom switch"));
    Serial.println(F("g <counts>     go there, closed loop, then disable"));
    Serial.println(F("c              clear a fault"));
    Serial.println(F("w              watch: print the state 5 times a second, or stop"));
    Serial.println(F("?              this, and the state"));
    Serial.println(F("any key stops whatever is moving"));
}

void printResult(Reject reject) {
    Serial.print(F("> "));
    Serial.println(rejectName(reject));
}

void startJog(int16_t pwm, uint16_t durationMs) {
    const Reject reject = lift.jog(pwm, durationMs);
    printResult(reject);
    if (reject != Reject::none) return;
    task = Task::jog;
    jogPwm = pwm;
    jogMs = 0;
    jogStart = lift.counts();
    jogSettled = jogStart;
    jogCurrentSum = 0;
    jogCurrentPeak = 0;
    jogLowestMillivolts = 0xFFFF;
}

// The line config.h's numbers come from.
void printJogResult() {
    Serial.print(F("jog pwm="));
    Serial.print(jogPwm);
    Serial.print(F(" ms="));
    Serial.print(jogMs);
    Serial.print(F(" moved="));
    Serial.print(lift.counts() - jogStart);
    if (jogMs > jogSettleMs) {
        Serial.print(F(" cps="));
        Serial.print((lift.counts() - jogSettled) * 1000 / (int32_t)(jogMs - jogSettleMs));
    }
    if (jogMs > 0) {
        Serial.print(F(" meanMa="));
        Serial.print(jogCurrentSum / (int32_t)jogMs);
    }
    Serial.print(F(" peakMa="));
    Serial.print(jogCurrentPeak);
    Serial.print(F(" lowestMv="));
    Serial.print(jogLowestMillivolts);
    Serial.print(F(" fault="));
    Serial.println(faultName(lift.fault()));
}

void endTask() {
    const Task finished = task;
    task = Task::none;
    if (finished == Task::jog) printJogResult();
    lift.disable();
    printState();
}

void runCommand() {
    line[lineLength] = 0;
    lineLength = 0;
    char *end = nullptr;
    const long first = strtol(line + 1, &end, 10);
    const long second = strtol(end, nullptr, 10);
    const uint16_t durationMs = second > 0 ? (uint16_t)min(second, 2000L) : 1000;

    switch (line[0]) {
    case 0: break;
    case 'u': startJog((int16_t)first, durationMs); break;
    case 'd': startJog((int16_t)-first, durationMs); break;
    case 'h': {
        const Reject reject = lift.home();
        printResult(reject);
        if (reject == Reject::none) task = Task::home;
        break;
    }
    case 'g': {
        Reject reject = lift.enable();
        moveTarget = first;
        moveMs = 0;
        if (reject == Reject::none) reject = lift.move(moveTarget, 0);
        printResult(reject);
        if (reject == Reject::none) task = Task::move;
        else lift.disable();
        break;
    }
    case 'c': printResult(lift.clear()); printState(); break;
    case 'w': watching = !watching; break;
    case '?':
        printCommands();
        printState();
        break;
    default: printResult(Reject::badCommand); break;
    }
}

void readConsole() {
    while (Serial.available() > 0) {
        const char c = (char)Serial.read();
        // A monitor's Enter can be CR LF: one key, and the LF must not stop what the CR just started.
        const bool secondHalfOfEnter = c == '\n' && lastChar == '\r';
        lastChar = c;
        if (secondHalfOfEnter) continue;
        if (task != Task::none) {
            Serial.println(F("stopped"));
            endTask();
        } else if (c == '\n' || c == '\r') {
            runCommand();
        } else if (lineLength < sizeof(line) - 1) {
            line[lineLength++] = c;
        }
    }
}

void runTask() {
    switch (task) {
    case Task::none:
        break;
    case Task::jog: {
        const Guards &guards = lift.guards();
        jogMs++;
        if (jogMs == jogSettleMs) jogSettled = lift.counts();
        jogCurrentSum += guards.milliamps();
        if (guards.milliamps() > jogCurrentPeak) jogCurrentPeak = guards.milliamps();
        if (guards.millivolts() < jogLowestMillivolts) jogLowestMillivolts = guards.millivolts();
        if (lift.mode() != Mode::jog) endTask();
        break;
    }
    case Task::home:
        lift.heard();
        if (lift.mode() != Mode::homing) endTask();
        break;
    case Task::move:
        moveMs++;
        if (lift.mode() != Mode::enabled || moveMs > moveLimitMs || (moveMs > 10 && lift.atRest())) {
            endTask();
        } else {
            lift.move(moveTarget, 0);
        }
        break;
    }
}

} // namespace

void setup() {
    board::begin(liftConfig);
    Serial.begin(1000000);
    Serial.println();
    Serial.println(F("lift bench"));
    printCommands();
}

void loop() {
    readConsole();

    bool late = false;
    if (board::tickDue(late)) {
        if (late) lift.countLateTick();
        board::drive(lift.tick(board::read()));
        board::feedWatchdog();
        runTask();
        if (watching && lift.boardMs() - lastPrintMs >= 200) {
            lastPrintMs = lift.boardMs();
            printState();
        }
    }
}

#endif
