#include "board.h"

#include <Arduino.h>
#include <Wire.h>
#include <avr/wdt.h>

namespace {

const uint8_t pinEna = 3; // OC2B. A 10 k pull-down holds it low until this code runs.
const uint8_t pinIn1 = 4;
const uint8_t pinIn2 = 7;
const uint8_t pinTopSwitch = 8;
const uint8_t pinBottomSwitch = 9;

const uint8_t as5600 = 0x36;
const uint8_t as5600Status = 0x0B;
const uint8_t as5600RawAngle = 0x0C;
const uint8_t as5600MagnetDetected = 1 << 5;
const uint8_t as5600MagnetMarginal = (1 << 4) | (1 << 3); // Too weak, too strong.

const uint8_t ina219 = 0x40;
const uint8_t ina219Config = 0x00;
const uint8_t ina219Shunt = 0x01;
const uint8_t ina219Bus = 0x02;
// 32 V range, +-320 mV shunt, 9-bit bus (84 us), 12-bit shunt (532 us, about two PWM periods), both continuous.
const uint16_t ina219Setting = 0x381F;

const uint32_t i2cClockHz = 400000;
const uint32_t i2cTimeoutUs = 1000;

bool motorInverted = false;
uint32_t lastTickUs = 0;
uint8_t slot = 0;

bool readBytes(uint8_t address, uint8_t reg, uint8_t *bytes, uint8_t count) {
    Wire.beginTransmission(address);
    Wire.write(reg);
    if (Wire.endTransmission() != 0) return false;
    if (Wire.requestFrom(address, count) != count) return false;
    for (uint8_t i = 0; i < count; i++) bytes[i] = (uint8_t)Wire.read();
    return true;
}

bool readRegister(uint8_t address, uint8_t reg, uint16_t &value) {
    uint8_t bytes[2];
    if (!readBytes(address, reg, bytes, 2)) return false;
    value = ((uint16_t)bytes[0] << 8) | bytes[1];
    return true;
}

} // namespace

namespace board {

void begin(const LiftConfig &config) {
    wdt_disable();
    motorInverted = config.motorInverted;

    digitalWrite(pinIn1, LOW);
    digitalWrite(pinIn2, LOW);
    pinMode(pinIn1, OUTPUT);
    pinMode(pinIn2, OUTPUT);

    // Timer 2, phase-correct 8-bit, clk/8: 3.9 kHz on D3, inside the DROK's 10 kHz. A duty of 0 stays low.
    OCR2B = 0;
    TCCR2A = _BV(COM2B1) | _BV(WGM20);
    TCCR2B = _BV(CS21);
    pinMode(pinEna, OUTPUT);

    // Normally closed to ground, so an open switch and a broken wire both read high.
    pinMode(pinTopSwitch, INPUT_PULLUP);
    pinMode(pinBottomSwitch, INPUT_PULLUP);

    Wire.begin();
    Wire.setClock(i2cClockHz);
    Wire.setWireTimeout(i2cTimeoutUs, true);

    Wire.beginTransmission(ina219);
    Wire.write(ina219Config);
    Wire.write((uint8_t)(ina219Setting >> 8));
    Wire.write((uint8_t)ina219Setting);
    Wire.endTransmission();

    lastTickUs = micros();
    wdt_enable(WDTO_60MS);
}

bool tickDue(bool &late) {
    const uint32_t now = micros();
    if (now - lastTickUs < 1000) return false;
    lastTickUs += 1000;
    late = now - lastTickUs >= 1000;
    if (late) lastTickUs = now;
    return true;
}

LiftInputs read() {
    LiftInputs in;
    uint16_t value = 0;

    if (readRegister(as5600, as5600RawAngle, value)) {
        in.angle = Reading::good;
        in.rawAngle = value;
    } else {
        in.angle = Reading::failed;
    }

    slot = (slot + 1) % 20;
    if (slot % 2 == 1) {
        if (readRegister(ina219, ina219Shunt, value)) {
            in.shunt = Reading::good;
            in.shuntRaw = (int16_t)value;
        } else {
            in.shunt = Reading::failed;
        }
    } else if (slot == 0) {
        if (readRegister(ina219, ina219Bus, value)) {
            in.bus = Reading::good;
            in.busMillivolts = (value >> 3) * 4; // Bits 15..3, 4 mV each.
        } else {
            in.bus = Reading::failed;
        }
    } else if (slot == 10) {
        uint8_t status = 0;
        if (readBytes(as5600, as5600Status, &status, 1)) {
            in.magnet = Reading::good;
            in.magnetPresent = (status & as5600MagnetDetected) != 0;
            in.magnetMarginal = (status & as5600MagnetMarginal) != 0;
        } else {
            in.magnet = Reading::failed;
        }
    }

    in.topOpen = digitalRead(pinTopSwitch) == HIGH;
    in.bottomOpen = digitalRead(pinBottomSwitch) == HIGH;
    return in;
}

void drive(int16_t pwm) {
    if (pwm == 0) {
        OCR2B = 0;
        digitalWrite(pinIn1, LOW);
        digitalWrite(pinIn2, LOW);
        return;
    }
    const bool up = pwm > 0;
    digitalWrite(pinIn1, up != motorInverted ? HIGH : LOW);
    digitalWrite(pinIn2, up != motorInverted ? LOW : HIGH);
    const int16_t size = up ? pwm : -pwm;
    OCR2B = size > 255 ? 255 : (uint8_t)size;
}

void feedWatchdog() { wdt_reset(); }

} // namespace board
