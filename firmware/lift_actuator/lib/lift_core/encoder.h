#pragma once

#include <stdint.h>

// Tracks whole turns from the AS5600's 12-bit angle, which it reads once per millisecond.
class Encoder {
  public:
    explicit Encoder(bool inverted) : inverted_(inverted) {}

    void update(bool readOk, uint16_t rawAngle);
    void setZeroHere() { zero_ = counts_; }

    int32_t counts() const { return counts_ - zero_; }
    int32_t velocityCps() const { return velocityCps_; }

    // Three failed reads in a row. Clears by itself on the next good read, but the turn count may be wrong by then.
    bool lost() const { return failures_ >= failureLimit; }

    // The angle moved further in one read than anything here can turn. Stays set until clearJump().
    bool jumped() const { return jumped_; }
    void clearJump() { jumped_ = false; }

  private:
    static const uint8_t failureLimit = 3;
    static const uint8_t historyLength = 16;

    bool inverted_;
    bool started_ = false;
    bool jumped_ = false;
    uint8_t failures_ = 0;
    uint8_t ticksSinceRead_ = 1;
    uint8_t historyIndex_ = 0;
    uint16_t lastAngle_ = 0;
    int32_t counts_ = 0;
    int32_t zero_ = 0;
    int32_t velocityCps_ = 0;
    int32_t history_[historyLength] = {};
};
