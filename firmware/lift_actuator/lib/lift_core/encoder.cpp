#include "encoder.h"

namespace {

const int16_t countsPerTurn = 4096;
const int16_t halfTurn = 2048;
// A quarter turn in a millisecond is 15,000 rpm at the magnet.
const int16_t maxCountsPerTick = 1024;

} // namespace

void Encoder::update(bool readOk, uint16_t rawAngle) {
    if (readOk && rawAngle < (uint16_t)countsPerTurn) {
        failures_ = 0;
        if (started_) {
            int16_t delta = (int16_t)rawAngle - (int16_t)lastAngle_;
            if (delta > halfTurn) delta -= countsPerTurn;
            if (delta < -halfTurn) delta += countsPerTurn;
            if (inverted_) delta = -delta;
            const int16_t size = delta < 0 ? -delta : delta;
            if ((int32_t)size > (int32_t)maxCountsPerTick * ticksSinceRead_) jumped_ = true;
            counts_ += delta;
        }
        started_ = true;
        lastAngle_ = rawAngle;
        ticksSinceRead_ = 1;
    } else {
        if (failures_ < failureLimit) failures_++;
        if (ticksSinceRead_ < 255) ticksSinceRead_++;
    }

    // Counts over the last 16 ms: 1000 / 16 = 125 / 2.
    velocityCps_ = (counts_ - history_[historyIndex_]) * 125 / 2;
    history_[historyIndex_] = counts_;
    historyIndex_ = (historyIndex_ + 1) % historyLength;
}
