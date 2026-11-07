#pragma once

#include <stdint.h>

#include "lift.h"
#include "protocol.h"

uint16_t crc16(const uint8_t *data, uint8_t length, uint16_t crc = 0xFFFF);

// Finds frames in the serial byte stream. A bad CRC or an impossible length drops the frame and looks for the next.
class FrameParser {
  public:
    // True when this byte completes a good frame, which stays readable until the next push.
    bool push(uint8_t byte);

    uint8_t type() const { return type_; }
    uint8_t length() const { return length_; }
    const uint8_t *payload() const { return payload_; }
    uint8_t errors() const { return errors_; }

  private:
    enum class Step : uint8_t { sync0, sync1, type, length, payload, crcLow, crcHigh };

    void countError();

    Step step_ = Step::sync0;
    uint8_t type_ = 0;
    uint8_t length_ = 0;
    uint8_t received_ = 0;
    uint8_t crcLow_ = 0;
    uint8_t errors_ = 0;
    uint8_t payload_[protocol::maxPayload] = {};
};

// Runs the frame's command and writes the reply. `reply` has room for protocol::maxReply bytes; returns its length.
uint8_t handleFrame(const FrameParser &frame, Lift &lift, uint8_t *reply);
