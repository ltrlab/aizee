#include "link.h"

namespace {

uint8_t *putU8(uint8_t *out, uint8_t value) {
    *out++ = value;
    return out;
}

uint8_t *putU16(uint8_t *out, uint16_t value) {
    *out++ = (uint8_t)value;
    *out++ = (uint8_t)(value >> 8);
    return out;
}

uint8_t *putU32(uint8_t *out, uint32_t value) {
    out = putU16(out, (uint16_t)value);
    return putU16(out, (uint16_t)(value >> 16));
}

uint16_t getU16(const uint8_t *in) { return (uint16_t)in[0] | ((uint16_t)in[1] << 8); }

int32_t getI32(const uint8_t *in) { return (int32_t)((uint32_t)getU16(in) | ((uint32_t)getU16(in + 2) << 16)); }

uint8_t finishFrame(uint8_t *reply, protocol::Reply type, uint8_t length) {
    reply[0] = protocol::sync0;
    reply[1] = protocol::sync1;
    reply[2] = (uint8_t)type;
    reply[3] = length;
    putU16(reply + 4 + length, crc16(reply + 2, length + 2));
    return length + protocol::frameOverhead;
}

uint8_t writeIdentity(uint8_t *reply, const Lift &lift, uint16_t sequence) {
    const LiftConfig &config = lift.config();
    uint8_t *out = reply + 4;
    out = putU16(out, sequence);
    out = putU8(out, 'L');
    out = putU8(out, 'I');
    out = putU8(out, 'F');
    out = putU8(out, 'T');
    out = putU16(out, protocol::version);
    out = putU16(out, protocol::firmwareVersion);
    out = putU16(out, crc16((const uint8_t *)&config, sizeof(LiftConfig)));
    out = putU32(out, (uint32_t)config.travelCounts);
    out = putU32(out, (uint32_t)config.clearanceCounts);
    out = putU32(out, (uint32_t)config.maxSpeedCps);
    putU8(out, lift.measured() ? 1 : 0);
    return finishFrame(reply, protocol::Reply::identity, protocol::identityLength);
}

uint8_t writeState(uint8_t *reply, const Lift &lift, uint16_t sequence, Reject reject, uint8_t linkErrors) {
    const Guards &guards = lift.guards();
    uint16_t flags = 0;
    if (lift.homed()) flags |= protocol::flagHomed;
    if (lift.encoderOk()) flags |= protocol::flagEncoderOk;
    if (guards.magnetMarginal()) flags |= protocol::flagMagnetMarginal;
    if (guards.sensorOk()) flags |= protocol::flagSensorOk;
    if (guards.atTop()) flags |= protocol::flagAtTop;
    if (guards.atBottom()) flags |= protocol::flagAtBottom;
    if (lift.measured()) flags |= protocol::flagMeasured;
    if (lift.atRest()) flags |= protocol::flagAtRest;

    uint8_t *out = reply + 4;
    out = putU16(out, sequence);
    out = putU32(out, lift.boardMs());
    out = putU32(out, (uint32_t)lift.counts());
    out = putU32(out, (uint32_t)lift.velocityCps());
    out = putU16(out, (uint16_t)guards.milliamps());
    out = putU16(out, guards.millivolts());
    out = putU16(out, (uint16_t)lift.pwm());
    out = putU16(out, flags);
    out = putU8(out, (uint8_t)lift.mode());
    out = putU8(out, (uint8_t)lift.fault());
    out = putU8(out, (uint8_t)reject);
    out = putU8(out, guards.dutyPercent());
    out = putU8(out, linkErrors);
    putU8(out, lift.lateTicks());
    return finishFrame(reply, protocol::Reply::state, protocol::stateLength);
}

} // namespace

uint16_t crc16(const uint8_t *data, uint8_t length, uint16_t crc) {
    for (uint8_t i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t bit = 0; bit < 8; bit++) {
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

void FrameParser::countError() {
    if (errors_ < 255) errors_++;
}

bool FrameParser::push(uint8_t byte) {
    switch (step_) {
    case Step::sync0:
        if (byte == protocol::sync0) step_ = Step::sync1;
        break;
    case Step::sync1:
        if (byte != protocol::sync0) step_ = byte == protocol::sync1 ? Step::type : Step::sync0;
        break;
    case Step::type:
        type_ = byte;
        step_ = Step::length;
        break;
    case Step::length:
        if (byte > protocol::maxPayload) {
            countError();
            step_ = Step::sync0;
        } else {
            length_ = byte;
            received_ = 0;
            step_ = byte == 0 ? Step::crcLow : Step::payload;
        }
        break;
    case Step::payload:
        payload_[received_++] = byte;
        if (received_ == length_) step_ = Step::crcLow;
        break;
    case Step::crcLow:
        crcLow_ = byte;
        step_ = Step::crcHigh;
        break;
    case Step::crcHigh: {
        step_ = Step::sync0;
        const uint8_t header[2] = {type_, length_};
        const uint16_t expected = crc16(payload_, length_, crc16(header, 2));
        if (expected == (uint16_t)(crcLow_ | ((uint16_t)byte << 8))) return true;
        countError();
        break;
    }
    }
    return false;
}

uint8_t handleFrame(const FrameParser &frame, Lift &lift, uint8_t *reply) {
    const uint8_t *payload = frame.payload();
    const uint8_t length = frame.length();
    const uint16_t sequence = length >= 2 ? getU16(payload) : 0;
    const auto command = (protocol::Command)frame.type();

    lift.heard();

    Reject reject = Reject::badCommand;
    if (command == protocol::Command::move) {
        if (length == 10) reject = lift.move(getI32(payload + 2), getI32(payload + 6));
    } else if (length == 2) {
        switch (command) {
        case protocol::Command::identify:
            return writeIdentity(reply, lift, sequence);
        case protocol::Command::state:
            reject = Reject::none;
            break;
        case protocol::Command::enable:
            reject = lift.enable();
            break;
        case protocol::Command::disable:
            reject = lift.disable();
            break;
        case protocol::Command::home:
            reject = lift.home();
            break;
        case protocol::Command::clear:
            reject = lift.clear();
            break;
        default:
            break;
        }
    }
    return writeState(reply, lift, sequence, reject, frame.errors());
}
