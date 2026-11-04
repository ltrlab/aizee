#include <Arduino.h>
#include <unity.h>

#include "../plant.h"
#include "link.h"

void setUp() {}
void tearDown() {}

namespace {

uint8_t buildFrame(uint8_t *out, uint8_t type, const uint8_t *payload, uint8_t length) {
    out[0] = protocol::sync0;
    out[1] = protocol::sync1;
    out[2] = type;
    out[3] = length;
    for (uint8_t i = 0; i < length; i++) out[4 + i] = payload[i];
    const uint16_t crc = crc16(out + 2, length + 2);
    out[4 + length] = (uint8_t)crc;
    out[5 + length] = (uint8_t)(crc >> 8);
    return length + 6;
}

bool feed(FrameParser &parser, const uint8_t *bytes, uint8_t count) {
    bool complete = false;
    for (uint8_t i = 0; i < count; i++) complete = parser.push(bytes[i]);
    return complete;
}

int32_t readI32(const uint8_t *p) {
    return (int32_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24));
}

uint16_t readU16(const uint8_t *p) { return (uint16_t)p[0] | ((uint16_t)p[1] << 8); }

// Sends one command through the parser and returns the reply's length.
uint8_t exchange(Lift &lift, uint8_t type, const uint8_t *payload, uint8_t length, uint8_t *reply) {
    FrameParser parser;
    uint8_t frame[protocol::maxPayload + protocol::frameOverhead];
    const uint8_t size = buildFrame(frame, type, payload, length);
    TEST_ASSERT_TRUE(feed(parser, frame, size));
    return handleFrame(parser, lift, reply);
}

const uint8_t stateOffsetMode = 4 + 20;
const uint8_t stateOffsetFault = 4 + 21;
const uint8_t stateOffsetReject = 4 + 22;

} // namespace

void crcMatchesTheStandardCheckValue() {
    const uint8_t check[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
    TEST_ASSERT_EQUAL_HEX16(0x29B1, crc16(check, 9));
}

void aFrameArrivesAcrossManyPushes() {
    FrameParser parser;
    const uint8_t payload[] = {0x34, 0x12};
    uint8_t frame[16];
    const uint8_t size = buildFrame(frame, 0x02, payload, 2);
    for (uint8_t i = 0; i + 1 < size; i++) TEST_ASSERT_FALSE(parser.push(frame[i]));
    TEST_ASSERT_TRUE(parser.push(frame[size - 1]));
    TEST_ASSERT_EQUAL_UINT8(0x02, parser.type());
    TEST_ASSERT_EQUAL_UINT8(2, parser.length());
    TEST_ASSERT_EQUAL_UINT8(0x34, parser.payload()[0]);
    TEST_ASSERT_EQUAL_UINT8(0, parser.errors());
}

void garbageBeforeAFrameIsSkipped() {
    FrameParser parser;
    const uint8_t noise[] = {0x00, 0xA5, 0xA5, 0x13, 0xFF, 0xA5};
    TEST_ASSERT_FALSE(feed(parser, noise, sizeof(noise)));
    const uint8_t payload[] = {1, 0};
    uint8_t frame[16];
    const uint8_t size = buildFrame(frame, 0x02, payload, 2);
    TEST_ASSERT_TRUE(feed(parser, frame, size));
}

void aBadCrcIsDroppedAndCounted() {
    FrameParser parser;
    const uint8_t payload[] = {1, 0};
    uint8_t frame[16];
    const uint8_t size = buildFrame(frame, 0x02, payload, 2);
    frame[size - 1] ^= 0x40;
    TEST_ASSERT_FALSE(feed(parser, frame, size));
    TEST_ASSERT_EQUAL_UINT8(1, parser.errors());

    frame[size - 1] ^= 0x40;
    TEST_ASSERT_TRUE(feed(parser, frame, size));
}

void anImpossibleLengthIsDropped() {
    FrameParser parser;
    const uint8_t frame[] = {protocol::sync0, protocol::sync1, 0x02, protocol::maxPayload + 1};
    TEST_ASSERT_FALSE(feed(parser, frame, sizeof(frame)));
    TEST_ASSERT_EQUAL_UINT8(1, parser.errors());
}

void stateRepliesInTheDocumentedLayout() {
    Lift lift(testConfig);
    Plant plant;
    wake(lift, plant);

    const uint8_t request[] = {0x34, 0x12};
    uint8_t reply[protocol::maxReply];
    const uint8_t size = exchange(lift, (uint8_t)protocol::Command::state, request, 2, reply);

    TEST_ASSERT_EQUAL_UINT8(protocol::frameOverhead + protocol::stateLength, size);
    TEST_ASSERT_EQUAL_UINT8(protocol::sync0, reply[0]);
    TEST_ASSERT_EQUAL_UINT8(protocol::sync1, reply[1]);
    TEST_ASSERT_EQUAL_UINT8((uint8_t)protocol::Reply::state, reply[2]);
    TEST_ASSERT_EQUAL_UINT8(protocol::stateLength, reply[3]);
    TEST_ASSERT_EQUAL_HEX16(0x1234, readU16(reply + 4));
    TEST_ASSERT_EQUAL_INT32(30, readI32(reply + 6));  // Board ms.
    TEST_ASSERT_EQUAL_INT32(lift.counts(), readI32(reply + 10));
    TEST_ASSERT_EQUAL_UINT16(12000, readU16(reply + 20)); // mV.
    const uint16_t flags = readU16(reply + 24);
    TEST_ASSERT_FALSE(flags & protocol::flagHomed);
    TEST_ASSERT_TRUE(flags & protocol::flagEncoderOk);
    TEST_ASSERT_TRUE(flags & protocol::flagSensorOk);
    TEST_ASSERT_TRUE(flags & protocol::flagMeasured);
    TEST_ASSERT_TRUE(flags & protocol::flagAtRest);
    TEST_ASSERT_EQUAL_UINT8((uint8_t)Mode::disabled, reply[stateOffsetMode + 2]);
    TEST_ASSERT_EQUAL_UINT8((uint8_t)Fault::none, reply[stateOffsetFault + 2]);
    TEST_ASSERT_EQUAL_UINT8((uint8_t)Reject::none, reply[stateOffsetReject + 2]);
    TEST_ASSERT_EQUAL_HEX16(crc16(reply + 2, protocol::stateLength + 2),
                            readU16(reply + 4 + protocol::stateLength));
}

void identifyNamesTheBoardAndItsColumn() {
    Lift lift(testConfig);
    const uint8_t request[] = {7, 0};
    uint8_t reply[protocol::maxReply];
    const uint8_t size = exchange(lift, (uint8_t)protocol::Command::identify, request, 2, reply);

    TEST_ASSERT_EQUAL_UINT8(protocol::frameOverhead + protocol::identityLength, size);
    TEST_ASSERT_EQUAL_UINT8((uint8_t)protocol::Reply::identity, reply[2]);
    TEST_ASSERT_EQUAL_UINT16(7, readU16(reply + 4));
    TEST_ASSERT_EQUAL_UINT8('L', reply[6]);
    TEST_ASSERT_EQUAL_UINT8('T', reply[9]);
    TEST_ASSERT_EQUAL_UINT16(protocol::version, readU16(reply + 10));
    TEST_ASSERT_EQUAL_INT32(testConfig.travelCounts, readI32(reply + 16));
    TEST_ASSERT_EQUAL_INT32(testConfig.clearanceCounts, readI32(reply + 20));
    TEST_ASSERT_EQUAL_INT32(testConfig.maxSpeedCps, readI32(reply + 24));
    TEST_ASSERT_EQUAL_UINT8(1, reply[28]);
}

void aMoveBeforeEnableIsRefusedInItsReply() {
    Lift lift(testConfig);
    Plant plant;
    wake(lift, plant);
    const uint8_t request[10] = {9, 0, 0x10, 0x27, 0, 0, 0, 0, 0, 0};
    uint8_t reply[protocol::maxReply];
    exchange(lift, (uint8_t)protocol::Command::move, request, 10, reply);
    TEST_ASSERT_EQUAL_UINT8((uint8_t)Reject::notEnabled, reply[stateOffsetReject + 2]);
}

void anUnknownCommandOrLengthIsRefused() {
    Lift lift(testConfig);
    const uint8_t request[3] = {1, 0, 0};
    uint8_t reply[protocol::maxReply];
    exchange(lift, 0x7F, request, 2, reply);
    TEST_ASSERT_EQUAL_UINT8((uint8_t)Reject::badCommand, reply[stateOffsetReject + 2]);
    exchange(lift, (uint8_t)protocol::Command::enable, request, 3, reply);
    TEST_ASSERT_EQUAL_UINT8((uint8_t)Reject::badCommand, reply[stateOffsetReject + 2]);
    TEST_ASSERT_EQUAL_UINT8((uint8_t)Mode::disabled, reply[stateOffsetMode + 2]);
}

void setup() {
    UNITY_BEGIN();
    RUN_TEST(crcMatchesTheStandardCheckValue);
    RUN_TEST(aFrameArrivesAcrossManyPushes);
    RUN_TEST(garbageBeforeAFrameIsSkipped);
    RUN_TEST(aBadCrcIsDroppedAndCounted);
    RUN_TEST(anImpossibleLengthIsDropped);
    RUN_TEST(stateRepliesInTheDocumentedLayout);
    RUN_TEST(identifyNamesTheBoardAndItsColumn);
    RUN_TEST(aMoveBeforeEnableIsRefusedInItsReply);
    RUN_TEST(anUnknownCommandOrLengthIsRefused);
    UNITY_END();
    stopSimulator();
}

void loop() {}
