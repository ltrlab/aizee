// OpenRB-150 leader-arm bridge for the AIZEE robot.
//
// Reads Present_Position on every poll from up to 7x Dynamixel XL330-M077-T
// servos (IDs 1..7, Protocol 2.0) and replies to the host with a compact
// binary frame.  The host implementation is python/teleop/openrb_leader.py.
//
// Wire protocol (host <-> board, USB-CDC):
//
//   Host -> MCU:
//     0x50                       IDENT  -> ASCII "AIZEE-OPENRB-LEADER\n"
//     0xA5                       POLL   -> [0xA5][N][int32 LE * N][crc8]
//     0x53                       SCAN   -> [0x53][N][(id, baud_code) * N][crc8]
//     0x52 [target_id]           REID   -> [0x52][status][found_id][baud_code][crc8]
//
//   crc8 = Dallas/Maxim, polynomial 0x31, seed 0x00, over [HDR, ...payload].
//
//   baud_code: 0=1Mbps, 1=57600, 2=115200, 3=2Mbps  (matches BAUDS table below)
//
//   REID status codes:
//     0x00 OK              servo successfully assigned target_id at 1Mbps
//     0x01 NOT_FOUND       no servo answered any baud during the scan
//     0x02 AMBIGUOUS       multiple servos answered (bus has more than one)
//     0x03 WRITE_FAILED    set-id or set-baud register write was rejected
//
// Normal-leader mode keeps every servo torque-disabled so the operator can
// backdrive the arm.  We rely on the host calibration math to handle the
// XL330's signed multi-turn Present_Position by reducing modulo 4096.
//
// First-time use: flash this firmware once, then run
//     python python/scripts/openrb_setup_arm.py
// to assign IDs 1..7 in joint order.

#include <Arduino.h>
#include <Dynamixel2Arduino.h>

// Pulls OPERATING_MODE, GOAL_POSITION, PRESENT_POSITION, etc. into scope so
// dxl.writeControlTableItem(...) calls don't need the ControlTableItem::
// namespace qualifier on every reference.
using namespace ControlTableItem;

// ─────────────────────────────────────────────────────────────────────────────
// Hardware bindings
// ─────────────────────────────────────────────────────────────────────────────

// OpenRB-150 routes its on-board Dynamixel TTL transceiver to Serial1 and
// drives the bus direction pin automatically when DXL_DIR_PIN = -1.
#define DXL_SERIAL    Serial1
#define DXL_DIR_PIN   -1

#ifndef DXL_BAUD
#define DXL_BAUD      1000000
#endif

#ifndef DXL_NUM_SERVOS
#define DXL_NUM_SERVOS 7
#endif

static const float    DXL_PROTOCOL_VERSION = 2.0f;
static const uint8_t  DXL_IDS[DXL_NUM_SERVOS] = {1, 2, 3, 4, 5, 6, 7};

// XL330 control-table addresses (Protocol 2.0).
static const uint16_t ADDR_PRESENT_POSITION = 132;
static const uint16_t LEN_PRESENT_POSITION  = 4;

// Wire protocol constants — must match python/teleop/openrb_leader.py.
static const uint8_t  CMD_IDENT        = 0x50;
static const uint8_t  CMD_POLL         = 0xA5;
static const uint8_t  CMD_SCAN         = 0x53;
static const uint8_t  CMD_REID         = 0x52;
static const uint8_t  CMD_CENTER       = 0xC0;   // payload [id]; one servo to 2048
static const uint8_t  REPLY_POLL_HDR   = 0xA5;
static const uint8_t  REPLY_SCAN_HDR   = 0x53;
static const uint8_t  REPLY_REID_HDR   = 0x52;
static const uint8_t  REPLY_CENTER_HDR = 0xC0;
static const char     IDENT_STR[]      = "AIZEE-OPENRB-LEADER";

// REID status codes.
static const uint8_t  STAT_OK            = 0x00;
static const uint8_t  STAT_NOT_FOUND     = 0x01;
static const uint8_t  STAT_AMBIGUOUS     = 0x02;
static const uint8_t  STAT_WRITE_FAILED  = 0x03;   // setID failed
static const uint8_t  STAT_BAUD_FAILED   = 0x04;   // setBaudrate failed
static const uint8_t  STAT_VERIFY_FAILED = 0x05;   // post-write ping failed

// CENTER status codes.
static const uint8_t  CENTER_OK        = 0x00;
static const uint8_t  CENTER_NOT_FOUND = 0x01;
static const uint8_t  CENTER_TIMEOUT   = 0x02;   // didn't reach target before deadline
static const uint8_t  CENTER_FAILED    = 0x03;   // a write failed (mode/torque)

// Centering parameters.  Profile velocity in raw XL330 units (0.229 rpm/lsb)
// — 60 ≈ 14 rpm ≈ 84°/s at the output shaft.  Profile acceleration in raw
// units (214.577 rev/min² / lsb) — 10 gives smooth ramp without inrush.
static const int32_t  CENTER_GOAL          = 2048;   // encoder centre
static const int32_t  CENTER_TOLERANCE     = 20;     // ticks (~1.7°) considered "at centre"
static const int32_t  CENTER_TOLERANCE_MAX = 60;     // ticks accepted as OK on timeout
static const uint32_t CENTER_TIMEOUT_MS    = 4000;
static const int32_t  CENTER_PROFILE_VEL   = 60;
static const int32_t  CENTER_PROFILE_ACC   = 10;

// Baud rates we sweep during scan/reid, in priority order.  baud_code is the
// index here.  Keep 1Mbps first so a freshly-configured bus is fastest.
static const uint32_t BAUDS[]      = {1000000, 57600, 115200, 2000000};
static const uint8_t  N_BAUDS      = sizeof(BAUDS) / sizeof(BAUDS[0]);
static const uint32_t TARGET_BAUD  = 1000000;
static const uint8_t  TARGET_BAUD_CODE = 0;

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

// ─────────────────────────────────────────────────────────────────────────────
// SyncRead user packet (sized at compile time)
// ─────────────────────────────────────────────────────────────────────────────
typedef struct __attribute__((packed)) {
    int32_t present_position;
} sr_data_t;

// 128 bytes is the size used by the Dynamixel2Arduino reference example —
// covers 7 servos * (status overhead + 4 data bytes) with comfortable slack.
static const uint16_t SR_PKT_CAP = 128;
static uint8_t        sr_pkt_buf[SR_PKT_CAP];

static sr_data_t                       sr_data[DXL_NUM_SERVOS];
static DYNAMIXEL::InfoSyncReadInst_t   sr_infos;
static DYNAMIXEL::XELInfoSyncRead_t    sr_xels[DXL_NUM_SERVOS];

// Last successful Present_Position per slot.  Sent unchanged when sync-read
// fails so the host always gets a well-formed frame and can retry.  Seeded
// to encoder centre for sane output before the first read succeeds.
static int32_t last_pos[DXL_NUM_SERVOS] = {2048, 2048, 2048, 2048, 2048, 2048, 2048};

// Currently-selected bus baud (so REID can leave it at TARGET_BAUD on success).
static uint32_t current_bus_baud = DXL_BAUD;

// ─────────────────────────────────────────────────────────────────────────────
// CRC-8 (Dallas/Maxim, poly 0x31, seed 0x00) — matches Python _crc8()
// ─────────────────────────────────────────────────────────────────────────────
static uint8_t crc8(const uint8_t* data, size_t len) {
    uint8_t crc = 0;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; ++b) {
            crc = (crc & 0x80) ? ((crc << 1) ^ 0x31) : (crc << 1);
        }
    }
    return crc;
}

static uint8_t baud_code_for(uint32_t baud) {
    for (uint8_t i = 0; i < N_BAUDS; ++i) {
        if (BAUDS[i] == baud) return i;
    }
    return 0xFF;
}

static void switch_bus_baud(uint32_t baud) {
    if (baud == current_bus_baud) return;
    // Drain TX, then fully re-init the UART.  The SAMD Uart driver can
    // leave stale state if begin() is called while the port is still open
    // — explicitly end() first guarantees the new baud takes effect.
    DXL_SERIAL.flush();
    DXL_SERIAL.end();
    delay(2);
    dxl.begin(baud);
    dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
    current_bus_baud = baud;
    delay(2);   // give the bus a beat to settle before next transaction
}

// ─────────────────────────────────────────────────────────────────────────────
// Setup
// ─────────────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(1000000);   // USB-CDC: baud is informational only

    // Bring up the Dynamixel bus at TARGET_BAUD.  Servos still on their
    // factory baud will simply not respond until openrb_setup_arm.py is run.
    dxl.begin(DXL_BAUD);
    dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
    current_bus_baud = DXL_BAUD;

    // Disable torque on whichever servos are present.  Missing servos just
    // get skipped — host-side calibration will surface that.
    for (uint8_t i = 0; i < DXL_NUM_SERVOS; ++i) {
        uint8_t id = DXL_IDS[i];
        if (dxl.ping(id)) {
            dxl.torqueOff(id);
        }
    }

    // Configure SyncRead packet: read Present_Position (4 bytes) from all IDs.
    sr_infos.packet.p_buf        = sr_pkt_buf;
    sr_infos.packet.buf_capacity = SR_PKT_CAP;
    sr_infos.packet.is_completed = false;
    sr_infos.addr                = ADDR_PRESENT_POSITION;
    sr_infos.addr_length         = LEN_PRESENT_POSITION;
    sr_infos.p_xels              = sr_xels;
    sr_infos.xel_count           = DXL_NUM_SERVOS;
    sr_infos.is_info_changed     = true;

    for (uint8_t i = 0; i < DXL_NUM_SERVOS; ++i) {
        sr_xels[i].id         = DXL_IDS[i];
        sr_xels[i].p_recv_buf = (uint8_t*)&sr_data[i];
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Frame handlers
// ─────────────────────────────────────────────────────────────────────────────
static void send_ident() {
    Serial.write((const uint8_t*)IDENT_STR, sizeof(IDENT_STR) - 1);
    Serial.write('\n');
    Serial.flush();
}

static void send_poll_reply() {
    // Sync-read all servos in one bus transaction.  Missing servos retain
    // their previous cached value so the host always gets N entries.
    if (dxl.syncRead(&sr_infos) > 0) {
        for (uint8_t i = 0; i < DXL_NUM_SERVOS; ++i) {
            last_pos[i] = sr_data[i].present_position;
        }
    }

    uint8_t frame[2 + DXL_NUM_SERVOS * 4 + 1];
    frame[0] = REPLY_POLL_HDR;
    frame[1] = DXL_NUM_SERVOS;
    for (uint8_t i = 0; i < DXL_NUM_SERVOS; ++i) {
        int32_t v = last_pos[i];
        frame[2 + i * 4 + 0] = (uint8_t)(v       & 0xFF);
        frame[2 + i * 4 + 1] = (uint8_t)((v >> 8)  & 0xFF);
        frame[2 + i * 4 + 2] = (uint8_t)((v >> 16) & 0xFF);
        frame[2 + i * 4 + 3] = (uint8_t)((v >> 24) & 0xFF);
    }
    frame[sizeof(frame) - 1] = crc8(frame, sizeof(frame) - 1);

    Serial.write(frame, sizeof(frame));
    Serial.flush();
}

// Sweep every supported baud, broadcast-pinging for any responding servo.
// Used by the setup wizard to discover what's currently on the bus.
//
// Reply: [0x53][N][(id, baud_code) * N][crc8]
static void send_scan_reply() {
    uint8_t found_ids   [N_BAUDS * 16];
    uint8_t found_codes [N_BAUDS * 16];
    uint8_t n_found = 0;

    for (uint8_t bi = 0; bi < N_BAUDS; ++bi) {
        switch_bus_baud(BAUDS[bi]);
        uint8_t resp_ids[16] = {0};
        // Broadcast-ping at the current baud and collect responders.  50 ms
        // is long enough for every bus servo to send its status reply
        // sequentially without making the scan feel sluggish.
        uint8_t count = dxl.ping(DXL_BROADCAST_ID, resp_ids, sizeof(resp_ids), 50);
        for (uint8_t i = 0; i < count && n_found < sizeof(found_ids); ++i) {
            found_ids  [n_found] = resp_ids[i];
            found_codes[n_found] = bi;
            n_found++;
        }
    }
    // Restore preferred bus baud after scan.
    switch_bus_baud(TARGET_BAUD);

    uint8_t frame[2 + 32 * 2 + 1];
    if ((size_t)(2 + n_found * 2 + 1) > sizeof(frame)) {
        n_found = (sizeof(frame) - 3) / 2;   // clamp; pathological case
    }
    frame[0] = REPLY_SCAN_HDR;
    frame[1] = n_found;
    for (uint8_t i = 0; i < n_found; ++i) {
        frame[2 + i * 2 + 0] = found_ids  [i];
        frame[2 + i * 2 + 1] = found_codes[i];
    }
    size_t len = 2 + n_found * 2;
    frame[len] = crc8(frame, len);
    Serial.write(frame, len + 1);
    Serial.flush();
}

// Find the single servo on the bus and reassign its ID + baud.  Intended
// for the first-time setup wizard, which plugs in one servo at a time.
//
// Payload : [target_id]
// Reply   : [0x52][status][found_id][baud_code][crc8]
static void handle_reid(uint8_t target_id) {
    uint8_t found_id   = 0;
    uint8_t found_code = 0xFF;
    uint8_t status     = STAT_NOT_FOUND;

    // Sweep baud rates looking for exactly one responder.  An ambiguous
    // result usually means the user forgot to unplug the previous servo.
    uint8_t total_responders = 0;

    for (uint8_t bi = 0; bi < N_BAUDS && total_responders <= 1; ++bi) {
        switch_bus_baud(BAUDS[bi]);
        uint8_t resp_ids[16] = {0};
        uint8_t count = dxl.ping(DXL_BROADCAST_ID, resp_ids, sizeof(resp_ids), 50);
        for (uint8_t i = 0; i < count; ++i) {
            if (total_responders == 0) {
                found_id   = resp_ids[i];
                found_code = bi;
            }
            total_responders++;
        }
    }

    if (total_responders == 0) {
        status = STAT_NOT_FOUND;
    } else if (total_responders > 1) {
        status = STAT_AMBIGUOUS;
    } else {
        // Switch to the baud the responder is on, then reassign.
        switch_bus_baud(BAUDS[found_code]);

        // Assign new ID first.  Idempotent if the servo already has this ID.
        bool ok_id = dxl.setID(found_id, target_id);
        if (!ok_id) {
            status = STAT_WRITE_FAILED;
        } else {
            // dxl.setBaudrate() reads the model number from a per-ID cache
            // populated only by the single-ID ping() overload; the
            // broadcast ping we used for discovery does NOT populate it
            // correctly.  Ping target_id here so the cache is hot before
            // the baud write — without this, setBaudrate returns false on
            // every freshly-discovered servo.
            (void)dxl.ping(target_id);

            // Bump the baud to TARGET_BAUD.  The servo accepts the write
            // at its current baud and applies the new value immediately.
            bool ok_baud = dxl.setBaudrate(target_id, TARGET_BAUD);
            if (!ok_baud) {
                status = STAT_BAUD_FAILED;
            } else {
                // The servo has just changed its bus baud.  Give its UART
                // a moment to commit (~10 ms is plenty), switch our bus
                // baud, then verify.  Retry the verify ping a few times
                // because the first one can race the servo's settle.
                delay(10);
                switch_bus_baud(TARGET_BAUD);
                bool verified = false;
                for (uint8_t attempt = 0; attempt < 5 && !verified; ++attempt) {
                    if (dxl.ping(target_id)) {
                        verified = true;
                        break;
                    }
                    delay(20);
                }
                if (verified) {
                    dxl.torqueOff(target_id);
                    status = STAT_OK;
                } else {
                    status = STAT_VERIFY_FAILED;
                }
            }
        }
    }

    // Always restore TARGET_BAUD before replying so the next normal POLL
    // works without an extra round-trip.
    switch_bus_baud(TARGET_BAUD);

    uint8_t frame[5];
    frame[0] = REPLY_REID_HDR;
    frame[1] = status;
    frame[2] = found_id;
    frame[3] = found_code;
    frame[4] = crc8(frame, 4);
    Serial.write(frame, sizeof(frame));
    Serial.flush();
}

// Send the CENTER reply: [0xC0][status][id][pos*4 bytes int32 LE][crc8]
static void send_center_reply(uint8_t status, uint8_t id, int32_t pos) {
    uint8_t frame[8];
    frame[0] = REPLY_CENTER_HDR;
    frame[1] = status;
    frame[2] = id;
    frame[3] = (uint8_t)( pos        & 0xFF);
    frame[4] = (uint8_t)((pos >> 8)  & 0xFF);
    frame[5] = (uint8_t)((pos >> 16) & 0xFF);
    frame[6] = (uint8_t)((pos >> 24) & 0xFF);
    frame[7] = crc8(frame, 7);
    Serial.write(frame, sizeof(frame));
    Serial.flush();
}

// Center one servo at *id*: enable position control with a slow profile,
// drive to encoder centre (2048), wait for arrival, disable torque.
//
// Blocks for up to CENTER_TIMEOUT_MS while the servo settles.  Sequential
// invocation by the host is intentional — driving all 7 at once would
// produce a current spike that browns out the OpenRB-150's USB rail.
static void handle_center(uint8_t id) {
    if (id == 0xFE || id == 0xFF) {
        send_center_reply(CENTER_NOT_FOUND, id, 0);
        return;
    }
    if (!dxl.ping(id)) {
        send_center_reply(CENTER_NOT_FOUND, id, 0);
        return;
    }
    // Operating mode + profile must be set with torque disabled.
    dxl.torqueOff(id);
    bool ok = true;
    ok &= dxl.writeControlTableItem(OPERATING_MODE,     id, 3);   // position
    ok &= dxl.writeControlTableItem(PROFILE_VELOCITY,   id, CENTER_PROFILE_VEL);
    ok &= dxl.writeControlTableItem(PROFILE_ACCELERATION, id, CENTER_PROFILE_ACC);
    if (!ok) {
        send_center_reply(CENTER_FAILED, id, 0);
        return;
    }
    if (!dxl.torqueOn(id)) {
        send_center_reply(CENTER_FAILED, id, 0);
        return;
    }
    if (!dxl.writeControlTableItem(GOAL_POSITION, id, CENTER_GOAL)) {
        dxl.torqueOff(id);
        send_center_reply(CENTER_FAILED, id, 0);
        return;
    }

    // Poll position until at centre or timeout.
    uint32_t deadline = millis() + CENTER_TIMEOUT_MS;
    int32_t  pos      = CENTER_GOAL;
    while ((int32_t)(millis() - deadline) < 0) {
        pos = dxl.readControlTableItem(PRESENT_POSITION, id);
        int32_t err = pos - CENTER_GOAL;
        if (err < 0) err = -err;
        if (err <= CENTER_TOLERANCE) break;
        delay(20);
    }
    int32_t err_final = pos - CENTER_GOAL;
    if (err_final < 0) err_final = -err_final;

    // Always disengage torque so the operator can backdrive again.
    dxl.torqueOff(id);

    uint8_t status = (err_final <= CENTER_TOLERANCE_MAX) ? CENTER_OK : CENTER_TIMEOUT;
    send_center_reply(status, id, pos);
}

// ─────────────────────────────────────────────────────────────────────────────
// Main loop
// ─────────────────────────────────────────────────────────────────────────────
void loop() {
    if (Serial.available() == 0) {
        return;
    }
    int c = Serial.read();
    if (c < 0) {
        return;
    }
    switch ((uint8_t)c) {
        case CMD_IDENT:
            send_ident();
            break;
        case CMD_POLL:
            send_poll_reply();
            break;
        case CMD_SCAN:
            send_scan_reply();
            break;
        case CMD_REID: {
            // Wait briefly for the target_id payload byte.
            uint32_t deadline = millis() + 100;
            while (Serial.available() == 0 && (int32_t)(millis() - deadline) < 0) {
                /* spin briefly — payload should arrive within ms */
            }
            if (Serial.available() == 0) {
                // Malformed command — respond with NOT_FOUND so the host
                // doesn't hang waiting for a reply.
                handle_reid(0xFF);
            } else {
                int t = Serial.read();
                handle_reid((uint8_t)(t < 0 ? 0xFF : t));
            }
            break;
        }
        case CMD_CENTER: {
            uint32_t deadline = millis() + 100;
            while (Serial.available() == 0 && (int32_t)(millis() - deadline) < 0) {
                /* spin briefly — payload should arrive within ms */
            }
            if (Serial.available() == 0) {
                send_center_reply(CENTER_NOT_FOUND, 0xFF, 0);
            } else {
                int t = Serial.read();
                handle_center((uint8_t)(t < 0 ? 0xFF : t));
            }
            break;
        }
        default:
            // Unknown byte — drop silently to keep the protocol
            // forward-compatible.  Older firmware ignores newer commands
            // instead of jamming with garbage replies.
            break;
    }
}
