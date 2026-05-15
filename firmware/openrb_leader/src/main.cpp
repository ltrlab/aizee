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
//     0xA5                       POLL   -> [0xA5][N][int32 LE * N]
//                                          [int16 joy_x][int16 joy_y]
//                                          [uint8 joy_btn][uint8 joy_status][crc8]
//     0x53                       SCAN   -> [0x53][N][(id, baud_code) * N][crc8]
//     0x52 [target_id]           REID   -> [0x52][status][found_id][baud_code][crc8]
//
//   POLL reply now embeds M5Stack Joystick2 state (I2C, addr 0x63 on Wire,
//   D11=SDA D12=SCL).  joy_x / joy_y are int16 LE in the unit's centred
//   12-bit range (~ -2048..+2047).  joy_btn is 0=pressed, 1=released.
//   joy_status is 0=ok, 1=not present, 2=read error — host should ignore
//   the joystick fields when joy_status != 0.
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
#include <Wire.h>

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
// M5Stack Joystick2 (U156) — optional I2C input on Wire (D11=SDA, D12=SCL).
//
// Used for operator drive + recording start/stop on the leader board so the
// host doesn't need a separate USB gamepad.  Wiring:
//
//     Joystick2 RED   (5V)  -> OpenRB-150 5V
//     Joystick2 BLACK (GND) -> OpenRB-150 GND
//     Joystick2 YELLOW (SDA) -> OpenRB-150 D11
//     Joystick2 WHITE  (SCL) -> OpenRB-150 D12
//
// Register map (M5UnitJoystick2 source):
//   0x50  int16 LE  X 12-bit centred offset (-2048..+2047, factory-cal'd)
//   0x52  int16 LE  Y 12-bit centred offset
//   0x20  uint8     button: 0=pressed, 1=released
//
// The host treats the joystick as absent if joy_status != 0; firmware never
// blocks waiting for the I2C device — if it's not there, we just ship neutral
// values forever.
// ─────────────────────────────────────────────────────────────────────────────
static const uint8_t  JOY_I2C_ADDR              = 0x63;
static const uint8_t  JOY_REG_OFFSET_12B        = 0x50;
static const uint8_t  JOY_REG_BUTTON            = 0x20;
// 100 kHz (standard mode) instead of 400 kHz fast mode.  At 400 kHz the bus
// was NACKing intermittently during heavy leader-arm motion — the DXL TTL
// bus is on adjacent header pins and the backdriven XL330 servos couple
// switching transients onto the shared 5V rail.  Standard mode quadruples
// bit setup/hold times and is much more tolerant of that environment, at
// a cost of ~340 µs more per joy_poll (still negligible vs the dominant
// DXL sync-read time).
static const uint32_t JOY_I2C_CLOCK             = 100000UL;

// joy_status byte (1 byte appended to POLL reply, after button):
//   0x00  ok — values are live
//   0x01  not present — Wire ping returned no ACK at startup
//   0x02  read error — last I2C read failed
static const uint8_t  JOY_STATUS_OK            = 0x00;
static const uint8_t  JOY_STATUS_NOT_PRESENT   = 0x01;
static const uint8_t  JOY_STATUS_READ_ERROR    = 0x02;

static int16_t joy_x       = 0;
static int16_t joy_y       = 0;
static uint8_t joy_button  = 1;     // 1 = released (matches M5 register convention)
static uint8_t joy_status  = JOY_STATUS_NOT_PRESENT;

// Recovery state.  When the I2C controller gets wedged (NACK loop, stuck
// slave, or clock-stretch hang) endTransmission keeps returning non-zero
// and reads keep failing forever — observed once in the field after ~1 min
// of normal use.  We count consecutive failures and on the threshold, tear
// down the SERCOM and re-init from scratch, which clears most stuck states.
// Recovery itself is logged via joy_status_recoveries (firmware-internal
// counter, not on the wire) so we can characterise frequency over time.
static const uint8_t  JOY_RECOVERY_THRESHOLD = 10;
static uint8_t        joy_fail_streak        = 0;

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

// ─────────────────────────────────────────────────────────────────────────────
// Joystick2 helpers
// ─────────────────────────────────────────────────────────────────────────────
//
// Probe by issuing a zero-length transmission to JOY_I2C_ADDR; endTransmission
// returns 0 only if the device ACKed.  Called once at boot so an absent unit
// permanently sets joy_status = NOT_PRESENT and we never waste bus time on it.
static void joy_init() {
    Wire.begin();
    Wire.setClock(JOY_I2C_CLOCK);
    Wire.beginTransmission(JOY_I2C_ADDR);
    uint8_t err = Wire.endTransmission();
    joy_status = (err == 0) ? JOY_STATUS_OK : JOY_STATUS_NOT_PRESENT;
}

// One I2C transaction: write the start register, then read N bytes.  Returns
// true on full success; partial reads (Wire.available() < n) are flagged as
// failures so the host sees joy_status == READ_ERROR rather than stale zeros.
static bool joy_read_bytes_once(uint8_t reg, uint8_t* buf, uint8_t n) {
    Wire.beginTransmission(JOY_I2C_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    uint8_t got = Wire.requestFrom(JOY_I2C_ADDR, n);
    if (got != n) return false;
    for (uint8_t i = 0; i < n; ++i) {
        if (!Wire.available()) return false;
        buf[i] = Wire.read();
    }
    return true;
}

// Same as above with one immediate retry on transient failure.  Most
// motion-induced glitches (DXL-bus EMI, 5V rail dip during sync-read TX
// bursts) are single-sample — a second attempt 100 µs later almost always
// succeeds, so the host never sees the transient as a READ_ERROR.  Two
// hard failures in a row still flag the error and feed the fail-streak
// counter that triggers full bus recovery further upstream.
static bool joy_read_bytes(uint8_t reg, uint8_t* buf, uint8_t n) {
    if (joy_read_bytes_once(reg, buf, n)) return true;
    return joy_read_bytes_once(reg, buf, n);
}

// Last-resort bus recovery — used after JOY_RECOVERY_THRESHOLD consecutive
// failed reads.  Tearing down and re-initialising the SERCOM clears the
// most common lockup modes (NACK loop, controller hung mid-byte).  Slave-
// side stuck-SDA can also be cleared by ending the controller — the slave
// times out and releases the line once SCL stops toggling.
//
// Costs a one-time ~3 ms hiccup on the joystick path but does NOT touch
// Dynamixel (which is on Serial1, not Wire), so leader telemetry stays
// uninterrupted.  joy_fail_streak is cleared so we don't immediately
// recover again on the next failure.
static void joy_recover_bus() {
    Wire.end();
    delay(2);
    Wire.begin();
    Wire.setClock(JOY_I2C_CLOCK);
    delay(1);
    joy_fail_streak = 0;
}

// Refresh joy_x / joy_y / joy_button from the unit.  No-op if we already
// know the device is absent, which keeps the per-poll cost ~zero on
// boards without the joystick attached.  Failed reads bump a streak
// counter; we trigger bus recovery once the streak hits the threshold.
static void joy_poll() {
    if (joy_status == JOY_STATUS_NOT_PRESENT) return;
    uint8_t axes[4];
    uint8_t btn;
    if (!joy_read_bytes(JOY_REG_OFFSET_12B, axes, 4) ||
        !joy_read_bytes(JOY_REG_BUTTON, &btn, 1)) {
        joy_status = JOY_STATUS_READ_ERROR;
        if (joy_fail_streak < 255) joy_fail_streak++;
        if (joy_fail_streak >= JOY_RECOVERY_THRESHOLD) {
            joy_recover_bus();
        }
        return;
    }
    // Registers are little-endian int16, matching SAMD21's native order.
    joy_x      = (int16_t)(axes[0] | (axes[1] << 8));
    joy_y      = (int16_t)(axes[2] | (axes[3] << 8));
    joy_button = btn;
    joy_status = JOY_STATUS_OK;
    joy_fail_streak = 0;
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

    // Optional M5 Joystick2 on the I2C bus (see header comment for wiring).
    // joy_status latches NOT_PRESENT here if no ACK; subsequent joy_poll()
    // calls become no-ops, so the host transparently degrades to "no
    // joystick" without per-frame I2C overhead.
    joy_init();
}

// ─────────────────────────────────────────────────────────────────────────────
// Frame handlers
// ─────────────────────────────────────────────────────────────────────────────
static void send_ident() {
    Serial.write((const uint8_t*)IDENT_STR, sizeof(IDENT_STR) - 1);
    Serial.write('\n');
    Serial.flush();
}

// POLL reply layout (host parser: python/teleop/openrb_leader.py):
//   [0]                    REPLY_POLL_HDR (0xA5)
//   [1]                    N = DXL_NUM_SERVOS
//   [2 .. 2+4N)            int32 LE * N — servo Present_Position
//   [2+4N .. 2+4N+2)       int16 LE     — joystick X (-2048..+2047 nominal)
//   [4+4N .. 4+4N+2)       int16 LE     — joystick Y
//   [6+4N]                 uint8        — joystick button (0=pressed, 1=released)
//   [7+4N]                 uint8        — joy_status (0=ok, 1=not present, 2=read err)
//   [8+4N]                 uint8        — crc8 over preceding bytes
// Total: 9 + 4N bytes (37 with N=7, was 31 before joystick fields).
static void send_poll_reply() {
    // Sync-read all servos in one bus transaction.  Missing servos retain
    // their previous cached value so the host always gets N entries.
    if (dxl.syncRead(&sr_infos) > 0) {
        for (uint8_t i = 0; i < DXL_NUM_SERVOS; ++i) {
            last_pos[i] = sr_data[i].present_position;
        }
    }

    // Refresh joystick state on every poll.  ~150 µs at 400 kHz for the two
    // I2C transactions (5 bytes total payload), well under the dominant
    // SyncRead cost above.  No-op if the unit was absent at boot.
    joy_poll();

    uint8_t frame[2 + DXL_NUM_SERVOS * 4 + 6 + 1];
    frame[0] = REPLY_POLL_HDR;
    frame[1] = DXL_NUM_SERVOS;
    for (uint8_t i = 0; i < DXL_NUM_SERVOS; ++i) {
        int32_t v = last_pos[i];
        frame[2 + i * 4 + 0] = (uint8_t)(v       & 0xFF);
        frame[2 + i * 4 + 1] = (uint8_t)((v >> 8)  & 0xFF);
        frame[2 + i * 4 + 2] = (uint8_t)((v >> 16) & 0xFF);
        frame[2 + i * 4 + 3] = (uint8_t)((v >> 24) & 0xFF);
    }
    uint8_t off = 2 + DXL_NUM_SERVOS * 4;
    frame[off + 0] = (uint8_t)( joy_x       & 0xFF);
    frame[off + 1] = (uint8_t)((joy_x >> 8) & 0xFF);
    frame[off + 2] = (uint8_t)( joy_y       & 0xFF);
    frame[off + 3] = (uint8_t)((joy_y >> 8) & 0xFF);
    frame[off + 4] = joy_button;
    frame[off + 5] = joy_status;
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
