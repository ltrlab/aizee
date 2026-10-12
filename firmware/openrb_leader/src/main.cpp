// OpenRB-150 leader input bridge for the shared robot platform.
//
// Reads Present_Position continuously from up to 7x Dynamixel XL330-M077-T
// servos (IDs 1..7, Protocol 2.0), one Sync Read after another, and answers
// each poll from the latest complete read with a compact binary frame, so a
// poll costs the USB round trip instead of a whole bus read. A poll is never
// answered twice with the same sample: one that arrives before a newer read
// completes waits for it, and one that arrives while reads are failing gets
// all_servos_valid = 0. The guarded host is src/robot_platform/teleop/openrb.py;
// the legacy host is python/teleop/openrb_leader.py.
//
// Wire protocol (host <-> board, USB-CDC):
//
//   Host -> MCU:
//     0x50                       IDENT  -> ASCII "AIZEE-OPENRB-LEADER\n"
//     0xA5                       POLL   -> [0xA5][N][int32 LE * N]
//                                          [int16 joy_x][int16 joy_y]
//                                          [uint8 joy_btn][uint8 joy_status][crc8]
//     0xA6                       FRESH_POLL -> POLL fields, followed by
//                                          [uint32 sample_sequence]
//                                          [uint32 sample_ms][uint32 reply_ms]
//                                          [uint8 all_servos_valid][crc8]
//   FRESH_POLL uses header 0xA6. Sequence advances only after a complete,
//   error-free seven-servo read. Failed/partial reads cannot refresh cached data.
//   sample_ms is the read's start rounded down and reply_ms the reply rounded
//   up, both from one microsecond clock, so their difference never understates
//   how old a cached sample is.
//     0x53                       SCAN   -> [0x53][N][(id, baud_code) * N][crc8]
//     0x52 [target_id]           REID   -> [0x52][status][found_id][baud_code][crc8]
//     0x49                       INFO   -> [0x49][firmware_version][N]
//                                          [(id, servo_firmware, return_delay, torque) * N]
//                                          [uint32 reads][uint32 failed_reads]
//                                          [uint32 last_read_us][uint32 max_read_us][crc8]
//                                          (0xFF for a field a servo did not answer)
//     0x44 [value][crc8]         RETURN_DELAY -> sets every torque-off servo's
//                                          Return Delay Time (EEPROM, 2 us/lsb);
//                                          [0x44][N][(id, before, after) * N][crc8].
//                                          crc8 over [0x44, value]; a bad crc or
//                                          force feedback on refuses it (no reply).
//     0xCC [N][int16 LE * N][crc8] FF_CURRENT -> no reply (see force feedback)
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
static const uint16_t ADDR_GOAL_CURRENT     = 102;
static const uint16_t LEN_GOAL_CURRENT      = 2;

// One-byte EEPROM/RAM items INFO and RETURN_DELAY read and write directly.
static const uint16_t ADDR_FIRMWARE_VERSION = 6;
static const uint16_t ADDR_RETURN_DELAY     = 9;
static const uint16_t ADDR_TORQUE_ENABLE    = 64;

// Reported by INFO. 2: continuous reads, cached replies, RETURN_DELAY, INFO.
static const uint8_t  LEADER_FIRMWARE_VERSION = 2;
// A Sync Read that fails this many times in a row marks cached replies invalid.
static const uint8_t  READ_FAIL_LIMIT       = 3;
// The joystick is read at most this often (it shares the loop with servo reads).
static const uint32_t JOY_PERIOD_US         = 10000;
// An incomplete command payload is dropped after this long.
static const uint32_t PAYLOAD_TIMEOUT_MS    = 100;   // as the old blocking waits

// Wire protocol constants — must match python/teleop/openrb_leader.py.
static const uint8_t  CMD_IDENT        = 0x50;
static const uint8_t  CMD_POLL         = 0xA5;
static const uint8_t  CMD_FRESH_POLL   = 0xA6;
static const uint8_t  CMD_SCAN         = 0x53;
static const uint8_t  CMD_REID         = 0x52;
static const uint8_t  CMD_CENTER       = 0xC0;   // payload [id]; one servo to 2048
static const uint8_t  CMD_FF_CURRENT   = 0xCC;   // payload [N][int16 LE * N][crc8]
static const uint8_t  CMD_INFO         = 0x49;
static const uint8_t  CMD_RETURN_DELAY = 0x44;   // payload [value][crc8]
static const uint8_t  REPLY_POLL_HDR   = 0xA5;
static const uint8_t  REPLY_SCAN_HDR   = 0x53;
static const uint8_t  REPLY_REID_HDR   = 0x52;
static const uint8_t  REPLY_CENTER_HDR = 0xC0;
static const char     IDENT_STR[]      = "AIZEE-OPENRB-LEADER";

// ─────────────────────────────────────────────────────────────────────────────
// Force-feedback (current-control) mode
// ─────────────────────────────────────────────────────────────────────────────
// Defaults to OFF — every servo starts torque-disabled so the operator can
// backdrive the arm freely.  The host opts in per-poll by sending
// CMD_FF_CURRENT with a vector of signed goal currents (mA).  Each entry is
// clamped to ±MAX_FF_CURRENT_MA in firmware; the sentinel value
// FF_DISABLE_SENTINEL means "stop applying torque on this servo, return it
// to backdrivable".  The host MUST resend currents periodically — two-stage
// watchdog (zero-on-stale, disable-on-very-stale) prevents a hung/crashed
// host from leaving torque applied indefinitely.
//
// The 5V supply on USB is not enough to drive these currents on more than
// one or two servos for any length of time; the user runs FF mode with an
// external 5V supply on the OpenRB rail.
//
// XL330-M077-T Goal_Current unit is 1 mA / lsb.
static const int16_t  MAX_FF_CURRENT_MA      = 200;   // hard cap per servo
static const int16_t  FF_DISABLE_SENTINEL    = INT16_MIN; // "torque off this slot"
static const uint32_t FF_WATCHDOG_ZERO_MS    = 200;   // stale -> goal=0 (still energised)
static const uint32_t FF_WATCHDOG_DISABLE_MS = 1000;  // very stale -> torque off entirely

// Per-slot FF state.
static bool     ff_active[DXL_NUM_SERVOS] = {false};
static uint32_t ff_last_update_ms         = 0;
static bool     ff_zeroed                 = false;   // currents have been zeroed by watchdog stage 1
static bool     ff_disabled               = true;    // post-boot state: no servo in current-control mode
// The latest CMD_FF_CURRENT, applied between servo reads (ff_apply).
static int16_t  ff_target[DXL_NUM_SERVOS];
static bool     ff_pending                = false;
// Goal currents go out in one Sync Write (no status replies) for the active slots.
static uint8_t                         ff_current_le[DXL_NUM_SERVOS][LEN_GOAL_CURRENT];
static uint8_t                         sw_pkt_buf[64];
static DYNAMIXEL::InfoSyncWriteInst_t  sw_infos;
static DYNAMIXEL::XELInfoSyncWrite_t   sw_xels[DXL_NUM_SERVOS];

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
static uint32_t position_sequence = 0;
static uint64_t position_sample_us = 0;   // start of the read that produced last_pos

// Continuous-read statistics (INFO) and the failure streak that invalidates replies.
static uint32_t reads = 0, failed_reads = 0, last_read_us = 0, max_read_us = 0;
static uint8_t  read_fail_streak = 0;

// A poll waits here until a newer complete read (or a failing one) can answer it.
static bool     poll_pending = false;
static bool     poll_with_freshness = false;
static uint32_t replied_sequence = 0;

// Commands with a payload collect it without blocking the read loop.
static uint8_t  cmd_code = 0;            // 0: waiting for a command byte
static uint8_t  cmd_buf[2 * DXL_NUM_SERVOS + 2];
static uint8_t  cmd_len = 0, cmd_need = 0;
static uint32_t cmd_started_ms = 0;
static uint32_t drain_until_ms = 0;      // after a malformed FF frame, drop bytes until then
static bool     draining = false;

static uint64_t joy_due_us = 0;

// micros() wraps every 71.6 minutes; this 64-bit count does not, so the
// millisecond fields derived from it wrap at 2^32 ms like millis(). Called
// every loop, far more often than once per wrap.
static uint64_t clock_us() {
    static uint32_t last = 0, high = 0;
    const uint32_t now = micros();
    if (now < last) ++high;
    last = now;
    return ((uint64_t)high << 32) | now;
}

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

    // Goal-current Sync Write: rebuilt for the active slots on every write.
    sw_infos.packet.p_buf        = sw_pkt_buf;
    sw_infos.packet.buf_capacity = sizeof(sw_pkt_buf);
    sw_infos.packet.is_completed = false;
    sw_infos.addr                = ADDR_GOAL_CURRENT;
    sw_infos.addr_length         = LEN_GOAL_CURRENT;
    sw_infos.p_xels              = sw_xels;
    sw_infos.xel_count           = 0;
    sw_infos.is_info_changed     = true;
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
//
// One Sync Read of every servo. Only a complete, error-free read refreshes
// the cached positions, their sequence and their start time (conservatively
// before any servo answered); missing servos keep their previous value.
static void read_servos() {
    const uint64_t started = clock_us();
    const uint8_t received = dxl.syncRead(&sr_infos);
    bool valid = received == DXL_NUM_SERVOS;
    for (uint8_t i = 0; i < received; ++i) {
        valid = valid && sr_xels[i].error == 0;
    }
    last_read_us = (uint32_t)(clock_us() - started);
    if (last_read_us > max_read_us) max_read_us = last_read_us;
    ++reads;
    if (!valid) {
        ++failed_reads;
        if (read_fail_streak < 255) ++read_fail_streak;
        return;
    }
    for (uint8_t i = 0; i < DXL_NUM_SERVOS; ++i) {
        last_pos[i] = sr_data[i].present_position;
    }
    position_sample_us = started;
    ++position_sequence;
    read_fail_streak = 0;
}

// Reply from the cache. all_servos_valid: a complete read exists and reads
// are not failing; a host skips (never reuses) a reply that says otherwise.
static void send_poll_reply(bool with_freshness, bool valid) {
    uint8_t frame[2 + DXL_NUM_SERVOS * 4 + 6 + 13 + 1];
    const size_t length = with_freshness ? sizeof(frame) : sizeof(frame) - 13;
    frame[0] = with_freshness ? CMD_FRESH_POLL : REPLY_POLL_HDR;
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
    if (with_freshness) {
        // Read start rounded down, reply rounded up: never understates the age.
        // The reply time never steps back either (a host faults on that).
        static uint32_t last_reply_ms = 0;
        uint32_t reply_ms = (uint32_t)((clock_us() + 999) / 1000);
        if ((int32_t)(reply_ms - last_reply_ms) < 0) reply_ms = last_reply_ms;
        last_reply_ms = reply_ms;
        const uint32_t fields[] = {position_sequence, (uint32_t)(position_sample_us / 1000), reply_ms};
        for (uint8_t field = 0; field < 3; ++field) {
            for (uint8_t byte = 0; byte < 4; ++byte) {
                frame[off + 6 + field * 4 + byte] = (uint8_t)(fields[field] >> (8 * byte));
            }
        }
        frame[off + 18] = valid ? 1 : 0;
    }
    frame[length - 1] = crc8(frame, length - 1);

    Serial.write(frame, length);
    Serial.flush();
}

// Answer a pending poll once it can be: with a sample newer than the last one
// sent, or as invalid while reads are failing. Never the same sample twice
// (a host faults on a repeated sequence); otherwise it waits for the next read.
static void answer_poll() {
    if (!poll_pending) return;
    const bool failing = read_fail_streak > 0;
    const bool fresh = position_sequence != replied_sequence && position_sequence > 0;
    if (!fresh && !failing) return;
    const bool valid = fresh && read_fail_streak < READ_FAIL_LIMIT;
    send_poll_reply(poll_with_freshness, valid);
    if (valid) replied_sequence = position_sequence;
    poll_pending = false;
}

// One byte from a servo's control table; 0xFF if it did not answer.
static uint8_t read_byte(uint8_t id, uint16_t addr) {
    uint8_t value = 0xFF;
    return dxl.read(id, addr, 1, &value, 1) == 1 ? value : 0xFF;
}

// INFO: firmware version, each servo's firmware, Return Delay Time and torque
// state, and the continuous-read statistics. Diagnostic: reads stop meanwhile.
static void send_info() {
    uint8_t frame[3 + DXL_NUM_SERVOS * 4 + 16 + 1];
    frame[0] = CMD_INFO;
    frame[1] = LEADER_FIRMWARE_VERSION;
    frame[2] = DXL_NUM_SERVOS;
    for (uint8_t i = 0; i < DXL_NUM_SERVOS; ++i) {
        const uint8_t id = DXL_IDS[i];
        frame[3 + i * 4] = id;
        frame[4 + i * 4] = read_byte(id, ADDR_FIRMWARE_VERSION);
        frame[5 + i * 4] = read_byte(id, ADDR_RETURN_DELAY);
        frame[6 + i * 4] = read_byte(id, ADDR_TORQUE_ENABLE);
    }
    const uint32_t stats[] = {reads, failed_reads, last_read_us, max_read_us};
    uint8_t off = 3 + DXL_NUM_SERVOS * 4;
    for (uint8_t s = 0; s < 4; ++s) {
        for (uint8_t byte = 0; byte < 4; ++byte) frame[off++] = (uint8_t)(stats[s] >> (8 * byte));
    }
    frame[off] = crc8(frame, off);
    Serial.write(frame, off + 1);
    Serial.flush();
}

// RETURN_DELAY: set every torque-off servo's Return Delay Time (EEPROM area,
// so torque must be off; a torque-on servo is left alone and reports it
// unchanged). Each servo is read before and after; 0xFF where it did not answer.
static void handle_return_delay(uint8_t value) {
    uint8_t frame[2 + DXL_NUM_SERVOS * 3 + 1];
    frame[0] = CMD_RETURN_DELAY;
    frame[1] = DXL_NUM_SERVOS;
    for (uint8_t i = 0; i < DXL_NUM_SERVOS; ++i) {
        const uint8_t id = DXL_IDS[i];
        const uint8_t before = read_byte(id, ADDR_RETURN_DELAY);
        uint8_t after = before;
        if (before != 0xFF && before != value && read_byte(id, ADDR_TORQUE_ENABLE) == 0) {
            dxl.write(id, ADDR_RETURN_DELAY, &value, 1);
            after = read_byte(id, ADDR_RETURN_DELAY);
        }
        frame[2 + i * 3] = id;
        frame[3 + i * 3] = before;
        frame[4 + i * 3] = after;
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

static void send_reid_reply(uint8_t status, uint8_t found_id, uint8_t found_code) {
    uint8_t frame[5];
    frame[0] = REPLY_REID_HDR;
    frame[1] = status;
    frame[2] = found_id;
    frame[3] = found_code;
    frame[4] = crc8(frame, 4);
    Serial.write(frame, sizeof(frame));
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
    send_reid_reply(status, found_id, found_code);
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
// Force-feedback handlers
// ─────────────────────────────────────────────────────────────────────────────
//
// Switch a single servo from torque-off (backdrive) mode into current
// control.  Idempotent — calling again on an already-active servo is cheap
// because Dynamixel2Arduino caches the mode internally.  Always:
//   1) torque off (mode writes require it)
//   2) set OPERATING_MODE = 0 (current control)
//   3) write Goal_Current = 0 (start gentle on transition)
//   4) torque on
// Returns true if the servo is now under current control.
static bool ff_enable_slot(uint8_t i) {
    if (ff_active[i]) return true;
    uint8_t id = DXL_IDS[i];
    // Skip silently if the servo isn't on the bus — keeps the rest of the
    // FF command alive even with a missing servo.
    if (!dxl.ping(id)) return false;
    dxl.torqueOff(id);
    if (!dxl.writeControlTableItem(OPERATING_MODE, id, 0)) return false;
    if (!dxl.writeControlTableItem(GOAL_CURRENT,   id, 0)) return false;
    if (!dxl.torqueOn(id)) return false;
    ff_active[i] = true;
    return true;
}

// Disable FF on a single servo, returning it to backdrive.  Safe to call
// whether the slot is currently active or not.
static void ff_disable_slot(uint8_t i) {
    if (!ff_active[i]) return;
    uint8_t id = DXL_IDS[i];
    // Best-effort: write goal current 0 first so even if torqueOff misses
    // (e.g. servo went offline), we're not actively driving torque.
    dxl.writeControlTableItem(GOAL_CURRENT, id, 0);
    dxl.torqueOff(id);
    ff_active[i] = false;
}

static void ff_disable_all() {
    for (uint8_t i = 0; i < DXL_NUM_SERVOS; ++i) ff_disable_slot(i);
    ff_disabled = true;
    ff_zeroed   = false;
}

// Accept a complete CMD_FF_CURRENT payload, collected by the parser after the
// 0xCC byte:
//     [1 byte  N]                 equals DXL_NUM_SERVOS (the parser checks it)
//     [int16 LE * N]              signed mA, INT16_MIN = disable that slot
//     [1 byte  crc8]              over [CMD, N, payload]
// A bad crc is dropped silently (the watchdog eventually disables FF);
// out-of-range currents are clamped before anything reaches a servo. The
// targets are applied between servo reads (ff_apply).
static void accept_ff_current(const uint8_t* payload) {
    uint8_t crc_in[2 + 2 * DXL_NUM_SERVOS];
    crc_in[0] = CMD_FF_CURRENT;
    for (uint8_t i = 0; i < 1 + 2 * DXL_NUM_SERVOS; ++i) crc_in[1 + i] = payload[i];
    if (payload[1 + 2 * DXL_NUM_SERVOS] != crc8(crc_in, sizeof(crc_in))) return;
    for (uint8_t i = 0; i < DXL_NUM_SERVOS; ++i) {
        int16_t v = (int16_t)(payload[1 + i * 2] | (payload[2 + i * 2] << 8));
        if (v != FF_DISABLE_SENTINEL) {
            if (v >  MAX_FF_CURRENT_MA) v =  MAX_FF_CURRENT_MA;
            if (v < -MAX_FF_CURRENT_MA) v = -MAX_FF_CURRENT_MA;
        }
        ff_target[i] = v;
    }
    ff_pending = true;
    ff_last_update_ms = millis();
    ff_zeroed   = false;
    ff_disabled = false;
}

// One Sync Write of goal current to every active slot. It has no status
// replies, so the bus is busy ~0.2 ms instead of a reply wait per servo.
static void ff_write_currents(bool zero) {
    uint8_t n = 0;
    for (uint8_t i = 0; i < DXL_NUM_SERVOS; ++i) {
        if (!ff_active[i]) continue;
        const int16_t v = zero ? 0 : ff_target[i];
        ff_current_le[i][0] = (uint8_t)(v & 0xFF);
        ff_current_le[i][1] = (uint8_t)((v >> 8) & 0xFF);
        sw_xels[n].id     = DXL_IDS[i];
        sw_xels[n].p_data = ff_current_le[i];
        ++n;
    }
    if (n == 0) return;
    sw_infos.xel_count       = n;
    sw_infos.is_info_changed = true;   // The packet holds a copy of the data: rebuild it every time.
    dxl.syncWrite(&sw_infos);
}

// Between servo reads: switch slots into or out of current control only when
// that changes (slow; Operating Mode is EEPROM), then write the currents.
static void ff_apply() {
    if (!ff_pending) return;
    ff_pending = false;
    for (uint8_t i = 0; i < DXL_NUM_SERVOS; ++i) {
        if (ff_target[i] == FF_DISABLE_SENTINEL) ff_disable_slot(i);
        else ff_enable_slot(i);   // A servo that does not answer stays out of the write.
    }
    ff_write_currents(false);
}

// Called every loop tick to enforce the two-stage host-liveness watchdog.
// Stage 1 (zero currents) kicks in at FF_WATCHDOG_ZERO_MS — the servos
// stay in current-control mode but apply no torque, so a brief host stall
// doesn't drop FF entirely (avoids the "torque toggling on/off as you
// stutter" feel).  Stage 2 (full disable) at FF_WATCHDOG_DISABLE_MS hard-
// resets to backdrive in case the host has actually crashed.
static void ff_watchdog_tick() {
    if (ff_disabled) return;
    uint32_t age = millis() - ff_last_update_ms;
    if (age >= FF_WATCHDOG_DISABLE_MS) {
        ff_disable_all();
        return;
    }
    if (age >= FF_WATCHDOG_ZERO_MS && !ff_zeroed) {
        ff_pending = false;   // Stale targets are never applied after the zeroing.
        ff_write_currents(true);
        ff_zeroed = true;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Host commands and the main loop
// ─────────────────────────────────────────────────────────────────────────────

// Payload bytes each command takes after its command byte.
static uint8_t payload_length(uint8_t code) {
    switch (code) {
        case CMD_REID:
        case CMD_CENTER:       return 1;
        case CMD_RETURN_DELAY: return 2;
        case CMD_FF_CURRENT:   return 2 + 2 * DXL_NUM_SERVOS;
        default:               return 0;
    }
}

static void run_command(uint8_t code, const uint8_t* payload) {
    switch (code) {
        case CMD_IDENT:      send_ident(); break;
        case CMD_POLL:       poll_pending = true; poll_with_freshness = false; break;
        case CMD_FRESH_POLL: poll_pending = true; poll_with_freshness = true;  break;
        case CMD_SCAN:       send_scan_reply(); break;
        case CMD_REID:       handle_reid(payload[0]); break;
        case CMD_CENTER:     handle_center(payload[0]); break;
        case CMD_INFO:       send_info(); break;
        case CMD_RETURN_DELAY: {
            // A persistent servo write: only with a valid crc and no force feedback on.
            const uint8_t check[] = {CMD_RETURN_DELAY, payload[0]};
            if (payload[1] == crc8(check, sizeof(check)) && ff_disabled) handle_return_delay(payload[0]);
            break;
        }
        // Fire-and-forget: no reply. The host owns the cadence; the watchdog the safe state.
        case CMD_FF_CURRENT: accept_ff_current(payload); break;
        // Unknown byte: dropped, so older firmware ignores newer commands instead of jamming.
        default: break;
    }
}

// Take whatever the host has sent without waiting for more: a command's
// payload collects across passes, so servo reads never stall on USB.
static void service_host() {
    if (draining && (int32_t)(millis() - drain_until_ms) >= 0) draining = false;
    if (cmd_code != 0 && millis() - cmd_started_ms > PAYLOAD_TIMEOUT_MS) {
        // Its payload never came: setup tools get the same failure reply as
        // before, and nothing is written to a servo.
        if (cmd_code == CMD_CENTER) send_center_reply(CENTER_NOT_FOUND, 0xFF, 0);
        else if (cmd_code == CMD_REID) send_reid_reply(STAT_NOT_FOUND, 0, 0xFF);
        cmd_code = 0;
    }
    while (Serial.available() > 0) {
        const int c = Serial.read();
        if (c < 0) break;
        if (draining) continue;
        if (cmd_code == 0) {
            const uint8_t need = payload_length((uint8_t)c);
            if (need == 0) {
                run_command((uint8_t)c, nullptr);
            } else {
                cmd_code = (uint8_t)c;
                cmd_need = need;
                cmd_len = 0;
                cmd_started_ms = millis();
            }
            continue;
        }
        cmd_buf[cmd_len++] = (uint8_t)c;
        if (cmd_code == CMD_FF_CURRENT && cmd_len == 1 && c != DXL_NUM_SERVOS) {
            // A wrong servo count: drop what follows for a moment to keep the framing aligned.
            cmd_code = 0;
            draining = true;
            drain_until_ms = millis() + 20;
            continue;
        }
        if (cmd_len == cmd_need) {
            const uint8_t code = cmd_code;
            cmd_code = 0;
            run_command(code, cmd_buf);
        }
    }
}

static void joy_tick() {
    const uint64_t now = clock_us();
    if (now < joy_due_us) return;
    joy_due_us = now + JOY_PERIOD_US;
    joy_poll();
}

// One Sync Read after another; host polls are answered right after each read
// and again after the slower steps, from the newest complete sample.
void loop() {
    // Every pass, so a host that stops sending cannot leave torque applied.
    ff_watchdog_tick();
    read_servos();
    service_host();
    answer_poll();
    ff_apply();
    joy_tick();
    service_host();
    answer_poll();
}
