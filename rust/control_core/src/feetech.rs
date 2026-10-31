//! Feetech STS bus servos (the head's two STS3250s) behind the receiver's `Hardware`, so the head
//! runs the same core, leases, limits and stops as the arms.
//!
//! A position servo, mapped onto the core's commands:
//! - `position` is the servo's Goal_Position;
//! - `kp`, `kd` are its P and D coefficients in register units (0..=255), written while its EEPROM
//!   stays locked so they apply until power-off and never persist;
//! - zero gains turn its torque off, the limp equivalent of the core's zero-gain Idle;
//! - `velocity` and `effort` are not sent: a position servo takes neither.
//!
//! Feedback: position, velocity, load (as effort, scaled to the servo's full load) and temperature.
//! Any status error bit faults the joint. Every reply is checked; a missing or malformed one fails
//! the read, naming the servo. Protocol: FF FF id length instruction params checksum, little-endian
//! (Feetech "protocol 0"), checksum = !(id + length + instruction + params).
use crate::{
    hardware::{Hardware, HardwareError, JointFeedback},
    protocol::JointCommand,
    robstride,
};
use serde::{Deserialize, Serialize};
use std::f64::consts::TAU;

pub const RESOLUTION: f64 = 4096.0;
pub const STS3250: u16 = 2825;
/// The fastest Goal_Position a servo is driven at, in steps/s (~4.6 rad/s): above the STS3250's
/// no-load speed at 12 V, so it never limits a valid command.
pub const MAX_SPEED_STEPS: f64 = 3000.0;
const BROADCAST: u8 = 0xFE;
const READ: u8 = 0x02;
const SYNC_READ: u8 = 0x82;
const SYNC_WRITE: u8 = 0x83;
const P_COEFFICIENT: u8 = 21; // D_Coefficient follows at 22.
const TORQUE_ENABLE: u8 = 40;
const GOAL_POSITION: u8 = 42;
/// One read covers 40..64: Torque_Enable, Goal_Position, Lock, Present_Position, Present_Velocity,
/// Present_Load, Present_Voltage, Present_Temperature.
const STATE_START: u8 = 40;
const STATE_BYTES: usize = 24;

pub fn checksum(body: &[u8]) -> u8 {
    !body.iter().fold(0u8, |sum, b| sum.wrapping_add(*b))
}

pub fn instruction(id: u8, code: u8, params: &[u8]) -> Vec<u8> {
    let mut packet = vec![0xFF, 0xFF, id, (params.len() + 2) as u8, code];
    packet.extend_from_slice(params);
    packet.push(checksum(&packet[2..]));
    packet
}

/// Sign-magnitude: bit `sign` holds the sign, the bits below it the magnitude.
pub fn signed(raw: u16, sign: u32) -> i32 {
    let magnitude = i32::from(raw & ((1u16 << sign) - 1));
    if raw >> sign & 1 == 1 {
        -magnitude
    } else {
        magnitude
    }
}

/// One status reply of `length` parameter bytes from `id`: (error bits, parameters).
pub fn parse_status(reply: &[u8], id: u8, length: usize) -> Result<(u8, &[u8]), String> {
    if reply.len() != length + 6 {
        return Err(format!(
            "servo {id}: {} of {} reply bytes arrived",
            reply.len(),
            length + 6
        ));
    }
    if reply[0] != 0xFF || reply[1] != 0xFF || reply[2] != id || usize::from(reply[3]) != length + 2
    {
        return Err(format!("servo {id}: malformed reply"));
    }
    if checksum(&reply[2..reply.len() - 1]) != reply[reply.len() - 1] {
        return Err(format!("servo {id}: reply checksum mismatch"));
    }
    Ok((reply[4], &reply[5..reply.len() - 1]))
}

pub fn alarm_names(bits: u8) -> String {
    let names: Vec<_> = [
        (1, "input voltage"),
        (2, "angle sensor"),
        (4, "overheat"),
        (8, "overcurrent"),
        (32, "overload"),
    ]
    .iter()
    .filter(|(bit, _)| bits & bit != 0)
    .map(|(_, name)| *name)
    .collect();
    if names.is_empty() {
        format!("error bits {bits:#04x}")
    } else {
        names.join(", ")
    }
}

/// A half-duplex serial bus: requests out, replies back, strictly one exchange at a time.
pub trait SerialPort {
    /// Drop anything unread (a late reply must not be taken for the next one).
    fn clear_input(&mut self) -> Result<(), HardwareError>;
    fn send(&mut self, bytes: &[u8]) -> Result<(), HardwareError>;
    /// Fill `buffer`, waiting at most `timeout_us`; returns how many bytes arrived.
    fn receive(&mut self, buffer: &mut [u8], timeout_us: u64) -> Result<usize, HardwareError>;
    /// Let simulated servos move for `dt` seconds; a real bus ignores it.
    fn advance(&mut self, _dt: f64) {}
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Servo {
    pub name: String,
    pub id: u8,
    pub model: u16,
    /// Firmware as the servo reports it, "major.minor".
    pub firmware: String,
    /// +1 or -1: the joint's direction against the servo's step count.
    pub direction: f64,
    /// The step count at joint zero.
    pub zero_steps: f64,
    /// Effort reported at full load (Present_Load 1000), N m.
    pub full_load_nm: f64,
}
impl Servo {
    pub fn steps(&self, position: f64) -> f64 {
        self.zero_steps + self.direction * position * RESOLUTION / TAU
    }
    pub fn validate(&self) -> Result<(), String> {
        if self.name.is_empty()
            || !(1..=253).contains(&self.id)
            || self.model != STS3250
            || self.firmware.is_empty()
            || (self.direction != 1.0 && self.direction != -1.0)
            || !(0.0..RESOLUTION).contains(&self.zero_steps)
            || !self.full_load_nm.is_finite()
            || self.full_load_nm <= 0.0
        {
            return Err(format!(
                "{}: invalid servo id, model, direction, zero or load scale",
                self.name
            ));
        }
        Ok(())
    }
}

/// The last reading of one servo, in its own units.
#[derive(Debug, Clone, Copy, Default)]
pub struct Reading {
    pub torque: bool,
    pub locked: bool,
    pub steps: i32,
    pub speed: i32,
    pub load: i32,
    pub voltage_dv: u8,
    pub temperature: u8,
    pub error: u8,
}

pub struct FeetechHardware<P: SerialPort> {
    pub port: P,
    pub servos: Vec<Servo>,
    pub readings: Vec<Reading>,
    samples: Vec<JointFeedback>,
    /// Torque each servo should hold, and the P/D last written to it.
    torque: Vec<bool>,
    gains: Vec<Option<(u8, u8)>>,
    pub enabled: bool,
    /// Observation never writes: only read requests go out.
    pub transmit: bool,
    now_ms: u64,
    pub requests: u64,
    pub replies: u64,
    pub reply_timeout_us: u64,
    virtual_enable: bool,
    reference: Option<crate::reference::Gate>,
    failure: Option<String>,
    dropped: Option<String>,
}
impl<P: SerialPort> Drop for FeetechHardware<P> {
    fn drop(&mut self) {
        let _ = self.disable();
    }
}
impl<P: SerialPort> FeetechHardware<P> {
    pub fn new(port: P, servos: Vec<Servo>, transmit: bool) -> Result<Self, String> {
        if servos.is_empty() || servos.len() > 16 {
            return Err("a Feetech bus holds 1..=16 servos here".into());
        }
        for (i, servo) in servos.iter().enumerate() {
            servo.validate()?;
            if servos[..i]
                .iter()
                .any(|s| s.id == servo.id || s.name == servo.name)
            {
                return Err(format!("{}: duplicate servo id or name", servo.name));
            }
        }
        let n = servos.len();
        Ok(Self {
            port,
            servos,
            readings: vec![Reading::default(); n],
            samples: vec![JointFeedback::default(); n],
            torque: vec![false; n],
            gains: vec![None; n],
            enabled: false,
            transmit,
            now_ms: 0,
            requests: 0,
            replies: 0,
            reply_timeout_us: 4000 + 1000 * n as u64,
            virtual_enable: false,
            reference: None,
            failure: None,
            dropped: None,
        })
    }
    pub fn failure(&self) -> Option<&str> {
        self.failure.as_deref()
    }
    pub fn dropped(&self) -> Option<&str> {
        self.dropped.as_deref()
    }
    /// Only for the simulated bus: enable without the reference-pose gate.
    pub fn allow_virtual_enable(&mut self) {
        self.virtual_enable = true;
    }
    pub fn configure_reference(
        &mut self,
        references: Vec<crate::reference::Reference>,
    ) -> Result<(), &'static str> {
        if self.enabled || !self.transmit {
            return Err("reference check requires a disabled transmitter");
        }
        let names: Vec<_> = self.servos.iter().map(|s| s.name.as_str()).collect();
        self.reference = Some(crate::reference::Gate::new(references, &names)?);
        self.virtual_enable = false;
        Ok(())
    }
    pub fn reference_status(&self) -> &'static str {
        if self.virtual_enable {
            "virtual_bus"
        } else {
            self.reference
                .as_ref()
                .map_or("unconfigured", |g| g.status(self.now_ms))
        }
    }
    /// The servos keep no command watchdog: one that stops hearing from the receiver holds its last goal.
    pub fn watchdog_status(&self) -> &'static str {
        "not_applicable"
    }

    fn send(&mut self, packet: &[u8]) -> Result<(), String> {
        self.port
            .clear_input()
            .map_err(|_| "serial input could not be cleared".to_string())?;
        self.port
            .send(packet)
            .map_err(|_| "serial write failed".to_string())?;
        self.requests += 1;
        Ok(())
    }
    fn write_all(&mut self, address: u8, values: &[(u8, Vec<u8>)]) -> Result<(), String> {
        if !self.transmit {
            return Err("observation never writes".into());
        }
        if values.is_empty() {
            return Ok(());
        }
        let mut params = vec![address, values[0].1.len() as u8];
        for (id, data) in values {
            params.push(*id);
            params.extend_from_slice(data);
        }
        self.send(&instruction(BROADCAST, SYNC_WRITE, &params))
    }
    /// The state block of every servo, in roster order.
    fn read_states(&mut self) -> Result<Vec<(u8, [u8; STATE_BYTES])>, String> {
        let mut params = vec![STATE_START, STATE_BYTES as u8];
        params.extend(self.servos.iter().map(|s| s.id));
        self.send(&instruction(BROADCAST, SYNC_READ, &params))?;
        let size = STATE_BYTES + 6;
        let mut replies = vec![0u8; size * self.servos.len()];
        let got = self
            .port
            .receive(&mut replies, self.reply_timeout_us)
            .map_err(|_| "serial read failed".to_string())?;
        let mut states = Vec::with_capacity(self.servos.len());
        for (i, servo) in self.servos.iter().enumerate() {
            let reply = &replies[i * size..((i + 1) * size).min(got.max(i * size))];
            if reply.len() < 3 || reply[2] != servo.id {
                return Err(format!(
                    "{}: no reply to the state read (servo id {})",
                    servo.name, servo.id
                ));
            }
            let (error, data) = parse_status(reply, servo.id, STATE_BYTES)
                .map_err(|e| format!("{}: {e}", servo.name))?;
            let mut block = [0u8; STATE_BYTES];
            block.copy_from_slice(data);
            states.push((error, block));
            self.replies += 1;
        }
        Ok(states)
    }
    /// Read one servo's registers (identity checks before a session).
    pub fn read_registers(
        &mut self,
        index: usize,
        address: u8,
        count: u8,
    ) -> Result<Vec<u8>, String> {
        let id = self.servos[index].id;
        self.send(&instruction(id, READ, &[address, count]))?;
        let mut reply = vec![0u8; usize::from(count) + 6];
        let got = self
            .port
            .receive(&mut reply, self.reply_timeout_us)
            .map_err(|_| "serial read failed".to_string())?;
        reply.truncate(got);
        let (error, data) = parse_status(&reply, id, usize::from(count))?;
        if error != 0 {
            return Err(format!(
                "{}: {}",
                self.servos[index].name,
                alarm_names(error)
            ));
        }
        self.replies += 1;
        Ok(data.to_vec())
    }
    /// Model and firmware of every servo against the roster: (model, "major.minor") per servo on mismatch.
    pub fn verify_identity(&mut self) -> Result<(), String> {
        for i in 0..self.servos.len() {
            let block = self.read_registers(i, 0, 5)?;
            let model = u16::from_le_bytes([block[3], block[4]]);
            let firmware = format!("{}.{}", block[0], block[1]);
            let servo = &self.servos[i];
            if model != servo.model || firmware != servo.firmware {
                return Err(format!(
                    "{}: servo id {} is model {model} firmware {firmware}, expected model {} firmware {}",
                    servo.name, servo.id, servo.model, servo.firmware
                ));
            }
        }
        Ok(())
    }

    fn read_inner(&mut self, now_ms: u64, output: &mut [JointFeedback]) -> Result<(), String> {
        self.now_ms = now_ms;
        if output.len() != self.servos.len() {
            return Err("feedback buffer does not match the servo roster".into());
        }
        for (i, (error, block)) in self.read_states()?.into_iter().enumerate() {
            let word = |at: usize| u16::from_le_bytes([block[at - 40], block[at - 39]]);
            let reading = Reading {
                torque: block[0] != 0,
                locked: block[55 - 40] != 0,
                steps: signed(word(56), 15),
                speed: signed(word(58), 15),
                load: signed(word(60), 10),
                voltage_dv: block[62 - 40],
                temperature: block[63 - 40],
                error,
            };
            let servo = &self.servos[i];
            if !reading.locked && self.transmit {
                return Err(format!(
                    "{}: EEPROM unlocked; written gains would persist",
                    servo.name
                ));
            }
            if self.enabled && self.torque[i] && !reading.torque && error == 0 {
                // It lost torque while driven without reporting a fault: it rebooted (lost power). Nothing
                // re-enables it here, so its readings are withheld and it reads as not answering.
                self.dropped.get_or_insert_with(|| {
                    format!("{}: lost torque while driven (it rebooted)", servo.name)
                });
                self.readings[i] = reading;
                continue;
            }
            let raw = f64::from(reading.steps) * TAU / RESOLUTION;
            let raw_velocity = f64::from(reading.speed) * TAU / RESOLUTION;
            if let Some(gate) = &mut self.reference {
                let feedback = robstride::Feedback {
                    motor_id: servo.id,
                    mode: if reading.torque { 2 } else { 0 },
                    error_bits: error,
                    position: raw,
                    velocity: raw_velocity,
                    effort: 0.0,
                    temperature: f64::from(reading.temperature),
                };
                gate.observe(i, feedback, now_ms, now_ms).map_err(|_| {
                    gate.failure
                        .clone()
                        .unwrap_or_else(|| "reference check failed".into())
                })?;
            }
            self.samples[i] = JointFeedback {
                position: servo.direction * (f64::from(reading.steps) - servo.zero_steps) * TAU
                    / RESOLUTION,
                velocity: servo.direction * raw_velocity,
                effort: servo.direction * f64::from(reading.load) / 1000.0 * servo.full_load_nm,
                temperature: f64::from(reading.temperature),
                sequence: self.samples[i]
                    .sequence
                    .checked_add(1)
                    .ok_or("feedback sequence overflow")?,
                sampled_at_ms: now_ms,
                fault: error != 0,
            };
            if error != 0 {
                self.failure
                    .get_or_insert_with(|| format!("{}: {}", servo.name, alarm_names(error)));
            }
            self.readings[i] = reading;
        }
        output.copy_from_slice(&self.samples);
        Ok(())
    }

    fn apply_inner(&mut self, commands: &[JointCommand]) -> Result<(), String> {
        if !self.enabled || commands.len() != self.servos.len() {
            return Err("commands while disabled, or for another roster".into());
        }
        // Convert and validate ALL commands before writing anything.
        let mut goals = Vec::with_capacity(commands.len());
        let mut wanted = Vec::with_capacity(commands.len());
        for (servo, command) in self.servos.iter().zip(commands) {
            let steps = servo.steps(command.position).round();
            let p = command.kp.round();
            let d = command.kd.round();
            if !steps.is_finite()
                || !(0.0..RESOLUTION).contains(&steps)
                || !(0.0..=255.0).contains(&p)
                || !(0.0..=255.0).contains(&d)
            {
                return Err(format!(
                    "{}: command outside the servo's position or gain range",
                    servo.name
                ));
            }
            goals.push((servo.id, (steps as u16).to_le_bytes().to_vec()));
            wanted.push((p > 0.0, (p as u8, d as u8)));
        }
        self.write_all(GOAL_POSITION, &goals)?;
        let gains: Vec<_> = (0..wanted.len())
            .filter(|&i| wanted[i].0 && self.gains[i] != Some(wanted[i].1))
            .map(|i| (self.servos[i].id, vec![wanted[i].1 .0, wanted[i].1 .1]))
            .collect();
        self.write_all(P_COEFFICIENT, &gains)?;
        for (written, (on, gains)) in self.gains.iter_mut().zip(&wanted) {
            if *on {
                *written = Some(*gains);
            }
        }
        let torque: Vec<_> = (0..wanted.len())
            .filter(|&i| wanted[i].0 != self.torque[i])
            .map(|i| (self.servos[i].id, vec![u8::from(wanted[i].0)]))
            .collect();
        self.write_all(TORQUE_ENABLE, &torque)?;
        for (i, (on, _)) in wanted.iter().enumerate() {
            self.torque[i] = *on;
        }
        Ok(())
    }
}

impl<P: SerialPort> Hardware for FeetechHardware<P> {
    fn read(&mut self, now_ms: u64, output: &mut [JointFeedback]) -> Result<(), HardwareError> {
        let Err(reason) = self.read_inner(now_ms, output) else {
            return Ok(());
        };
        self.failure.get_or_insert(reason);
        if let Some(gate) = &mut self.reference {
            gate.revoke();
        }
        Err(HardwareError)
    }
    fn enable(&mut self) -> Result<(), HardwareError> {
        if !self.transmit {
            return Err(HardwareError);
        }
        if !self.virtual_enable {
            self.reference
                .as_mut()
                .ok_or(HardwareError)?
                .consume(self.now_ms)?;
        }
        // Goals start where each servo is, so torque coming on never jumps; torque itself follows the first
        // command's gains (zero gains stay limp).
        let goals: Vec<_> = self
            .servos
            .iter()
            .zip(&self.readings)
            .map(|(s, r)| (s.id, (r.steps.clamp(0, 4095) as u16).to_le_bytes().to_vec()))
            .collect();
        self.enabled = true;
        self.write_all(GOAL_POSITION, &goals).map_err(|reason| {
            self.failure.get_or_insert(reason);
            HardwareError
        })
    }
    fn disable(&mut self) -> Result<(), HardwareError> {
        let was_enabled = self.enabled;
        self.enabled = false;
        if let Some(gate) = &mut self.reference {
            gate.revoke();
        }
        if !self.transmit || !was_enabled {
            return Ok(());
        }
        let off: Vec<_> = self.servos.iter().map(|s| (s.id, vec![0u8])).collect();
        self.torque.fill(false);
        self.write_all(TORQUE_ENABLE, &off)
            .map_err(|_| HardwareError)
    }
    fn apply(&mut self, commands: &[JointCommand]) -> Result<(), HardwareError> {
        self.apply_inner(commands).map_err(|reason| {
            self.failure.get_or_insert(reason);
            HardwareError
        })
    }
}

/// Simulated STS servos answering the same bytes, for tests and the virtual head receiver. A slew
/// toward the goal at a fixed speed, not the servo's dynamics.
pub mod sim {
    use super::*;

    #[derive(Clone)]
    pub struct SimServo {
        pub memory: [u8; 128],
        pub silent: bool,
        pub error_bits: u8,
        pub speed_steps: f64,
        position: f64,
    }
    impl SimServo {
        pub fn new(id: u8, steps: u16) -> Self {
            let mut memory = [0u8; 128];
            memory[0] = 3;
            memory[1] = 10;
            memory[3..5].copy_from_slice(&STS3250.to_le_bytes());
            memory[5] = id;
            memory[21] = 32;
            memory[22] = 32;
            memory[55] = 1;
            memory[56..58].copy_from_slice(&steps.to_le_bytes());
            memory[42..44].copy_from_slice(&steps.to_le_bytes());
            memory[62] = 120;
            memory[63] = 30;
            Self {
                memory,
                silent: false,
                error_bits: 0,
                speed_steps: 2900.0,
                position: f64::from(steps),
            }
        }
        pub fn word(&self, at: usize) -> u16 {
            u16::from_le_bytes([self.memory[at], self.memory[at + 1]])
        }
        pub fn advance(&mut self, dt: f64) {
            let torque = self.memory[40] != 0;
            let goal = f64::from(self.word(42) & 0x7FFF);
            let step = if torque {
                (goal - self.position).clamp(-self.speed_steps * dt, self.speed_steps * dt)
            } else {
                0.0
            };
            self.position += step;
            let velocity = if dt > 0.0 { step / dt } else { 0.0 };
            let encode = |value: f64| -> u16 {
                let magnitude = value.abs().round() as u16 & 0x7FFF;
                if value < 0.0 {
                    magnitude | 0x8000
                } else {
                    magnitude
                }
            };
            let steps = self.position.round() as u16;
            self.memory[56..58].copy_from_slice(&steps.to_le_bytes());
            self.memory[58..60].copy_from_slice(&encode(velocity).to_le_bytes());
        }
        /// A hand turns a limp servo.
        pub fn turn_to(&mut self, steps: u16) {
            if self.memory[40] == 0 {
                self.position = f64::from(steps);
                self.memory[56..58].copy_from_slice(&steps.to_le_bytes());
            }
        }
    }

    #[derive(Default)]
    pub struct SimBus {
        pub servos: Vec<SimServo>,
        pending: Vec<u8>,
        pub log: Vec<(u8, u8)>,
    }
    impl SimBus {
        pub fn new(servos: Vec<SimServo>) -> Self {
            Self {
                servos,
                ..Self::default()
            }
        }
        fn reply(&mut self, index: usize, params: &[u8]) {
            let servo = &self.servos[index];
            if servo.silent {
                return;
            }
            let mut packet = vec![
                0xFF,
                0xFF,
                servo.memory[5],
                (params.len() + 2) as u8,
                servo.error_bits,
            ];
            packet.extend_from_slice(params);
            packet.push(checksum(&packet[2..]));
            self.pending.extend(packet);
        }
        fn find(&self, id: u8) -> Option<usize> {
            self.servos.iter().position(|s| s.memory[5] == id)
        }
    }
    impl SerialPort for SimBus {
        fn clear_input(&mut self) -> Result<(), HardwareError> {
            self.pending.clear();
            Ok(())
        }
        fn advance(&mut self, dt: f64) {
            for servo in &mut self.servos {
                servo.advance(dt);
            }
        }
        fn send(&mut self, bytes: &[u8]) -> Result<(), HardwareError> {
            if bytes.len() < 6
                || bytes[0] != 0xFF
                || bytes[1] != 0xFF
                || usize::from(bytes[3]) != bytes.len() - 4
                || checksum(&bytes[2..bytes.len() - 1]) != bytes[bytes.len() - 1]
            {
                return Ok(()); // A servo ignores what it cannot frame.
            }
            let (id, code, params) = (bytes[2], bytes[4], &bytes[5..bytes.len() - 1]);
            self.log.push((code, id));
            match code {
                READ if params.len() == 2 => {
                    if let Some(i) = self.find(id) {
                        let (at, n) = (usize::from(params[0]), usize::from(params[1]));
                        let data = self.servos[i].memory[at..at + n].to_vec();
                        self.reply(i, &data);
                    }
                }
                SYNC_READ if id == BROADCAST && params.len() >= 3 => {
                    let (at, n) = (usize::from(params[0]), usize::from(params[1]));
                    for &target in &params[2..] {
                        if let Some(i) = self.find(target) {
                            let data = self.servos[i].memory[at..at + n].to_vec();
                            self.reply(i, &data);
                        }
                    }
                }
                SYNC_WRITE if id == BROADCAST && params.len() >= 2 => {
                    let (at, n) = (usize::from(params[0]), usize::from(params[1]));
                    for chunk in params[2..].chunks(n + 1) {
                        if chunk.len() == n + 1 {
                            if let Some(i) = self.find(chunk[0]) {
                                self.servos[i].memory[at..at + n].copy_from_slice(&chunk[1..]);
                            }
                        }
                    }
                }
                _ => {}
            }
            Ok(())
        }
        fn receive(&mut self, buffer: &mut [u8], _timeout_us: u64) -> Result<usize, HardwareError> {
            let n = buffer.len().min(self.pending.len());
            buffer[..n].copy_from_slice(&self.pending[..n]);
            self.pending.drain(..n);
            Ok(n)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::sim::{SimBus, SimServo};
    use super::*;
    use crate::hardware::Hardware;

    fn servo(name: &str, id: u8, zero: f64) -> Servo {
        Servo {
            name: name.into(),
            id,
            model: STS3250,
            firmware: "3.10".into(),
            direction: if id == 2 { -1.0 } else { 1.0 },
            zero_steps: zero,
            full_load_nm: 4.9,
        }
    }
    fn head(transmit: bool) -> FeetechHardware<SimBus> {
        let bus = SimBus::new(vec![SimServo::new(1, 2048), SimServo::new(2, 1000)]);
        FeetechHardware::new(
            bus,
            vec![servo("head_pan", 1, 2048.0), servo("head_tilt", 2, 1024.0)],
            transmit,
        )
        .unwrap()
    }
    fn hold(position: f64, kp: f64) -> JointCommand {
        JointCommand {
            position,
            velocity: 0.0,
            effort: 0.0,
            kp,
            kd: if kp > 0.0 { 8.0 } else { 0.0 },
        }
    }

    #[test]
    fn packets_and_encodings_match_the_protocol() {
        assert_eq!(
            instruction(1, 0x01, &[]),
            [0xFF, 0xFF, 0x01, 0x02, 0x01, 0xFB]
        );
        assert_eq!(
            instruction(1, READ, &[0x38, 2]),
            [0xFF, 0xFF, 0x01, 0x04, 0x02, 0x38, 0x02, 0xBE]
        );
        assert_eq!(
            (signed(0x8005, 15), signed(5, 15), signed(0x0400 | 100, 10)),
            (-5, 5, -100)
        );
        assert!(parse_status(&[0xFF, 0xFF, 1, 2, 0, 0xFC], 1, 0).is_ok());
        assert!(parse_status(&[0xFF, 0xFF, 1, 2, 0, 0xFD], 1, 0)
            .unwrap_err()
            .contains("checksum"));
        assert!(parse_status(&[0xFF, 0xFF, 2, 2, 0, 0xFB], 1, 0)
            .unwrap_err()
            .contains("malformed"));
        assert_eq!(alarm_names(0x24), "overheat, overload");
    }

    #[test]
    fn reads_map_steps_to_joint_radians_and_verify_identity() {
        let mut hw = head(true);
        hw.verify_identity().unwrap();
        let mut feedback = [JointFeedback::default(); 2];
        hw.read(10, &mut feedback).unwrap();
        assert!(feedback[0].position.abs() < 1e-12);
        // Tilt reads 1000 steps against its zero of 1024, reversed: +24 steps.
        assert!((feedback[1].position - 24.0 * TAU / RESOLUTION).abs() < 1e-12);
        assert_eq!(
            (
                feedback[0].sequence,
                feedback[0].sampled_at_ms,
                feedback[0].temperature
            ),
            (1, 10, 30.0)
        );
        hw.port.servos[1].memory[3] = 7; // Another model (2823) answers at id 2.
        assert!(hw.verify_identity().unwrap_err().contains("head_tilt"));
    }

    #[test]
    fn zero_gains_stay_limp_and_gains_hold_the_goal() {
        let mut hw = head(true);
        hw.allow_virtual_enable();
        let mut feedback = [JointFeedback::default(); 2];
        hw.read(0, &mut feedback).unwrap();
        hw.enable().unwrap();
        hw.apply(&[hold(0.0, 0.0), hold(feedback[1].position, 0.0)])
            .unwrap();
        assert_eq!(
            (hw.port.servos[0].memory[40], hw.port.servos[1].memory[40]),
            (0, 0)
        );
        hw.apply(&[hold(0.5, 20.0), hold(feedback[1].position, 16.0)])
            .unwrap();
        let pan = &hw.port.servos[0];
        assert_eq!((pan.memory[40], pan.memory[21], pan.memory[22]), (1, 20, 8));
        assert_eq!(
            pan.word(42),
            (2048.0 + 0.5 * RESOLUTION / TAU).round() as u16
        );
        hw.port.advance(1.0);
        hw.read(1000, &mut feedback).unwrap();
        assert!((feedback[0].position - 0.5).abs() < TAU / RESOLUTION);
        let writes = hw
            .port
            .log
            .iter()
            .filter(|(code, _)| *code == SYNC_WRITE)
            .count();
        hw.apply(&[hold(0.5, 20.0), hold(feedback[1].position, 16.0)])
            .unwrap();
        // Unchanged gains and torque: only the goals are written again.
        assert_eq!(
            hw.port
                .log
                .iter()
                .filter(|(code, _)| *code == SYNC_WRITE)
                .count(),
            writes + 1
        );
        hw.disable().unwrap();
        assert_eq!(
            (hw.port.servos[0].memory[40], hw.port.servos[1].memory[40]),
            (0, 0)
        );
    }

    #[test]
    fn faults_silence_and_reboots_are_reported_by_name() {
        let mut hw = head(true);
        let mut feedback = [JointFeedback::default(); 2];
        hw.port.servos[1].silent = true;
        assert!(hw.read(0, &mut feedback).is_err());
        assert!(hw.failure().unwrap().contains("head_tilt: no reply"));
        let mut hot = head(true);
        hot.port.servos[0].error_bits = 4;
        hot.read(0, &mut feedback).unwrap();
        assert!(feedback[0].fault && hot.failure().unwrap().contains("head_pan: overheat"));
        let mut rebooted = head(true);
        rebooted.allow_virtual_enable();
        rebooted.read(0, &mut feedback).unwrap();
        rebooted.enable().unwrap();
        rebooted
            .apply(&[hold(0.0, 20.0), hold(feedback[1].position, 20.0)])
            .unwrap();
        rebooted.port.servos[0].memory[40] = 0; // Lost power and came back limp.
        rebooted.read(5, &mut feedback).unwrap();
        assert!(rebooted
            .dropped()
            .unwrap()
            .contains("head_pan: lost torque"));
        assert_eq!(
            feedback[0].sequence, 1,
            "a rebooted servo's reading is withheld"
        );
    }

    #[test]
    fn observation_never_writes_and_bad_commands_write_nothing() {
        let mut watcher = head(false);
        let mut feedback = [JointFeedback::default(); 2];
        watcher.read(0, &mut feedback).unwrap();
        assert!(watcher.enable().is_err());
        assert!(watcher.port.log.iter().all(|(code, _)| *code == SYNC_READ));
        let mut hw = head(true);
        hw.allow_virtual_enable();
        hw.read(0, &mut feedback).unwrap();
        hw.enable().unwrap();
        let before = hw.port.log.len();
        assert!(hw.apply(&[hold(4.0, 20.0), hold(0.0, 20.0)]).is_err()); // Beyond the single turn.
        assert!(hw.apply(&[hold(0.0, 300.0), hold(0.0, 20.0)]).is_err()); // Beyond the gain register.
        assert_eq!(hw.port.log.len(), before);
        let mut unlocked = head(true);
        unlocked.port.servos[0].memory[55] = 0;
        assert!(unlocked.read(1, &mut feedback).is_err());
        assert!(unlocked
            .failure()
            .unwrap()
            .contains("head_pan: EEPROM unlocked"));
    }

    #[test]
    fn enable_needs_the_reference_pose() {
        let mut hw = head(true);
        let at = |steps: f64| steps * TAU / RESOLUTION;
        hw.configure_reference(vec![
            crate::reference::Reference {
                name: "head_pan".into(),
                raw_position_rad: at(2048.0),
                tolerance_rad: 0.05,
            },
            crate::reference::Reference {
                name: "head_tilt".into(),
                raw_position_rad: at(1000.0),
                tolerance_rad: 0.05,
            },
        ])
        .unwrap();
        let mut feedback = [JointFeedback::default(); 2];
        assert!(hw.enable().is_err());
        for t in [0, 50, 100, 120] {
            hw.read(t, &mut feedback).unwrap();
        }
        assert_eq!(hw.reference_status(), "ready");
        hw.enable().unwrap();
        assert_eq!(hw.reference_status(), "consumed");
    }

    #[test]
    fn the_reference_pose_and_the_first_enable_share_one_clock() {
        // Helm's first live head jog (2026-10-30): the service proved the pose on one clock, then served on one
        // restarted at zero. Readings stamped before the gate's newest are ignored, so the enable 91 ms in was
        // refused and the receiver faulted. The service now proves it on its serve loop's own clock.
        let at = |steps: f64| steps * TAU / RESOLUTION;
        let proved = || {
            let mut hw = head(true);
            hw.configure_reference(vec![
                crate::reference::Reference {
                    name: "head_pan".into(),
                    raw_position_rad: at(2048.0),
                    tolerance_rad: 0.05,
                },
                crate::reference::Reference {
                    name: "head_tilt".into(),
                    raw_position_rad: at(1000.0),
                    tolerance_rad: 0.05,
                },
            ])
            .unwrap();
            let mut feedback = [JointFeedback::default(); 2];
            for t in (0..=110).step_by(10) {
                hw.read(t, &mut feedback).unwrap();
            }
            assert_eq!(hw.reference_status(), "ready");
            hw
        };
        let mut feedback = [JointFeedback::default(); 2];
        let mut restarted = proved();
        for t in (0..=91).step_by(10) {
            restarted.read(t, &mut feedback).unwrap();
        }
        assert_eq!(restarted.reference_status(), "unverified");
        assert!(restarted.enable().is_err());
        let mut continued = proved();
        continued.read(120, &mut feedback).unwrap();
        continued.enable().unwrap();
        assert_eq!(continued.reference_status(), "consumed");
    }
}
