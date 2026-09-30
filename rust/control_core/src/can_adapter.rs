//! Bounded generic CAN hardware adapter, shared by Linux and deterministic tests.
use crate::{
    hardware::{Hardware, HardwareError, JointFeedback},
    protocol::JointCommand,
    robstride::{self, Frame, Ranges},
};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Motor {
    pub name: String,
    pub bus: usize,
    pub id: u8,
    pub ranges: Ranges,
    /// q_joint = scale * (q_motor - zero). Gains/effort preserve virtual work.
    pub scale: f64,
    pub zero: f64,
}
impl Motor {
    pub fn to_motor(&self, c: JointCommand) -> JointCommand {
        JointCommand {
            position: c.position / self.scale + self.zero,
            velocity: c.velocity / self.scale,
            effort: c.effort * self.scale,
            kp: c.kp * self.scale * self.scale,
            kd: c.kd * self.scale * self.scale,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct Received {
    pub bus: usize,
    pub frame: Frame,
    /// Kernel reception time converted to the receiver's monotonic domain.
    pub sampled_at_ms: u64,
}
pub trait CanIo {
    /// At most one nonblocking read. Errors include overflow/bus-off/timestamp loss.
    fn receive(&mut self, now_ms: u64) -> Result<Option<Received>, HardwareError>;
    /// One nonblocking write. EAGAIN/ENOBUFS is an error; never retry/sleep here.
    fn send(&mut self, bus: usize, frame: Frame) -> Result<(), HardwareError>;
}

pub struct CanHardware<I: CanIo> {
    pub io: I,
    pub motors: Vec<Motor>,
    host: u8,
    samples: Vec<JointFeedback>,
    modes: Vec<u8>,
    /// Reported Run mode since the last enable: leaving it later means the drive dropped out.
    running: Vec<bool>,
    frames: Vec<Frame>,
    pub enabled: bool,
    /// Physical observation mode blocks ALL writes, including disable.
    pub transmit: bool,
    enabled_at: u64,
    now_ms: u64,
    pub tx_frames: u64,
    pub rx_frames: u64,
    pub max_drain: usize,
    watchdog: Option<crate::watchdog::Gate>,
    virtual_enable: bool,
    reference: Option<crate::reference::Gate>,
    /// The first read failure, in words: which motor and check, and the value that failed it.
    failure: Option<String>,
    /// The first drive that left Run mode while driven, in words (not a failure: it reads as quiet).
    dropped: Option<String>,
}
impl<I: CanIo> Drop for CanHardware<I> {
    fn drop(&mut self) {
        let _ = self.disable();
    }
}
impl<I: CanIo> CanHardware<I> {
    pub fn new(io: I, motors: Vec<Motor>, host: u8, transmit: bool) -> Result<Self, &'static str> {
        if motors.is_empty() || motors.len() > 256 || host == 0 {
            return Err("invalid CAN roster");
        }
        for (i, motor) in motors.iter().enumerate() {
            motor.ranges.validate()?;
            if motor.name.is_empty()
                || motor.id == 0
                || motor.id == host
                || !motor.scale.is_finite()
                || motor.scale.abs() < 1e-9
                || !motor.zero.is_finite()
                || motors[..i]
                    .iter()
                    .any(|m| m.name == motor.name || (m.bus == motor.bus && m.id == motor.id))
            {
                return Err("invalid or duplicate motor mapping");
            }
        }
        let n = motors.len();
        Ok(Self {
            io,
            motors,
            host,
            samples: vec![JointFeedback::default(); n],
            modes: vec![0; n],
            running: vec![false; n],
            frames: vec![
                Frame {
                    id: 0,
                    data: [0; 8]
                };
                n
            ],
            enabled: false,
            transmit,
            enabled_at: 0,
            now_ms: 0,
            tx_frames: 0,
            rx_frames: 0,
            max_drain: 128,
            watchdog: None,
            virtual_enable: false,
            reference: None,
            failure: None,
            dropped: None,
        })
    }
    /// Why the first failed read failed, for operators and evidence.
    pub fn failure(&self) -> Option<&str> {
        self.failure.as_deref()
    }
    /// Which drive first left Run mode while driven, for operators and evidence.
    pub fn dropped(&self) -> Option<&str> {
        self.dropped.as_deref()
    }
    /// Linux callers may use this only after verifying kernel vcan interfaces.
    pub fn allow_virtual_enable(&mut self) {
        self.virtual_enable = true;
    }
    pub fn configure_reference(
        &mut self,
        references: Vec<crate::reference::Reference>,
    ) -> Result<(), &'static str> {
        if self.enabled || !self.transmit {
            return Err("reference check requires disabled transmitter");
        }
        let names: Vec<_> = self.motors.iter().map(|m| m.name.as_str()).collect();
        self.reference = Some(crate::reference::Gate::new(references, &names)?);
        self.virtual_enable = false;
        Ok(())
    }
    pub fn reference_status(&self) -> &'static str {
        if self.virtual_enable {
            "virtual_can"
        } else {
            self.reference
                .as_ref()
                .map_or("unconfigured", |g| g.status(self.now_ms))
        }
    }
    pub fn configure_watchdogs(
        &mut self,
        identities: Vec<crate::watchdog_trial::Identity>,
    ) -> Result<(), &'static str> {
        if self.enabled || !self.transmit {
            return Err("watchdog setup requires disarmed transmitter");
        }
        self.watchdog = Some(crate::watchdog::Gate::new(identities, self.motors.len())?);
        self.virtual_enable = false;
        Ok(())
    }
    pub fn watchdog_status(&self) -> &'static str {
        if self.virtual_enable {
            "virtual_can"
        } else {
            self.watchdog
                .as_ref()
                .map_or("unconfigured", |g| g.status(self.now_ms))
        }
    }
    fn send(&mut self, index: usize, frame: Frame) -> Result<(), HardwareError> {
        if !self.transmit {
            return Err(HardwareError);
        }
        self.io.send(self.motors[index].bus, frame)?;
        self.tx_frames += 1;
        Ok(())
    }
    /// Disarmed parameter observation is a separate, explicitly authorized trial.
    /// It does not populate feedback with incomplete/default measurements.
    pub fn probe(&mut self, index: usize, parameter: u16) -> Result<(), HardwareError> {
        if self.enabled
            || index >= self.motors.len()
            || !matches!(parameter, 0x7019 | 0x701b | 0x701c)
        {
            return Err(HardwareError);
        }
        self.send(
            index,
            robstride::read_parameter(self.motors[index].id, self.host, parameter),
        )
    }

    /// Explicit commissioning diagnostics. Stop/version frames may remove torque;
    /// none of these commands enable, clear faults, zero, or write parameters.
    pub fn diagnostic(&mut self, index: usize, kind: Diagnostic) -> Result<(), HardwareError> {
        if self.enabled || index >= self.motors.len() {
            return Err(HardwareError);
        }
        let id = self.motors[index].id;
        let frame = match kind {
            Diagnostic::Identity => robstride::management(id, self.host, 0),
            Diagnostic::Version => robstride::version_request(id, self.host),
            Diagnostic::StopFeedback => robstride::management(id, self.host, 4),
            Diagnostic::Parameter { parameter } => {
                if !matches!(
                    parameter,
                    0x7005 | 0x700b | 0x7017..=0x701c | 0x7026 | 0x7028
                ) {
                    return Err(HardwareError);
                }
                robstride::read_parameter(id, self.host, parameter)
            }
        };
        self.send(index, frame)
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum Diagnostic {
    Identity,
    Version,
    StopFeedback,
    Parameter { parameter: u16 },
}
impl<I: CanIo> CanHardware<I> {
    fn read_inner(&mut self, now_ms: u64, output: &mut [JointFeedback]) -> Result<(), String> {
        self.now_ms = now_ms;
        if output.len() != self.samples.len() {
            return Err("feedback buffer does not match the motor roster".into());
        }
        let mut empty = false;
        for _ in 0..self.max_drain {
            let Some(rx) = self.io.receive(now_ms).map_err(|_| {
                String::from(
                    "CAN receive failed (overflow, bus-off, error frame or lost timestamp)",
                )
            })?
            else {
                empty = true;
                break;
            };
            self.rx_frames += 1;
            if let Some(gate) = &mut self.watchdog {
                gate.observe(rx, &self.motors, self.host, now_ms)
                    .map_err(|_| {
                        gate.failure
                            .clone()
                            .unwrap_or_else(|| "watchdog check failed".into())
                    })?;
            }
            // A frame for another host/type cannot refresh feedback.
            if rx.frame.id >> 24 != 2
                || rx.frame.id & 255 != u32::from(self.host)
                || robstride::is_version(rx.frame)
            {
                continue;
            }
            let id = ((rx.frame.id >> 8) & 255) as u8;
            let Some(i) = self
                .motors
                .iter()
                .position(|m| m.id == id && m.bus == rx.bus)
            else {
                continue;
            };
            let m = &self.motors[i];
            let f = robstride::feedback(rx.frame, self.host, m.ranges)
                .map_err(|_| format!("{}: undecodable feedback", m.name))?;
            if self.enabled && f.mode == 2 {
                self.running[i] = true;
            } else if self.enabled && self.running[i] && f.mode == 0 && f.error_bits == 0 {
                // It left Run mode while driven, without a fault: it rebooted (lost power) or its own
                // command-loss watchdog disabled it, and no longer holds its joint. Nothing re-enables it
                // here (its watchdog would not be armed), so its readings are withheld: it reads as not
                // answering, and the receiver holds the arm on the drives still running. A drive that
                // reports a fault still cuts torque.
                self.dropped.get_or_insert_with(|| {
                    format!("{}: left Run mode while driven (it rebooted, or its own watchdog disabled it)",
                            m.name)
                });
                continue;
            }
            if let Some(gate) = &mut self.reference {
                gate.observe(i, f, rx.sampled_at_ms, now_ms).map_err(|_| {
                    gate.failure
                        .clone()
                        .unwrap_or_else(|| "reference check failed".into())
                })?;
            }
            if rx.sampled_at_ms > now_ms || rx.sampled_at_ms < self.samples[i].sampled_at_ms {
                return Err(format!(
                    "{}: feedback stamped {} ms, now {} ms, previous sample {} ms",
                    m.name, rx.sampled_at_ms, now_ms, self.samples[i].sampled_at_ms
                ));
            }
            self.samples[i] = JointFeedback {
                position: m.scale * (f.position - m.zero),
                velocity: m.scale * f.velocity,
                effort: f.effort / m.scale,
                temperature: f.temperature,
                sequence: self.samples[i]
                    .sequence
                    .checked_add(1)
                    .ok_or_else(|| String::from("feedback sequence overflow"))?,
                sampled_at_ms: rx.sampled_at_ms,
                fault: f.error_bits != 0 || f.mode == 1,
            };
            self.modes[i] = f.mode;
        }
        // Never process an unbounded CAN backlog or give queued frames a new age.
        if !empty {
            return Err(format!(
                "more than {} CAN frames queued in one cycle",
                self.max_drain
            ));
        }
        if let Some(gate) = &mut self.watchdog {
            self.tx_frames += gate
                .poll(&mut self.io, &self.motors, self.host, now_ms)
                .map_err(|_| {
                    gate.failure
                        .clone()
                        .unwrap_or_else(|| "watchdog check failed".into())
                })?;
        }
        output.copy_from_slice(&self.samples);
        for (i, _sample) in output.iter_mut().enumerate() {
            if self.enabled && now_ms.saturating_sub(self.enabled_at) >= 50 && self.modes[i] != 2 {
                return Err(format!(
                    "{}: not in Run mode 50 ms after enable (mode {})",
                    self.motors[i].name, self.modes[i]
                ));
            }
        }
        Ok(())
    }
}
impl<I: CanIo> Hardware for CanHardware<I> {
    fn read(&mut self, now_ms: u64, output: &mut [JointFeedback]) -> Result<(), HardwareError> {
        let Err(reason) = self.read_inner(now_ms, output) else {
            return Ok(());
        };
        self.failure.get_or_insert(reason);
        if let Some(gate) = &mut self.reference {
            gate.revoke();
        }
        if let Some(gate) = &mut self.watchdog {
            self.tx_frames += gate.abort(&mut self.io, &self.motors, self.host);
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
            self.watchdog
                .as_mut()
                .ok_or(HardwareError)?
                .consume(self.now_ms)?;
        }
        self.enabled = true; // Partial enable must be followed by best-effort disable.
        self.enabled_at = self.now_ms;
        self.running.fill(false);
        // Stage a neutral command for every actuator before sending any enable.
        // Whether a particular firmware accepts it while disabled is a physical
        // commissioning question; the initial hold is also sent immediately.
        for i in 0..self.motors.len() {
            let m = &self.motors[i];
            self.frames[i] = robstride::control(
                m.id,
                m.ranges,
                m.to_motor(JointCommand {
                    position: self.samples[i].position,
                    velocity: 0.0,
                    effort: 0.0,
                    kp: 0.0,
                    kd: 0.0,
                }),
            )
            .map_err(|_| HardwareError)?;
        }
        for i in 0..self.motors.len() {
            // Physical startup verified the existing mode; never overwrite it.
            if self.virtual_enable {
                self.send(i, robstride::motion_mode(self.motors[i].id, self.host))?;
            }
            self.send(i, self.frames[i])?;
        }
        for i in 0..self.motors.len() {
            self.send(i, robstride::management(self.motors[i].id, self.host, 3))?;
        }
        Ok(())
    }
    fn disable(&mut self) -> Result<(), HardwareError> {
        let was_enabled = self.enabled;
        self.enabled = false;
        if let Some(gate) = &mut self.reference {
            gate.revoke();
        }
        if let Some(gate) = &mut self.watchdog {
            if self.transmit && !was_enabled && gate.status(self.now_ms) != "consumed" {
                self.tx_frames += gate.abort(&mut self.io, &self.motors, self.host);
            }
            gate.revoke();
        }
        if !self.transmit || !was_enabled {
            return Ok(());
        }
        let mut failed = false;
        for i in 0..self.motors.len() {
            failed |= self
                .send(i, robstride::management(self.motors[i].id, self.host, 4))
                .is_err();
        }
        if failed {
            Err(HardwareError)
        } else {
            Ok(())
        }
    }
    fn apply(&mut self, commands: &[JointCommand]) -> Result<(), HardwareError> {
        if !self.enabled || commands.len() != self.motors.len() {
            return Err(HardwareError);
        }
        // Convert/validate ALL entries before sending any frame.
        for ((m, command), frame) in self.motors.iter().zip(commands).zip(&mut self.frames) {
            *frame = robstride::control(m.id, m.ranges, m.to_motor(*command))
                .map_err(|_| HardwareError)?;
        }
        for i in 0..commands.len() {
            self.send(i, self.frames[i])?;
        }
        Ok(())
    }
}
