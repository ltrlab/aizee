//! Finite, single-actuator watchdog experiment with an optional bounded micro-jog.
//! Identity and watchdog readback must pass before nominal-zero-gain enable;
//! the optional jog additionally requires observed command-loss Reset behavior.
//! A final stop is attempted on every post-identity exit.
use crate::{
    can_adapter::{CanIo, Motor},
    protocol::JointCommand,
    robstride::{self, Frame},
};
use serde::{Deserialize, Serialize};

pub trait Clock {
    fn now_ms(&self) -> u64;
    fn wait_ms(&mut self, milliseconds: u64);
    fn interrupted(&self) -> bool {
        false
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Identity {
    pub uid: [u8; 8],
    pub firmware: [u8; 4],
}

#[derive(Serialize)]
pub struct Event {
    pub direction: &'static str,
    pub at_ms: u64,
    pub id: u32,
    pub data: [u8; 8],
}

#[derive(Default, Serialize)]
pub struct Report {
    pub success: bool,
    pub error: Option<String>,
    pub enable_requested: bool,
    pub previous_watchdog_ticks: Option<u32>,
    pub written_watchdog_ticks: Option<u32>,
    pub run_confirmed: bool,
    pub reset_after_silence: bool,
    pub silence_ms: u64,
    pub final_stop_confirmed: bool,
    pub final_stop_error: Option<String>,
    /// Reset acknowledges disabled drive, not zero velocity. Preserve both.
    pub final_stop_feedback: Option<robstride::Feedback>,
    pub jog: Option<JogReport>,
    pub breakaway: Option<BreakawayReport>,
    pub events: Vec<Event>,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Breakaway {
    pub direction: i8,
    pub expected_position_rad: f64,
    #[serde(default)]
    pub stop: BreakawayStop,
}
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum BreakawayStop {
    #[default]
    Immediate,
    Damped,
}

#[derive(Default, Serialize)]
pub struct DampedStopReport {
    pub started_at_ms: u64,
    pub finished_at_ms: Option<u64>,
    pub phases: Vec<StopPhase>,
    pub maximum_predicted_effort_nm: f64,
    pub settled_before_disable: bool,
}
#[derive(Serialize)]
pub struct StopPhase {
    pub name: &'static str,
    pub started_at_ms: u64,
    pub samples: u32,
}

/// Require a continuous quiet interval; one low velocity reading is insufficient.
struct Settling {
    since: u64,
    last: u64,
    minimum: f64,
    maximum: f64,
    samples: u32,
}
impl Settling {
    fn new(now: u64, position: f64) -> Self {
        Self {
            since: now,
            last: now,
            minimum: position,
            maximum: position,
            samples: 0,
        }
    }
    fn observe(&mut self, now: u64, f: robstride::Feedback) {
        self.minimum = self.minimum.min(f.position);
        self.maximum = self.maximum.max(f.position);
        if f.velocity.abs() > 0.1 || self.maximum - self.minimum > 0.0008 {
            *self = Self::new(now, f.position);
        } else {
            self.samples += 1;
        }
        self.last = now;
    }
    fn settled(&self) -> bool {
        self.samples >= 10 && self.last.saturating_sub(self.since) >= 100
    }
}
#[derive(Default, Serialize)]
pub struct BreakawayReport {
    pub direction: i8,
    pub stop: BreakawayStop,
    pub damped_stop: Option<DampedStopReport>,
    pub initial_position_rad: f64,
    pub final_position_rad: f64,
    pub maximum_displacement_rad: f64,
    pub maximum_reported_effort_nm: f64,
    pub maximum_reported_velocity_rad_s: f64,
    pub maximum_commanded_effort_nm: f64,
    pub command_before_motion_nm: f64,
    pub command_at_motion_nm: Option<f64>,
    pub reported_effort_at_motion_nm: Option<f64>,
    pub samples: u32,
    pub movement_observed: bool,
}
impl BreakawayReport {
    fn observe(&mut self, f: robstride::Feedback) -> bool {
        self.final_position_rad = f.position;
        self.maximum_displacement_rad = self
            .maximum_displacement_rad
            .max((f.position - self.initial_position_rad).abs());
        self.maximum_reported_velocity_rad_s =
            self.maximum_reported_velocity_rad_s.max(f.velocity.abs());
        self.maximum_reported_effort_nm = self.maximum_reported_effort_nm.max(f.effort.abs());
        f.velocity.abs() <= 0.2
            && f.effort.abs() <= 0.35
            && (f.position - self.initial_position_rad).abs() <= 0.02
    }
}

#[derive(Default, Serialize)]
pub struct JogReport {
    pub kp_nm_per_rad: f64,
    pub kd_nm_s_per_rad: f64,
    pub requested_displacement_rad: f64,
    pub initial_position_rad: f64,
    pub final_position_rad: f64,
    pub maximum_displacement_rad: f64,
    pub maximum_reported_effort_nm: f64,
    pub maximum_reported_velocity_rad_s: f64,
    pub samples: u32,
    pub movement_observed: bool,
}

struct Trial<'a, I: CanIo, C: Clock> {
    io: &'a mut I,
    clock: &'a mut C,
    motor: &'a Motor,
    host: u8,
    identity_verified: bool,
    started: u64,
    budget_ms: u64,
    report: Report,
}

impl<I: CanIo, C: Clock> Drop for Trial<'_, I, C> {
    fn drop(&mut self) {
        if self.identity_verified && !self.report.final_stop_confirmed {
            let _ = self.io.send(
                self.motor.bus,
                robstride::management(self.motor.id, self.host, 4),
            );
        }
    }
}

impl<I: CanIo, C: Clock> Trial<'_, I, C> {
    fn check_time(&self) -> Result<(), &'static str> {
        if self.clock.interrupted()
            || self.clock.now_ms().saturating_sub(self.started) > self.budget_ms
        {
            Err("trial interrupted or exceeded time budget")
        } else {
            Ok(())
        }
    }

    fn send(&mut self, frame: Frame) -> Result<u64, &'static str> {
        let now = self.clock.now_ms();
        self.io
            .send(self.motor.bus, frame)
            .map_err(|_| "CAN transmit failed")?;
        self.report.events.push(Event {
            direction: "tx",
            at_ms: now,
            id: frame.id,
            data: frame.data,
        });
        Ok(now)
    }

    fn receive(&mut self) -> Result<Option<(u64, Frame)>, &'static str> {
        let now = self.clock.now_ms();
        match self.io.receive(now).map_err(|_| "CAN receive failed")? {
            None => Ok(None),
            Some(rx) => {
                if rx.sampled_at_ms > now {
                    return Err("invalid reception timestamp");
                }
                if self.report.events.len() >= 2048 {
                    return Err("trial CAN event budget exhausted");
                }
                self.report.events.push(Event {
                    direction: "rx",
                    at_ms: rx.sampled_at_ms,
                    id: rx.frame.id,
                    data: rx.frame.data,
                });
                // The profile supplies exactly one CAN bus.
                if rx.bus != self.motor.bus {
                    return Err("unexpected bus");
                }
                Ok(Some((rx.sampled_at_ms, rx.frame)))
            }
        }
    }

    fn exchange(
        &mut self,
        frame: Frame,
        matches: impl Fn(Frame) -> bool,
    ) -> Result<Frame, &'static str> {
        self.check_time()?;
        // Drain old frames before issuing a request. They cannot acknowledge it.
        let mut empty = false;
        for _ in 0..128 {
            if self.receive()?.is_none() {
                empty = true;
                break;
            }
        }
        if !empty {
            return Err("CAN backlog before request");
        }
        let sent = self.send(frame)?;
        loop {
            self.check_time()?;
            if self.clock.now_ms().saturating_sub(sent) > 40 {
                return Err("diagnostic reply timeout");
            }
            let mut empty = false;
            for _ in 0..128 {
                match self.receive()? {
                    Some((stamp, reply)) if stamp >= sent && matches(reply) => return Ok(reply),
                    Some(_) => {}
                    None => {
                        empty = true;
                        break;
                    }
                }
            }
            if !empty {
                return Err("CAN receive flood");
            }
            self.clock.wait_ms(2);
        }
    }

    fn parameter(&mut self, parameter: u16) -> Result<Frame, &'static str> {
        let expected = (17 << 24) | (u32::from(self.motor.id) << 8) | u32::from(self.host);
        self.exchange(
            robstride::read_parameter(self.motor.id, self.host, parameter),
            |f| f.id == expected && f.data[..2] == parameter.to_le_bytes(),
        )
    }

    fn feedback(&mut self, frame: Frame) -> Result<robstride::Feedback, &'static str> {
        let id = self.motor.id;
        let host = self.host;
        let raw = self.exchange(frame, |f| {
            f.id >> 24 == 2
                && (f.id & 255) == u32::from(host)
                && ((f.id >> 8) & 255) == u32::from(id)
                && !robstride::is_version(f)
        })?;
        let feedback = robstride::feedback(raw, host, self.motor.ranges)?;
        if feedback.error_bits != 0 || feedback.temperature > 60.0 {
            return Err("motor fault or excessive temperature");
        }
        Ok(feedback)
    }

    fn execute(&mut self, identity: &Identity) -> Result<(), &'static str> {
        self.motor.ranges.validate()?;
        let id = self.motor.id;
        let host = self.host;
        let uid = self.exchange(robstride::management(id, host, 0), |f| {
            f.id == (u32::from(id) << 8) | 0xfe
        })?;
        if uid.data != identity.uid {
            return Err("motor UID mismatch");
        }
        self.identity_verified = true;
        let version = self.exchange(robstride::version_request(id, host), |f| {
            robstride::is_version(f)
                && ((f.id >> 8) & 255) == u32::from(id)
                && [u32::from(id), u32::from(host)].contains(&(f.id & 255))
        })?;
        if version.data[3..7] != identity.firmware {
            return Err("firmware mismatch");
        }
        let stopped = self.feedback(robstride::management(id, host, 4))?;
        if stopped.mode != 0 || stopped.velocity.abs() > 0.2 {
            return Err("motor must start stopped and stationary");
        }
        if self.parameter(0x7005)?.data[4] != 0 {
            return Err("motor must already use motion-control mode");
        }
        let old = self.parameter(0x7028)?;
        self.report.previous_watchdog_ticks =
            Some(u32::from_le_bytes(old.data[4..8].try_into().unwrap()));
        // 600 ticks is 30 ms in current manuals, 100 ms in some older firmware.
        // Verify behavior empirically; never infer it solely from a register value.
        let mut write = robstride::management(id, host, 18);
        write.data[..2].copy_from_slice(&0x7028u16.to_le_bytes());
        write.data[4..].copy_from_slice(&crate::watchdog::TICKS.to_le_bytes());
        self.check_time()?;
        self.send(write)?;
        let readback = self.parameter(0x7028)?;
        if u32::from_le_bytes(readback.data[4..8].try_into().unwrap()) != crate::watchdog::TICKS {
            return Err("watchdog readback mismatch; motor was not enabled");
        }
        self.report.written_watchdog_ticks = Some(crate::watchdog::TICKS);
        let neutral = robstride::control(
            id,
            self.motor.ranges,
            JointCommand {
                position: stopped.position,
                velocity: 0.0,
                effort: 0.0,
                kp: 0.0,
                kd: 0.0,
            },
        )?;
        if self.feedback(neutral)?.mode != 0 {
            return Err("neutral frame unexpectedly enabled motor");
        }
        self.enable_verified()?;
        let running = self.feedback(neutral)?;
        if running.mode != 2 {
            return Err("motor did not remain in Run mode");
        }
        self.report.run_confirmed = true;
        let silence_start = self.clock.now_ms();
        while self.clock.now_ms().saturating_sub(silence_start) < 150 {
            self.check_time()?;
            self.clock.wait_ms(2);
        }
        self.report.silence_ms = self.clock.now_ms().saturating_sub(silence_start);
        if self.report.silence_ms > 170 {
            return Err("trial scheduler exceeded silence budget");
        }
        // A neutral frame was proven above not to enable a stopped motor. Its
        // reply distinguishes autonomous Reset from our subsequent final stop.
        let after = self.feedback(neutral)?;
        self.report.reset_after_silence = after.mode == 0;
        if !self.report.reset_after_silence {
            return Err("motor did not enter Reset during command silence");
        }
        Ok(())
    }

    fn enable_verified(&mut self) -> Result<(), &'static str> {
        // Never reuse the readback from an earlier enable/disable cycle.
        let readback = self.parameter(0x7028)?;
        if u32::from_le_bytes(readback.data[4..].try_into().unwrap()) != crate::watchdog::TICKS {
            return Err("watchdog changed before enable");
        }
        self.report.enable_requested = true;
        if self
            .feedback(robstride::management(self.motor.id, self.host, 3))?
            .mode
            != 2
        {
            return Err("motor did not confirm Run mode");
        }
        Ok(())
    }

    fn breakaway(&mut self, config: Breakaway) -> Result<(), &'static str> {
        let id = self.motor.id;
        let host = self.host;
        let mut f = self.feedback(robstride::management(id, host, 4))?;
        if f.mode != 0
            || f.velocity.abs() > 0.2
            || f.effort.abs() > 0.08
            || (f.position - config.expected_position_rad).abs() > 0.02
        {
            return Err("breakaway requires the prepared stationary, low-effort starting pose");
        }
        let initial = f.position;
        if initial.abs() + 0.02 > self.motor.ranges.position {
            return Err("breakaway too near encoding boundary");
        }
        let ranges = self.motor.ranges;
        let command = |effort| {
            robstride::control(
                id,
                ranges,
                JointCommand {
                    position: initial,
                    velocity: 0.,
                    effort,
                    kp: 0.,
                    kd: 0.05,
                },
            )
        };
        // Stage a genuinely zero-gain neutral frame before enable.
        let neutral = robstride::control(
            id,
            self.motor.ranges,
            JointCommand {
                position: initial,
                velocity: 0.,
                effort: 0.,
                kp: 0.,
                kd: 0.,
            },
        )?;
        if self.feedback(neutral)?.mode != 0 {
            return Err("neutral command enabled motor");
        }
        self.enable_verified()?;
        self.report.breakaway = Some(BreakawayReport {
            direction: config.direction,
            stop: config.stop,
            initial_position_rad: initial,
            final_position_rad: initial,
            ..BreakawayReport::default()
        });
        let start = self.clock.now_ms();
        let mut previous = start;
        loop {
            self.check_time()?;
            let now = self.clock.now_ms();
            if now.saturating_sub(previous) > 20 {
                return Err("breakaway command interval exceeded 20 ms");
            }
            previous = now;
            let elapsed = now.saturating_sub(start);
            if elapsed >= 2000 {
                return Err("no breakaway observed within 0.25 Nm ramp; no automatic increase");
            }
            let effort = f64::from(config.direction) * 0.125 * elapsed as f64 / 1000.;
            f = self.feedback(command(effort)?)?;
            let report = self.report.breakaway.as_mut().unwrap();
            report.samples += 1;
            report.final_position_rad = f.position;
            report.maximum_commanded_effort_nm =
                report.maximum_commanded_effort_nm.max(effort.abs());
            report.maximum_reported_effort_nm =
                report.maximum_reported_effort_nm.max(f.effort.abs());
            report.maximum_reported_velocity_rad_s =
                report.maximum_reported_velocity_rad_s.max(f.velocity.abs());
            report.maximum_displacement_rad = report
                .maximum_displacement_rad
                .max((f.position - initial).abs());
            if f.mode != 2
                || f.velocity.abs() > 0.2
                || f.effort.abs() > 0.35
                || (f.position - initial).abs() > 0.02
            {
                return Err("breakaway measured envelope exceeded");
            }
            let directed = (f.position - initial) * f64::from(config.direction);
            if directed < -0.003 {
                return Err("breakaway moved opposite the commanded direction");
            }
            if directed >= 0.003 {
                report.movement_observed = true;
                report.command_at_motion_nm = Some(effort);
                report.reported_effort_at_motion_nm = Some(f.effort);
                if config.stop == BreakawayStop::Damped {
                    self.damped_breakaway_stop(f, effort)?;
                }
                // Disable only after the selected stop sequence, then observe
                // recoil. Fault exits bypass active damping and stop immediately.
                for _ in 0..if config.stop == BreakawayStop::Damped {
                    20
                } else {
                    10
                } {
                    f = self.feedback(robstride::management(id, host, 4))?;
                    let report = self.report.breakaway.as_mut().unwrap();
                    report.final_position_rad = f.position;
                    report.maximum_displacement_rad = report
                        .maximum_displacement_rad
                        .max((f.position - initial).abs());
                    report.maximum_reported_effort_nm =
                        report.maximum_reported_effort_nm.max(f.effort.abs());
                    report.maximum_reported_velocity_rad_s =
                        report.maximum_reported_velocity_rad_s.max(f.velocity.abs());
                    if f.mode != 0
                        || (f.position - initial).abs() > 0.02
                        || f.velocity.abs() > 0.2
                        || f.effort.abs() > 0.35
                    {
                        return Err("breakaway post-stop envelope exceeded");
                    }
                    self.clock.wait_ms(5);
                }
                return Ok(());
            }
            report.command_before_motion_nm = effort;
            self.clock.wait_ms(5);
        }
    }

    fn damped_breakaway_stop(
        &mut self,
        mut f: robstride::Feedback,
        initial_effort: f64,
    ) -> Result<(), &'static str> {
        let start = self.clock.now_ms();
        let initial = self.report.breakaway.as_ref().unwrap().initial_position_rad;
        self.report.breakaway.as_mut().unwrap().damped_stop = Some(DampedStopReport {
            started_at_ms: start,
            ..DampedStopReport::default()
        });
        let mut previous = start;
        for (name, duration, require_settled) in [
            ("release_drive", 100, false),
            ("damp", 200, true),
            ("fade_damping", 200, false),
            ("zero_gain", 150, true),
        ] {
            let phase_start = self.clock.now_ms();
            let mut settling = Settling::new(phase_start, f.position);
            self.report
                .breakaway
                .as_mut()
                .unwrap()
                .damped_stop
                .as_mut()
                .unwrap()
                .phases
                .push(StopPhase {
                    name,
                    started_at_ms: phase_start,
                    samples: 0,
                });
            while self.clock.now_ms().saturating_sub(phase_start) < duration {
                self.check_time()?;
                let now = self.clock.now_ms();
                if now.saturating_sub(previous) > 20 {
                    return Err("damped stop command interval exceeded 20 ms");
                }
                previous = now;
                let fraction = (now - phase_start) as f64 / duration as f64;
                let fade = 1. - fraction * fraction * (3. - 2. * fraction);
                let effort = if name == "release_drive" {
                    initial_effort * fade
                } else {
                    0.
                };
                let kd = match name {
                    "zero_gain" => 0.,
                    "fade_damping" => 0.2 * fade,
                    _ => 0.2,
                };
                let predicted = (effort - kd * f.velocity).abs();
                if predicted > 0.25 {
                    return Err("damped stop predicted effort exceeded 0.25 Nm");
                }
                let command = robstride::control(
                    self.motor.id,
                    self.motor.ranges,
                    JointCommand {
                        position: initial,
                        velocity: 0.,
                        effort,
                        kp: 0.,
                        kd,
                    },
                )?;
                f = self.feedback(command)?;
                let report = self.report.breakaway.as_mut().unwrap();
                let within_bounds = report.observe(f);
                let stop = report.damped_stop.as_mut().unwrap();
                stop.phases.last_mut().unwrap().samples += 1;
                stop.maximum_predicted_effort_nm = stop.maximum_predicted_effort_nm.max(predicted);
                if f.mode != 2 || !within_bounds {
                    return Err("damped stop measured envelope exceeded");
                }
                settling.observe(self.clock.now_ms(), f);
                self.clock.wait_ms(5);
            }
            if require_settled && !settling.settled() {
                return Err("damped stop did not settle within phase budget");
            }
        }
        let stop = self
            .report
            .breakaway
            .as_mut()
            .unwrap()
            .damped_stop
            .as_mut()
            .unwrap();
        stop.finished_at_ms = Some(self.clock.now_ms());
        stop.settled_before_disable = true;
        Ok(())
    }

    /// Explicit motor-space micro-jog on an operator-cleared mechanism. This is
    /// a commissioning experiment, not permission for whole-robot motion.
    fn jog(&mut self, displacement: f64) -> Result<(), &'static str> {
        let id = self.motor.id;
        let host = self.host;
        let mut f = self.feedback(robstride::management(id, host, 4))?;
        if f.mode != 0 || f.velocity.abs() > 0.2 || f.effort.abs() > 0.08 {
            return Err("jog requires stopped low-effort motor");
        }
        let initial = f.position;
        if (initial + displacement).abs() > self.motor.ranges.position {
            return Err("jog would exceed motor encoding range");
        }
        let neutral = robstride::control(
            id,
            self.motor.ranges,
            JointCommand {
                position: initial,
                velocity: 0.0,
                effort: 0.0,
                kp: 0.0,
                kd: 0.0,
            },
        )?;
        if self.feedback(neutral)?.mode != 0 {
            return Err("neutral pre-jog command enabled motor");
        }
        self.enable_verified()?;
        self.report.jog = Some(JogReport {
            kp_nm_per_rad: 2.0,
            kd_nm_s_per_rad: 0.05,
            requested_displacement_rad: displacement,
            initial_position_rad: initial,
            ..JogReport::default()
        });
        let start = self.clock.now_ms();
        let mut previous = start;
        loop {
            self.check_time()?;
            let now = self.clock.now_ms();
            if now.saturating_sub(previous) > 20 {
                return Err("jog command interval exceeded 20 ms");
            }
            previous = now;
            let elapsed = now.saturating_sub(start);
            if elapsed >= 1600 {
                break;
            }
            // 0.6 s out, 0.2 s dwell, 0.6 s return, 0.2 s dwell. Target
            // speed is <=0.05 rad/s for the largest allowed 0.03 rad jog.
            let fraction = match elapsed {
                0..600 => elapsed as f64 / 600.0,
                600..800 => 1.0,
                800..1400 => (1400 - elapsed) as f64 / 600.0,
                _ => 0.0,
            };
            let target = initial + displacement * fraction;
            if (f.position - initial).abs() > 0.04
                || f.velocity.abs() > 0.2
                || f.effort.abs() > 0.08
                || 2.0 * (target - f.position).abs() + 0.05 * f.velocity.abs() > 0.08
            {
                return Err("jog position, speed or torque envelope exceeded");
            }
            let command = robstride::control(
                id,
                self.motor.ranges,
                JointCommand {
                    position: target,
                    velocity: 0.0,
                    effort: 0.0,
                    kp: 2.0,
                    kd: 0.05,
                },
            )?;
            f = self.feedback(command)?;
            if f.mode != 2 {
                return Err("motor left Run mode during jog");
            }
            let report = self.report.jog.as_mut().unwrap();
            report.samples += 1;
            report.final_position_rad = f.position;
            report.maximum_displacement_rad = report
                .maximum_displacement_rad
                .max((f.position - initial).abs());
            report.maximum_reported_effort_nm =
                report.maximum_reported_effort_nm.max(f.effort.abs());
            report.maximum_reported_velocity_rad_s =
                report.maximum_reported_velocity_rad_s.max(f.velocity.abs());
            // Check the last sample as well; the next iteration may reach expiry.
            if (f.position - initial).abs() > 0.04
                || f.velocity.abs() > 0.2
                || f.effort.abs() > 0.08
            {
                return Err("jog measured envelope exceeded");
            }
            self.clock.wait_ms(5);
        }
        let report = self.report.jog.as_mut().unwrap();
        report.movement_observed = report.maximum_displacement_rad >= 0.003;
        if !report.movement_observed {
            return Err("jog completed without measurable motion; gains were not increased");
        }
        if (f.position - initial).abs() > 0.01 {
            return Err("jog did not return within 0.01 rad");
        }
        Ok(())
    }
}

pub fn run<I: CanIo, C: Clock>(
    io: &mut I,
    clock: &mut C,
    motor: &Motor,
    host: u8,
    identity: &Identity,
) -> Report {
    run_with_jog(io, clock, motor, host, identity, None)
}

pub fn run_with_jog<I: CanIo, C: Clock>(
    io: &mut I,
    clock: &mut C,
    motor: &Motor,
    host: u8,
    identity: &Identity,
    jog_radians: Option<f64>,
) -> Report {
    run_experiment(io, clock, motor, host, identity, jog_radians, None)
}

pub fn run_breakaway<I: CanIo, C: Clock>(
    io: &mut I,
    clock: &mut C,
    motor: &Motor,
    host: u8,
    identity: &Identity,
    config: Breakaway,
) -> Report {
    run_experiment(io, clock, motor, host, identity, None, Some(config))
}

fn run_experiment<I: CanIo, C: Clock>(
    io: &mut I,
    clock: &mut C,
    motor: &Motor,
    host: u8,
    identity: &Identity,
    jog_radians: Option<f64>,
    breakaway: Option<Breakaway>,
) -> Report {
    if breakaway
        .is_some_and(|c| ![-1, 1].contains(&c.direction) || !c.expected_position_rad.is_finite())
    {
        return Report {
            error: Some("invalid breakaway direction or starting position".into()),
            ..Report::default()
        };
    }
    if jog_radians.is_some_and(|q| !q.is_finite() || q.abs() < 0.005 || q.abs() > 0.03) {
        return Report {
            error: Some("jog displacement must be finite, with magnitude 0.005..0.03 rad".into()),
            ..Report::default()
        };
    }
    let started = clock.now_ms();
    let mut trial = Trial {
        io,
        clock,
        motor,
        host,
        identity_verified: false,
        started,
        budget_ms: if jog_radians.is_some() || breakaway.is_some() {
            4000
        } else {
            2000
        },
        report: Report::default(),
    };
    let mut outcome = trial.execute(identity).and_then(|()| match jog_radians {
        Some(displacement) => trial.jog(displacement),
        None => match breakaway {
            Some(config) => trial.breakaway(config),
            None => Ok(()),
        },
    });
    if let Err(reason) = outcome {
        trial.report.error = Some(reason.into());
    }
    if trial.identity_verified {
        // Stop must still be attempted if the normal timing/signal gate tripped.
        let stop = robstride::management(motor.id, host, 4);
        if let Err(e) = trial.send(stop) {
            trial.report.final_stop_error = Some(e.into());
        } else if !trial.clock.interrupted() {
            // A separate acknowledged stop provides evidence beyond a successful send.
            match trial.feedback(stop) {
                Ok(f) => {
                    trial.report.final_stop_feedback = Some(f);
                    trial.report.final_stop_confirmed = f.mode == 0;
                    if f.mode != 0 {
                        trial.report.final_stop_error =
                            Some("final stop did not report Reset".into());
                    }
                    if let Some(b) = &mut trial.report.breakaway {
                        b.final_position_rad = f.position;
                        b.maximum_displacement_rad = b
                            .maximum_displacement_rad
                            .max((f.position - b.initial_position_rad).abs());
                        b.maximum_reported_velocity_rad_s =
                            b.maximum_reported_velocity_rad_s.max(f.velocity.abs());
                        b.maximum_reported_effort_nm =
                            b.maximum_reported_effort_nm.max(f.effort.abs());
                        if outcome.is_ok()
                            && (f.velocity.abs() > 0.2
                                || f.effort.abs() > 0.35
                                || (f.position - b.initial_position_rad).abs() > 0.02)
                        {
                            outcome = Err("breakaway final-stop envelope exceeded");
                            trial.report.error =
                                Some("breakaway final-stop envelope exceeded".into());
                        }
                    }
                }
                Err(e) => trial.report.final_stop_error = Some(e.into()),
            }
        }
    }
    trial.report.success = outcome.is_ok() && trial.report.final_stop_confirmed;
    if trial.identity_verified && !trial.report.final_stop_confirmed {
        let _ = trial.send(robstride::management(motor.id, host, 4));
    }
    // All deliberate exit transmissions above are in the report. Drop remains
    // a fallback for unwinding during the experiment, not an unlogged extra stop.
    trial.identity_verified = false;
    std::mem::take(&mut trial.report)
}
