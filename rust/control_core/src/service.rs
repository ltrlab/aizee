//! Linux commissioning service. Physical motion is deliberately not exposed.
//! The same core/CAN loop is exercised on vcan; physical modes are receive-only,
//! parameter-read, or a bounded single-actuator zero-torque trial.
pub mod linux_can;
use crate::{
    can_adapter::{CanHardware, Diagnostic, Motor},
    hardware::{Hardware, JointFeedback},
    protocol::*,
    Config, Controller,
};
use serde::{Deserialize, Serialize};
use std::{
    fs,
    time::{Duration, Instant},
};

static SHUTDOWN: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);
extern "C" fn shutdown_signal(_: libc::c_int) {
    SHUTDOWN.store(true, std::sync::atomic::Ordering::Relaxed);
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Mode {
    VirtualCan,
    Observe,
    ParameterProbe,
    MotorDiagnostics,
    ZeroTorqueTrial,
    BoundedJogTrial,
    BreakawayTrial,
}
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Bus {
    pub interface: String,
    pub usb_serial: Option<String>,
}
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ServiceConfig {
    pub mode: Mode,
    pub endpoint: String,
    pub period_us: u64,
    pub maximum_lateness_us: u64,
    pub host_id: u8,
    pub buses: Vec<Bus>,
    pub conflicting_services: Vec<String>,
    pub motors: Vec<Motor>,
    pub controller: Config,
    #[serde(default)]
    pub watchdog_identities: Vec<crate::watchdog_trial::Identity>,
    #[serde(default)]
    pub reference_positions: Vec<crate::reference::Reference>,
}
impl ServiceConfig {
    pub fn validate(&self) -> Result<(), String> {
        self.controller.validate()?;
        if self.conflicting_services.len() > 32
            || self.conflicting_services.iter().any(|name| {
                name.is_empty()
                    || name.len() > 128
                    || !name
                        .bytes()
                        .all(|c| c.is_ascii_alphanumeric() || b"_.@-".contains(&c))
            })
        {
            return Err("invalid conflicting service names".into());
        }
        if !(1000..=10000).contains(&self.period_us)
            || self.maximum_lateness_us < self.period_us
            || self.maximum_lateness_us > 20000
            || !self.endpoint.starts_with("tcp://127.0.0.1:")
            || self.endpoint[16..]
                .parse::<u16>()
                .ok()
                .filter(|p| *p > 1024)
                .is_none()
            || self.buses.is_empty()
            || self.buses.len() > 8
            || self.motors.len() != self.controller.joints.len()
        {
            return Err("invalid service rates, loopback endpoint or roster".into());
        }
        for (i, b) in self.buses.iter().enumerate() {
            if b.interface.is_empty()
                || b.interface.len() > 15
                || !b
                    .interface
                    .bytes()
                    .all(|c| c.is_ascii_alphanumeric() || c == b'_')
                || self.buses[..i].iter().any(|x| x.interface == b.interface)
                || (self.mode == Mode::VirtualCan && b.usb_serial.is_some())
                || (self.mode != Mode::VirtualCan
                    && b.usb_serial.as_ref().is_none_or(|s| s.is_empty()))
            {
                return Err("invalid or duplicate CAN interface identity".into());
            }
        }
        for (m, j) in self.motors.iter().zip(&self.controller.joints) {
            m.ranges.validate()?;
            if m.name != j.name || m.bus >= self.buses.len() {
                return Err("joint/CAN roster mismatch".into());
            }
        }
        if matches!(
            self.mode,
            Mode::ZeroTorqueTrial | Mode::BoundedJogTrial | Mode::BreakawayTrial
        ) && (self.motors.len() != 1
            || self.controller.lease_ms > 250
            || self
                .controller
                .joints
                .iter()
                .any(|j| j.hold_kp != 0.0 || j.hold_kd != 0.0))
        {
            return Err(
                "zero-torque trial requires one motor, zero gains and lease <=250 ms".into(),
            );
        }
        if self.mode == Mode::ZeroTorqueTrial && self.watchdog_identities.len() != self.motors.len()
        {
            return Err("physical enable requires pinned watchdog identities".into());
        }
        if self.mode == Mode::ZeroTorqueTrial {
            let names: Vec<_> = self.motors.iter().map(|m| m.name.as_str()).collect();
            crate::reference::Gate::new(self.reference_positions.clone(), &names)?;
        } else if !self.reference_positions.is_empty() {
            return Err(
                "reference positions only supported by explicit physical enable mode".into(),
            );
        }
        if self.mode != Mode::ZeroTorqueTrial && !self.watchdog_identities.is_empty() {
            return Err(
                "watchdog setup only available in explicit zero-torque service mode".into(),
            );
        }
        Ok(())
    }
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Envelope {
    pub protocol_version: u32,
    pub request: ServiceRequest,
}
#[derive(Deserialize)]
#[serde(untagged)]
pub enum ServiceRequest {
    Core(Request),
    Probe(Probe),
}
#[derive(Deserialize)]
#[serde(tag = "type", rename_all = "snake_case", deny_unknown_fields)]
pub enum Probe {
    ProbeParameter {
        motor_index: usize,
        parameter: u16,
    },
    DiagnoseMotor {
        motor_index: usize,
        diagnostic: Diagnostic,
    },
}

#[derive(Default, Serialize)]
pub struct Timing {
    pub ticks: u64,
    pub overruns: u64,
    pub maximum_lateness_us: u64,
    pub maximum_work_us: u64,
    pub rejected_frames: u64,
    pub deadline_misses: u64,
}
#[derive(Serialize)]
struct Output<'a> {
    reply: Option<Reply>,
    status: Status,
    feedback: &'a [JointFeedback],
    error: Option<&'a str>,
    mode: Mode,
    hardware_enabled: bool,
    watchdog: &'static str,
    reference: &'static str,
    tx_frames: u64,
    rx_frames: u64,
    timing: &'a Timing,
    observations: &'a [linux_can::Observation],
}

pub fn run(
    config: ServiceConfig,
    run_seconds: Option<f64>,
) -> Result<(), Box<dyn std::error::Error>> {
    config.validate()?;
    unsafe {
        libc::signal(
            libc::SIGINT,
            shutdown_signal as *const () as libc::sighandler_t,
        );
        libc::signal(
            libc::SIGTERM,
            shutdown_signal as *const () as libc::sighandler_t,
        );
    }
    let clock = Instant::now();
    let io = linux_can::LinuxCan::open(
        &config.buses,
        config.mode,
        &config.conflicting_services,
        clock,
    )?;
    let mut hardware = CanHardware::new(
        io,
        config.motors.clone(),
        config.host_id,
        config.mode != Mode::Observe,
    )?;
    if config.mode == Mode::VirtualCan {
        hardware.allow_virtual_enable();
    } else if config.mode == Mode::ZeroTorqueTrial {
        hardware.configure_reference(config.reference_positions.clone())?;
        hardware.configure_watchdogs(config.watchdog_identities.clone())?;
        // Complete stopped preflight before exposing the command socket. The
        // core correctly rejects missing feedback; startup must not feed it
        // empty samples while identity/parameter exchanges are still pending.
        let mut feedback = vec![JointFeedback::default(); config.motors.len()];
        let deadline = Instant::now() + Duration::from_millis(500);
        loop {
            if SHUTDOWN.load(std::sync::atomic::Ordering::Relaxed) || Instant::now() >= deadline {
                return Err("watchdog startup interrupted or timed out".into());
            }
            hardware
                .read(clock.elapsed().as_millis() as u64, &mut feedback)
                .map_err(|_| "watchdog startup verification failed")?;
            if hardware.watchdog_status() == "ready" && hardware.reference_status() == "ready" {
                break;
            }
            std::thread::sleep(Duration::from_micros(config.period_us));
        }
    }
    let epoch = fs::read_to_string("/proc/sys/kernel/random/uuid")?
        .trim()
        .to_owned();
    let mut controller = Controller::new(config.controller.clone(), epoch)?;
    let context = zmq::Context::new();
    let socket = context.socket(zmq::REP)?;
    socket.set_linger(0)?;
    socket.set_rcvhwm(4)?;
    socket.set_sndhwm(4)?;
    socket.set_maxmsgsize(65536)?;
    socket.bind(&config.endpoint)?;
    let period = Duration::from_micros(config.period_us);
    let mut scheduled = Instant::now();
    let mut timing = Timing::default();
    let mut response_pending = false;
    let mut pending = Vec::new();
    let mut physical_trial_deadline = None;
    // Opening the socket never arms. Explicit physical zero-torque mode already
    // completed stopped watchdog preflight; other modes never write on startup.
    loop {
        if SHUTDOWN.load(std::sync::atomic::Ordering::Relaxed) {
            break;
        }
        let started = Instant::now();
        let now_ms = clock.elapsed().as_millis() as u64;
        let lateness = started.saturating_duration_since(scheduled).as_micros() as u64;
        timing.maximum_lateness_us = timing.maximum_lateness_us.max(lateness);
        if lateness > config.maximum_lateness_us {
            timing.overruns += 1;
            controller.handle_deferred(now_ms, Request::EmergencyStop, &mut hardware);
        }
        if physical_trial_deadline.is_some_and(|deadline| now_ms >= deadline) {
            controller.handle_deferred(now_ms, Request::EmergencyStop, &mut hardware);
            physical_trial_deadline = None;
        }
        controller.tick(now_ms, &mut hardware);
        timing.ticks += 1;
        if !response_pending {
            match socket.recv_bytes(zmq::DONTWAIT) {
                Ok(bytes) => {
                    let mut error = None;
                    let mut reply = None;
                    let parsed = rmp_serde::from_slice::<Envelope>(&bytes);
                    match parsed {
                        Ok(frame) if frame.protocol_version == PROTOCOL_VERSION => {
                            match frame.request {
                                ServiceRequest::Core(request) => {
                                    let is_arm = matches!(request, Request::Arm { .. });
                                    let physical_move = config.mode != Mode::VirtualCan
                                        && matches!(request, Request::Move { .. });
                                    let prohibited_arm = is_arm
                                        && !matches!(
                                            config.mode,
                                            Mode::VirtualCan | Mode::ZeroTorqueTrial
                                        );
                                    // Continuous Idle enables every motor without the finite
                                    // trial deadline; it waits for a whole-robot physical mode.
                                    let prohibited_idle = config.mode != Mode::VirtualCan
                                        && matches!(request, Request::Idle { .. });
                                    if physical_move || prohibited_arm || prohibited_idle {
                                        error = Some("operation_not_available_in_this_mode");
                                    } else {
                                        let result = controller.handle_deferred(
                                            clock.elapsed().as_millis() as u64,
                                            request,
                                            &mut hardware,
                                        );
                                        if is_arm
                                            && result.accepted
                                            && config.mode == Mode::ZeroTorqueTrial
                                        {
                                            // Renewal cannot extend a physical trial. Disabling at expiry is
                                            // deliberate: user confirmed support loss is tolerated by the stand.
                                            physical_trial_deadline =
                                                Some(now_ms + config.controller.lease_ms);
                                        }
                                        reply = Some(result);
                                    }
                                }
                                ServiceRequest::Probe(Probe::ProbeParameter {
                                    motor_index,
                                    parameter,
                                }) => {
                                    if config.mode != Mode::ParameterProbe
                                        || hardware.probe(motor_index, parameter).is_err()
                                    {
                                        error = Some("parameter_probe_rejected");
                                    }
                                }
                                ServiceRequest::Probe(Probe::DiagnoseMotor {
                                    motor_index,
                                    diagnostic,
                                }) => {
                                    if config.mode != Mode::MotorDiagnostics
                                        || hardware.diagnostic(motor_index, diagnostic).is_err()
                                    {
                                        error = Some("motor_diagnostic_rejected");
                                    }
                                }
                            }
                        }
                        _ => {
                            timing.rejected_frames += 1;
                            controller.handle_deferred(
                                clock.elapsed().as_millis() as u64,
                                Request::EmergencyStop,
                                &mut hardware,
                            );
                            error = Some("invalid_guarded_protocol_frame");
                        }
                    }
                    pending = rmp_serde::to_vec_named(&Output {
                        reply,
                        status: controller.status(),
                        feedback: controller.feedback(),
                        error,
                        mode: config.mode,
                        hardware_enabled: hardware.enabled,
                        watchdog: hardware.watchdog_status(),
                        reference: hardware.reference_status(),
                        tx_frames: hardware.tx_frames,
                        rx_frames: hardware.rx_frames,
                        timing: &timing,
                        observations: &hardware.io.observations,
                    })?;
                    response_pending = true;
                }
                Err(zmq::Error::EAGAIN) => {}
                Err(e) => {
                    let _ = hardware.disable();
                    return Err(e.into());
                }
            }
        }
        if response_pending {
            match socket.send(pending.as_slice(), zmq::DONTWAIT) {
                Ok(()) => response_pending = false,
                Err(zmq::Error::EAGAIN) => {} // Independent ticks continue while peer is stalled.
                Err(e) => {
                    let _ = hardware.disable();
                    return Err(e.into());
                }
            }
        }
        let work_us = started.elapsed().as_micros() as u64;
        timing.maximum_work_us = timing.maximum_work_us.max(work_us);
        if work_us.saturating_add(lateness) > config.period_us {
            timing.deadline_misses += 1;
        }
        if work_us > config.maximum_lateness_us {
            timing.overruns += 1;
            controller.handle_deferred(
                clock.elapsed().as_millis() as u64,
                Request::EmergencyStop,
                &mut hardware,
            );
        }
        if run_seconds.is_some_and(|seconds| clock.elapsed().as_secs_f64() >= seconds) {
            break;
        }
        scheduled += period;
        if scheduled < Instant::now() {
            scheduled = Instant::now() + period;
        }
        std::thread::sleep(scheduled.saturating_duration_since(Instant::now()));
    }
    // A bounded bench run always exits disarmed. This is still best effort CAN.
    let disable_attempted = hardware.enabled && hardware.transmit;
    let disabled = hardware.disable().is_ok();
    println!(
        "{}",
        serde_json::json!({"timing": timing, "tx_frames": hardware.tx_frames,
        "rx_frames": hardware.rx_frames, "disable_operation_succeeded": disabled,
        "disable_transmit_attempted": disable_attempted, "mode": config.mode})
    );
    Ok(())
}
