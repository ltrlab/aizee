//! Linux head receiver: the arms' receiver loop (the same core, recorder, protocol and replies) on the head's
//! Feetech bus. Virtual mode runs simulated servos in-process; observation only reads; head teleop enables
//! after the servos' identity and the reference-pose gate pass. Nothing here writes a servo's EEPROM.
//!
//! Unlike the arms' receiver, a late tick (lateness or work over maximum_lateness_us) is counted, never a stop.
//! The servos keep no watchdog: through a late tick each holds the last goal it was given, every one inside its
//! limits, and a stall long enough for the client's command or lease to expire still brakes the head to a hold
//! with its torque kept (Controller::tick). Stopping cut its torque instead and dropped the head mid-session over
//! one 15.2 ms tick (2026-11-03).
use crate::{
    feetech::{sim, FeetechHardware, SerialPort},
    hardware::{Hardware, HardwareError, JointFeedback},
    head_config::{HeadMode, HeadServiceConfig, SerialBus},
    protocol::*,
    recorder::Recorder,
    service::{Envelope, ServiceRequest, Timing},
    Controller,
};
use serde::Serialize;
use std::{
    ffi::CString,
    fs,
    time::{Duration, Instant},
};

static SHUTDOWN: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);
extern "C" fn shutdown_signal(_: libc::c_int) {
    SHUTDOWN.store(true, std::sync::atomic::Ordering::Relaxed);
}

/// The adapter's tty, raw at 1 Mbaud, opened exclusively. Reads wait on poll() up to their timeout.
pub struct LinuxSerial {
    fd: libc::c_int,
}
impl LinuxSerial {
    pub fn open(bus: &SerialBus) -> Result<Self, String> {
        let path = CString::new(bus.device.as_str()).map_err(|_| "invalid serial device path")?;
        let fd = unsafe {
            libc::open(
                path.as_ptr(),
                libc::O_RDWR | libc::O_NOCTTY | libc::O_NONBLOCK | libc::O_CLOEXEC,
            )
        };
        if fd < 0 {
            return Err(format!(
                "{}: {}",
                bus.device,
                std::io::Error::last_os_error()
            ));
        }
        let port = Self { fd };
        let mut tio: libc::termios = unsafe { std::mem::zeroed() };
        let configured = unsafe {
            libc::ioctl(fd, libc::TIOCEXCL) == 0 && libc::tcgetattr(fd, &mut tio) == 0 && {
                libc::cfmakeraw(&mut tio);
                tio.c_cflag |= libc::CLOCAL | libc::CREAD;
                tio.c_cflag &= !(libc::CRTSCTS | libc::CSTOPB | libc::PARENB);
                tio.c_cc[libc::VMIN] = 0;
                tio.c_cc[libc::VTIME] = 0;
                libc::cfsetispeed(&mut tio, libc::B1000000) == 0
                    && libc::cfsetospeed(&mut tio, libc::B1000000) == 0
                    && libc::tcsetattr(fd, libc::TCSANOW, &tio) == 0
                    && libc::tcflush(fd, libc::TCIOFLUSH) == 0
            }
        };
        if !configured {
            return Err(format!(
                "{}: could not configure the port exclusively at 1 Mbaud: {}",
                bus.device,
                std::io::Error::last_os_error()
            ));
        }
        Ok(port)
    }
}
impl Drop for LinuxSerial {
    fn drop(&mut self) {
        unsafe { libc::close(self.fd) };
    }
}
impl SerialPort for LinuxSerial {
    fn clear_input(&mut self) -> Result<(), HardwareError> {
        if unsafe { libc::tcflush(self.fd, libc::TCIFLUSH) } != 0 {
            return Err(HardwareError);
        }
        Ok(())
    }
    fn send(&mut self, bytes: &[u8]) -> Result<(), HardwareError> {
        let mut written = 0;
        let deadline = Instant::now() + Duration::from_millis(5);
        while written < bytes.len() {
            let n = unsafe {
                libc::write(
                    self.fd,
                    bytes[written..].as_ptr().cast(),
                    bytes.len() - written,
                )
            };
            if n > 0 {
                written += n as usize;
            } else if std::io::Error::last_os_error().kind() != std::io::ErrorKind::WouldBlock
                || Instant::now() >= deadline
            {
                return Err(HardwareError);
            }
        }
        Ok(())
    }
    fn receive(&mut self, buffer: &mut [u8], timeout_us: u64) -> Result<usize, HardwareError> {
        let deadline = Instant::now() + Duration::from_micros(timeout_us);
        let mut got = 0;
        while got < buffer.len() {
            let now = Instant::now();
            if now >= deadline {
                break;
            }
            let wait_ms = (deadline - now).as_micros().div_ceil(1000) as libc::c_int;
            let mut poll = libc::pollfd {
                fd: self.fd,
                events: libc::POLLIN,
                revents: 0,
            };
            let ready = unsafe { libc::poll(&mut poll, 1, wait_ms) };
            if ready < 0 {
                if std::io::Error::last_os_error().kind() == std::io::ErrorKind::Interrupted {
                    continue;
                }
                return Err(HardwareError);
            }
            if ready == 0 {
                break;
            }
            if poll.revents & (libc::POLLERR | libc::POLLHUP | libc::POLLNVAL) != 0 {
                return Err(HardwareError); // The adapter went away.
            }
            let n = unsafe {
                libc::read(
                    self.fd,
                    buffer[got..].as_mut_ptr().cast(),
                    buffer.len() - got,
                )
            };
            if n > 0 {
                got += n as usize;
            } else if n == 0
                || std::io::Error::last_os_error().kind() != std::io::ErrorKind::WouldBlock
            {
                return Err(HardwareError);
            }
        }
        Ok(got)
    }
}

/// The head's reply: the arms' fields and names, so one client reads both receivers.
#[derive(Serialize)]
struct Output<'a> {
    reply: Option<Reply>,
    status: Status,
    feedback: &'a [JointFeedback],
    error: Option<&'a str>,
    mode: HeadMode,
    hardware_enabled: bool,
    hardware_failure: Option<&'a str>,
    hardware_dropped: Option<&'a str>,
    watchdog: &'static str,
    reference: &'static str,
    /// Bus requests sent and replies received.
    tx_frames: u64,
    rx_frames: u64,
    timing: &'a Timing,
    /// Per servo, its temperature readings taken and held back (feetech::TemperatureCheck).
    temperatures: &'a [crate::feetech::TemperatureCheck],
    observations: [u8; 0],
    recent: Vec<&'a [f64]>,
    recent_dropped: u64,
}

pub fn run(
    config: HeadServiceConfig,
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
    match config.mode {
        HeadMode::VirtualBus => {
            let servos = config
                .servos
                .iter()
                .map(|s| sim::SimServo::new(s.id, s.zero_steps.round() as u16))
                .collect();
            let mut hardware =
                FeetechHardware::new(sim::SimBus::new(servos), config.servos.clone(), true)?;
            hardware.allow_virtual_enable();
            serve(config, hardware, run_seconds)
        }
        HeadMode::Observe | HeadMode::TeleopDirect => {
            let port = LinuxSerial::open(&config.bus)?;
            let transmit = config.mode == HeadMode::TeleopDirect;
            let mut hardware = FeetechHardware::new(port, config.servos.clone(), transmit)?;
            hardware
                .verify_identity()
                .map_err(|e| format!("head servo identity: {e}"))?;
            if transmit {
                hardware.configure_reference(config.reference_positions.clone())?;
            }
            serve(config, hardware, run_seconds)
        }
    }
}

/// Reads the head until its reference-pose gate is ready, before the command socket opens, stamping each reading on
/// the serve loop's own `clock`: the gate ignores readings stamped before its newest, so on a restarted clock an
/// enable is refused until the new clock catches up (Helm's first live head jog, 2026-10-30, armed 91 ms in).
fn prove_reference<P: SerialPort>(
    hardware: &mut FeetechHardware<P>,
    clock: Instant,
    servos: usize,
    period_us: u64,
) -> Result<(), String> {
    let mut feedback = vec![JointFeedback::default(); servos];
    let deadline = Instant::now() + Duration::from_millis(1000);
    loop {
        if SHUTDOWN.load(std::sync::atomic::Ordering::Relaxed) || Instant::now() >= deadline {
            return Err("head reference check interrupted or timed out".into());
        }
        hardware
            .read(clock.elapsed().as_millis() as u64, &mut feedback)
            .map_err(|_| {
                format!(
                    "head reference check failed: {}",
                    hardware.failure().unwrap_or("no detail")
                )
            })?;
        if hardware.reference_status() == "ready" {
            return Ok(());
        }
        std::thread::sleep(Duration::from_micros(period_us));
    }
}

fn serve<P: SerialPort>(
    config: HeadServiceConfig,
    mut hardware: FeetechHardware<P>,
    run_seconds: Option<f64>,
) -> Result<(), Box<dyn std::error::Error>> {
    let clock = Instant::now();
    if config.mode == HeadMode::TeleopDirect {
        prove_reference(&mut hardware, clock, config.servos.len(), config.period_us)?;
    }
    let epoch = fs::read_to_string("/proc/sys/kernel/random/uuid")?
        .trim()
        .to_owned();
    let mut controller = Controller::new(config.controller.clone(), epoch)?;
    let mut recorder = Recorder::new(config.servos.len());
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
    let mut last_tick = clock.elapsed();
    loop {
        if SHUTDOWN.load(std::sync::atomic::Ordering::Relaxed) {
            break;
        }
        let started = Instant::now();
        let now_ms = clock.elapsed().as_millis() as u64;
        let lateness = started.saturating_duration_since(scheduled).as_micros() as u64;
        timing.maximum_lateness_us = timing.maximum_lateness_us.max(lateness);
        if lateness > config.maximum_lateness_us {
            timing.overruns += 1; // Counted, never a stop: see the module's notes.
        }
        if config.mode == HeadMode::VirtualBus {
            // Simulated servos move in real time between reads.
            let elapsed = clock.elapsed();
            hardware.port.advance((elapsed - last_tick).as_secs_f64());
            last_tick = elapsed;
        }
        controller.tick(now_ms, &mut hardware);
        recorder.record(
            clock.elapsed().as_secs_f64() * 1000.0,
            controller.feedback(),
            controller.commands(),
        );
        timing.ticks += 1;
        if !response_pending {
            match socket.recv_bytes(zmq::DONTWAIT) {
                Ok(bytes) => {
                    let mut error = None;
                    let mut reply = None;
                    match rmp_serde::from_slice::<Envelope>(&bytes) {
                        Ok(frame) if frame.protocol_version == PROTOCOL_VERSION => match frame
                            .request
                        {
                            ServiceRequest::Core(request) => {
                                // Observation never arms, idles or moves.
                                let motion = matches!(
                                    request,
                                    Request::Arm { .. }
                                        | Request::Idle { .. }
                                        | Request::Move { .. }
                                        | Request::DirectUpdate { .. }
                                );
                                if motion && config.mode == HeadMode::Observe {
                                    error = Some("operation_not_available_in_this_mode");
                                } else {
                                    reply = Some(controller.handle_deferred(
                                        clock.elapsed().as_millis() as u64,
                                        request,
                                        &mut hardware,
                                    ));
                                }
                            }
                            ServiceRequest::Probe(_) => error = Some("parameter_probe_rejected"),
                        },
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
                    let (recent, recent_dropped) = recorder.take();
                    pending = rmp_serde::to_vec_named(&Output {
                        reply,
                        status: controller.status(),
                        feedback: controller.feedback(),
                        error,
                        mode: config.mode,
                        hardware_enabled: hardware.enabled,
                        hardware_failure: hardware.failure(),
                        hardware_dropped: hardware.dropped(),
                        watchdog: hardware.watchdog_status(),
                        reference: hardware.reference_status(),
                        tx_frames: hardware.requests,
                        rx_frames: hardware.replies,
                        timing: &timing,
                        temperatures: hardware.temperatures(),
                        observations: [],
                        recent,
                        recent_dropped,
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
                Err(zmq::Error::EAGAIN) => {}
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
            timing.overruns += 1; // Counted, never a stop: see the module's notes.
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
    let disabled = hardware.disable().is_ok();
    println!(
        "{}",
        serde_json::json!({"timing": timing, "requests": hardware.requests, "replies": hardware.replies,
        "disable_operation_succeeded": disabled, "mode": config.mode,
        "hardware_failure": hardware.failure(), "hardware_dropped": hardware.dropped(),
        "temperatures": hardware.temperatures()})
    );
    Ok(())
}
