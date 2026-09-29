//! Deterministic, offline harness for the SAME guarded core used by adapters.
//! stdin/stdout: little-endian u32 length followed by MessagePack named maps.
//! Clock and fault injection exist only in this test executable, never on CAN.
use robot_control::{
    hardware::{JointFeedback, Loopback},
    protocol::*,
    Config, Controller,
};
use serde::{Deserialize, Serialize};
use std::io::{self, Read, Write};

const MAX_FRAME: usize = 256 * 1024;

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Setup {
    config: Config,
    initial_positions: Vec<f64>,
}

#[derive(Deserialize, Default)]
#[serde(deny_unknown_fields)]
struct Faults {
    #[serde(default)]
    frozen: bool,
    #[serde(default)]
    fail_read: bool,
    #[serde(default)]
    fail_write: bool,
    #[serde(default)]
    motor_fault: bool,
}

/// A measured joint state to report from the next read, as a simulated plant computes it.
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Measured {
    position: f64,
    velocity: f64,
    effort: f64,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Input {
    protocol_version: u32,
    now_ms: u64,
    request: Option<Request>,
    faults: Option<Faults>,
    /// Drive the feedback from a plant instead of following the commands (the default).
    #[serde(default)]
    feedback: Option<Vec<Measured>>,
}

#[derive(Serialize)]
struct Output {
    reply: Option<Reply>,
    status: Status,
    hardware_enabled: bool,
    writes: u64,
    feedback: Vec<JointFeedback>,
    error: Option<String>,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let path = std::env::args().nth(1).unwrap_or_else(|| "-".into());
    let mut input = io::stdin().lock();
    let mut output = io::stdout().lock();
    let setup: Setup = if path == "-" {
        let mut size = [0u8; 4];
        input.read_exact(&mut size)?;
        let len = u32::from_le_bytes(size) as usize;
        if len == 0 || len > MAX_FRAME {
            return Err("invalid setup frame length".into());
        }
        let mut bytes = vec![0u8; len];
        input.read_exact(&mut bytes)?;
        rmp_serde::from_slice(&bytes)?
    } else {
        serde_json::from_reader(std::fs::File::open(path)?)?
    };
    if setup.initial_positions.len() != setup.config.joints.len()
        || setup
            .initial_positions
            .iter()
            .zip(&setup.config.joints)
            .any(|(q, l)| !q.is_finite() || *q < l.lower || *q > l.upper)
    {
        return Err("invalid initial positions".into());
    }
    // Epoch isolates process restarts. It is an identity, not authentication.
    let epoch = format!(
        "harness-{}-{}",
        std::process::id(),
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)?
            .as_nanos()
    );
    let mut controller = Controller::new(setup.config, epoch)?;
    let mut hardware = Loopback::new(&setup.initial_positions);
    loop {
        let mut size = [0u8; 4];
        match input.read_exact(&mut size) {
            Ok(()) => {}
            Err(e) if e.kind() == io::ErrorKind::UnexpectedEof => break,
            Err(e) => return Err(e.into()),
        }
        let len = u32::from_le_bytes(size) as usize;
        if len == 0 || len > MAX_FRAME {
            return Err("frame length exceeds harness limit".into());
        }
        let mut bytes = vec![0u8; len];
        input.read_exact(&mut bytes)?;
        let parsed = rmp_serde::from_slice::<Input>(&bytes);
        let (reply, error) = match parsed {
            Ok(frame) if frame.protocol_version == PROTOCOL_VERSION => {
                if let Some(f) = frame.faults {
                    hardware.frozen = f.frozen;
                    hardware.fail_read = f.fail_read;
                    hardware.fail_write = f.fail_write;
                    hardware.motor_fault = f.motor_fault;
                }
                if let Some(measured) = frame.feedback {
                    if measured.len() != hardware.feedback.len() {
                        return Err("feedback must cover every joint".into());
                    }
                    for (joint, m) in hardware.feedback.iter_mut().zip(measured) {
                        joint.position = m.position;
                        joint.velocity = m.velocity;
                        joint.effort = m.effort;
                    }
                }
                let reply = if let Some(request) = frame.request {
                    Some(controller.handle(frame.now_ms, request, &mut hardware))
                } else {
                    controller.tick(frame.now_ms, &mut hardware);
                    None
                };
                (reply, None)
            }
            other => {
                let error = match other {
                    Err(e) => e.to_string(),
                    _ => "unsupported protocol version".into(),
                };
                let now = controller.status().now_ms;
                controller.handle(now, Request::EmergencyStop, &mut hardware);
                (None, Some(error))
            }
        };
        let result = Output {
            reply,
            status: controller.status(),
            hardware_enabled: hardware.enabled,
            writes: hardware.writes,
            feedback: controller.feedback().to_vec(),
            error,
        };
        let bytes = rmp_serde::to_vec_named(&result)?;
        output.write_all(&(bytes.len() as u32).to_le_bytes())?;
        output.write_all(&bytes)?;
        output.flush()?;
    }
    Ok(())
}
