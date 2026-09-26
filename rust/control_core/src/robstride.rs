//! Strict, allocation-free codec for RobStride's extended-ID motion protocol.
//! Ranges are explicit commissioning data: never infer a motor model from its ID.
//! No flash writes, zero changes, automatic fault clearing or implicit clamping.
use crate::protocol::JointCommand;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Frame {
    pub id: u32,
    pub data: [u8; 8],
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Ranges {
    pub position: f64,
    pub velocity: f64,
    pub effort: f64,
    pub kp: f64,
    pub kd: f64,
}
impl Ranges {
    pub fn validate(self) -> Result<(), &'static str> {
        if [self.position, self.velocity, self.effort, self.kp, self.kd]
            .iter()
            .any(|x| !x.is_finite() || *x <= 0.0)
        {
            return Err("invalid motor protocol ranges");
        }
        Ok(())
    }
}

#[derive(Debug, Clone, Copy, Serialize)]
pub struct Feedback {
    pub motor_id: u8,
    pub mode: u8,
    pub error_bits: u8,
    pub position: f64,
    pub velocity: f64,
    pub effort: f64,
    pub temperature: f64,
}

fn encode(value: f64, min: f64, max: f64) -> Result<u16, &'static str> {
    if !value.is_finite() || value < min || value > max {
        return Err("motor command outside protocol range");
    }
    // Protocol affine conversion uses the full 65535 span in both directions.
    Ok(((value - min) * 65535.0 / (max - min)) as u16)
}
fn decode(value: u16, min: f64, max: f64) -> f64 {
    f64::from(value) * (max - min) / 65535.0 + min
}

pub fn control(id: u8, r: Ranges, c: JointCommand) -> Result<Frame, &'static str> {
    r.validate()?;
    let values = [
        encode(c.position, -r.position, r.position)?,
        encode(c.velocity, -r.velocity, r.velocity)?,
        encode(c.kp, 0.0, r.kp)?,
        encode(c.kd, 0.0, r.kd)?,
    ];
    let torque = encode(c.effort, -r.effort, r.effort)?;
    let mut data = [0; 8];
    for (slot, value) in data.as_chunks_mut::<2>().0.iter_mut().zip(values) {
        slot.copy_from_slice(&value.to_be_bytes());
    }
    Ok(Frame {
        id: (1 << 24) | (u32::from(torque) << 8) | u32::from(id),
        data,
    })
}

pub fn feedback(frame: Frame, host: u8, r: Ranges) -> Result<Feedback, &'static str> {
    if frame.id >> 24 != 2 || (frame.id & 255) != u32::from(host) || is_version(frame) {
        return Err("not feedback for this host");
    }
    r.validate()?;
    let value = |offset| u16::from_be_bytes([frame.data[offset], frame.data[offset + 1]]);
    let mode = ((frame.id >> 22) & 3) as u8;
    if mode == 3 {
        return Err("reserved motor mode");
    }
    Ok(Feedback {
        motor_id: ((frame.id >> 8) & 255) as u8,
        mode,
        error_bits: ((frame.id >> 16) & 63) as u8,
        position: decode(value(0), -r.position, r.position),
        velocity: decode(value(2), -r.velocity, r.velocity),
        effort: decode(value(4), -r.effort, r.effort),
        temperature: f64::from(value(6)) / 10.0,
    })
}

/// Firmware replies reuse type 2 but contain a version, not joint measurements.
pub fn is_version(frame: Frame) -> bool {
    frame.id >> 24 == 2 && frame.data[..3] == [0, 0xc4, 0x56]
}

pub fn version_request(id: u8, host: u8) -> Frame {
    let mut frame = management(id, host, 4);
    frame.data[1] = 0xc4;
    frame
}

pub fn management(id: u8, host: u8, kind: u8) -> Frame {
    Frame {
        id: (u32::from(kind) << 24) | (u32::from(host) << 8) | u32::from(id),
        data: [0; 8],
    }
}
pub fn motion_mode(id: u8, host: u8) -> Frame {
    let mut frame = management(id, host, 18);
    frame.data[..2].copy_from_slice(&0x7005u16.to_le_bytes());
    frame // run_mode=0; never writes flash
}
pub fn read_parameter(id: u8, host: u8, parameter: u16) -> Frame {
    let mut frame = management(id, host, 17);
    frame.data[..2].copy_from_slice(&parameter.to_le_bytes());
    frame
}
