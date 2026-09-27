//! Single-use pre-enable reference-pose check, in RAW motor radians (no wrapping).
//! Encoder data cannot prove the physical pose. The operator must independently
//! match the documented reference; this gate never writes or repairs a zero.
use crate::{hardware::HardwareError, robstride::Feedback};
use serde::{Deserialize, Serialize};

/// A still motor's drive still reports speed noise: on Minerva, motors at rest read
/// about -0.06..+0.01 rad/s, and an unmoving wrist once read +0.211 rad/s. Real
/// motion persists, so a motor counts as moving only when MOVING_READINGS readings
/// in a row exceed STILL_RAD_S.
pub const STILL_RAD_S: f64 = 0.2;
pub const MOVING_READINGS: u8 = 3;

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Reference {
    pub name: String,
    pub raw_position_rad: f64,
    pub tolerance_rad: f64,
}

pub struct Gate {
    references: Vec<Reference>,
    first: Vec<Option<u64>>,
    last: Vec<Option<u64>>,
    count: Vec<u32>,
    /// Readings in a row over STILL_RAD_S, per motor.
    moving: Vec<u8>,
    failed: bool,
    consumed: bool,
    /// The first failure, naming the motor and the value that failed it.
    pub failure: Option<String>,
}
impl Gate {
    pub fn new(references: Vec<Reference>, names: &[&str]) -> Result<Self, &'static str> {
        if references.is_empty()
            || references.len() != names.len()
            || references.iter().zip(names).any(|(r, name)| {
                r.name != *name
                    || !r.raw_position_rad.is_finite()
                    || !r.tolerance_rad.is_finite()
                    || r.tolerance_rad <= 0.0
                    || r.tolerance_rad > 0.1
            })
        {
            return Err("reference requires ordered raw motor positions and tolerances <=0.1 rad");
        }
        let n = references.len();
        Ok(Self {
            references,
            first: vec![None; n],
            last: vec![None; n],
            count: vec![0; n],
            moving: vec![0; n],
            failed: false,
            consumed: false,
            failure: None,
        })
    }
    pub fn observe(
        &mut self,
        index: usize,
        f: Feedback,
        stamp: u64,
        now: u64,
    ) -> Result<(), HardwareError> {
        if self.consumed {
            return Ok(());
        }
        let r = &self.references[index];
        let moving = if f.velocity.abs() > STILL_RAD_S {
            self.moving[index].saturating_add(1)
        } else {
            0
        };
        let problem = if self.failed {
            Some(format!(
                "{}: the reference check had already failed or was revoked",
                r.name
            ))
        } else if stamp > now || now - stamp > 40 {
            Some(format!(
                "{}: feedback read {} ms after it arrived (limit 40 ms)",
                r.name,
                now.saturating_sub(stamp)
            ))
        } else if f.mode != 0 {
            Some(format!("{}: in mode {} (not Reset)", r.name, f.mode))
        } else if f.error_bits != 0 {
            Some(format!("{}: fault bits {:#04x}", r.name, f.error_bits))
        } else if !f.position.is_finite() || !f.velocity.is_finite() {
            Some(format!("{}: non-finite feedback", r.name))
        } else if (f.position - r.raw_position_rad).abs() > r.tolerance_rad {
            Some(format!(
                "{}: at {:+.4} rad, {:.4} rad from its reference (tolerance {} rad)",
                r.name,
                f.position,
                (f.position - r.raw_position_rad).abs(),
                r.tolerance_rad
            ))
        } else if moving >= MOVING_READINGS {
            Some(format!(
                "{}: moving at {:+.3} rad/s, {moving} readings in a row over {STILL_RAD_S} rad/s",
                r.name, f.velocity
            ))
        } else {
            None
        };
        self.moving[index] = moving;
        if let Some(reason) = problem {
            self.failed = true;
            self.failure.get_or_insert(reason);
            return Err(HardwareError);
        }
        if self.last[index].is_some_and(|last| stamp <= last) {
            return Ok(());
        }
        self.first[index].get_or_insert(stamp);
        self.last[index] = Some(stamp);
        self.count[index] = self.count[index].saturating_add(1);
        Ok(())
    }
    pub fn status(&self, now: u64) -> &'static str {
        if self.failed {
            "failed"
        } else if self.consumed {
            "consumed"
        } else if (0..self.references.len()).all(|i| {
            self.count[i] >= 3
                && self.first[i]
                    .zip(self.last[i])
                    .is_some_and(|(first, last)| {
                        last.saturating_sub(first) >= 100 && now >= last && now - last <= 40
                    })
        }) {
            "ready"
        } else {
            "unverified"
        }
    }
    pub fn consume(&mut self, now: u64) -> Result<(), HardwareError> {
        if self.status(now) != "ready" {
            return Err(HardwareError);
        }
        self.consumed = true;
        Ok(())
    }
    pub fn revoke(&mut self) {
        self.failed = true;
    }
}
