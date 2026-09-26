//! Single-use pre-enable reference-pose check, in RAW motor radians (no wrapping).
//! Encoder data cannot prove the physical pose. The operator must independently
//! match the documented reference; this gate never writes or repairs a zero.
use crate::{hardware::HardwareError, robstride::Feedback};
use serde::{Deserialize, Serialize};

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
    failed: bool,
    consumed: bool,
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
            failed: false,
            consumed: false,
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
        if self.failed
            || stamp > now
            || now - stamp > 40
            || f.mode != 0
            || f.error_bits != 0
            || !f.position.is_finite()
            || !f.velocity.is_finite()
            || (f.position - r.raw_position_rad).abs() > r.tolerance_rad
            || f.velocity.abs() > 0.2
        {
            self.failed = true;
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
