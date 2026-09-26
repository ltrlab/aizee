//! Nonblocking pre-enable gate. Only the volatile watchdog register is written.
//! Pinned firmware must already have passed the separate command-loss trial.
use crate::{
    can_adapter::{CanIo, Motor, Received},
    hardware::HardwareError,
    protocol::JointCommand,
    robstride,
    watchdog_trial::Identity,
};

pub const TICKS: u32 = 600;
const FRESH_MS: u64 = 40;
struct Entry {
    identity: Identity,
    phase: u8,
    sent: Option<u64>,
    verified: Option<u64>,
    identified: bool,
    position: f64,
}
pub struct Gate {
    entries: Vec<Entry>,
    consumed: bool,
    failed: bool,
}
impl Gate {
    pub fn new(identities: Vec<Identity>, count: usize) -> Result<Self, &'static str> {
        if identities.len() != count || count == 0 || identities.iter().any(|p| p.uid == [0; 8]) {
            return Err("watchdog requires one pinned identity per motor");
        }
        Ok(Self {
            entries: identities
                .into_iter()
                .map(|identity| Entry {
                    identity,
                    phase: 0,
                    sent: None,
                    verified: None,
                    identified: false,
                    position: 0.,
                })
                .collect(),
            consumed: false,
            failed: false,
        })
    }
    pub fn status(&self, now: u64) -> &'static str {
        if self.failed {
            "failed"
        } else if self.consumed {
            "consumed"
        } else if self
            .entries
            .iter()
            .all(|e| e.verified.is_some_and(|t| now >= t && now - t < FRESH_MS))
        {
            "ready"
        } else {
            "preparing"
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
        self.consumed = true;
    }
    pub fn abort<I: CanIo>(&mut self, io: &mut I, motors: &[Motor], host: u8) -> u64 {
        if self.failed {
            return 0;
        }
        self.failed = true;
        let mut sent = 0;
        // A neutral command might have unexpectedly enabled defective firmware.
        for (e, m) in self.entries.iter().zip(motors) {
            if e.identified && io.send(m.bus, robstride::management(m.id, host, 4)).is_ok() {
                sent += 1;
            }
        }
        sent
    }
    pub fn observe(
        &mut self,
        rx: Received,
        motors: &[Motor],
        host: u8,
        now: u64,
    ) -> Result<(), HardwareError> {
        if self.failed || self.consumed {
            return Ok(());
        }
        let Some(i) = motors
            .iter()
            .position(|m| m.bus == rx.bus && m.id == ((rx.frame.id >> 8) & 255) as u8)
        else {
            return Ok(());
        };
        let e = &mut self.entries[i];
        let m = &motors[i];
        let f = rx.frame;
        let Some(sent) = e.sent else {
            return Ok(());
        };
        if rx.sampled_at_ms < sent {
            return Ok(());
        }
        if rx.sampled_at_ms > now || now.saturating_sub(sent) > FRESH_MS {
            return Err(HardwareError);
        }
        match e.phase {
            0 => {
                if f.id != (u32::from(m.id) << 8) | 0xfe {
                    return Ok(());
                }
                if f.data != e.identity.uid {
                    return Err(HardwareError);
                }
                e.identified = true;
            }
            1 => {
                if !robstride::is_version(f) || ![host, m.id].contains(&(f.id as u8)) {
                    return Ok(());
                }
                if f.data[3..7] != e.identity.firmware || (f.id >> 16) & 63 != 0 {
                    return Err(HardwareError);
                }
            }
            2 | 6 | 7 => {
                if f.id >> 24 != 2 || f.id as u8 != host || robstride::is_version(f) {
                    return Ok(());
                }
                let fb = robstride::feedback(f, host, m.ranges).map_err(|_| HardwareError)?;
                if fb.mode != 0
                    || fb.error_bits != 0
                    || fb.temperature > 60.
                    || fb.velocity.abs() > 0.2
                {
                    return Err(HardwareError);
                }
                e.position = fb.position;
                if e.phase == 6 {
                    e.verified = Some(rx.sampled_at_ms);
                }
            }
            3 | 5 | 8 => {
                let parameter = if e.phase == 5 { 0x7028u16 } else { 0x7005u16 };
                if f.id & 0xff00ffff != (17 << 24) | (u32::from(m.id) << 8) | u32::from(host)
                    || f.data[..2] != parameter.to_le_bytes()
                {
                    return Ok(());
                }
                if (f.id >> 16) & 255 != 0 {
                    return Err(HardwareError);
                }
                if (e.phase == 5 && u32::from_le_bytes(f.data[4..].try_into().unwrap()) != TICKS)
                    || (e.phase != 5 && f.data[4] != 0)
                {
                    return Err(HardwareError);
                }
            }
            _ => return Ok(()),
        }
        e.sent = None;
        e.phase = match e.phase {
            5 if e.verified.is_some() => 7,
            5 => 6,
            6 => 9,
            8 => 6,
            p => p + 1,
        };
        Ok(())
    }
    /// Called only after the adapter drained its bounded RX queue. Old queued
    /// replies cannot acknowledge a request that has not yet been sent.
    pub fn poll<I: CanIo>(
        &mut self,
        io: &mut I,
        motors: &[Motor],
        host: u8,
        now: u64,
    ) -> Result<u64, HardwareError> {
        if self.failed {
            return Err(HardwareError);
        }
        if self.consumed {
            return Ok(0);
        }
        let mut sent_count = 0;
        for (e, m) in self.entries.iter_mut().zip(motors) {
            if let Some(sent) = e.sent {
                if now.saturating_sub(sent) > FRESH_MS {
                    return Err(HardwareError);
                }
                continue;
            }
            if e.phase == 9 {
                if now.saturating_sub(e.verified.unwrap()) < 10 {
                    continue;
                }
                e.phase = 5; // Audit without rewriting: resets/changed settings fail closed.
            }
            let frame = match e.phase {
                0 => robstride::management(m.id, host, 0),
                1 => robstride::version_request(m.id, host),
                2 | 7 => robstride::management(m.id, host, 4),
                3 | 8 => robstride::read_parameter(m.id, host, 0x7005),
                4 => {
                    let mut f = robstride::management(m.id, host, 18);
                    f.data[..2].copy_from_slice(&0x7028u16.to_le_bytes());
                    f.data[4..].copy_from_slice(&TICKS.to_le_bytes());
                    f
                }
                5 => robstride::read_parameter(m.id, host, 0x7028),
                6 => robstride::control(
                    m.id,
                    m.ranges,
                    JointCommand {
                        position: e.position,
                        velocity: 0.,
                        effort: 0.,
                        kp: 0.,
                        kd: 0.,
                    },
                )
                .map_err(|_| HardwareError)?,
                _ => return Err(HardwareError),
            };
            io.send(m.bus, frame)?;
            sent_count += 1;
            if e.phase == 4 {
                e.phase = 5;
            } else {
                e.sent = Some(now);
            }
        }
        Ok(sent_count)
    }
}
