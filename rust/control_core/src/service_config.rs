//! Guarded service configuration and its validation, portable so the rules are
//! unit-tested on every platform. The Linux service (service.rs) runs it.
use crate::{can_adapter::Motor, Config};
use serde::{Deserialize, Serialize};

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
    /// Guarded teleoperation of one arm on physical CAN: Idle, Arm and Move under
    /// the receiver's leases, collision envelopes and stops, after the same
    /// watchdog and reference-pose gates as the zero-torque trial.
    Teleop,
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
    /// Modes that may physically enable motors after the watchdog and reference gates.
    pub fn physical_enable(&self) -> bool {
        matches!(self.mode, Mode::ZeroTorqueTrial | Mode::Teleop)
    }

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
        if self.mode == Mode::Teleop {
            // One bus drives one arm: the other arm is never enabled until a
            // two-arm collision bound exists. Every Arm/Move needs a receiver-checked
            // envelope, and stale feedback or an idle owner must stop it quickly.
            if self.buses.len() != 1 {
                return Err(
                    "physical teleop drives one arm (one CAN bus) until a two-arm collision bound exists"
                        .into(),
                );
            }
            if self.controller.collision.is_none()
                || self.controller.lease_ms > 250
                || self.controller.feedback_timeout_ms > 100
            {
                return Err(
                    "physical teleop requires a collision policy, lease <=250 ms and feedback timeout <=100 ms"
                        .into(),
                );
            }
        }
        if self.physical_enable() && self.watchdog_identities.len() != self.motors.len() {
            return Err("physical enable requires pinned watchdog identities".into());
        }
        if self.physical_enable() {
            let names: Vec<_> = self.motors.iter().map(|m| m.name.as_str()).collect();
            crate::reference::Gate::new(self.reference_positions.clone(), &names)?;
        } else if !self.reference_positions.is_empty() {
            return Err(
                "reference positions only supported by explicit physical enable mode".into(),
            );
        }
        if !self.physical_enable() && !self.watchdog_identities.is_empty() {
            return Err("watchdog setup only available in explicit physical enable modes".into());
        }
        Ok(())
    }
}
