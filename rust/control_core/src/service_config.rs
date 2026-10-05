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
    /// Guarded teleoperation of one arm on physical CAN (both, on two buses, under a
    /// motion collision bound): Idle, Arm and Move under
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
            // Every command the controller may accept must fit its motor's wire protocol. One that
            // does not fails to encode, which the receiver takes as a drive fault, cutting torque:
            // right J4's damping ceiling of 5.6 against its motor's 5.0 (2026-09-28 and -30).
            let (r, s) = (m.ranges, m.scale.abs());
            let position = |q: f64| (q / m.scale + m.zero).abs();
            if j.max_kp * s * s > r.kp
                || j.max_kd * s * s > r.kd
                || j.max_effort * s > r.effort
                || j.max_velocity / s > r.velocity
                || position(j.lower) > r.position
                || position(j.upper) > r.position
            {
                return Err(format!(
                    "{}: its limits exceed its motor's protocol ranges",
                    j.name
                ));
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
            // One bus drives one arm. Both arms (two buses) only under a motion collision bound,
            // whose boxes are tight enough to certify one arm against the other. Every Arm/Move
            // needs a receiver-checked envelope, and stale feedback or an idle owner must stop it
            // quickly.
            let motion = self
                .controller
                .collision
                .as_ref()
                .is_some_and(|policy| policy.bound == crate::collision::Bound::Motion);
            if self.buses.len() > 2 || (self.buses.len() == 2 && !motion) {
                return Err(
                    "physical teleop drives one arm (one CAN bus), or both (two) under a motion collision bound"
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
