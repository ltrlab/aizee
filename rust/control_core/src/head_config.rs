//! The head receiver's configuration and its validation, portable so the rules are unit-tested on every
//! platform. The Linux head service (head_service.rs) runs it: the same core, leases, limits and stops as
//! the arms' receiver, on the head's Feetech bus instead of CAN.
use crate::{feetech, feetech::Servo, reference::Reference, Config, JointUnit};
use serde::{Deserialize, Serialize};
use std::f64::consts::TAU;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum HeadMode {
    /// Simulated servos on an in-process bus: the virtual head receiver.
    VirtualBus,
    /// The physical servos, read only: requests go out, nothing is written, nothing enables.
    Observe,
    /// The physical servos under the core's leases, limits and stops, after the reference-pose gate.
    /// Collision checks stay off: the arms keep out of everywhere the head can reach instead.
    TeleopDirect,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SerialBus {
    /// The adapter's stable path, named by its USB serial: /dev/serial/by-id/usb-...<serial>...
    pub device: String,
    pub usb_serial: Option<String>,
    pub baud: u32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct HeadServiceConfig {
    pub mode: HeadMode,
    pub endpoint: String,
    pub period_us: u64,
    pub maximum_lateness_us: u64,
    pub bus: SerialBus,
    pub servos: Vec<Servo>,
    pub controller: Config,
    #[serde(default)]
    pub reference_positions: Vec<Reference>,
}
impl HeadServiceConfig {
    pub fn physical(&self) -> bool {
        self.mode != HeadMode::VirtualBus
    }

    pub fn validate(&self) -> Result<(), String> {
        self.controller.validate()?;
        if !(2000..=20000).contains(&self.period_us)
            || self.maximum_lateness_us < self.period_us
            || self.maximum_lateness_us > 30000
            || !self.endpoint.starts_with("tcp://127.0.0.1:")
            || self.endpoint[16..]
                .parse::<u16>()
                .ok()
                .filter(|p| *p > 1024)
                .is_none()
            || self.servos.is_empty()
            || self.servos.len() != self.controller.joints.len()
            || self.bus.baud != 1_000_000
        {
            return Err("invalid head rates, loopback endpoint, roster or baud rate".into());
        }
        let serial = self.bus.usb_serial.as_deref().unwrap_or("");
        let pinned = self.bus.device.starts_with("/dev/serial/by-id/")
            && !serial.is_empty()
            && serial.bytes().all(|c| c.is_ascii_alphanumeric())
            && self.bus.device.contains(serial);
        if self.physical() != pinned || (!self.physical() && !self.bus.device.is_empty()) {
            return Err(
                "the physical head bus is its adapter's /dev/serial/by-id path naming its USB serial; the virtual bus has none"
                    .into(),
            );
        }
        for (i, (servo, joint)) in self.servos.iter().zip(&self.controller.joints).enumerate() {
            servo.validate()?;
            if servo.name != joint.name
                || self.servos[..i]
                    .iter()
                    .any(|s| s.id == servo.id || s.name == servo.name)
            {
                return Err("head joint/servo roster mismatch or duplicate".into());
            }
            // Every command the core may accept must fit the servo: one turn of steps, the gain registers,
            // its speed and its full load.
            let in_turn =
                |q: f64| (0.0..=feetech::RESOLUTION - 1.0).contains(&servo.steps(q).round());
            if !matches!(joint.unit, JointUnit::Radians)
                || !in_turn(joint.lower)
                || !in_turn(joint.upper)
                || joint.max_kp > 255.0
                || joint.max_kd > 255.0
                || joint.max_velocity * feetech::RESOLUTION / TAU > feetech::MAX_SPEED_STEPS
                || joint.max_effort > servo.full_load_nm
            {
                return Err(format!(
                    "{}: its limits exceed its servo's ranges",
                    joint.name
                ));
            }
        }
        if self.mode == HeadMode::TeleopDirect
            && (self.controller.direct.is_none()
                || self.controller.collision.is_some()
                || self.controller.lease_ms > 250
                || self.controller.feedback_timeout_ms > 100)
        {
            return Err(
                "head teleop requires the direct policy (collision off), lease <=250 ms and feedback timeout <=100 ms"
                    .into(),
            );
        }
        if self.mode == HeadMode::TeleopDirect {
            let names: Vec<_> = self.servos.iter().map(|s| s.name.as_str()).collect();
            crate::reference::Gate::new(self.reference_positions.clone(), &names)?;
        } else if !self.reference_positions.is_empty() {
            return Err("reference positions only gate physical head teleop".into());
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn config(mode: HeadMode) -> HeadServiceConfig {
        let joints: Vec<_> = ["head_pan", "head_tilt"]
            .iter()
            .map(|name| {
                serde_json::json!({
                    "name": name, "unit": "radians", "lower": -1.2, "upper": 1.2, "max_velocity": 2.0,
                    "max_effort": 3.0, "max_kp": 64.0, "max_kd": 64.0, "stop_acceleration": 8.0,
                    "hold_kp": 32.0, "hold_kd": 32.0, "max_tracking_error": 0.3, "max_temperature": 65.0
                })
            })
            .collect();
        let mut value = serde_json::json!({
            "mode": mode,
            "endpoint": "tcp://127.0.0.1:5591",
            "period_us": 10000,
            "maximum_lateness_us": 15000,
            "bus": {"device": "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5A46083062-if00",
                    "usb_serial": "5A46083062", "baud": 1000000},
            "servos": [
                {"name": "head_pan", "id": 1, "model": 2825, "firmware": "3.10", "direction": 1.0,
                 "zero_steps": 2048.0, "full_load_nm": 4.9},
                {"name": "head_tilt", "id": 2, "model": 2825, "firmware": "3.10", "direction": -1.0,
                 "zero_steps": 2048.0, "full_load_nm": 4.9}
            ],
            "controller": {
                "group": "head", "lease_ms": 200, "feedback_timeout_ms": 50, "settled_velocity": 0.05,
                "identity": {"model": "minerva-head", "calibration": "head-test", "collision_policy": "disabled-v1"},
                "direct": {"command_ms": 50, "velocity_timeout_ms": 50, "limit_reaction_ms": 100},
                "joints": joints
            },
            "reference_positions": [
                {"name": "head_pan", "raw_position_rad": std::f64::consts::PI, "tolerance_rad": 0.05},
                {"name": "head_tilt", "raw_position_rad": std::f64::consts::PI, "tolerance_rad": 0.05}
            ]
        });
        if mode == HeadMode::VirtualBus {
            value["bus"] = serde_json::json!({"device": "", "usb_serial": null, "baud": 1000000});
        }
        if mode != HeadMode::TeleopDirect {
            value["reference_positions"] = serde_json::json!([]);
        }
        serde_json::from_value(value).unwrap()
    }

    #[test]
    fn a_pinned_physical_head_and_a_virtual_one_validate() {
        for mode in [
            HeadMode::TeleopDirect,
            HeadMode::Observe,
            HeadMode::VirtualBus,
        ] {
            config(mode).validate().unwrap();
        }
    }

    #[test]
    fn the_bus_must_be_pinned_by_usb_serial() {
        let mut c = config(HeadMode::TeleopDirect);
        c.bus.device = "/dev/ttyACM0".into();
        assert!(c.validate().is_err());
        let mut c = config(HeadMode::TeleopDirect);
        c.bus.usb_serial = Some("OTHER123".into());
        assert!(c.validate().is_err());
        let mut c = config(HeadMode::VirtualBus);
        c.bus.device = "/dev/serial/by-id/x".into();
        assert!(c.validate().is_err());
    }

    #[test]
    fn limits_must_fit_the_servo() {
        let mut c = config(HeadMode::TeleopDirect);
        c.controller.joints[0].upper = 3.2; // Past the servo's single turn from zero at 2048.
        assert!(c.validate().unwrap_err().contains("head_pan"));
        let mut c = config(HeadMode::TeleopDirect);
        c.controller.joints[1].max_kp = 300.0;
        assert!(c.validate().unwrap_err().contains("head_tilt"));
        let mut c = config(HeadMode::TeleopDirect);
        c.controller.joints[1].max_effort = 6.0;
        assert!(c.validate().is_err());
        let mut c = config(HeadMode::TeleopDirect);
        c.servos[1].name = "neck".into();
        assert!(c.validate().is_err());
    }

    #[test]
    fn teleop_needs_the_direct_policy_and_the_reference_pose() {
        let mut c = config(HeadMode::TeleopDirect);
        c.controller.direct = None;
        assert!(c.validate().is_err());
        let mut c = config(HeadMode::TeleopDirect);
        c.reference_positions.clear();
        assert!(c.validate().is_err());
        let mut c = config(HeadMode::Observe);
        c.reference_positions = config(HeadMode::TeleopDirect).reference_positions;
        assert!(c.validate().is_err());
    }
}
