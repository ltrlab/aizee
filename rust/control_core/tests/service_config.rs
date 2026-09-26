use robot_control::service_config::{Mode, ServiceConfig};
use serde_json::{json, Value};

fn joint(name: &str, hold_kp: f64) -> Value {
    json!({"name": name, "unit": "radians", "lower": -1.0, "upper": 1.0, "max_velocity": 1.0,
           "max_effort": 14.0, "max_kp": 5.0, "max_kd": 2.0, "stop_acceleration": 4.0,
           "hold_kp": hold_kp, "hold_kd": hold_kp / 2.0, "max_tracking_error": 0.1,
           "max_temperature": 70.0})
}

fn motor(name: &str, bus: usize, id: u8) -> Value {
    json!({"name": name, "bus": bus, "id": id, "scale": 1.0, "zero": 0.0,
           "ranges": {"position": 12.566, "velocity": 33.0, "effort": 14.0, "kp": 500.0, "kd": 5.0}})
}

/// A physical profile over `names`, on one bus per arm prefix.
fn profile(mode: &str, names: &[&str], hold_kp: f64) -> Value {
    let arms: Vec<&str> = names.iter().map(|n| n.split('_').next().unwrap()).collect();
    let mut buses: Vec<&str> = arms.clone();
    buses.dedup();
    json!({
        "mode": mode, "endpoint": "tcp://127.0.0.1:5590", "period_us": 2500, "maximum_lateness_us": 10000,
        "host_id": 170, "conflicting_services": ["aizee-minerva-left"],
        "buses": buses.iter().map(|arm| json!({"interface": format!("can_{arm}"), "usb_serial": format!("serial-{arm}")})).collect::<Vec<_>>(),
        "motors": names.iter().zip(&arms).enumerate().map(|(i, (name, arm))|
            motor(name, buses.iter().position(|b| b == arm).unwrap(), 4 + (i % 7) as u8)).collect::<Vec<_>>(),
        "controller": {"group": "fixture", "identity": {"model": "m", "calibration": "c", "collision_policy": "p"},
                       "lease_ms": 150, "feedback_timeout_ms": 30, "settled_velocity": 0.1,
                       "joints": names.iter().map(|name| joint(name, hold_kp)).collect::<Vec<_>>()},
        "watchdog_identities": names.iter().map(|_| json!({"uid": [1, 2, 3, 4, 5, 6, 7, 8], "firmware": [0, 2, 3, 34]})).collect::<Vec<_>>(),
        "reference_positions": names.iter().map(|name| json!({"name": name, "raw_position_rad": 0.0, "tolerance_rad": 0.1})).collect::<Vec<_>>(),
    })
}

fn parse(value: Value) -> ServiceConfig {
    serde_json::from_value(value).expect("fixture parses")
}

fn error(value: Value) -> String {
    parse(value).validate().expect_err("fixture must be rejected")
}

#[test]
fn a_zero_torque_trial_needs_one_motor_zero_gains_watchdogs_and_references() {
    let valid = parse(profile("zero_torque_trial", &["left_gripper"], 0.0));
    assert_eq!(valid.mode, Mode::ZeroTorqueTrial);
    valid.validate().unwrap();
    assert!(error(profile("zero_torque_trial", &["left_gripper"], 2.0)).contains("zero gains"));
    assert!(error(profile("zero_torque_trial", &["left_arm_j6", "left_gripper"], 0.0)).contains("one motor"));
    let mut unpinned = profile("zero_torque_trial", &["left_gripper"], 0.0);
    unpinned["watchdog_identities"] = json!([]);
    assert!(error(unpinned).contains("watchdog"));
    let mut unreferenced = profile("zero_torque_trial", &["left_gripper"], 0.0);
    unreferenced["reference_positions"][0]["tolerance_rad"] = json!(0.2);
    assert!(error(unreferenced).contains("reference"));
}

#[test]
fn observation_modes_never_carry_enable_gates_and_physical_buses_need_serials() {
    let mut observe = profile("observe", &["left_arm_j1"], 0.0);
    assert!(error(observe.clone()).contains("reference positions only"));
    observe["reference_positions"] = json!([]);
    assert!(error(observe.clone()).contains("watchdog setup only"));
    observe["watchdog_identities"] = json!([]);
    parse(observe.clone()).validate().unwrap();
    observe["buses"][0]["usb_serial"] = Value::Null;
    assert!(error(observe).contains("CAN interface identity"));
}
