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

#[test]
fn direct_teleop_is_explicit_and_keeps_physical_gates_for_one_or_two_arms() {
    for names in [vec!["left_arm_j1"], vec!["left_arm_j1", "right_arm_j1"]] {
        let mut value = profile("teleop_direct", &names, 2.0);
        value["controller"]["identity"]["collision_policy"] = json!("disabled-v1");
        value["controller"]["direct"] = json!({"command_ms": 30, "velocity_timeout_ms": 10, "limit_reaction_ms": 10});
        let config = parse(value.clone());
        config.validate().unwrap();
        assert!(config.physical_enable());
        let mut invalid = value.clone();
        invalid["mode"] = json!("teleop");
        assert!(error(invalid).contains("explicit"));
        let mut invalid = value.clone();
        invalid["watchdog_identities"] = json!([]);
        assert!(error(invalid).contains("watchdog"));
        let mut invalid = value.clone();
        invalid["reference_positions"] = json!([]);
        assert!(error(invalid).contains("reference"));
        let mut invalid = value.clone();
        invalid["controller"]["collision"] = json!({"world_revision":"w", "scope":"all", "max_verdict_ms":80, "reaction_ms":10});
        assert!(error(invalid).contains("direct policy"));
    }
}

fn error(value: Value) -> String {
    parse(value)
        .validate()
        .expect_err("fixture must be rejected")
}

#[test]
fn a_zero_torque_trial_needs_one_motor_zero_gains_watchdogs_and_references() {
    let valid = parse(profile("zero_torque_trial", &["left_gripper"], 0.0));
    assert_eq!(valid.mode, Mode::ZeroTorqueTrial);
    valid.validate().unwrap();
    assert!(error(profile("zero_torque_trial", &["left_gripper"], 2.0)).contains("zero gains"));
    assert!(error(profile(
        "zero_torque_trial",
        &["left_arm_j6", "left_gripper"],
        0.0
    ))
    .contains("one motor"));
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

const LEFT_ARM: [&str; 7] = [
    "left_arm_j1",
    "left_arm_j2",
    "left_arm_j3",
    "left_arm_j4",
    "left_arm_j5",
    "left_arm_j6",
    "left_gripper",
];

fn teleop(names: &[&str]) -> Value {
    let mut value = profile("teleop", names, 2.0);
    value["controller"]["collision"] = json!({"world_revision": "w", "scope": "inter_branch", "max_verdict_ms": 80, "reaction_ms": 10});
    value
}

#[test]
fn teleop_drives_one_gated_arm_with_holding_gains_and_collision_envelopes() {
    let valid = parse(teleop(&LEFT_ARM));
    assert_eq!(valid.mode, Mode::Teleop);
    assert!(valid.physical_enable());
    valid.validate().unwrap();
    let both: Vec<&str> = LEFT_ARM.iter().copied().chain(["right_arm_j1"]).collect();
    assert!(error(teleop(&both)).contains("one arm"));
    let mut unguarded = teleop(&LEFT_ARM);
    unguarded["controller"]["collision"] = Value::Null;
    assert!(error(unguarded).contains("collision policy"));
    let mut slow = teleop(&LEFT_ARM);
    slow["controller"]["feedback_timeout_ms"] = json!(150);
    assert!(error(slow).contains("feedback timeout"));
    let mut long = teleop(&LEFT_ARM);
    long["controller"]["lease_ms"] = json!(300);
    long["controller"]["collision"]["max_verdict_ms"] = json!(80);
    assert!(error(long).contains("lease"));
    let mut unpinned = teleop(&LEFT_ARM);
    unpinned["watchdog_identities"]
        .as_array_mut()
        .unwrap()
        .pop();
    assert!(error(unpinned).contains("watchdog"));
    let mut unreferenced = teleop(&LEFT_ARM);
    unreferenced["reference_positions"] = json!([]);
    assert!(error(unreferenced).contains("reference"));
    let mut virtual_bus = teleop(&LEFT_ARM);
    virtual_bus["buses"][0]["usb_serial"] = Value::Null;
    assert!(error(virtual_bus).contains("CAN interface identity"));
}

#[test]
fn teleop_drives_both_arms_only_under_a_motion_collision_bound() {
    let both: Vec<String> = LEFT_ARM
        .iter()
        .map(|name| name.to_string())
        .chain(
            LEFT_ARM
                .iter()
                .map(|name| name.replacen("left", "right", 1)),
        )
        .collect();
    let both: Vec<&str> = both.iter().map(String::as_str).collect();
    // A reserve bound's boxes are too wide to certify one arm against the other.
    assert!(error(teleop(&both)).contains("one arm"));
    let mut motion = teleop(&both);
    motion["controller"]["collision"]["bound"] = json!("motion");
    motion["controller"]["interpolation_ms"] = json!(20);
    let valid = parse(motion.clone());
    assert_eq!(valid.buses.len(), 2);
    valid.validate().unwrap();
    // A motion bound covers the reference's ramp: commands must be ramped.
    let mut unramped = motion.clone();
    unramped["controller"]["interpolation_ms"] = json!(0);
    assert!(error(unramped).contains("interpolated"));
    let mut three = motion;
    three["buses"]
        .as_array_mut()
        .unwrap()
        .push(json!({"interface": "can_head", "usb_serial": "serial-head"}));
    assert!(error(three).contains("one arm"));
}

#[test]
fn every_command_limit_fits_the_motors_protocol_ranges() {
    // A damping ceiling over the motor's wire range (right J4: 5.6 against 5.0) failed to encode
    // mid-session, which the receiver took as a drive fault and cut torque.
    parse(teleop(&LEFT_ARM)).validate().unwrap();
    for (key, value) in [
        ("max_kd", 5.6),
        ("max_kp", 501.0),
        ("max_effort", 14.5),
        ("max_velocity", 34.0),
        ("upper", 12.6),
        ("lower", -12.6),
    ] {
        let mut over = teleop(&LEFT_ARM);
        over["controller"]["joints"][1][key] = json!(value);
        assert!(error(over).contains("protocol ranges"), "{key}");
    }
}
