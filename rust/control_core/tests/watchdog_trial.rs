use robot_control::{
    can_adapter::{CanHardware, CanIo, Motor, Received},
    hardware::{Hardware, HardwareError, JointFeedback},
    robstride::{Frame, Ranges},
    watchdog_trial::{self, Clock, Identity},
};
use std::{cell::Cell, collections::VecDeque, rc::Rc};

struct TestClock(Rc<Cell<u64>>);
impl Clock for TestClock {
    fn now_ms(&self) -> u64 {
        self.0.get()
    }
    fn wait_ms(&mut self, ms: u64) {
        self.0.set(self.0.get() + ms);
    }
}
struct Bus {
    now: Rc<Cell<u64>>,
    rx: VecDeque<Received>,
    tx: Vec<Frame>,
    enabled: bool,
    timeout: u32,
    last_command: u64,
    accept_timeout: bool,
    obey_timeout: bool,
    auto_enable: bool,
    uid: [u8; 8],
    firmware: [u8; 4],
    fail_enable: bool,
    allow_jog: bool,
    follow_jog: bool,
    excess_effort: bool,
    position: u16,
    breakaway: bool,
    receive_error: bool,
    post_stop_speed: bool,
    damping_fault: Option<&'static str>,
}
impl CanIo for Bus {
    fn receive(&mut self, _: u64) -> Result<Option<Received>, HardwareError> {
        if self.receive_error {
            return Err(HardwareError);
        }
        Ok(self.rx.pop_front())
    }
    fn send(&mut self, bus: usize, frame: Frame) -> Result<(), HardwareError> {
        assert_eq!(bus, 0);
        if self.obey_timeout
            && self.enabled
            && self.timeout > 0
            && self.now.get() - self.last_command >= 30
        {
            self.enabled = false;
        }
        self.last_command = self.now.get();
        self.tx.push(frame);
        let mut reply = Frame {
            id: 0x02000aaa,
            data: [0x7f, 0xff, 0x7f, 0xff, 0x7f, 0xff, 0, 220],
        };
        reply.data[..2].copy_from_slice(&self.position.to_be_bytes());
        match frame.id >> 24 {
            0 => {
                reply = Frame {
                    id: 0xafe,
                    data: self.uid,
                }
            }
            4 => {
                self.enabled = false;
                if self.post_stop_speed && self.position != 32767 {
                    reply.data[2..4].copy_from_slice(&33016u16.to_be_bytes());
                }
                if frame.data[1] == 0xc4 {
                    reply.data = [0, 0xc4, 0x56, 0, 0, 0, 0, 0];
                    reply.data[3..7].copy_from_slice(&self.firmware);
                }
            }
            17 => {
                reply = Frame {
                    id: 0x11000aaa,
                    data: frame.data,
                };
                if frame.data[..2] == [0x28, 0x70] {
                    reply.data[4..].copy_from_slice(&self.timeout.to_le_bytes());
                }
            }
            18 => {
                assert_eq!(frame.data[..4], [0x28, 0x70, 0, 0]);
                assert_eq!(u32::from_le_bytes(frame.data[4..].try_into().unwrap()), 600);
                if self.accept_timeout {
                    self.timeout = 600;
                }
                return Ok(());
            }
            3 => {
                self.enabled = true;
                if self.fail_enable {
                    return Err(HardwareError);
                }
            }
            1 => {
                if self.allow_jog {
                    assert!(u16::from_be_bytes([frame.data[4], frame.data[5]]) <= 263);
                    assert!(
                        u16::from_be_bytes([frame.data[6], frame.data[7]])
                            <= if self.breakaway { 2622 } else { 656 }
                    );
                } else {
                    assert_eq!(&frame.data[4..], &[0; 4]); // Watchdog-only never applies gains.
                }
                let torque = ((frame.id >> 8) & 65535) as f64 * 28. / 65535. - 14.;
                if self.breakaway {
                    assert!(torque.abs() <= 0.251);
                    if self.enabled && self.follow_jog && torque.abs() > 0.1 {
                        self.position =
                            (i32::from(self.position) + if torque > 0. { 10 } else { -10 }) as u16;
                        reply.data[..2].copy_from_slice(&self.position.to_be_bytes());
                    }
                    if self.enabled && self.excess_effort {
                        reply.data[4..6].copy_from_slice(&40000u16.to_be_bytes());
                    }
                    if self.enabled && u16::from_be_bytes([frame.data[6], frame.data[7]]) > 656 {
                        match self.damping_fault {
                            Some("lost_reply") => return Ok(()),
                            Some("reset") => self.enabled = false,
                            Some("speed") => {
                                reply.data[2..4].copy_from_slice(&33016u16.to_be_bytes())
                            }
                            Some("drift") => {
                                reply.data[..2].copy_from_slice(&32840u16.to_be_bytes())
                            }
                            Some("not_settled") => {
                                reply.data[2..4].copy_from_slice(&32887u16.to_be_bytes())
                            }
                            Some("gap") => self.now.set(self.now.get() + 25),
                            _ => {}
                        }
                    }
                } else {
                    assert_eq!((frame.id >> 8) & 65535, 32767);
                }
                if self.auto_enable {
                    self.enabled = true;
                }
                if self.enabled && frame.data[5] != 0 {
                    if self.follow_jog {
                        self.position = u16::from_be_bytes([frame.data[0], frame.data[1]]);
                        reply.data[..2].copy_from_slice(&self.position.to_be_bytes());
                    }
                    if self.excess_effort {
                        reply.data[4..6].copy_from_slice(&40000u16.to_be_bytes());
                    }
                }
            }
            kind => panic!("prohibited transmission kind {kind}"),
        }
        if reply.id >> 24 == 2 && self.enabled {
            reply.id |= 2 << 22;
        }
        self.rx.push_back(Received {
            bus,
            frame: reply,
            sampled_at_ms: self.now.get(),
        });
        Ok(())
    }
}
fn fixture() -> (Bus, TestClock, Motor, Identity) {
    let now = Rc::new(Cell::new(0));
    let identity = Identity {
        uid: [1; 8],
        firmware: [0, 0, 3, 32],
    };
    let bus = Bus {
        now: now.clone(),
        rx: VecDeque::new(),
        tx: Vec::new(),
        enabled: false,
        timeout: 0,
        last_command: 0,
        accept_timeout: true,
        obey_timeout: true,
        auto_enable: false,
        uid: identity.uid,
        firmware: identity.firmware,
        fail_enable: false,
        allow_jog: false,
        follow_jog: true,
        excess_effort: false,
        position: 32767,
        breakaway: false,
        receive_error: false,
        post_stop_speed: false,
        damping_fault: None,
    };
    let motor = Motor {
        name: "test_gripper".into(),
        bus: 0,
        id: 10,
        ranges: Ranges {
            position: 4.0 * std::f64::consts::PI,
            velocity: 33.0,
            effort: 14.0,
            kp: 500.0,
            kd: 5.0,
        },
        scale: 1.0,
        zero: 0.0,
    };
    (bus, TestClock(now), motor, identity)
}

#[test]
fn verifies_identity_watchdog_and_autonomous_reset_before_final_stop() {
    let (mut bus, mut clock, motor, identity) = fixture();
    let result = watchdog_trial::run(&mut bus, &mut clock, &motor, 170, &identity);
    assert!(result.success, "{:?}", result.error);
    assert!(
        result.enable_requested
            && result.run_confirmed
            && result.reset_after_silence
            && result.final_stop_confirmed
    );
    assert_eq!(result.silence_ms, 150);
    assert_eq!(result.previous_watchdog_ticks, Some(0));
    assert_eq!(result.written_watchdog_ticks, Some(600));
    assert!(!bus.enabled);
    assert_eq!(
        bus.tx.len(),
        result.events.iter().filter(|e| e.direction == "tx").count()
    );
}

#[test]
fn wrong_identity_or_unwritable_watchdog_never_enables() {
    for failure in 0..3 {
        let (mut bus, mut clock, motor, identity) = fixture();
        match failure {
            0 => bus.uid = [2; 8],
            1 => bus.firmware = [9; 4],
            _ => bus.accept_timeout = false,
        }
        let result = watchdog_trial::run(&mut bus, &mut clock, &motor, 170, &identity);
        assert!(!result.success);
        assert!(!result.enable_requested);
        assert!(bus.tx.iter().all(|f| f.id >> 24 != 3));
        if failure == 0 {
            assert_eq!(bus.tx.len(), 1);
        }
    }
}

#[test]
fn ineffective_watchdog_or_auto_enable_is_rejected_and_stopped() {
    for auto_enable in [false, true] {
        let (mut bus, mut clock, motor, identity) = fixture();
        bus.obey_timeout = false;
        bus.auto_enable = auto_enable;
        let result = watchdog_trial::run(&mut bus, &mut clock, &motor, 170, &identity);
        assert!(!result.success);
        assert!(!result.reset_after_silence);
        assert!(result.final_stop_confirmed);
        assert!(!bus.enabled);
        assert_eq!(result.enable_requested, !auto_enable);
    }
}

#[test]
fn ambiguous_enable_transmit_failure_still_stops() {
    let (mut bus, mut clock, motor, identity) = fixture();
    bus.fail_enable = true;
    let result = watchdog_trial::run(&mut bus, &mut clock, &motor, 170, &identity);
    assert!(!result.success);
    assert!(result.enable_requested && result.final_stop_confirmed);
    assert!(!bus.enabled);
}

#[test]
fn bounded_jog_requires_watchdog_then_returns_to_start_and_stops() {
    let (mut bus, mut clock, motor, identity) = fixture();
    bus.allow_jog = true;
    let result =
        watchdog_trial::run_with_jog(&mut bus, &mut clock, &motor, 170, &identity, Some(0.025));
    assert!(result.success, "{:?}", result.error);
    let jog = result.jog.unwrap();
    assert!(jog.movement_observed);
    assert!(jog.maximum_displacement_rad > 0.024 && jog.maximum_displacement_rad < 0.026);
    assert!((jog.final_position_rad - jog.initial_position_rad).abs() < 0.001);
    assert!(!bus.enabled);
}

#[test]
fn bounded_jog_stops_on_measured_effort_or_failure_to_move() {
    for effort in [true, false] {
        let (mut bus, mut clock, motor, identity) = fixture();
        bus.allow_jog = true;
        bus.excess_effort = effort;
        bus.follow_jog = effort;
        let result =
            watchdog_trial::run_with_jog(&mut bus, &mut clock, &motor, 170, &identity, Some(0.025));
        assert!(!result.success);
        assert!(result.final_stop_confirmed);
        assert!(!bus.enabled);
    }
}

#[test]
fn bounded_jog_refuses_bad_extent_and_failed_watchdog_before_gains() {
    let (mut bus, mut clock, motor, identity) = fixture();
    for value in [f64::NAN, 0.0, 0.1] {
        assert!(
            !watchdog_trial::run_with_jog(
                &mut bus,
                &mut clock,
                &motor,
                170,
                &identity,
                Some(value)
            )
            .success
        );
        assert!(bus.tx.is_empty());
    }
    bus.obey_timeout = false;
    let result =
        watchdog_trial::run_with_jog(&mut bus, &mut clock, &motor, 170, &identity, Some(0.025));
    assert!(!result.success);
    assert!(result.jog.is_none());
}

#[test]
fn physical_adapter_requires_fresh_verified_watchdog_and_consumes_permission() {
    let (bus, mut clock, motor, identity) = fixture();
    let mut hw = CanHardware::new(bus, vec![motor], 170, true).unwrap();
    assert!(hw.enable().is_err());
    assert!(hw.io.tx.is_empty());
    hw.configure_watchdogs(vec![identity]).unwrap();
    hw.configure_reference(vec![robot_control::reference::Reference {
        name: hw.motors[0].name.clone(),
        raw_position_rad: 0.0,
        tolerance_rad: 0.03,
    }])
    .unwrap();
    let mut out = [JointFeedback::default()];
    for _ in 0..80 {
        hw.read(clock.now_ms(), &mut out).unwrap();
        clock.wait_ms(2);
    }
    assert_eq!(hw.watchdog_status(), "ready");
    assert!(!hw.io.enabled);
    assert_eq!(hw.io.timeout, 600);
    assert!(hw.io.tx.iter().all(|f| f.id >> 24 != 3));
    hw.enable().unwrap();
    assert!(hw.io.enabled);
    hw.disable().unwrap();
    assert!(hw.enable().is_err());
    assert!(!hw.io.enabled);
}

#[test]
fn startup_watchdog_rejects_identity_firmware_readback_and_lost_reply() {
    for failure in 0..4 {
        let (mut bus, mut clock, motor, identity) = fixture();
        match failure {
            0 => bus.uid = [9; 8],
            1 => bus.firmware = [9; 4],
            2 => bus.accept_timeout = false,
            _ => {}
        }
        let mut hw = CanHardware::new(bus, vec![motor], 170, true).unwrap();
        hw.configure_watchdogs(vec![identity]).unwrap();
        let mut out = [JointFeedback::default()];
        let mut rejected = false;
        for _ in 0..60 {
            if failure == 3 {
                hw.io.rx.clear();
            }
            if hw.read(clock.now_ms(), &mut out).is_err() {
                rejected = true;
                break;
            }
            clock.wait_ms(2);
        }
        assert!(rejected);
        assert!(hw.enable().is_err());
        assert!(hw.io.tx.iter().all(|f| f.id >> 24 != 3));
    }
}

#[test]
fn rebooted_motor_invalidates_startup_proof_without_rewriting_watchdog() {
    let (bus, mut clock, motor, identity) = fixture();
    let mut hw = CanHardware::new(bus, vec![motor], 170, true).unwrap();
    hw.configure_watchdogs(vec![identity]).unwrap();
    let mut out = [JointFeedback::default()];
    for _ in 0..30 {
        hw.read(clock.now_ms(), &mut out).unwrap();
        clock.wait_ms(2);
    }
    assert_eq!(hw.watchdog_status(), "ready");
    hw.io.timeout = 0;
    let mut rejected = false;
    for _ in 0..30 {
        if hw.read(clock.now_ms(), &mut out).is_err() {
            rejected = true;
            break;
        }
        clock.wait_ms(2);
    }
    assert!(rejected && hw.enable().is_err());
    assert_eq!(hw.io.tx.iter().filter(|f| f.id >> 24 == 18).count(), 1);
}

#[test]
fn receive_failure_or_shutdown_revokes_preflight_and_stops_identified_motor() {
    for receive_failure in [true, false] {
        let (bus, mut clock, motor, identity) = fixture();
        let mut hw = CanHardware::new(bus, vec![motor], 170, true).unwrap();
        hw.configure_watchdogs(vec![identity]).unwrap();
        let mut out = [JointFeedback::default()];
        for _ in 0..30 {
            hw.read(clock.now_ms(), &mut out).unwrap();
            clock.wait_ms(2);
        }
        assert_eq!(hw.watchdog_status(), "ready");
        hw.io.enabled = true; // Unexpected auto-enable must still get a stop attempt.
        if receive_failure {
            hw.io.receive_error = true;
            assert!(hw.read(clock.now_ms(), &mut out).is_err());
        } else {
            hw.disable().unwrap();
        }
        assert!(!hw.io.enabled);
        assert!(hw.enable().is_err());
        assert_eq!(hw.io.tx.last().unwrap().id >> 24, 4);
    }
}

#[test]
fn stale_startup_proof_and_wrong_bus_reply_cannot_authorize_enable() {
    let (bus, mut clock, motor, identity) = fixture();
    let mut hw = CanHardware::new(bus, vec![motor], 170, true).unwrap();
    hw.configure_watchdogs(vec![identity]).unwrap();
    let mut out = [JointFeedback::default()];
    for _ in 0..30 {
        hw.read(clock.now_ms(), &mut out).unwrap();
        clock.wait_ms(2);
    }
    clock.wait_ms(100);
    let _ = hw.read(clock.now_ms(), &mut out);
    assert!(hw.enable().is_err());
    let (bus, mut clock, motor, identity) = fixture();
    let mut hw = CanHardware::new(bus, vec![motor], 170, true).unwrap();
    hw.configure_watchdogs(vec![identity]).unwrap();
    for _ in 0..30 {
        for r in &mut hw.io.rx {
            r.bus = 1;
        }
        if hw.read(clock.now_ms(), &mut out).is_err() {
            break;
        }
        clock.wait_ms(2);
    }
    assert!(hw.enable().is_err());
}

#[test]
fn breakaway_stops_at_first_motion_in_both_directions() {
    for direction in [-1, 1] {
        let (mut bus, mut clock, motor, identity) = fixture();
        bus.breakaway = true;
        bus.allow_jog = true;
        let result = watchdog_trial::run_breakaway(
            &mut bus,
            &mut clock,
            &motor,
            170,
            &identity,
            watchdog_trial::Breakaway {
                direction,
                expected_position_rad: 0.,
                stop: watchdog_trial::BreakawayStop::Immediate,
            },
        );
        assert!(result.success, "{:?}", result.error);
        let b = result.breakaway.unwrap();
        assert!(b.movement_observed && b.maximum_displacement_rad < 0.01);
        assert!(b.command_at_motion_nm.unwrap().abs() >= 0.1);
        assert!(b.maximum_commanded_effort_nm < 0.11);
        assert!(!bus.enabled);
    }
}

#[test]
fn breakaway_caps_force_and_stops_on_no_motion_effort_or_changed_pose() {
    for failure in 0..3 {
        let (mut bus, mut clock, motor, identity) = fixture();
        bus.breakaway = true;
        bus.allow_jog = true;
        match failure {
            0 => bus.follow_jog = false,
            1 => bus.excess_effort = true,
            _ => {}
        }
        let result = watchdog_trial::run_breakaway(
            &mut bus,
            &mut clock,
            &motor,
            170,
            &identity,
            watchdog_trial::Breakaway {
                direction: 1,
                expected_position_rad: if failure == 2 { 1. } else { 0. },
                stop: watchdog_trial::BreakawayStop::Immediate,
            },
        );
        assert!(!result.success);
        assert!(result.final_stop_confirmed && !bus.enabled);
        if failure == 2 {
            assert!(result.breakaway.is_none());
        }
    }
}

#[test]
fn movement_alone_does_not_pass_a_breakaway_with_post_stop_overspeed() {
    let (mut bus, mut clock, motor, identity) = fixture();
    bus.breakaway = true;
    bus.allow_jog = true;
    bus.post_stop_speed = true;
    let result = watchdog_trial::run_breakaway(
        &mut bus,
        &mut clock,
        &motor,
        170,
        &identity,
        watchdog_trial::Breakaway {
            direction: -1,
            expected_position_rad: 0.,
            stop: watchdog_trial::BreakawayStop::Immediate,
        },
    );
    assert!(!result.success);
    assert_eq!(
        result.error.as_deref(),
        Some("breakaway post-stop envelope exceeded")
    );
    assert!(result.breakaway.unwrap().movement_observed);
    assert!(result.final_stop_feedback.unwrap().velocity > 0.2);
    assert!(result.final_stop_confirmed && !bus.enabled);
}

#[test]
fn damped_stop_tapers_drive_then_damping_and_proves_zero_gain_settling() {
    for direction in [-1, 1] {
        let (mut bus, mut clock, motor, identity) = fixture();
        bus.breakaway = true;
        bus.allow_jog = true;
        let result = watchdog_trial::run_breakaway(
            &mut bus,
            &mut clock,
            &motor,
            170,
            &identity,
            watchdog_trial::Breakaway {
                direction,
                expected_position_rad: 0.,
                stop: watchdog_trial::BreakawayStop::Damped,
            },
        );
        assert!(result.success, "{:?}", result.error);
        let b = result.breakaway.unwrap();
        let stop = b.damped_stop.unwrap();
        assert!(b.movement_observed && stop.settled_before_disable);
        assert!(stop.finished_at_ms.unwrap() - stop.started_at_ms >= 650);
        assert_eq!(
            stop.phases.iter().map(|p| p.name).collect::<Vec<_>>(),
            ["release_drive", "damp", "fade_damping", "zero_gain"]
        );
        // Final control traffic must actually be neutral before disabling.
        let commands: Vec<_> = bus.tx.iter().filter(|f| f.id >> 24 == 1).collect();
        for f in commands.iter().rev().take(20) {
            assert_eq!((f.id >> 8) & 65535, 32767);
            assert_eq!(f.data[4..], [0; 4]);
        }
        assert!(result.final_stop_confirmed && !bus.enabled);
        assert!(result.events.len() < 2048);
    }
}

#[test]
fn damping_faults_stop_immediately_without_completing_or_rearming() {
    for fault in [
        "lost_reply",
        "reset",
        "speed",
        "drift",
        "not_settled",
        "gap",
    ] {
        let (mut bus, mut clock, motor, identity) = fixture();
        bus.breakaway = true;
        bus.allow_jog = true;
        bus.damping_fault = Some(fault);
        let result = watchdog_trial::run_breakaway(
            &mut bus,
            &mut clock,
            &motor,
            170,
            &identity,
            watchdog_trial::Breakaway {
                direction: 1,
                expected_position_rad: 0.,
                stop: watchdog_trial::BreakawayStop::Damped,
            },
        );
        assert!(!result.success, "{fault}");
        assert!(
            !result
                .breakaway
                .unwrap()
                .damped_stop
                .unwrap()
                .settled_before_disable,
            "{fault}"
        );
        assert!(result.final_stop_confirmed && !bus.enabled, "{fault}");
        assert_eq!(bus.tx.last().unwrap().id >> 24, 4);
        assert_eq!(bus.tx.iter().filter(|f| f.id >> 24 == 3).count(), 2);
    }
}
