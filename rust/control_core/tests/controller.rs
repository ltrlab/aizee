use robot_control::{
    hardware::Loopback, protocol::*, stop::JointStop, Config, Controller, JointLimits, JointUnit,
};

fn identity() -> Identity {
    Identity {
        model: "fixture-model".into(),
        calibration: "synthetic".into(),
        collision_policy: "fixture-policy".into(),
    }
}
fn config(count: usize) -> Config {
    Config {
        collision: None,
        group: "arbitrary-group".into(),
        identity: identity(),
        lease_ms: 100,
        feedback_timeout_ms: 20,
        interpolation_ms: 0,
        settled_velocity: 0.001,
        joints: (0..count)
            .map(|i| JointLimits {
                name: format!("axis-{i}"),
                unit: JointUnit::Radians,
                lower: -3.0,
                upper: 3.0,
                max_velocity: 2.0,
                max_effort: 5.0,
                max_kp: 20.0,
                max_kd: 5.0,
                stop_acceleration: 2.0,
                hold_kp: 10.0,
                hold_kd: 1.0,
                max_tracking_error: 0.5,
                max_temperature: 70.0,
                gripper: false,
            })
            .collect(),
    }
}
fn fixture(count: usize) -> (Controller, Loopback) {
    (
        Controller::new(config(count), "boot-a".into()).unwrap(),
        Loopback::new(&vec![0.0; count]),
    )
}
fn offer(c: &mut Controller, h: &mut Loopback, now: u64) -> u64 {
    c.handle(now, Request::Offer, h).offer.unwrap().id
}
fn claim(c: &mut Controller, h: &mut Loopback, now: u64) -> Reply {
    let id = offer(c, h, now);
    c.handle(
        now,
        Request::Acquire {
            epoch: "boot-a".into(),
            offer: id,
            session: "client-a".into(),
            group: "arbitrary-group".into(),
            identity: identity(),
        },
        h,
    )
}
fn auth(c: &Controller, sequence: u64) -> Authority {
    let status = c.status();
    let lease = status.lease.unwrap();
    Authority {
        epoch: status.epoch,
        session: lease.session,
        lease: lease.id,
        sequence,
    }
}
fn armed(count: usize) -> (Controller, Loopback) {
    let (mut c, mut h) = fixture(count);
    assert!(claim(&mut c, &mut h, 0).accepted);
    assert!(
        c.handle(
            1,
            Request::Arm {
                collision: None,
                authority: auth(&c, 1)
            },
            &mut h
        )
        .accepted
    );
    (c, h)
}
fn joints(count: usize, q: f64, v: f64) -> Vec<JointCommand> {
    vec![
        JointCommand {
            position: q,
            velocity: v,
            effort: 0.0,
            kp: 10.0,
            kd: 1.0
        };
        count
    ]
}
fn move_request(c: &Controller, sequence: u64, count: usize, q: f64, v: f64) -> Request {
    Request::Move {
        collision: None,
        authority: auth(c, sequence),
        identity: identity(),
        joints: joints(count, q, v),
    }
}

fn collision_fixture() -> (Controller, Loopback) {
    let mut cfg = config(2);
    cfg.collision = Some(robot_control::collision::CollisionPolicy {
        world_revision: "fixed-world".into(),
        scope: "inter_branch".into(),
        max_verdict_ms: 40,
        reaction_ms: 10,
    });
    let mut c = Controller::new(cfg, "boot-a".into()).unwrap();
    let mut h = Loopback::new(&[0.0, 0.0]);
    assert!(claim(&mut c, &mut h, 0).accepted);
    (c, h)
}

fn envelope(
    c: &mut Controller,
    h: &mut Loopback,
    now: u64,
) -> robot_control::collision::MotionEnvelope {
    robot_control::collision::MotionEnvelope {
        offer: offer(c, h, now),
        world_revision: "fixed-world".into(),
        scope: "inter_branch".into(),
        lower: vec![-2.5; 2],
        upper: vec![2.5; 2],
    }
}

#[test]
fn collision_policy_requires_a_verdict_before_arm_or_move() {
    let (mut c, mut h) = collision_fixture();
    assert_eq!(
        c.handle(
            1,
            Request::Arm {
                authority: auth(&c, 1),
                collision: None
            },
            &mut h
        )
        .reason,
        Reason::CollisionRequired
    );
    assert!(!h.enabled);
    let checked = envelope(&mut c, &mut h, 2);
    assert!(
        c.handle(
            2,
            Request::Arm {
                authority: auth(&c, 1),
                collision: Some(checked)
            },
            &mut h
        )
        .accepted
    );
    let writes = h.writes;
    assert_eq!(
        c.handle_deferred(3, move_request(&c, 2, 2, 0.1, 0.), &mut h)
            .reason,
        Reason::CollisionRequired
    );
    assert_eq!(h.writes, writes);
    assert_eq!(c.status().joints[0].position, 0.);
}

#[test]
fn collision_expiry_survives_lease_renewal_and_stops_without_client_messages() {
    let (mut c, mut h) = collision_fixture();
    let checked = envelope(&mut c, &mut h, 1);
    assert!(
        c.handle(
            1,
            Request::Arm {
                authority: auth(&c, 1),
                collision: Some(checked)
            },
            &mut h
        )
        .accepted
    );
    let checked = envelope(&mut c, &mut h, 2);
    assert!(
        c.handle(
            2,
            Request::Move {
                authority: auth(&c, 2),
                identity: identity(),
                joints: joints(2, 0.1, 0.0),
                collision: Some(checked)
            },
            &mut h
        )
        .accepted
    );
    let renewal = offer(&mut c, &mut h, 20);
    assert!(
        c.handle(
            20,
            Request::Renew {
                authority: auth(&c, 3),
                offer: renewal
            },
            &mut h
        )
        .accepted
    );
    assert_eq!(c.status().collision_expires_at_ms, Some(42));
    // Position-only setpoints still require braking from measured velocity.
    for sample in &mut h.feedback {
        sample.velocity = 0.2;
    }
    c.tick(42, &mut h);
    assert_eq!(c.status().reason, Reason::CollisionExpired);
    assert_eq!(c.status().state, State::Stopping);
    assert!(c.status().lease.is_none());
    for now in 43..160 {
        c.tick(now, &mut h);
    }
    assert_eq!(c.status().state, State::Stopped);
    assert!((c.status().joints[0].position - 0.11).abs() < 1e-10);
    h.feedback[0].position = 2.6;
    c.tick(160, &mut h);
    assert_eq!(c.status().reason, Reason::CollisionViolation);
    assert!(!h.enabled);
}

#[test]
fn collision_offer_cannot_be_delayed_reused_or_given_a_new_lifetime() {
    let (mut c, mut h) = collision_fixture();
    let late = envelope(&mut c, &mut h, 0);
    assert_eq!(
        c.handle(
            40,
            Request::Arm {
                authority: auth(&c, 1),
                collision: Some(late)
            },
            &mut h
        )
        .reason,
        Reason::CollisionExpired
    );
    let checked = envelope(&mut c, &mut h, 41);
    assert!(
        c.handle(
            41,
            Request::Arm {
                authority: auth(&c, 1),
                collision: Some(checked.clone())
            },
            &mut h
        )
        .accepted
    );
    assert_eq!(
        c.handle_deferred(
            42,
            Request::Move {
                authority: auth(&c, 2),
                identity: identity(),
                joints: joints(2, 0.1, 0.),
                collision: Some(checked)
            },
            &mut h
        )
        .reason,
        Reason::InvalidOffer
    );
    assert_eq!(c.status().collision_expires_at_ms, Some(81));
}

#[test]
fn collision_envelopes_validate_scope_world_dimensions_finiteness_and_stopping_reserve() {
    for failure in 0..7 {
        let (mut c, mut h) = collision_fixture();
        let mut checked = envelope(&mut c, &mut h, 1);
        match failure {
            0 => checked.world_revision = "other-world".into(),
            1 => checked.scope = "all".into(),
            2 => {
                checked.upper.pop();
            }
            3 => checked.lower[0] = f64::NAN,
            4 => checked.lower[0] = -0.1, // contains current pose but lacks stopping reserve
            5 => checked.upper[1] = 0.1,
            _ => checked.upper[0] = 4., // outside operational limits
        }
        let result = c.handle(
            1,
            Request::Arm {
                authority: auth(&c, 1),
                collision: Some(checked),
            },
            &mut h,
        );
        assert!(!result.accepted);
        assert!(!h.enabled);
        assert_eq!(h.writes, 0);
    }
}

#[test]
fn deferred_requests_do_not_accelerate_periodic_writes_or_defer_emergency_stop() {
    let (mut c, mut h) = armed(1);
    let before = h.writes;
    let request = move_request(&c, 2, 1, 0.1, 0.0);
    assert!(c.handle_deferred(2, request, &mut h).accepted);
    for now in 3..10 {
        assert!(c.handle_deferred(now, Request::Status, &mut h).accepted);
    }
    assert_eq!(h.writes, before);
    assert_eq!(h.feedback[0].position, 0.0);
    c.tick(10, &mut h);
    assert_eq!(h.writes, before + 1);
    assert_eq!(h.feedback[0].position, 0.1);
    c.handle_deferred(11, Request::EmergencyStop, &mut h);
    assert!(!h.enabled);
    assert_eq!(h.writes, before + 1);
}

#[test]
fn acquire_never_enables_and_owner_is_exclusive() {
    let (mut c, mut h) = fixture(3);
    assert!(claim(&mut c, &mut h, 0).accepted);
    assert!(!h.enabled);
    assert_eq!(claim(&mut c, &mut h, 1).reason, Reason::Busy);
    assert_eq!(c.status().state, State::Disarmed);
}

#[test]
fn delayed_offer_and_acquire_cannot_start_new_window() {
    let (mut c, mut h) = fixture(1);
    let id = offer(&mut c, &mut h, 0);
    let request = Request::Acquire {
        epoch: "boot-a".into(),
        offer: id,
        session: "client".into(),
        group: "arbitrary-group".into(),
        identity: identity(),
    };
    assert_eq!(c.handle(100, request, &mut h).reason, Reason::OfferExpired);
    assert!(!h.enabled);
    assert!(c.status().lease.is_none());
}

#[test]
fn permit_expiry_is_issuance_time_not_arrival_time() {
    let (mut c, mut h) = fixture(1);
    let id = offer(&mut c, &mut h, 0);
    let r = c.handle(
        80,
        Request::Acquire {
            epoch: "boot-a".into(),
            offer: id,
            session: "client-a".into(),
            group: "arbitrary-group".into(),
            identity: identity(),
        },
        &mut h,
    );
    assert!(r.accepted);
    assert_eq!(r.status.lease.unwrap().expires_at_ms, 100);
    c.handle(
        81,
        Request::Arm {
            collision: None,
            authority: auth(&c, 1),
        },
        &mut h,
    );
    let queued = move_request(&c, 2, 1, 0.1, 0.0);
    assert!(!c.handle(100, queued, &mut h).accepted);
    assert_eq!(c.status().reason, Reason::LeaseExpired);
}

#[test]
fn rejects_duplicates_reordering_wrong_owner_and_previous_boot() {
    let (mut c, mut h) = armed(2);
    let request = move_request(&c, 4, 2, 0.1, 0.0);
    assert!(c.handle(2, request.clone(), &mut h).accepted);
    assert_eq!(c.handle(3, request, &mut h).reason, Reason::Sequence);
    let old = move_request(&c, 3, 2, 0.2, 0.0);
    assert_eq!(c.handle(4, old, &mut h).reason, Reason::Sequence);
    let mut wrong = auth(&c, 5);
    wrong.session = "client-b".into();
    assert_eq!(
        c.handle(
            5,
            Request::Arm {
                collision: None,
                authority: wrong.clone()
            },
            &mut h
        )
        .reason,
        Reason::WrongOwner
    );
    wrong.epoch = "previous-boot".into();
    assert_eq!(
        c.handle(
            6,
            Request::Arm {
                authority: wrong,
                collision: None
            },
            &mut h
        )
        .reason,
        Reason::WrongEpoch
    );
    assert_eq!(c.status().joints[0].position, 0.1);
}

#[test]
fn entire_batch_and_identity_are_validated_before_mutation() {
    let (mut c, mut h) = armed(2);
    let before = c.status().joints;
    for bad in [f64::NAN, f64::INFINITY, 4.0] {
        let mut values = joints(2, 0.1, 0.0);
        values[1].position = bad;
        let r = c.handle(
            2,
            Request::Move {
                collision: None,
                authority: auth(&c, 2),
                identity: identity(),
                joints: values,
            },
            &mut h,
        );
        assert_eq!(r.reason, Reason::InvalidCommand);
        assert_eq!(c.status().joints, before);
        assert_eq!(h.feedback[0].position, 0.0);
    }
    let bad_length = move_request(&c, 2, 1, 0.1, 0.0);
    assert_eq!(
        c.handle(2, bad_length, &mut h).reason,
        Reason::InvalidCommand
    );
    let mut changed = identity();
    changed.calibration = "other".into();
    let r = c.handle(
        2,
        Request::Move {
            collision: None,
            authority: auth(&c, 2),
            identity: changed,
            joints: joints(2, 0.1, 0.0),
        },
        &mut h,
    );
    assert_eq!(r.reason, Reason::WrongIdentity);
    assert_eq!(c.status().lease.unwrap().last_sequence, 1);
}

#[test]
fn rejected_motion_does_not_renew_deadline() {
    let (mut c, mut h) = armed(1);
    let r = move_request(&c, 2, 1, 10.0, 0.0);
    assert!(!c.handle(90, r, &mut h).accepted);
    c.tick(100, &mut h);
    assert_eq!(c.status().reason, Reason::LeaseExpired);
    assert!(c.status().lease.is_none());
}

#[test]
fn renewal_does_not_extend_old_motion_and_invalidates_old_permit() {
    let (mut c, mut h) = armed(1);
    let old_command = move_request(&c, 3, 1, 0.1, 0.0);
    let id = offer(&mut c, &mut h, 50);
    let r = c.handle(
        60,
        Request::Renew {
            authority: auth(&c, 2),
            offer: id,
        },
        &mut h,
    );
    assert!(r.accepted);
    assert_eq!(r.status.lease.unwrap().expires_at_ms, 150);
    assert_eq!(r.status.command_expires_at_ms, Some(100));
    assert_eq!(c.handle(70, old_command, &mut h).reason, Reason::WrongLease);
    c.tick(100, &mut h);
    assert_eq!(c.status().reason, Reason::CommandExpired);
    assert!(c.status().lease.is_none());
}

#[test]
fn new_motion_after_renewal_gets_only_new_offer_deadline() {
    let (mut c, mut h) = armed(1);
    let id = offer(&mut c, &mut h, 50);
    c.handle(
        55,
        Request::Renew {
            authority: auth(&c, 2),
            offer: id,
        },
        &mut h,
    );
    let r = move_request(&c, 3, 1, 0.1, 0.0);
    assert!(c.handle(60, r, &mut h).accepted);
    c.tick(110, &mut h);
    assert_eq!(c.status().state, State::Executing);
    c.tick(150, &mut h);
    assert_eq!(c.status().reason, Reason::LeaseExpired);
}

#[test]
fn expiry_stops_locally_then_requires_reset_and_arm() {
    let (mut c, mut h) = armed(2);
    let r = move_request(&c, 2, 2, 0.1, 0.2);
    assert!(c.handle(2, r, &mut h).accepted);
    c.tick(100, &mut h);
    assert_eq!(c.status().state, State::Stopping);
    c.tick(150, &mut h);
    assert!((c.status().joints[0].velocity - 0.1).abs() < 1e-10);
    c.tick(200, &mut h);
    c.tick(201, &mut h);
    assert_eq!(c.status().state, State::Stopped);
    assert!((c.status().joints[0].position - 0.11).abs() < 1e-10);
    assert!(claim(&mut c, &mut h, 202).accepted);
    assert_eq!(
        c.handle(
            203,
            Request::Arm {
                collision: None,
                authority: auth(&c, 1)
            },
            &mut h
        )
        .reason,
        Reason::InvalidState
    );
    assert!(
        c.handle(
            204,
            Request::Reset {
                authority: auth(&c, 1)
            },
            &mut h
        )
        .accepted
    );
    assert!(!h.enabled);
    assert_eq!(c.status().state, State::Disarmed);
    assert!(
        c.handle(
            205,
            Request::Arm {
                collision: None,
                authority: auth(&c, 2)
            },
            &mut h
        )
        .accepted
    );
}

#[test]
fn emergency_stop_bypasses_ownership_and_never_auto_resumes() {
    let (mut c, mut h) = armed(1);
    let queued = move_request(&c, 2, 1, 0.1, 0.0);
    assert!(c.handle(2, Request::EmergencyStop, &mut h).accepted);
    assert!(!h.enabled);
    assert_eq!(c.status().state, State::Faulted);
    assert!(!c.handle(3, queued, &mut h).accepted);
    c.tick(200, &mut h);
    assert_eq!(c.status().state, State::Faulted);
    assert!(!h.enabled);
}

#[test]
fn frozen_feedback_and_hardware_failures_revoke_authority() {
    for kind in 0..4 {
        let (mut c, mut h) = armed(1);
        match kind {
            0 => h.frozen = true,
            1 => h.fail_read = true,
            2 => h.fail_write = true,
            _ => h.motor_fault = true,
        }
        c.tick(21, &mut h);
        assert_eq!(c.status().state, State::Faulted, "fault kind {kind}");
        assert_eq!(
            c.status().reason,
            if kind == 0 {
                Reason::FeedbackStale
            } else {
                Reason::HardwareFault
            }
        );
        assert!(c.status().lease.is_none());
        assert!(!h.enabled);
    }
}

#[test]
fn one_overspeed_reading_is_drive_noise_but_a_run_of_them_faults() {
    let (mut c, mut h) = armed(1);
    h.feedback[0].velocity = 2.5; // Over the 2.0 rad/s cap once...
    c.tick(2, &mut h);
    h.feedback[0].velocity = 0.0;
    c.tick(3, &mut h);
    assert_eq!(c.status().state, State::Holding);
    for now in 4..7 {
        h.feedback[0].velocity = 2.5; // ...then three samples in a row.
        c.tick(now, &mut h);
    }
    assert_eq!(c.status().reason, Reason::LimitViolation);
    assert!(!h.enabled);
}

#[test]
fn a_reading_far_over_the_speed_cap_faults_at_once() {
    let (mut c, mut h) = armed(1);
    h.feedback[0].velocity = 6.5; // Over three times the 2.0 rad/s cap.
    c.tick(2, &mut h);
    assert_eq!(c.status().reason, Reason::LimitViolation);
    assert!(!h.enabled);
}

#[test]
fn braking_starts_from_at_most_the_speed_cap() {
    // Braking from a lone 2.5 rad/s spike would carry the joint 1.56 rad on;
    // from the 2.0 rad/s cap it is 1.0 rad at 2 rad/s^2.
    let (mut c, mut h) = armed(1);
    h.feedback[0].velocity = 2.5;
    let reply = c.handle(
        2,
        Request::Stop {
            authority: auth(&c, 2),
        },
        &mut h,
    );
    assert!(reply.accepted, "{:?}", reply.reason);
    assert_eq!(c.status().state, State::Stopping);
    for now in 3..1500 {
        c.tick(now, &mut h);
    }
    assert_eq!(c.status().state, State::Stopped);
    assert!((c.status().joints[0].position - 1.0).abs() < 1e-9);
}

#[test]
fn clock_regression_and_unsafe_stopping_envelope_latch_faults() {
    let (mut c, mut h) = armed(1);
    c.tick(0, &mut h);
    assert_eq!(c.status().reason, Reason::ClockRegression);
    let (mut c, mut h) = armed(1);
    let r = move_request(&c, 2, 1, 2.99, 1.0);
    c.handle(2, r, &mut h);
    c.tick(100, &mut h);
    assert_eq!(c.status().reason, Reason::LimitViolation);
    assert!(!h.enabled);
}

#[test]
fn config_rejects_duplicate_names_nonfinite_and_invalid_gains() {
    let mut c = config(2);
    c.joints[1].name = c.joints[0].name.clone();
    assert!(c.validate().is_err());
    c = config(1);
    c.joints[0].stop_acceleration = f64::NAN;
    assert!(c.validate().is_err());
    c = config(1);
    c.joints[0].hold_kp = 21.0;
    assert!(c.validate().is_err());
}

#[test]
fn stopping_equations_cover_both_directions_and_zero() {
    for velocity in [-2.0, 0.0, 2.0] {
        let stop = JointStop::new(0.5, velocity, 4.0).unwrap();
        let end = stop.sample(100.0, 1.0, 1.0);
        assert_eq!(end.velocity, 0.0);
        assert_eq!(end.position, 0.5 + velocity * velocity.abs() / 8.0);
        assert_eq!(stop.sample(0.0, 1.0, 1.0).velocity, velocity);
    }
    assert!(JointStop::new(0.0, 1.0, 0.0).is_none());
}

fn idle_request(c: &Controller, sequence: u64) -> Request {
    Request::Idle {
        authority: auth(c, sequence),
    }
}

fn disable_request(c: &Controller, sequence: u64) -> Request {
    Request::Disable {
        authority: auth(c, sequence),
    }
}

#[test]
fn idle_is_zero_gain_backdrive_that_follows_the_measured_pose() {
    let (mut c, mut h) = fixture(2);
    assert!(claim(&mut c, &mut h, 0).accepted);
    h.feedback[0].position = 0.4;
    let reply = c.handle(1, idle_request(&c, 1), &mut h);
    assert!(reply.accepted);
    assert_eq!(reply.status.state, State::Idle);
    assert!(h.enabled);
    assert_eq!(h.enables, 1);
    assert_eq!(reply.status.joints[0].position, 0.4);
    assert!(reply
        .status
        .joints
        .iter()
        .all(|j| j.kp == 0.0 && j.kd == 0.0 && j.velocity == 0.0 && j.effort == 0.0));
    // Backdriving far beyond the tracking bound is not a fault with zero gains.
    h.feedback[1].position = -1.2;
    c.tick(2, &mut h);
    let status = c.status();
    assert_eq!(status.state, State::Idle);
    assert_eq!(status.joints[1].position, -1.2);
    assert_eq!(status.joints[1].kp, 0.0);
}

#[test]
fn enable_from_idle_holds_measured_pose_without_a_second_enable() {
    let (mut c, mut h) = fixture(1);
    assert!(claim(&mut c, &mut h, 0).accepted);
    assert!(c.handle(1, idle_request(&c, 1), &mut h).accepted);
    h.feedback[0].position = 0.25;
    c.tick(2, &mut h);
    let reply = c.handle(
        3,
        Request::Arm {
            collision: None,
            authority: auth(&c, 2),
        },
        &mut h,
    );
    assert!(reply.accepted);
    assert_eq!(reply.status.state, State::Holding);
    assert_eq!(h.enables, 1);
    assert_eq!(reply.status.joints[0].position, 0.25);
    assert_eq!(reply.status.joints[0].kp, 10.0);
}

#[test]
fn enable_from_idle_is_refused_while_backdriven() {
    let (mut c, mut h) = fixture(1);
    assert!(claim(&mut c, &mut h, 0).accepted);
    assert!(c.handle(1, idle_request(&c, 1), &mut h).accepted);
    // Backdriven: every new sample reads fast (Idle's zero-gain apply resets the loopback).
    for now in 2..5 {
        h.feedback[0].velocity = 0.5;
        c.tick(now, &mut h);
    }
    h.feedback[0].velocity = 0.5;
    let reply = c.handle(
        5,
        Request::Arm {
            collision: None,
            authority: auth(&c, 2),
        },
        &mut h,
    );
    assert_eq!(reply.reason, Reason::InvalidState);
    assert_eq!(c.status().state, State::Idle);
    assert!(h.enabled);
}

#[test]
fn one_fast_reading_while_idle_is_drive_noise_not_a_backdriven_arm() {
    let (mut c, mut h) = fixture(1);
    assert!(claim(&mut c, &mut h, 0).accepted);
    assert!(c.handle(1, idle_request(&c, 1), &mut h).accepted);
    h.feedback[0].velocity = 0.5;
    c.tick(2, &mut h);
    h.feedback[0].velocity = 0.0;
    c.tick(3, &mut h);
    h.feedback[0].velocity = 0.5;
    c.tick(4, &mut h);
    let reply = c.handle(
        5,
        Request::Arm {
            collision: None,
            authority: auth(&c, 2),
        },
        &mut h,
    );
    assert!(reply.accepted, "{:?}", reply.reason);
    assert_eq!(c.status().state, State::Holding);
}

#[test]
fn move_requires_enable_after_idle() {
    let (mut c, mut h) = fixture(1);
    assert!(claim(&mut c, &mut h, 0).accepted);
    assert!(c.handle(1, idle_request(&c, 1), &mut h).accepted);
    let reply = c.handle(2, move_request(&c, 2, 1, 0.1, 0.0), &mut h);
    assert_eq!(reply.reason, Reason::InvalidState);
    assert_eq!(c.status().state, State::Idle);
}

#[test]
fn idle_owner_loss_disables_instead_of_holding() {
    let (mut c, mut h) = fixture(1);
    assert!(claim(&mut c, &mut h, 0).accepted);
    assert!(c.handle(1, idle_request(&c, 1), &mut h).accepted);
    c.tick(99, &mut h);
    assert_eq!(c.status().state, State::Idle);
    c.tick(100, &mut h);
    let status = c.status();
    assert_eq!(status.state, State::Disarmed);
    assert_eq!(status.reason, Reason::LeaseExpired);
    assert!(status.lease.is_none());
    assert!(!h.enabled);
    let writes = h.writes;
    c.tick(150, &mut h);
    assert_eq!(h.writes, writes);
}

#[test]
fn idle_must_be_refreshed_like_any_command() {
    let (mut c, mut h) = fixture(1);
    assert!(claim(&mut c, &mut h, 0).accepted);
    assert!(c.handle(1, idle_request(&c, 1), &mut h).accepted);
    let next = offer(&mut c, &mut h, 50);
    assert!(
        c.handle(
            50,
            Request::Renew {
                authority: auth(&c, 2),
                offer: next
            },
            &mut h
        )
        .accepted
    );
    c.tick(100, &mut h);
    assert_eq!(c.status().state, State::Disarmed);
    assert_eq!(c.status().reason, Reason::CommandExpired);
    assert!(!h.enabled);
}

#[test]
fn deferred_idle_refresh_cannot_accelerate_periodic_writes() {
    let (mut c, mut h) = fixture(1);
    assert!(claim(&mut c, &mut h, 0).accepted);
    assert!(c.handle_deferred(1, idle_request(&c, 1), &mut h).accepted);
    let writes = h.writes;
    assert_eq!(writes, 1);
    for sequence in 2..6 {
        assert!(
            c.handle_deferred(2, idle_request(&c, sequence), &mut h)
                .accepted
        );
    }
    assert_eq!(h.writes, writes);
    c.tick(3, &mut h);
    assert_eq!(h.writes, writes + 1);
}

#[test]
fn idle_from_hold_drops_gains_now_and_clears_collision_bounds() {
    let (mut c, mut h) = collision_fixture();
    let checked = envelope(&mut c, &mut h, 1);
    assert!(
        c.handle(
            1,
            Request::Arm {
                authority: auth(&c, 1),
                collision: Some(checked)
            },
            &mut h
        )
        .accepted
    );
    let writes = h.writes;
    let reply = c.handle_deferred(2, idle_request(&c, 2), &mut h);
    assert!(reply.accepted);
    assert_eq!(reply.status.state, State::Idle);
    assert!(reply.status.collision_expires_at_ms.is_none());
    assert!(reply
        .status
        .joints
        .iter()
        .all(|j| j.kp == 0.0 && j.kd == 0.0));
    assert_eq!(h.writes, writes + 1);
    assert_eq!(h.enables, 1);
    // Hand motion outside the former collision box is not a collision fault.
    h.feedback[0].position = 2.8;
    c.tick(3, &mut h);
    assert_eq!(c.status().state, State::Idle);
}

#[test]
fn disable_removes_torque_immediately_and_releases_ownership() {
    for moving in [false, true] {
        let (mut c, mut h) = armed(2);
        let mut sequence = 2;
        if moving {
            assert!(
                c.handle(2, move_request(&c, sequence, 2, 0.2, 0.0), &mut h)
                    .accepted
            );
            sequence += 1;
            assert_eq!(c.status().state, State::Executing);
        }
        let reply = c.handle(3, disable_request(&c, sequence), &mut h);
        assert!(reply.accepted);
        assert_eq!(reply.status.state, State::Disarmed);
        assert!(reply.status.lease.is_none());
        assert!(reply.status.command_expires_at_ms.is_none());
        assert!(reply
            .status
            .joints
            .iter()
            .all(|j| j.kp == 0.0 && j.kd == 0.0));
        assert!(!h.enabled);
        let writes = h.writes;
        c.tick(4, &mut h);
        assert_eq!(h.writes, writes);
        // A new owner must explicitly start again; nothing re-arms by itself.
        assert!(claim(&mut c, &mut h, 5).accepted);
        assert!(!h.enabled);
        assert_eq!(c.status().state, State::Disarmed);
    }
}

#[test]
fn idle_and_disable_never_clear_a_fault() {
    let (mut c, mut h) = armed(1);
    assert!(c.handle(2, Request::EmergencyStop, &mut h).accepted);
    assert!(claim(&mut c, &mut h, 3).accepted);
    assert_eq!(
        c.handle(4, idle_request(&c, 1), &mut h).reason,
        Reason::InvalidState
    );
    assert!(!h.enabled);
    let reply = c.handle(5, disable_request(&c, 2), &mut h);
    assert!(reply.accepted);
    assert_eq!(reply.status.state, State::Faulted);
    assert_eq!(reply.status.reason, Reason::EmergencyStop);
    assert!(reply.status.lease.is_none());
    assert!(!h.enabled);
}

#[test]
fn idle_and_disable_require_the_current_owner() {
    let (mut c, mut h) = fixture(1);
    assert!(claim(&mut c, &mut h, 0).accepted);
    let mut stale = auth(&c, 1);
    stale.session = "someone-else".into();
    let reply = c.handle(
        1,
        Request::Idle {
            authority: stale.clone(),
        },
        &mut h,
    );
    assert_eq!(reply.reason, Reason::WrongOwner);
    assert!(!h.enabled);
    assert_eq!(
        c.handle(2, Request::Disable { authority: stale }, &mut h)
            .reason,
        Reason::WrongOwner
    );
    assert!(c.status().lease.is_some());
}

fn with_gripper(mut cfg: Config) -> Config {
    let last = cfg.joints.len() - 1;
    cfg.joints[last].gripper = true;
    cfg
}

#[test]
fn gripper_config_field_defaults_to_an_ordinary_joint() {
    let text = r#"{"name":"a","unit":"radians","lower":-1,"upper":1,"max_velocity":1,"max_effort":1,
        "max_kp":1,"max_kd":1,"stop_acceleration":1,"hold_kp":1,"hold_kd":1,"max_tracking_error":1,
        "max_temperature":70}"#;
    let joint: JointLimits = serde_json::from_str(text).unwrap();
    assert!(!joint.gripper);
}

#[test]
fn a_gripper_needs_no_stopping_reserve_at_its_ends() {
    let mut cfg = with_gripper(config(2));
    cfg.collision = Some(robot_control::collision::CollisionPolicy {
        world_revision: "fixed-world".into(),
        scope: "inter_branch".into(),
        max_verdict_ms: 40,
        reaction_ms: 10,
    });
    for (gripper_low, accepted) in [(0.0, true), (0.01, false)] {
        let mut c = Controller::new(cfg.clone(), "boot-a".into()).unwrap();
        let mut h = Loopback::new(&[0.0, 0.0]);
        assert!(claim(&mut c, &mut h, 0).accepted);
        let mut checked = envelope(&mut c, &mut h, 1);
        // The gripper's box may start exactly at its position; it must still contain it.
        checked.lower[1] = gripper_low;
        let reply = c.handle(
            1,
            Request::Arm {
                authority: auth(&c, 1),
                collision: Some(checked),
            },
            &mut h,
        );
        assert_eq!(reply.accepted, accepted);
    }
}

#[test]
fn a_stopped_gripper_keeps_its_last_target_while_the_arm_brakes() {
    let mut c = Controller::new(with_gripper(config(2)), "boot-a".into()).unwrap();
    let mut h = Loopback::new(&[0.0, 0.0]);
    assert!(claim(&mut c, &mut h, 0).accepted);
    assert!(
        c.handle(
            1,
            Request::Arm {
                collision: None,
                authority: auth(&c, 1)
            },
            &mut h
        )
        .accepted
    );
    let r = move_request(&c, 2, 2, 0.1, 0.2);
    assert!(c.handle(2, r, &mut h).accepted);
    c.tick(100, &mut h);
    assert_eq!(c.status().state, State::Stopping);
    c.tick(150, &mut h);
    let joints = c.status().joints;
    assert!((joints[0].velocity - 0.1).abs() < 1e-10); // Braking at 2 rad/s^2.
    assert_eq!((joints[1].position, joints[1].velocity), (0.1, 0.0)); // Holding its target.
    c.tick(200, &mut h);
    c.tick(201, &mut h);
    assert_eq!(c.status().state, State::Stopped);
    assert_eq!(c.status().joints[1].position, 0.1);
}

#[test]
fn a_gripper_stalled_on_an_object_does_not_fault_but_an_arm_joint_does() {
    for gripper_stalled in [true, false] {
        let mut c = Controller::new(with_gripper(config(2)), "boot-a".into()).unwrap();
        let mut h = Loopback::new(&[0.0, 0.0]);
        assert!(claim(&mut c, &mut h, 0).accepted);
        assert!(
            c.handle(
                1,
                Request::Arm {
                    collision: None,
                    authority: auth(&c, 1)
                },
                &mut h
            )
            .accepted
        );
        let r = move_request(&c, 2, 2, 1.0, 0.0);
        assert!(c.handle(2, r, &mut h).accepted);
        // The jaws (or an arm joint) stop short of the target, 1 rad behind.
        let stalled = if gripper_stalled { 1 } else { 0 };
        h.feedback[stalled].position = 0.0;
        c.tick(3, &mut h);
        let status = c.status();
        if gripper_stalled {
            assert_eq!(status.state, State::Executing);
        } else {
            assert_eq!(
                (status.state, status.reason),
                (State::Faulted, Reason::TrackingError)
            );
        }
    }
}

// Braking curves: in the fixture's collision policy a joint (limits +-3, cap 2 rad/s,
// 2 rad/s^2) may approach a limit only as fast as it can still stop at it, reacting within
// the feedback timeout and reaction allowance (30 ms), down to 0.5 rad/s, whose stopping
// margin (0.0775 rad) is all it keeps at each end.

fn arm_at(position: f64, upper: f64) -> (Controller, Loopback, Reply) {
    let (mut c, mut h) = collision_fixture();
    h.feedback[0].position = position;
    c.tick(1, &mut h);
    let mut checked = envelope(&mut c, &mut h, 1);
    checked.upper[0] = upper;
    let reply = c.handle(
        1,
        Request::Arm {
            authority: auth(&c, 1),
            collision: Some(checked),
        },
        &mut h,
    );
    (c, h, reply)
}

#[test]
fn limit_speed_is_the_cap_in_open_travel_and_stops_the_joint_at_the_limit() {
    use robot_control::collision::{limit_reserve, limit_speed, APPROACH_FLOOR_RAD_S};
    let mut cfg = config(1);
    let policy = robot_control::collision::CollisionPolicy {
        world_revision: "fixed-world".into(),
        scope: "inter_branch".into(),
        max_verdict_ms: 40,
        reaction_ms: 10,
    };
    cfg.collision = Some(policy.clone());
    let joint = &cfg.joints[0];
    assert_eq!(limit_speed(&policy, &cfg, joint, 3.0), 2.0);
    // One radian from the limit: under the cap, and it stops exactly at the limit.
    let v = limit_speed(&policy, &cfg, joint, 1.0);
    assert!(v > APPROACH_FLOOR_RAD_S && v < 2.0, "{v}");
    assert!((limit_reserve(&policy, &cfg, joint, 1.0) - 1.0).abs() < 1e-9);
    // Within the floor's stopping margin the reserve is that margin: more than the distance.
    assert_eq!(
        limit_speed(&policy, &cfg, joint, 0.05),
        APPROACH_FLOOR_RAD_S
    );
    assert!((limit_reserve(&policy, &cfg, joint, 0.05) - 0.0775).abs() < 1e-9);
    let mut gripper = joint.clone();
    gripper.gripper = true;
    assert_eq!(limit_speed(&policy, &cfg, &gripper, 0.01), 2.0);
    assert_eq!(limit_reserve(&policy, &cfg, &gripper, 0.01), 0.0);
}

#[test]
fn a_joint_near_its_limit_arms_with_a_box_reaching_the_limit() {
    let (c, h, reply) = arm_at(2.0, 3.0);
    assert!(reply.accepted, "{:?}", reply.reason);
    assert_eq!(c.status().state, State::Holding);
    assert!(h.enabled);
    // A box stopping short of the limit lacks the reserve.
    let (_, h, reply) = arm_at(2.0, 2.9);
    assert!(!reply.accepted);
    assert_eq!(reply.reason, Reason::CollisionViolation);
    assert!(!h.enabled);
}

#[test]
fn a_joint_uses_its_travel_up_to_the_floors_stopping_margin() {
    let (_, h, reply) = arm_at(2.95, 3.0); // 0.05 rad from the limit; the margin is 0.0775.
    assert!(!reply.accepted);
    assert_eq!(reply.reason, Reason::CollisionViolation);
    assert!(!h.enabled);
    let (_, _, reply) = arm_at(2.9, 3.0); // 0.1 rad: on the curve, which stops it at the limit.
    assert!(reply.accepted, "{:?}", reply.reason);
}

#[test]
fn approaching_a_limit_faster_than_the_curve_allows_faults_but_leaving_it_does_not() {
    // 0.6 rad from the upper limit a joint may approach it at about 1.49 rad/s.
    let (mut c, mut h, reply) = arm_at(2.4, 3.0);
    assert!(reply.accepted, "{:?}", reply.reason);
    for now in 2..5 {
        h.feedback[0].velocity = -1.8; // Away from it, within the 2 rad/s cap.
        c.tick(now, &mut h);
    }
    assert_eq!(c.status().state, State::Holding);
    for now in 5..8 {
        h.feedback[0].velocity = 1.8; // Toward it, three readings in a row.
        c.tick(now, &mut h);
    }
    assert_eq!(c.status().reason, Reason::LimitViolation);
    assert!(!h.enabled);
}

fn interpolated_fixture() -> (Controller, Loopback) {
    let mut cfg = config(2);
    cfg.interpolation_ms = 40;
    cfg.collision = Some(robot_control::collision::CollisionPolicy {
        world_revision: "fixed-world".into(),
        scope: "inter_branch".into(),
        max_verdict_ms: 100, // Longer than a ramp, so the verdict stays fresh through these tests.
        reaction_ms: 10,
    });
    let mut c = Controller::new(cfg, "boot-a".into()).unwrap();
    let mut h = Loopback::new(&[0.0, 0.0]);
    assert!(claim(&mut c, &mut h, 0).accepted);
    let checked = envelope(&mut c, &mut h, 1);
    assert!(
        c.handle(
            1,
            Request::Arm {
                authority: auth(&c, 1),
                collision: Some(checked),
            },
            &mut h,
        )
        .accepted
    );
    (c, h)
}

#[allow(clippy::too_many_arguments)]
fn move_with(
    c: &mut Controller,
    h: &mut Loopback,
    now: u64,
    sequence: u64,
    q: f64,
    v: f64,
    effort: f64,
    upper: f64,
) -> Reply {
    let mut checked = envelope(c, h, now);
    checked.upper = vec![upper; 2];
    let mut commanded = joints(2, q, v);
    for joint in commanded.iter_mut() {
        joint.effort = effort;
    }
    c.handle(
        now,
        Request::Move {
            collision: Some(checked),
            authority: auth(c, sequence),
            identity: identity(),
            joints: commanded,
        },
        h,
    )
}

#[test]
fn velocity_and_effort_feed_forward_are_accepted_with_a_collision_box() {
    let (mut c, mut h) = interpolated_fixture();
    let reply = move_with(&mut c, &mut h, 2, 2, 0.2, 1.0, 1.5, 2.5);
    assert!(reply.accepted, "{:?}", reply.reason);
    let status = c.status();
    assert_eq!(status.state, State::Executing);
    assert_eq!(
        (status.joints[0].velocity, status.joints[0].effort),
        (1.0, 1.5)
    );
}

#[test]
fn a_move_ramps_from_where_the_stream_came_from_then_holds() {
    let (mut c, mut h) = interpolated_fixture();
    // 0.4 rad at 1 rad/s over the 40 ms period: the reference starts at 0.36.
    assert!(move_with(&mut c, &mut h, 10, 2, 0.4, 1.0, 0.0, 2.5).accepted);
    assert!((c.status().joints[0].position - 0.36).abs() < 1e-9);
    c.tick(30, &mut h);
    let halfway = c.status().joints[0];
    assert!((halfway.position - 0.38).abs() < 1e-9 && halfway.velocity == 1.0);
    c.tick(50, &mut h);
    let there = c.status().joints[0];
    assert!((there.position - 0.4).abs() < 1e-9 && there.velocity == 0.0);
}

#[test]
fn a_ramp_that_would_start_within_the_stopping_margin_is_refused() {
    let (mut c, mut h) = interpolated_fixture();
    // Ends 0.1 rad from the limit, but ramps from 2.98 (moving away at 2 rad/s over 40 ms).
    let reply = move_with(&mut c, &mut h, 2, 2, 2.9, -2.0, 0.0, 3.0);
    assert!(!reply.accepted);
    assert_eq!(reply.reason, Reason::CollisionViolation);
}

#[test]
fn a_stop_keeps_the_last_commands_feed_forward_holding_the_joint() {
    // A joint held out against gravity by 3 Nm of feed-forward: without it, stopping would drop
    // the load onto the position loop and sag the joint past its tracking bound.
    let (mut c, mut h) = interpolated_fixture();
    assert!(move_with(&mut c, &mut h, 2, 2, 0.0, 0.0, 3.0, 2.5).accepted);
    let reply = c.handle(
        3,
        Request::Stop {
            authority: auth(&c, 3),
        },
        &mut h,
    );
    assert!(reply.accepted, "{:?}", reply.reason);
    c.tick(4, &mut h);
    assert_eq!(c.status().state, State::Stopped);
    assert_eq!(c.status().joints[0].effort, 3.0);
}
