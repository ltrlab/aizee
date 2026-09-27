use robot_control::{
    reference::{Gate, Reference},
    robstride::Feedback,
};

fn gate() -> Gate {
    Gate::new(
        vec![Reference {
            name: "joint".into(),
            raw_position_rad: 0.,
            tolerance_rad: 0.03,
        }],
        &["joint"],
    )
    .unwrap()
}
fn sample(position: f64) -> Feedback {
    Feedback {
        motor_id: 4,
        mode: 0,
        error_bits: 0,
        position,
        velocity: 0.,
        effort: 0.,
        temperature: 25.,
    }
}
#[test]
fn requires_repeated_fresh_samples_and_is_single_use() {
    let mut g = gate();
    assert!(g.consume(0).is_err());
    for t in [0, 50, 100] {
        g.observe(0, sample(0.001), t, t).unwrap();
    }
    assert_eq!(g.status(100), "ready");
    assert_eq!(g.status(141), "unverified");
    g.consume(100).unwrap();
    assert!(g.consume(100).is_err());
}
#[test]
fn shifted_zero_and_whole_revolutions_latch_failure() {
    for q in [
        0.031,
        std::f64::consts::TAU,
        -std::f64::consts::TAU,
        f64::NAN,
    ] {
        let mut g = gate();
        assert!(g.observe(0, sample(q), 1, 1).is_err());
        assert!(g.observe(0, sample(0.), 2, 2).is_err());
        assert!(g.consume(2).is_err());
    }
}
#[test]
fn a_failure_names_the_joint_and_the_value_and_keeps_the_first_reason() {
    let mut g = gate();
    let mut f = sample(0.);
    f.velocity = 0.21;
    assert!(g.observe(0, f, 0, 0).is_ok() && g.observe(0, f, 1, 1).is_ok());
    assert!(g.observe(0, f, 2, 2).is_err());
    let moving = Some("joint: moving at +0.210 rad/s, 3 readings in a row over 0.2 rad/s");
    assert_eq!(g.failure.as_deref(), moving);
    assert!(g.observe(0, sample(0.05), 3, 3).is_err());
    assert_eq!(g.failure.as_deref(), moving);
    let mut g = gate();
    assert!(g.observe(0, sample(0.031), 1, 1).is_err());
    assert_eq!(
        g.failure.as_deref(),
        Some("joint: at +0.0310 rad, 0.0310 rad from its reference (tolerance 0.03 rad)")
    );
    let mut g = gate();
    assert!(g.observe(0, sample(0.), 0, 41).is_err());
    assert_eq!(
        g.failure.as_deref(),
        Some("joint: feedback read 41 ms after it arrived (limit 40 ms)")
    );
}
#[test]
fn speed_noise_on_a_still_motor_is_not_motion() {
    // Isolated readings over the still speed, as a resting drive reports them.
    let mut g = gate();
    let mut noisy = sample(0.001);
    noisy.velocity = -0.25;
    for t in [0, 25, 50, 75, 100] {
        g.observe(0, noisy, t, t).unwrap();
        g.observe(0, sample(0.001), t + 1, t + 1).unwrap();
    }
    assert_eq!(g.status(101), "ready");
    assert!(g.failure.is_none());
}
#[test]
fn old_duplicate_moving_and_enabled_samples_cannot_verify() {
    let mut g = gate();
    for _ in 0..10 {
        g.observe(0, sample(0.), 0, 0).unwrap();
    }
    assert!(g.consume(100).is_err());
    assert!(gate().observe(0, sample(0.), 0, 41).is_err());
    let mut f = sample(0.);
    f.mode = 2;
    assert!(gate().observe(0, f, 0, 0).is_err());
    f.mode = 0;
    f.velocity = 0.21;
    let mut g = gate();
    for t in 0..2 {
        g.observe(0, f, t, t).unwrap();
    }
    assert!(g.observe(0, f, 2, 2).is_err());
    let mut g = gate();
    g.revoke();
    assert!(g.observe(0, sample(0.), 0, 0).is_err());
}
