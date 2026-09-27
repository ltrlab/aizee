use robot_control::{hardware::JointFeedback, protocol::JointCommand, recorder::Recorder};

fn sample(position: f64, velocity: f64, effort: f64) -> JointFeedback {
    JointFeedback {
        position,
        velocity,
        effort,
        temperature: 30.0,
        sequence: 1,
        sampled_at_ms: 0,
        fault: false,
    }
}
fn command(position: f64) -> JointCommand {
    JointCommand {
        position,
        velocity: 0.0,
        effort: 0.0,
        kp: 10.0,
        kd: 1.0,
    }
}

#[test]
fn each_row_holds_time_then_measured_and_commanded_per_joint_and_is_reported_once() {
    let mut recorder = Recorder::new(2);
    let feedback = [sample(0.1, 0.2, 0.3), sample(1.1, 1.2, 1.3)];
    let commands = [command(0.4), command(1.4)];
    recorder.record(2.5, &feedback, &commands);
    recorder.record(5.0, &feedback, &commands);
    let (rows, dropped) = recorder.take();
    assert_eq!(dropped, 0);
    assert_eq!(rows.len(), 2);
    assert_eq!(rows[0], &[2.5, 0.1, 0.2, 0.3, 0.4, 1.1, 1.2, 1.3, 1.4]);
    assert_eq!(rows[1][0], 5.0);
    assert!(recorder.take().0.is_empty()); // Reported rows are not sent again.
    recorder.record(7.5, &feedback, &commands);
    assert_eq!(recorder.take().0.len(), 1);
}

#[test]
fn a_slow_reader_gets_the_newest_rows_and_the_count_it_missed() {
    let mut recorder = Recorder::new(1);
    let feedback = [sample(0.0, 0.0, 0.0)];
    let commands = [command(0.0)];
    let total = Recorder::CAPACITY + 10;
    for tick in 0..total {
        recorder.record(tick as f64, &feedback, &commands);
    }
    let (rows, dropped) = recorder.take();
    assert_eq!(dropped, 10);
    assert_eq!(rows.len(), Recorder::CAPACITY);
    assert_eq!(rows[0][0], 10.0); // Oldest kept, in order.
    assert_eq!(rows[Recorder::CAPACITY - 1][0], (total - 1) as f64);
}
