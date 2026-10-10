#[cfg(all(target_os = "linux", feature = "linux-service"))]
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut args = std::env::args().skip(1);
    let path = args.next().ok_or(
        "usage: guarded_controller CONFIG.json [--validate] [--seconds N] [--allow-powered-trial] [--allow-powered-teleop] [--confirm-reference-pose]",
    )?;
    let config: robot_control::service::ServiceConfig =
        serde_json::from_reader(std::fs::File::open(path)?)?;
    config.validate()?;
    let mut duration = None;
    let mut allow_powered = false;
    let mut allow_teleop = false;
    let mut reference_confirmed = false;
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--validate" => {
                println!("configuration valid; no sockets opened");
                return Ok(());
            }
            "--seconds" => {
                let value: f64 = args.next().ok_or("missing duration")?.parse()?;
                if !value.is_finite() || value <= 0.0 || value > 3600.0 {
                    return Err("duration must be in (0, 3600] seconds".into());
                }
                duration = Some(value);
            }
            "--allow-powered-trial" => allow_powered = true,
            "--allow-powered-teleop" => allow_teleop = true,
            "--confirm-reference-pose" => reference_confirmed = true,
            _ => return Err("unknown argument".into()),
        }
    }
    if matches!(
        config.mode,
        robot_control::service::Mode::ParameterProbe
            | robot_control::service::Mode::MotorDiagnostics
            | robot_control::service::Mode::ZeroTorqueTrial
            | robot_control::service::Mode::BoundedJogTrial
            | robot_control::service::Mode::BreakawayTrial
    ) && (!allow_powered || duration.is_none_or(|seconds| seconds > 30.0))
    {
        return Err("powered commissioning requires --allow-powered-trial and --seconds <=30; this does not approve joint motion".into());
    }
    if config.mode == robot_control::service::Mode::ZeroTorqueTrial && !reference_confirmed {
        return Err("physical enable requires operator verification of the reference pose and --confirm-reference-pose".into());
    }
    if matches!(config.mode, robot_control::service::Mode::Teleop | robot_control::service::Mode::TeleopDirect)
        && !(allow_teleop && reference_confirmed)
    {
        return Err("physical teleop requires --allow-powered-teleop and --confirm-reference-pose after the operator verifies the arm's reference pose".into());
    }
    robot_control::service::run(config, duration)
}
#[cfg(not(target_os = "linux"))]
fn main() {
    eprintln!("guarded_controller requires Linux SocketCAN");
    std::process::exit(2);
}
