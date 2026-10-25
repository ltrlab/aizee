#[cfg(all(target_os = "linux", feature = "linux-service"))]
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut args = std::env::args().skip(1);
    let path = args.next().ok_or(
        "usage: head_controller CONFIG.json [--validate] [--seconds N] [--allow-powered-teleop] [--confirm-reference-pose]",
    )?;
    let config: robot_control::head_config::HeadServiceConfig =
        serde_json::from_reader(std::fs::File::open(path)?)?;
    config.validate()?;
    let mut duration = None;
    let mut allow_teleop = false;
    let mut reference_confirmed = false;
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--validate" => {
                println!("configuration valid; no ports or sockets opened");
                return Ok(());
            }
            "--seconds" => {
                let value: f64 = args.next().ok_or("missing duration")?.parse()?;
                if !value.is_finite() || value <= 0.0 || value > 3600.0 {
                    return Err("duration must be in (0, 3600] seconds".into());
                }
                duration = Some(value);
            }
            "--allow-powered-teleop" => allow_teleop = true,
            "--confirm-reference-pose" => reference_confirmed = true,
            _ => return Err("unknown argument".into()),
        }
    }
    if config.mode == robot_control::head_config::HeadMode::TeleopDirect
        && !(allow_teleop && reference_confirmed)
    {
        return Err("physical head teleop requires --allow-powered-teleop and --confirm-reference-pose after the operator verifies the head's reference pose".into());
    }
    robot_control::head_service::run(config, duration)
}
#[cfg(not(all(target_os = "linux", feature = "linux-service")))]
fn main() {
    eprintln!("head_controller requires Linux and the linux-service feature");
    std::process::exit(2);
}
