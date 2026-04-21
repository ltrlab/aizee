// AIZEE LiDAR Control - RPLiDAR A1M8 Scanner Interface

mod scanner;

use anyhow::{Context, Result};
use comms::messages::{LidarScan, TelemetryMessage};
use scanner::{LidarConfig, LidarScanner};
use std::time::Duration;
use tokio::sync::mpsc;
use tokio::time::interval;
use tracing::{error, info, warn};
use tracing_subscriber;

#[derive(Debug, Clone, serde::Deserialize)]
struct Config {
    lidars: Vec<LidarConfigYaml>,
    network: NetworkConfig,
}

#[derive(Debug, Clone, serde::Deserialize)]
struct LidarConfigYaml {
    id: String,
    device: String,
    #[serde(default)]
    scan_mode: String,
}

#[derive(Debug, Clone, serde::Deserialize)]
struct NetworkConfig {
    #[serde(alias = "jetson")]  // Backward compatibility
    device: DeviceConfig,
}

#[derive(Debug, Clone, serde::Deserialize)]
struct DeviceConfig {
    #[serde(default)]
    ip: String,
    #[serde(default)]
    hostname: String,
    zmq: ZmqConfig,
}

#[derive(Debug, Clone, serde::Deserialize)]
struct ZmqConfig {
    #[serde(default)]
    command_sub: Option<String>,
    #[serde(default)]
    telemetry_pub: Option<String>,
    #[serde(default)]
    lidar_pub: Option<String>,
}

fn load_config() -> Result<Config> {
    let config_path = std::env::var("AIZEE_CONFIG")
        .unwrap_or_else(|_| "config/hardware_jetson_rover.yaml".to_string());

    info!("Loading config from: {}", config_path);

    let config_str = std::fs::read_to_string(&config_path)
        .with_context(|| format!("Failed to read config from {}", config_path))?;
    let config: Config = serde_yaml::from_str(&config_str)?;

    Ok(config)
}

fn initialize_scanners(config: &Config) -> Result<Vec<LidarScanner>> {
    let mut scanners = Vec::new();

    for lidar_config in &config.lidars {
        info!("Initializing LiDAR: {} at {}", lidar_config.id, lidar_config.device);

        let scanner_config = LidarConfig {
            id: lidar_config.id.clone(),
            device: lidar_config.device.clone(),
        };

        match LidarScanner::new(scanner_config) {
            Ok(scanner) => {
                info!("LiDAR {} initialized successfully", lidar_config.id);
                scanners.push(scanner);
            }
            Err(e) => {
                error!("Failed to initialize LiDAR {}: {}", lidar_config.id, e);
                // Continue with other sensors even if one fails
            }
        }
    }

    if scanners.is_empty() {
        anyhow::bail!("No LiDAR sensors initialized successfully");
    }

    Ok(scanners)
}

fn publish_scans(context: &zmq::Context, endpoint: &str, scans: &[LidarScan]) -> Result<()> {
    let socket = context.socket(zmq::PUB)
        .context("Failed to create ZMQ PUB socket")?;

    socket.bind(endpoint)
        .with_context(|| format!("Failed to bind to {}", endpoint))?;

    let mut telemetry_msg = TelemetryMessage::new();
    telemetry_msg.lidar_scans = Some(scans.to_vec());

    let json = serde_json::to_string(&telemetry_msg)
        .context("Failed to serialize telemetry message")?;

    socket.send(&json, 0)
        .context("Failed to send telemetry message")?;

    Ok(())
}

#[tokio::main]
async fn main() -> Result<()> {
    // Initialize logging
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| tracing_subscriber::EnvFilter::new("info"))
        )
        .init();

    info!("AIZEE LiDAR Control starting...");

    // Load configuration
    let config = load_config()?;

    // Get ZMQ endpoint
    let lidar_pub_endpoint = config.network.device.zmq.lidar_pub
        .as_ref()
        .context("lidar_pub endpoint not configured")?;

    info!("LiDAR telemetry will publish on: {}", lidar_pub_endpoint);

    // Initialize ZMQ context
    let zmq_context = zmq::Context::new();
    let socket = zmq_context.socket(zmq::PUB)
        .context("Failed to create ZMQ PUB socket")?;

    socket.bind(lidar_pub_endpoint)
        .with_context(|| format!("Failed to bind to {}", lidar_pub_endpoint))?;

    info!("ZMQ socket bound to {}", lidar_pub_endpoint);

    // Give ZMQ time to establish connections
    tokio::time::sleep(Duration::from_millis(100)).await;

    // Initialize scanners
    let scanners = initialize_scanners(&config)?;
    info!("Initialized {} LiDAR sensor(s)", scanners.len());

    // Create scan aggregation channel
    let (scan_tx, mut scan_rx) = mpsc::channel::<LidarScan>(8);

    // Spawn blocking tasks for each sensor
    for scanner in scanners {
        let tx = scan_tx.clone();
        tokio::task::spawn_blocking(move || {
            if let Err(e) = scanner.run_loop(tx) {
                error!("Scanner task failed: {}", e);
            }
        });
    }

    // Drop the original sender so channel closes when all scanners exit
    drop(scan_tx);

    // Main loop: aggregate and publish scans
    let mut publish_interval = interval(Duration::from_millis(200)); // 5Hz
    let mut scan_buffer = Vec::new();

    info!("LiDAR control loop started");

    loop {
        tokio::select! {
            Some(scan) = scan_rx.recv() => {
                info!("Received scan from {}: {} points", scan.sensor_id, scan.ranges.len());
                scan_buffer.push(scan);
            }

            _ = publish_interval.tick() => {
                if !scan_buffer.is_empty() {
                    // Create telemetry message
                    let mut telemetry_msg = TelemetryMessage::new();
                    telemetry_msg.lidar_scans = Some(scan_buffer.clone());

                    // Serialize and publish
                    match serde_json::to_string(&telemetry_msg) {
                        Ok(json) => {
                            if let Err(e) = socket.send(&json, 0) {
                                error!("Failed to publish LiDAR telemetry: {}", e);
                            } else {
                                info!("Published {} LiDAR scan(s)", scan_buffer.len());
                            }
                        }
                        Err(e) => {
                            error!("Failed to serialize telemetry: {}", e);
                        }
                    }

                    scan_buffer.clear();
                }
            }

            else => {
                warn!("All scan channels closed, exiting");
                break;
            }
        }
    }

    info!("LiDAR control shutting down");
    Ok(())
}
