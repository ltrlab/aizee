// ZeroMQ pub/sub bridge implementation

use crate::messages::{CommandMessage, TelemetryMessage};
use anyhow::{Context, Result};
use std::time::Duration;

/// Subscribe to command messages from teleop
pub struct CommandSubscriber {
    socket: zmq::Socket,
}

impl CommandSubscriber {
    pub fn new(endpoint: &str) -> Result<Self> {
        let context = zmq::Context::new();
        let socket = context.socket(zmq::SUB)?;
        socket.connect(endpoint)?;
        socket.set_subscribe(b"")?; // Subscribe to all messages
        socket.set_rcvtimeo(100)?; // 100ms receive timeout

        tracing::info!("Command subscriber connected to {}", endpoint);

        Ok(Self { socket })
    }

    /// Receive next command (non-blocking with timeout)
    pub fn recv_command(&mut self) -> Result<Option<CommandMessage>> {
        match self.socket.recv_bytes(0) {
            Ok(bytes) => {
                let cmd: CommandMessage = serde_json::from_slice(&bytes)
                    .context("Failed to deserialize command")?;
                Ok(Some(cmd))
            }
            Err(zmq::Error::EAGAIN) => Ok(None), // Timeout, no message available
            Err(e) => Err(e.into()),
        }
    }
}

/// Publish telemetry messages
pub struct TelemetryPublisher {
    socket: zmq::Socket,
}

impl TelemetryPublisher {
    pub fn new(endpoint: &str) -> Result<Self> {
        let context = zmq::Context::new();
        let socket = context.socket(zmq::PUB)?;
        socket.bind(endpoint)?;

        tracing::info!("Telemetry publisher bound to {}", endpoint);

        Ok(Self { socket })
    }

    /// Publish telemetry message
    pub fn publish(&mut self, msg: &TelemetryMessage) -> Result<()> {
        let json = serde_json::to_string(msg)?;
        self.socket.send(&json, 0)?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::messages::{MotorTelemetry, TelemetryMessage};
    use std::thread;

    #[test]
    #[ignore] // Requires ZeroMQ sockets, run manually
    fn test_pub_sub() {
        // Publisher
        let mut publisher = TelemetryPublisher::new("tcp://127.0.0.1:15556").unwrap();

        // Subscriber in separate thread
        let handle = thread::spawn(|| {
            let mut subscriber = CommandSubscriber::new("tcp://127.0.0.1:15556").unwrap();
            thread::sleep(Duration::from_millis(100)); // Wait for connection

            for _ in 0..5 {
                if let Ok(Some(_msg)) = subscriber.recv_command() {
                    return true;
                }
                thread::sleep(Duration::from_millis(10));
            }
            false
        });

        thread::sleep(Duration::from_millis(50)); // Wait for subscriber to connect

        let mut msg = TelemetryMessage::new();
        msg.add_motor(
            "test".to_string(),
            MotorTelemetry {
                position: 0.0,
                velocity: 0.0,
                torque: 0.0,
                temperature: 25.0,
                error: None,
            },
        );
        publisher.publish(&msg).unwrap();

        // Note: This test may fail due to timing; it's primarily for manual verification
    }
}
