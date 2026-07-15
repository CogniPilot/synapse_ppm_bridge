use std::io::Write;
use std::time::Duration;

use clap::Parser;
use synapse_fbs::topic::ManualControlData;
use synapse_ppm_bridge::{
    ChannelMap, PpmChannels, build_packet, channel_map_from_slice, manual_control_to_channels,
};
use thiserror::Error;
use zenoh::{Wait, config::Config};

#[derive(Debug, Parser)]
#[command(
    name = "synapse-ppm-bridge",
    version,
    about = "Bridge Synapse manual-control data on Zenoh to a PPM encoder serial link",
    long_about = "Subscribes to a Synapse ManualControlData bare struct over Zenoh, maps normalized \
manual-control axes to PWM microsecond values, and sends the same 14-byte serial packet used by \
the ROS ppm_bridge Arduino encoder.",
    next_line_help = true,
    after_help = "\
Examples:
  synapse-ppm-bridge --serial-device /dev/ttyACM0
  synapse-ppm-bridge --topic manual --channel-map 1,2,0,3,4

Environment:
  ZENOH_CONNECT, ZENOH_TOPIC
  PPM_SERIAL_DEVICE, PPM_BAUD_RATE, PPM_CHANNEL_MAP"
)]
struct Cli {
    #[command(flatten)]
    zenoh: ZenohArgs,

    #[command(flatten)]
    serial: SerialArgs,

    #[command(flatten)]
    ppm: PpmArgs,
}

#[derive(Debug, Parser)]
#[command(next_help_heading = "Zenoh")]
struct ZenohArgs {
    #[arg(
        long = "zenoh-connect",
        env = "ZENOH_CONNECT",
        value_name = "LOCATOR",
        default_value = "udp/127.0.0.1:7447",
        help = "Zenoh router locator"
    )]
    zenoh_connect: String,

    #[arg(
        long = "topic",
        alias = "zenoh-topic",
        env = "ZENOH_TOPIC",
        value_name = "KEYEXPR",
        default_value = "manual",
        help = "Zenoh key expression carrying synapse.topic.ManualControlData bare structs"
    )]
    topic: String,
}

#[derive(Debug, Parser)]
#[command(next_help_heading = "Serial")]
struct SerialArgs {
    #[arg(
        long = "serial-device",
        env = "PPM_SERIAL_DEVICE",
        value_name = "PATH",
        default_value = "/dev/ttyACM0",
        help = "Serial device connected to the PPM encoder"
    )]
    serial_device: String,

    #[arg(
        long = "baud-rate",
        env = "PPM_BAUD_RATE",
        value_name = "BAUD",
        default_value_t = 57_600,
        help = "Serial baud rate"
    )]
    baud_rate: u32,

    #[arg(
        long = "serial-timeout-ms",
        env = "PPM_SERIAL_TIMEOUT_MS",
        value_name = "MS",
        default_value_t = 100,
        help = "Serial write timeout"
    )]
    serial_timeout_ms: u64,
}

#[derive(Debug, Parser)]
#[command(next_help_heading = "PPM")]
struct PpmArgs {
    #[arg(
        long = "channel-map",
        env = "PPM_CHANNEL_MAP",
        value_delimiter = ',',
        value_name = "INDEXES",
        default_value = "0,1,2,3,4",
        help = "Comma-separated output channel map over base order throttle,roll,pitch,yaw,mode"
    )]
    channel_map: Vec<usize>,
}

#[derive(Debug, Error)]
enum BridgeError {
    #[error("zenoh error: {0}")]
    Zenoh(String),
    #[error("serial error: {0}")]
    Serial(#[from] serialport::Error),
    #[error("serial write error: {0}")]
    SerialWrite(#[from] std::io::Error),
    #[error("channel map error: {0}")]
    ChannelMap(#[from] synapse_ppm_bridge::ChannelMapError),
    #[error("manual control payload is {actual} bytes, expected {expected}")]
    InvalidManualControlSize { expected: usize, actual: usize },
}

/// Wire size of a bare `synapse.topic.ManualControlData` struct.
const MANUAL_CONTROL_PAYLOAD_SIZE: usize = 40;

type Result<T> = std::result::Result<T, BridgeError>;

fn main() -> Result<()> {
    let cli = Cli::parse();
    let channel_map = channel_map_from_slice(&cli.ppm.channel_map)?;
    run(cli, channel_map)
}

fn run(cli: Cli, channel_map: ChannelMap) -> Result<()> {
    let session = zenoh::open(zenoh_config(&cli)?)
        .wait()
        .map_err(|error| BridgeError::Zenoh(error.to_string()))?;
    let subscriber = session
        .declare_subscriber(cli.zenoh.topic.clone())
        .wait()
        .map_err(|error| BridgeError::Zenoh(error.to_string()))?;

    let mut serial = serialport::new(&cli.serial.serial_device, cli.serial.baud_rate)
        .timeout(Duration::from_millis(cli.serial.serial_timeout_ms))
        .open()?;

    println!(
        "listening on {} and writing {} at {} baud with channel map {:?}",
        cli.zenoh.topic, cli.serial.serial_device, cli.serial.baud_rate, channel_map.0
    );

    loop {
        let sample = subscriber
            .recv()
            .map_err(|error| BridgeError::Zenoh(error.to_string()))?;
        let payload = sample.payload().to_bytes();
        let channels = match channels_from_payload(&payload, channel_map) {
            Ok(channels) => channels,
            Err(BridgeError::InvalidManualControlSize { expected, actual }) => {
                eprintln!(
                    "dropping manual control payload with {actual} bytes; expected {expected}"
                );
                continue;
            }
            Err(error) => return Err(error),
        };
        serial.write_all(&build_packet(channels))?;
    }
}

fn zenoh_config(cli: &Cli) -> Result<Config> {
    let mut config = Config::default();
    config
        .insert_json5("mode", "\"client\"")
        .map_err(|error| BridgeError::Zenoh(error.to_string()))?;
    config
        .insert_json5(
            "connect/endpoints",
            &format!("[\"{}\"]", cli.zenoh.zenoh_connect),
        )
        .map_err(|error| BridgeError::Zenoh(error.to_string()))?;
    Ok(config)
}

fn channels_from_payload(payload: &[u8], channel_map: ChannelMap) -> Result<PpmChannels> {
    if payload.len() != MANUAL_CONTROL_PAYLOAD_SIZE {
        return Err(BridgeError::InvalidManualControlSize {
            expected: MANUAL_CONTROL_PAYLOAD_SIZE,
            actual: payload.len(),
        });
    }

    // Safety: FlatBuffers fixed-layout structs use unaligned accessors and the
    // exact-size check above covers the complete struct at offset zero.
    let data = unsafe { <ManualControlData as flatbuffers::Follow>::follow(payload, 0) };
    Ok(channel_map.apply(manual_control_to_channels(data)))
}

#[cfg(test)]
mod tests {
    use super::*;
    use synapse_fbs::topic::{ManualControlAxes, ManualControlFlags};

    #[test]
    fn decodes_synapse_0_8_bare_manual_control_payload() {
        let axes = ManualControlAxes::Pitch
            | ManualControlAxes::Roll
            | ManualControlAxes::Throttle
            | ManualControlAxes::Yaw;
        let flags = ManualControlFlags::Active | ManualControlFlags::Valid;
        let data = ManualControlData::new(
            42,
            0,
            axes.bits(),
            -250,
            500,
            750,
            -500,
            0,
            0,
            0,
            0,
            0,
            0,
            1,
            flags.bits(),
        );

        let channels = channels_from_payload(&data.0, ChannelMap([0, 1, 2, 3, 4])).unwrap();
        assert_eq!(channels, PpmChannels([1750, 1750, 1625, 1250, 2000]));
    }

    #[test]
    fn rejects_non_struct_payload_sizes() {
        let error = channels_from_payload(&[0; 39], ChannelMap([0, 1, 2, 3, 4])).unwrap_err();
        assert!(matches!(
            error,
            BridgeError::InvalidManualControlSize {
                expected: 40,
                actual: 39
            }
        ));
    }
}
