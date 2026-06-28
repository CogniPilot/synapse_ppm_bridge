use std::io::Write;
use std::time::Duration;

use clap::Parser;
use synapse_fbs::topic::ManualControl;
use synapse_ppm_bridge::{
    ChannelMap, PpmChannels, build_packet, channel_map_from_slice, manual_control_to_channels,
};
use thiserror::Error;
use zenoh::{Wait, config::Config};

#[derive(Debug, Parser)]
#[command(
    name = "synapse-ppm-bridge",
    version,
    about = "Bridge Synapse ManualControl FlatBuffers on Zenoh to a PPM encoder serial link",
    long_about = "Subscribes to a Synapse ManualControl FlatBuffer over Zenoh, maps normalized \
manual-control axes to PWM microsecond values, and sends the same 14-byte serial packet used by \
the ROS ppm_bridge Arduino encoder.",
    next_line_help = true,
    after_help = "\
Examples:
  synapse-ppm-bridge --serial-device /dev/ttyACM0
  synapse-ppm-bridge --topic synapse/manual_control --channel-map 1,2,0,3,4

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
        default_value = "synapse/manual_control",
        help = "Zenoh key expression carrying synapse.topic.ManualControl payloads"
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
    #[error("manual control payload is missing data")]
    MissingManualControlData,
    #[error("invalid ManualControl flatbuffer: {0}")]
    InvalidFlatbuffer(#[from] flatbuffers::InvalidFlatbuffer),
}

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
            Err(BridgeError::InvalidFlatbuffer(error)) => {
                eprintln!("dropping invalid ManualControl flatbuffer: {error}");
                continue;
            }
            Err(BridgeError::MissingManualControlData) => {
                eprintln!("dropping ManualControl payload without data");
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
    let manual_control = flatbuffers::root::<ManualControl>(payload)?;
    let data = manual_control
        .data()
        .ok_or(BridgeError::MissingManualControlData)?;
    Ok(channel_map.apply(manual_control_to_channels(data)))
}
