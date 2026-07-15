use synapse_fbs::topic::{ManualControlData, ManualControlFlags};

pub const NUM_CHANNELS: usize = 5;
pub const PACKET_LEN: usize = 14;
pub const PACKET_HEADER: u16 = 0xffff;
pub const DEFAULT_CHANNEL_MAP: [usize; NUM_CHANNELS] = [0, 1, 2, 3, 4];
pub const FAILSAFE_CHANNELS: [u16; NUM_CHANNELS] = [1000, 1500, 1500, 1500, 2000];

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PpmChannels(pub [u16; NUM_CHANNELS]);

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ChannelMap(pub [usize; NUM_CHANNELS]);

impl Default for ChannelMap {
    fn default() -> Self {
        Self(DEFAULT_CHANNEL_MAP)
    }
}

impl ChannelMap {
    pub fn new(channels: [usize; NUM_CHANNELS]) -> Result<Self, ChannelMapError> {
        if let Some(channel) = channels.iter().find(|&&channel| channel >= NUM_CHANNELS) {
            return Err(ChannelMapError::OutOfRange(*channel));
        }

        Ok(Self(channels))
    }

    pub fn apply(self, channels: PpmChannels) -> PpmChannels {
        PpmChannels(self.0.map(|index| channels.0[index]))
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ChannelMapError {
    WrongLength { expected: usize, actual: usize },
    OutOfRange(usize),
}

impl std::fmt::Display for ChannelMapError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::WrongLength { expected, actual } => {
                write!(f, "expected {expected} channel-map entries, got {actual}")
            }
            Self::OutOfRange(channel) => {
                write!(
                    f,
                    "channel map entry {channel} is out of range 0..{}",
                    NUM_CHANNELS - 1
                )
            }
        }
    }
}

impl std::error::Error for ChannelMapError {}

pub fn channel_map_from_slice(channels: &[usize]) -> Result<ChannelMap, ChannelMapError> {
    let channels: [usize; NUM_CHANNELS] =
        channels
            .try_into()
            .map_err(|_| ChannelMapError::WrongLength {
                expected: NUM_CHANNELS,
                actual: channels.len(),
            })?;
    ChannelMap::new(channels)
}

pub fn manual_control_to_channels(data: &ManualControlData) -> PpmChannels {
    let flags = ManualControlFlags::from_bits_retain(data.flags());
    if !flags.contains(ManualControlFlags::Valid)
        || !flags.contains(ManualControlFlags::Active)
        || flags.contains(ManualControlFlags::KillSwitch)
    {
        return PpmChannels(FAILSAFE_CHANNELS);
    }

    PpmChannels([
        throttle_to_pwm(milli_to_normalized(data.throttle_milli())),
        centered_to_pwm(milli_to_normalized(data.roll_milli())),
        centered_to_pwm(-milli_to_normalized(data.pitch_milli())),
        centered_to_pwm(milli_to_normalized(data.yaw_milli())),
        mode_to_pwm(data.flight_mode()),
    ])
}

pub fn build_packet(channels: PpmChannels) -> [u8; PACKET_LEN] {
    let mut packet = [0_u8; PACKET_LEN];
    packet[0..2].copy_from_slice(&PACKET_HEADER.to_le_bytes());

    for (index, channel) in channels.0.iter().enumerate() {
        let start = 2 + index * 2;
        packet[start..start + 2].copy_from_slice(&channel.to_le_bytes());
    }

    let checksum = checksum(channels);
    packet[12..14].copy_from_slice(&checksum.to_le_bytes());
    packet
}

pub fn checksum(channels: PpmChannels) -> u16 {
    channels
        .0
        .iter()
        .fold(0_u16, |sum, channel| sum.wrapping_add(*channel))
}

fn throttle_to_pwm(value: f32) -> u16 {
    scale_to_pwm(value, 0.0, 1.0, 1000.0, 2000.0)
}

fn centered_to_pwm(value: f32) -> u16 {
    scale_to_pwm(value, -1.0, 1.0, 1000.0, 2000.0)
}

fn mode_to_pwm(flight_mode: u8) -> u16 {
    if flight_mode == 0 { 1000 } else { 2000 }
}

fn milli_to_normalized(value: i16) -> f32 {
    f32::from(value) / 1000.0
}

fn scale_to_pwm(value: f32, in_min: f32, in_max: f32, out_min: f32, out_max: f32) -> u16 {
    let value = if value.is_finite() { value } else { in_min };
    let normalized = ((value - in_min) / (in_max - in_min)).clamp(0.0, 1.0);
    (out_min + normalized * (out_max - out_min)).round() as u16
}

#[cfg(test)]
mod tests {
    use super::*;
    use synapse_fbs::topic::{ManualControlAxes, ManualControlData, ManualControlFlags};

    const STICK_AXES: ManualControlAxes = ManualControlAxes::Pitch
        .union(ManualControlAxes::Roll)
        .union(ManualControlAxes::Throttle)
        .union(ManualControlAxes::Yaw);

    fn to_milli(value: f32) -> i16 {
        (value * 1000.0).round().clamp(-1000.0, 1000.0) as i16
    }

    fn manual_control_data_with_flags(
        roll: f32,
        pitch: f32,
        yaw: f32,
        throttle: f32,
        flight_mode: u8,
        flags: ManualControlFlags,
    ) -> ManualControlData {
        ManualControlData::new(
            42,
            0,
            STICK_AXES.bits(),
            to_milli(pitch),
            to_milli(roll),
            to_milli(throttle),
            to_milli(yaw),
            0,
            0,
            0,
            0,
            0,
            0,
            flight_mode,
            flags.bits(),
        )
    }

    fn manual_control_data(
        roll: f32,
        pitch: f32,
        yaw: f32,
        throttle: f32,
        flight_mode: u8,
    ) -> ManualControlData {
        manual_control_data_with_flags(
            roll,
            pitch,
            yaw,
            throttle,
            flight_mode,
            ManualControlFlags::Active | ManualControlFlags::Valid,
        )
    }

    #[test]
    fn maps_manual_control_to_base_ppm_channels() {
        let data = manual_control_data(0.5, 0.25, -0.5, 0.75, 1);
        assert_eq!(
            manual_control_to_channels(&data),
            PpmChannels([1750, 1750, 1375, 1250, 2000])
        );
    }

    #[test]
    fn invalid_or_kill_switch_messages_use_failsafe_channels() {
        let invalid =
            manual_control_data_with_flags(1.0, 1.0, 1.0, 1.0, 1, ManualControlFlags::Active);
        let killed = manual_control_data_with_flags(
            1.0,
            1.0,
            1.0,
            1.0,
            1,
            ManualControlFlags::Active | ManualControlFlags::Valid | ManualControlFlags::KillSwitch,
        );

        assert_eq!(
            manual_control_to_channels(&invalid),
            PpmChannels(FAILSAFE_CHANNELS)
        );
        assert_eq!(
            manual_control_to_channels(&killed),
            PpmChannels(FAILSAFE_CHANNELS)
        );
    }

    #[test]
    fn applies_channel_map_to_base_channels() {
        let map = ChannelMap::new([1, 2, 0, 3, 4]).unwrap();
        assert_eq!(
            map.apply(PpmChannels([1000, 1500, 1600, 1700, 2000])),
            PpmChannels([1500, 1600, 1000, 1700, 2000])
        );
    }

    #[test]
    fn encodes_reference_serial_packet_format() {
        let packet = build_packet(PpmChannels([1000, 1500, 1500, 1500, 2000]));

        assert_eq!(packet[0..2], [0xff, 0xff]);
        assert_eq!(
            packet[2..12],
            [0xe8, 0x03, 0xdc, 0x05, 0xdc, 0x05, 0xdc, 0x05, 0xd0, 0x07]
        );
        assert_eq!(packet[12..14], 7500_u16.to_le_bytes());
    }

    #[test]
    fn rejects_bad_channel_maps() {
        assert_eq!(
            channel_map_from_slice(&[0, 1, 2]).unwrap_err(),
            ChannelMapError::WrongLength {
                expected: 5,
                actual: 3
            }
        );
        assert_eq!(
            channel_map_from_slice(&[0, 1, 2, 3, 5]).unwrap_err(),
            ChannelMapError::OutOfRange(5)
        );
    }
}
