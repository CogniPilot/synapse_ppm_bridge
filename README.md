# synapse_ppm_bridge

Rust bridge from Synapse `ManualControl` FlatBuffers on Zenoh to the serial
packet consumed by the `ppm_bridge` Arduino encoder.

Run with the defaults used by the original ROS 2 bridge:

```sh
cargo run --bin synapse-ppm-bridge -- \
  --zenoh-connect udp/127.0.0.1:7447 \
  --serial-device /dev/ttyACM0 \
  --baud-rate 57600
```

The default input topic is `synapse/manual_control`. Payloads must be
`synapse.topic.ManualControl` FlatBuffers from the `synapse_fbs` crate.

Useful environment variables:

```sh
ZENOH_CONNECT=udp/127.0.0.1:7447 \
ZENOH_TOPIC=synapse/manual_control \
PPM_SERIAL_DEVICE=/dev/ttyACM0 \
PPM_BAUD_RATE=57600 \
PPM_CHANNEL_MAP=1,2,0,3,4 \
cargo run --bin synapse-ppm-bridge
```

Base channel order before `PPM_CHANNEL_MAP` is:

```text
0 throttle
1 aileron / roll
2 elevator / pitch
3 rudder / yaw
4 mode
```

The serial packet is 14 bytes:

```text
0xffff header, five little-endian u16 channels, little-endian u16 checksum
```

The checksum is the wrapping sum of the five transmitted channel values, matching
the encoder firmware in https://github.com/wsribunma/ppm_bridge.
