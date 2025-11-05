# ESP32-S3 WiFi Network Scanner (Embassy)

This is an Embassy-based implementation for ESP32-S3 that scans and lists available WiFi networks.

## Features

- Asynchronous WiFi network scanning using Embassy
- Displays network information including SSID, signal strength, authentication method, and channel
- Continuous scanning every 10 seconds
- Uses esp-hal-embassy for async runtime

## Building

Make sure you have the Rust ESP32 toolchain installed:

```bash
cargo install espflash
```

Build the project:

```bash
cargo build --release
```

## Flashing

Flash to your ESP32-S3 device:

```bash
cargo run --release
```

## Output

The application will output discovered WiFi networks in the following format:

```
Found 5 networks:
  1: SSID: MyNetwork, Signal: Some(-45), Auth: WPA2Personal, Channel: 6
  2: SSID: OpenWiFi, Signal: Some(-67), Auth: None, Channel: 1
  ...
```

## Dependencies

This project uses:
- `embassy-executor` for async task execution
- `embassy-time` for async timers
- `esp-hal-embassy` for Embassy integration with ESP32-S3
- `esp-radio` with Embassy support for WiFi functionality
- `esp-hal` 1.0.0 for hardware abstraction

## Differences from no-std version

- Uses Embassy's async runtime instead of blocking calls
- Implements WiFi scanning as an async task
- Uses `scan_n_async` for non-blocking network scanning
- Better resource management with async/await patterns
