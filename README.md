# List of WiFi networks examples for ESP32

The application for ESP32 which demonstrates how to list WiFi using different technologies.

## Examples

* esp32-s3-box-3-esp-idf - ESP32-S3-BOX-3 with ESP-IDF implementation
* esp32-s3-rust-no-std - ESP32-S3 with Rust no\_std (synchronous implementation)
* esp32-s3-rust-embassy - ESP32-S3 with Rust no\_std + Embassy (asynchronous implementation)
* esp32-s3-rust-std - ESP32-S3 with Rust std (using ESP-IDF v5.3)

## Technology Comparison

| Implementation | Language | Runtime | Async Support |
|----------------|----------|---------|---------------|
| esp32-s3-box-3-esp-idf | C | ESP-IDF | Task-based |
| esp32-s3-rust-no-std | Rust | Bare metal | No |
| esp32-s3-rust-embassy | Rust | Embassy | Yes |
| esp32-s3-rust-std | Rust | ESP-IDF | Limited |

### Key Differences

**esp32-s3-rust-no-std**: Pure Rust implementation without any async runtime. Uses blocking calls for WiFi operations.

**esp32-s3-rust-embassy**: Rust implementation with Embassy async runtime. Provides better concurrency and resource management through async/await patterns. WiFi scanning runs as an async task.

