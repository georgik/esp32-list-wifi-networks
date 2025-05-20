#![no_std]
#![no_main]

use esp_hal::clock::CpuClock;
use esp_hal::main;
use esp_hal::time::{Duration, Instant};
use esp_hal::timer::timg::TimerGroup;
use esp_println::println;

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}

extern crate alloc;

#[main]
fn main() -> ! {

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(size: 72 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let _init = esp_wifi::init(
        timg0.timer0,
        esp_hal::rng::Rng::new(peripherals.RNG),
        peripherals.RADIO_CLK,
    )
    .unwrap();

    // Enable logging via UART (if not already done elsewhere)
    esp_println::logger::init_logger_from_env();
    info!("Starting WiFi scan...");

    use log::info;
    use esp_wifi::{
        init,
        wifi::{self, AccessPointInfo, ClientConfiguration, Configuration, WifiMode},
    };
    // Create and start the WiFi driver
    let (mut controller, _interfaces) =
        wifi::new(&_init, peripherals.WIFI).unwrap();

    // Set mode and configuration
    controller.set_mode(WifiMode::Sta).unwrap();
    controller.set_configuration(&Configuration::Client(ClientConfiguration {
        ssid: heapless::String::new(),
        password: heapless::String::new(),
        ..Default::default()
    })).unwrap();

    // Start controller before scanning
    controller.start().unwrap();

    // Perform a scan
    let (results, _count) = controller.scan_n::<16>().unwrap();

    for ap in results {
        info!(
            "SSID: {}, Signal: {:?}, Auth: {:?}",
            ap.ssid.as_str(),
            ap.signal_strength,
            ap.auth_method
        );
    }

    loop {
        let delay_start = Instant::now();
        while delay_start.elapsed() < Duration::from_millis(500) {}
    }
    
}
