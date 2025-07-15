#![no_std]
#![no_main]

use embassy_executor::Spawner;

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::{clock::CpuClock, rng::Rng, timer::timg::TimerGroup};
use esp_wifi::{
    EspWifiController,
    init,
    wifi::{ClientConfiguration, Configuration, WifiController},
};
use log::info;

extern crate alloc;
use alloc::string::String;

// When you are okay with using a nightly compiler it's better to use https://docs.rs/static_cell/2.1.0/static_cell/macro.make_static.html
macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) -> ! {
    esp_println::logger::init_logger_from_env();
    info!("Logger initialized");
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    info!("Config created");
    let peripherals = esp_hal::init(config);
    info!("Peripherals initialized");

    esp_alloc::heap_allocator!(size: 72 * 1024);
    info!("Heap allocator initialized");

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    info!("Timer group created");
    let rng = Rng::new(peripherals.RNG);
    info!("RNG created");

    info!("Initializing WiFi...");
    let esp_wifi_ctrl = &*mk_static!(
        EspWifiController<'static>,
        init(timg0.timer0, rng.clone(), peripherals.RADIO_CLK).unwrap()
    );
    info!("WiFi controller initialized");

    let (controller, _interfaces) = esp_wifi::wifi::new(&esp_wifi_ctrl, peripherals.WIFI).unwrap();
    info!("WiFi interface created");

    // For ESP32-S3, we use SystemTimer
    use esp_hal::timer::systimer::SystemTimer;
    let systimer = SystemTimer::new(peripherals.SYSTIMER);
    esp_hal_embassy::init(systimer.alarm0);

    spawner.spawn(wifi_scan_task(controller)).ok();

    loop {
        info!("Main loop running...");
        Timer::after(Duration::from_millis(5000)).await;
    }
}

#[embassy_executor::task]
async fn wifi_scan_task(mut controller: WifiController<'static>) {
    info!("Starting WiFi scan task");
    info!("Device capabilities: {:?}", controller.capabilities());
    
    // Set up the WiFi controller in station mode
    let client_config = Configuration::Client(ClientConfiguration {
        ssid: String::new(),
        password: String::new(),
        ..Default::default()
    });
    
    controller.set_configuration(&client_config).unwrap();
    
    // Start WiFi
    info!("Starting WiFi...");
    controller.start_async().await.unwrap();
    info!("WiFi started!");

    loop {
        info!("Performing WiFi scan...");
        
        // Perform a scan for up to 16 networks
        match controller.scan_n_async(16).await {
            Ok(results) => {
                info!("Found {} networks:", results.len());
                for (i, ap) in results.iter().enumerate() {
                    info!(
                        "  {}: SSID: {}, Signal: {:?}, Auth: {:?}, Channel: {}",
                        i + 1,
                        ap.ssid.as_str(),
                        ap.signal_strength,
                        ap.auth_method,
                        ap.channel
                    );
                }
            }
            Err(e) => {
                info!("WiFi scan failed: {:?}", e);
            }
        }
        
        // Wait 10 seconds before next scan
        Timer::after(Duration::from_millis(10000)).await;
    }
}
