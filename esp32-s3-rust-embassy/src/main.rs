#![no_std]
#![no_main]

extern crate alloc;

// Add ESP-IDF App Descriptor - required for flashing
esp_bootloader_esp_idf::esp_app_desc!();

use embassy_executor::Spawner;

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}
use embassy_time::{Duration, Timer};
use esp_alloc as _;
use esp_hal::{clock::CpuClock, rng::Rng, timer::timg::TimerGroup};
use esp_radio::wifi::{ClientConfig, ModeConfig, ScanConfig, WifiController};
use log::info;

macro_rules! mk_static {
    ($t:ty, $val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[allow(unsafe_code, unused_unsafe)]
        unsafe {
            STATIC_CELL.init_with(|| $val)
        }
    }};
}

#[esp_rtos::main]
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
    let _rng = Rng::new();
    info!("RNG created");

    // Initialize Wi-Fi controller with esp-rtos
    esp_rtos::start(timg0.timer0);

    info!("Initializing WiFi...");
    let radio_init = &*mk_static!(
        esp_radio::Controller<'static>,
        esp_radio::init().expect("Failed to initialize Wi-Fi/BLE controller")
    );
    info!("WiFi controller initialized");

    let (controller, _interfaces) =
        esp_radio::wifi::new(radio_init, peripherals.WIFI, Default::default())
            .expect("Failed to initialize Wi-Fi controller");
    info!("WiFi interface created");

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

    // Set WiFi to Station mode for scanning
    let client_config = ModeConfig::Client(ClientConfig::default());
    if let Err(e) = controller.set_config(&client_config) {
        info!("Failed to set WiFi config: {:?}", e);
        return;
    }

    // Start WiFi
    info!("Starting WiFi...");
    if let Err(e) = controller.start_async().await {
        info!("Failed to start WiFi: {:?}", e);
        return;
    }
    info!("WiFi started!");

    loop {
        info!("Performing WiFi scan...");

        // Perform a scan for up to 16 networks
        let scan_config = ScanConfig::default().with_max(16);
        match controller.scan_with_config_async(scan_config).await {
            Ok(results) => {
                info!("Found {} networks:", results.len());
                for (i, ap) in results.iter().enumerate() {
                    info!(
                        "  {}: SSID: {}, Signal: {}, Auth: {:?}, Channel: {}",
                        i + 1,
                        ap.ssid,
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
