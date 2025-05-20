use esp_idf_svc::wifi::*;
use esp_idf_svc::nvs::*;
use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_svc::log::EspLogger;

fn main() {
    esp_idf_svc::sys::link_patches();

    EspLogger::initialize_default();
    log::info!("Hello, WiFi!");

    let peripherals = Peripherals::take().unwrap();
    let sysloop = EspSystemEventLoop::take().unwrap();
    let nvs = EspDefaultNvsPartition::take().unwrap();

    let mut wifi = EspWifi::new(peripherals.modem, sysloop, Some(nvs)).unwrap();

    wifi.set_configuration(&Configuration::Client(ClientConfiguration::default())).unwrap();

    log::info!("Starting WiFi scan...");
    wifi.start().unwrap();
    let access_points = wifi.scan().unwrap();

    log::info!("Found {} access points:", access_points.len());
    for (i, ap) in access_points.iter().enumerate() {
        log::info!(
            "[{}] SSID: {}, Signal: {:?}, Auth: {:?}",
            i,
            ap.ssid.as_str(),
            ap.signal_strength,
            ap.auth_method
        );
    }
}