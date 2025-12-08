//! WiFi scanning module for ESP32-S3
//!
//! This module provides functionality to scan for WiFi networks
//! and expose them to the main application for display using esp-radio.

use embassy_time::Duration;
use esp_radio::wifi::{
    AccessPointInfo, ClientConfig, ModeConfig, ScanConfig, WifiController, WifiError,
};
use heapless::{String, Vec};
use log::{error, info, warn};

/// Information about a detected WiFi network
#[derive(Debug, Clone)]
pub struct NetworkInfo {
    /// Service Set Identifier (network name)
    pub ssid: heapless::String<32>,
    /// BSSID (MAC address)
    pub bssid: [u8; 6],
    /// Received Signal Strength Indicator in dBm
    pub rssi: i8,
    /// Authentication/security method
    pub auth_method: Option<String<32>>,
    /// Channel number
    pub channel: u8,
}

impl NetworkInfo {
    /// Create a new NetworkInfo from AccessPointInfo
    pub fn from_ap_info(ap_info: &AccessPointInfo) -> Option<Self> {
        // Try to convert the SSID to a heapless string
        let ssid = ap_info.ssid.as_str();
        if ssid.is_empty() {
            return None;
        }

        let ssid_result = heapless::String::try_from(ssid);
        match ssid_result {
            Ok(ssid_string) => {
                // For now, we'll use a simplified auth method string
                // TODO: Implement proper auth method mapping from esp-radio
                let auth_method = Some(String::<32>::try_from("WPA2").unwrap());

                Some(Self {
                    ssid: ssid_string,
                    bssid: ap_info.bssid,
                    rssi: ap_info.signal_strength,
                    auth_method,
                    channel: ap_info.channel,
                })
            }
            Err(_) => {
                warn!("Failed to create SSID string for network");
                None
            }
        }
    }

    /// Get signal strength as a percentage (0-100)
    pub fn signal_strength_percent(&self) -> u8 {
        // Convert RSSI (-100 to 0 dBm) to percentage (0-100%)
        let clamped_rssi = self.rssi.clamp(-100, 0);
        ((clamped_rssi + 100) * 100 / 100) as u8
    }

    /// Get signal strength as bars (1-5)
    pub fn signal_strength_bars(&self) -> u8 {
        match self.signal_strength_percent() {
            0..=20 => 1,
            21..=40 => 2,
            41..=60 => 3,
            61..=80 => 4,
            81..=100 => 5,
            _ => 0,
        }
    }

    /// Get display name for authentication mode
    pub fn auth_mode_name(&self) -> &str {
        match self.auth_method.as_ref() {
            Some(mode) => mode.as_str(),
            None => "Open",
        }
    }
}

/// Macro for creating static values using static_cell
macro_rules! mk_static {
    ($t:ty, $val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        unsafe { STATIC_CELL.init_with(|| $val) }
    }};
}

/// WiFi scanner structure
pub struct WifiScanner {
    controller: Option<WifiController<'static>>,
    initialized: bool,
}

impl WifiScanner {
    /// Create a new WiFi scanner
    pub fn new(wifi_peripheral: esp_hal::peripherals::WIFI<'static>) -> Result<Self, WifiError> {
        info!("Creating WiFi scanner with esp-radio");

        // Initialize esp-radio controller with static lifetime
        let esp_controller = mk_static!(
            esp_radio::Controller<'static>,
            esp_radio::init().expect("Failed to initialize Wi-Fi/BLE controller")
        );

        // Create WiFi controller with real WiFi peripheral
        match esp_radio::wifi::new(esp_controller, wifi_peripheral, Default::default()) {
            Ok((controller, _interfaces)) => {
                info!("WiFi controller created successfully");
                Ok(Self {
                    controller: Some(controller),
                    initialized: false,
                })
            }
            Err(e) => {
                error!("Failed to create WiFi controller: {:?}", e);
                Err(e)
            }
        }
    }

    /// Initialize WiFi controller for scanning
    pub async fn init(&mut self) -> Result<(), WifiError> {
        if self.controller.is_none() {
            return Err(WifiError::NotInitialized);
        }

        info!("Initializing WiFi scanner");

        // Get mutable reference to controller
        let controller = self.controller.as_mut().unwrap();

        // Set WiFi to Station mode
        let client_config = ModeConfig::Client(ClientConfig::default());
        controller.set_config(&client_config)?;

        // Start WiFi using async method
        match controller.start_async().await {
            Ok(_) => {
                info!("WiFi controller started successfully");

                // Wait for initialization
                embassy_time::Timer::after(Duration::from_millis(2000)).await;

                self.initialized = true;
                Ok(())
            }
            Err(e) => {
                error!("Failed to start WiFi controller: {:?}", e);
                Err(e)
            }
        }
    }

    /// Scan for available WiFi networks
    pub async fn scan(&mut self) -> Result<Vec<NetworkInfo, 10>, WifiError> {
        if !self.initialized || self.controller.is_none() {
            return Err(WifiError::NotInitialized);
        }

        info!("Starting WiFi scan");

        let controller = self.controller.as_mut().unwrap();

        // Perform async scan with configuration
        let scan_config = ScanConfig::default().with_max(10);
        let scan_results = match controller.scan_with_config_async(scan_config).await {
            Ok(results) => results,
            Err(e) => {
                error!("WiFi scan failed: {:?}", e);
                return Err(e);
            }
        };

        // Convert scan results to NetworkInfo
        let mut networks = Vec::<NetworkInfo, 10>::new();

        for ap in scan_results {
            if let Some(network) = NetworkInfo::from_ap_info(&ap) {
                if networks.push(network).is_err() {
                    warn!("Too many networks, skipping some");
                    break;
                }

                info!(
                    "Found: {} ({}dBm, CH: {})",
                    ap.ssid.as_str(),
                    ap.signal_strength,
                    ap.channel,
                );
            }
        }

        info!("Found {} networks", networks.len());
        Ok(networks)
    }
}
