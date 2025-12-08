#![no_std]
#![no_main]

extern crate alloc;

// ESP-IDF App Descriptor required by newer espflash
esp_bootloader_esp_idf::esp_app_desc!();

// Module declarations
mod wifi;

// Include Slint UI
slint::include_modules!();

use alloc::boxed::Box;
use alloc::rc::Rc;
use alloc::string::String;
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Ticker};

// ESP-HAL imports
use esp_hal::clock::CpuClock::_240MHz;
use esp_hal::dma::{CHUNK_SIZE, DmaDescriptor, DmaTxBuf};
use esp_hal::interrupt::software::SoftwareInterruptControl;
use esp_hal::lcd_cam::{
    LcdCam,
    lcd::{
        ClockMode, Phase, Polarity,
        dpi::{Config, Dpi, Format, FrameTiming},
    },
};
use esp_hal::{
    Blocking,
    delay::Delay,
    gpio::{Level, Output, OutputConfig},
    i2c::master::{Config as I2cConfig, I2c},
    system::Stack,
    time::Rate,
    timer::{AnyTimer, timg::TimerGroup},
};
use esp_println::logger::init_logger_from_env;
use log::{error, info, warn};

// Slint platform integration imports
use core::cell::RefCell;
use slint::platform::software_renderer::Rgb565Pixel;

// Import our custom modules
use crate::wifi::WifiScanner;

// Embassy multicore: allocate app core stack
static APP_CORE_STACK: static_cell::StaticCell<Stack<8192>> = static_cell::StaticCell::new();

// --- DISPLAY CONFIGURATION ---
pub const LCD_H_RES: u16 = 480;
pub const LCD_V_RES: u16 = 480;
pub const FRAME_BYTES: usize = (LCD_H_RES as usize * LCD_V_RES as usize) * 2;
pub const FRAME_PIXELS: usize = (LCD_H_RES as usize) * (LCD_V_RES as usize);
pub const NUM_DMA_DESC: usize = (FRAME_BYTES + CHUNK_SIZE - 1) / CHUNK_SIZE;

#[unsafe(link_section = ".dma")]
static mut TX_DESCRIPTORS: [DmaDescriptor; NUM_DMA_DESC] = [DmaDescriptor::EMPTY; NUM_DMA_DESC];

// --- I2C expander (TCA9554) ---
struct Tca9554 {
    i2c: I2c<'static, esp_hal::Blocking>,
    address: u8,
}

impl Tca9554 {
    pub fn new(i2c: I2c<'static, esp_hal::Blocking>) -> Self {
        Self { i2c, address: 0x20 }
    }
    pub fn write_direction_reg(&mut self, value: u8) -> Result<(), esp_hal::i2c::master::Error> {
        self.i2c.write(self.address, &[0x03, value])
    }
    pub fn write_output_reg(&mut self, value: u8) -> Result<(), esp_hal::i2c::master::Error> {
        self.i2c.write(self.address, &[0x01, value])
    }
}

// Display initialization commands for the ESP32-S3-LCD-EV-Board
#[derive(Copy, Clone, Debug)]
pub enum InitCmd {
    Cmd(u8, &'static [u8]),
    Delay(u8),
}

const INIT_CMDS: &[InitCmd] = &[
    InitCmd::Cmd(0xf0, &[0x55, 0xaa, 0x52, 0x08, 0x00]),
    InitCmd::Cmd(0xf6, &[0x5a, 0x87]),
    InitCmd::Cmd(0xc1, &[0x3f]),
    InitCmd::Cmd(0xc2, &[0x0e]),
    InitCmd::Cmd(0xc6, &[0xf8]),
    InitCmd::Cmd(0xc9, &[0x10]),
    InitCmd::Cmd(0xcd, &[0x25]),
    InitCmd::Cmd(0xf8, &[0x8a]),
    InitCmd::Cmd(0xac, &[0x45]),
    InitCmd::Cmd(0xa0, &[0xdd]),
    InitCmd::Cmd(0xa7, &[0x47]),
    InitCmd::Cmd(0xfa, &[0x00, 0x00, 0x00, 0x04]),
    InitCmd::Cmd(0x86, &[0x99, 0xa3, 0xa3, 0x51]),
    InitCmd::Cmd(0xa3, &[0xee]),
    InitCmd::Cmd(0xfd, &[0x3c, 0x3]),
    InitCmd::Cmd(0x71, &[0x48]),
    InitCmd::Cmd(0x72, &[0x48]),
    InitCmd::Cmd(0x73, &[0x00, 0x44]),
    InitCmd::Cmd(0x97, &[0xee]),
    InitCmd::Cmd(0x83, &[0x93]),
    InitCmd::Cmd(0x9a, &[0x72]),
    InitCmd::Cmd(0x9b, &[0x5a]),
    InitCmd::Cmd(0x82, &[0x2c, 0x2c]),
    InitCmd::Cmd(0xB1, &[0x10]),
    InitCmd::Cmd(
        0x6d,
        &[
            0x00, 0x1f, 0x19, 0x1a, 0x10, 0x0e, 0x0c, 0x0a, 0x02, 0x07, 0x1e, 0x1e, 0x1e, 0x1e,
            0x1e, 0x1e, 0x1e, 0x1e, 0x1e, 0x1e, 0x1e, 0x1e, 0x08, 0x01, 0x09, 0x0b, 0x0d, 0x0f,
            0x1a, 0x19, 0x1f, 0x00,
        ],
    ),
    InitCmd::Cmd(
        0x64,
        &[
            0x38, 0x05, 0x01, 0xdb, 0x03, 0x03, 0x38, 0x04, 0x01, 0xdc, 0x03, 0x03, 0x7a, 0x7a,
            0x7a, 0x7a,
        ],
    ),
    InitCmd::Cmd(
        0x65,
        &[
            0x38, 0x03, 0x01, 0xdd, 0x03, 0x03, 0x38, 0x02, 0x01, 0xde, 0x03, 0x03, 0x7a, 0x7a,
            0x7a, 0x7a,
        ],
    ),
    InitCmd::Cmd(
        0x66,
        &[
            0x38, 0x01, 0x01, 0xdf, 0x03, 0x03, 0x38, 0x00, 0x01, 0xe0, 0x03, 0x03, 0x7a, 0x7a,
            0x7a, 0x7a,
        ],
    ),
    InitCmd::Cmd(
        0x67,
        &[
            0x30, 0x01, 0x01, 0xe1, 0x03, 0x03, 0x30, 0x02, 0x01, 0xe2, 0x03, 0x03, 0x7a, 0x7a,
            0x7a, 0x7a,
        ],
    ),
    InitCmd::Cmd(
        0x68,
        &[
            0x00, 0x08, 0x15, 0x08, 0x15, 0x7a, 0x7a, 0x08, 0x15, 0x08, 0x15, 0x7a, 0x7a,
        ],
    ),
    InitCmd::Cmd(0x60, &[0x38, 0x08, 0x7a, 0x7a, 0x38, 0x09, 0x7a, 0x7a]),
    InitCmd::Cmd(0x63, &[0x31, 0xe4, 0x7a, 0x7a, 0x31, 0xe5, 0x7a, 0x7a]),
    InitCmd::Cmd(0x69, &[0x04, 0x22, 0x14, 0x22, 0x14, 0x22, 0x08]),
    InitCmd::Cmd(0x6b, &[0x07]),
    InitCmd::Cmd(0x7a, &[0x08, 0x13]),
    InitCmd::Cmd(0x7b, &[0x08, 0x13]),
    InitCmd::Cmd(
        0xd1,
        &[
            0x00, 0x00, 0x00, 0x04, 0x00, 0x12, 0x00, 0x18, 0x00, 0x21, 0x00, 0x2a, 0x00, 0x35,
            0x00, 0x47, 0x00, 0x56, 0x00, 0x90, 0x00, 0xe5, 0x01, 0x68, 0x01, 0xd5, 0x01, 0xd7,
            0x02, 0x36, 0x02, 0xa6, 0x02, 0xee, 0x03, 0x48, 0x03, 0xa0, 0x03, 0xba, 0x03, 0xc5,
            0x03, 0xd0, 0x03, 0xe0, 0x03, 0xea, 0x03, 0xfa, 0x03, 0xff,
        ],
    ),
    InitCmd::Cmd(0x36, &[0x00]),
    InitCmd::Cmd(0x2A, &[0x00, 0x00, 0x01, 0xDF]), // 0 to 479 (0x1DF)
    InitCmd::Cmd(0x2B, &[0x00, 0x00, 0x01, 0xDF]), // 0 to 479 (0x1DF)
    InitCmd::Cmd(0x3A, &[0x66]),
    InitCmd::Cmd(0x11, &[]),
    InitCmd::Delay(120),
    InitCmd::Cmd(0x29, &[]),
    InitCmd::Delay(20),
];

// Shared state between cores
static WIFI_RESULTS: Signal<
    CriticalSectionRawMutex,
    Option<heapless::Vec<crate::wifi::NetworkInfo, 10>>,
> = Signal::new();
static UI_UPDATE_REQUEST: Signal<CriticalSectionRawMutex, ()> = Signal::new();

// Custom Slint platform for embedded environment
struct EspBackend {
    window: RefCell<Option<Rc<slint::platform::software_renderer::MinimalSoftwareWindow>>>,
}

impl Default for EspBackend {
    fn default() -> Self {
        EspBackend {
            window: RefCell::new(None),
        }
    }
}

impl slint::platform::Platform for EspBackend {
    fn create_window_adapter(
        &self,
    ) -> Result<Rc<dyn slint::platform::WindowAdapter>, slint::PlatformError> {
        let window = slint::platform::software_renderer::MinimalSoftwareWindow::new(
            slint::platform::software_renderer::RepaintBufferType::ReusedBuffer,
        );
        self.window.replace(Some(window.clone()));
        Ok(window)
    }

    fn duration_since_start(&self) -> core::time::Duration {
        core::time::Duration::from_millis(
            esp_hal::time::Instant::now()
                .duration_since_epoch()
                .as_millis(),
        )
    }

    fn run_event_loop(&self) -> Result<(), slint::PlatformError> {
        // This method should not be called in our architecture
        // The display is managed by the main loop directly
        panic!("run_event_loop should not be called in this architecture");
    }
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    error!("Panic: {}", _info);
    loop {}
}

// Core 0: Display streaming task - following Conway's game_logic_task pattern
#[embassy_executor::task]
async fn display_streaming_task(
    mut dpi: Dpi<'static, esp_hal::Blocking>,
    mut dma_tx: DmaTxBuf,
    pixel_buf: &'static mut [Rgb565Pixel],
    window: Rc<slint::platform::software_renderer::MinimalSoftwareWindow>,
    ui: WifiListWindow,
) {
    info!("[CORE 0] Display streaming task started - following Conway pattern");

    let mut frame_count: u32 = 0;
    let mut scan_count: u32 = 0;

    // Initialize pixel buffer with initial frame
    for pixel in pixel_buf.iter_mut() {
        *pixel = Rgb565Pixel(0); // Black background
    }

    // Copy initial frame to DMA buffer
    let dst = dma_tx.as_mut_slice();
    for (i, px) in pixel_buf.iter().enumerate() {
        let bytes = px.0.to_le_bytes();
        dst[2 * i] = bytes[0];
        dst[2 * i + 1] = bytes[1];
    }

    info!("[CORE 0] Starting continuous display streaming loop");

    // Main continuous streaming loop - like Conway's implementation
    loop {
        frame_count = frame_count.wrapping_add(1);

        // Update Slint timers and animations
        slint::platform::update_timers_and_animations();

        // Check for WiFi updates
        if UI_UPDATE_REQUEST.try_take().is_some() {
            if let Some(networks) = WIFI_RESULTS.try_take() {
                scan_count = scan_count.wrapping_add(1);

                if let Some(ref networks) = networks {
                    // Update Slint UI status
                    ui.set_status_text(
                        alloc::format!("Found {} networks (scan #{})", networks.len(), scan_count)
                            .into(),
                    );

                    // Convert networks to display strings for Slint
                    let mut ap_strings = alloc::vec::Vec::<slint::SharedString>::new();
                    for (i, network) in networks.iter().enumerate() {
                        let signal_bars = "▂▄▆█"
                            .chars()
                            .take(network.signal_strength_bars() as usize)
                            .collect::<String>();
                        let ap_string = alloc::format!(
                            "📶 {} ({}dBm {}) CH:{}",
                            network.ssid.as_str(),
                            network.rssi,
                            signal_bars,
                            network.channel
                        );
                        ap_strings.push(ap_string.into());

                        if i >= 9 {
                            break;
                        } // Limit to 10 networks for UI
                    }

                    if networks.is_empty() {
                        ui.set_status_text("Scanning for networks...".into());
                    }

                    ui.set_access_points(slint::ModelRc::from(ap_strings.as_slice()));
                    info!("[CORE 0] UI updated with {} networks", networks.len());
                } else {
                    ui.set_status_text("WiFi scan failed".into());
                }
            }
        }

        // Render the UI into Slint's software renderer buffer
        window.draw_if_needed(|renderer| {
            let _dirty = renderer.render(pixel_buf, LCD_H_RES as usize);
        });

        // Copy pixel buffer to DMA buffer for next transfer (while current DMA is happening)
        let dst = dma_tx.as_mut_slice();
        for (i, px) in pixel_buf.iter().enumerate() {
            let bytes = px.0.to_le_bytes();
            dst[2 * i] = bytes[0];
            dst[2 * i + 1] = bytes[1];
        }

        // Start DMA transfer immediately - like Conway's continuous streaming
        dma_tx.set_length(FRAME_BYTES);
        match dpi.send(false, dma_tx) {
            Ok(xfer) => {
                let (res, new_dpi, new_dma_tx) = xfer.wait();
                dpi = new_dpi;
                dma_tx = new_dma_tx;
                if let Err(e) = res {
                    error!("[CORE 0] DMA transfer error: {:?}", e);
                }
            }
            Err((e, new_dpi, new_dma_tx)) => {
                error!("[CORE 0] DMA send error: {:?}", e);
                dpi = new_dpi;
                dma_tx = new_dma_tx;
            }
        }

        // Log progress periodically
        if frame_count % 60 == 0 {
            info!(
                "[CORE 0] Frame: {}, WiFi scans: {}",
                frame_count, scan_count
            );
        }
    }
}

// Core 1: WiFi scanning task
#[embassy_executor::task]
async fn wifi_scanner_task(wifi_peripheral: esp_hal::peripherals::WIFI<'static>) -> ! {
    info!("[CORE 1] WiFi scanner task started");

    // Create and initialize WiFi scanner with the WiFi peripheral passed from main
    let mut wifi_scanner = match WifiScanner::new(wifi_peripheral) {
        Ok(scanner) => scanner,
        Err(e) => {
            error!("Failed to create WiFi scanner: {e:?}");
            loop {
                embassy_time::Timer::after(Duration::from_secs(5)).await;
                warn!("WiFi scanner initialization failed, retrying...");
            }
        }
    };

    if let Err(e) = wifi_scanner.init().await {
        error!("Failed to initialize WiFi scanner: {e:?}");
        loop {
            embassy_time::Timer::after(Duration::from_secs(5)).await;
            warn!("WiFi scanner initialization failed, retrying...");
        }
    }

    let mut scan_count = 0;
    let mut ticker = Ticker::every(Duration::from_secs(5)); // Scan every 5 seconds

    loop {
        info!("[CORE 1] Starting WiFi scan #{}", scan_count + 1);

        // Request UI to show scanning state
        let empty_networks: heapless::Vec<crate::wifi::NetworkInfo, 10> = heapless::Vec::new();
        WIFI_RESULTS.signal(Some(empty_networks));
        UI_UPDATE_REQUEST.signal(());

        embassy_time::Timer::after(Duration::from_millis(500)).await; // Give UI time to update

        // Perform WiFi scan
        match wifi_scanner.scan().await {
            Ok(networks) => {
                scan_count += 1;
                info!(
                    "[CORE 1] WiFi scan #{} found {} networks",
                    scan_count,
                    networks.len()
                );

                // Update shared state and signal UI
                WIFI_RESULTS.signal(Some(networks));
                UI_UPDATE_REQUEST.signal(());
            }
            Err(e) => {
                error!("[CORE 1] WiFi scan #{} failed: {:?}", scan_count + 1, e);

                // Update UI with error state
                WIFI_RESULTS.signal(None);
                UI_UPDATE_REQUEST.signal(());
            }
        }

        // Wait for next scan cycle
        ticker.next().await;
    }
}

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default().with_cpu_clock(_240MHz));

    // Initialize BOTH heap allocators - WiFi first in internal RAM, then PSRAM for GUI
    // Step 1: Initialize internal RAM heap for WiFi (must be first)
    esp_alloc::heap_allocator!(size: 180 * 1024);

    // Step 2: Initialize PSRAM heap for GUI and other data
    esp_alloc::psram_allocator!(peripherals.PSRAM, esp_hal::psram);

    init_logger_from_env();
    info!("Dual heap allocation initialized - internal RAM: 180KB, PSRAM: available");

    // Initialize Embassy timer
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let timer0: AnyTimer = timg0.timer0.into();
    esp_rtos::start(timer0);

    info!("Starting ESP32-S3 LCD EV Board WiFi Scanner with Multicore Slint UI");

    // Extract CPU_CTRL first (before moving peripherals)
    let cpu_ctrl = peripherals.CPU_CTRL;

    // Set up app core stack
    let app_core_stack = APP_CORE_STACK.init(Stack::new());

    // Initialize shared state
    WIFI_RESULTS.signal(None);
    UI_UPDATE_REQUEST.signal(());

    // Create and install the Slint backend
    let backend = EspBackend::default();
    slint::platform::set_platform(Box::new(backend)).expect("Slint platform already initialized");

    // Extract WiFi peripheral for the second core
    let wifi_peripheral = peripherals.WIFI;

    // Set up software interrupts for multicore communication before consuming peripherals
    let sw_ints = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);

    // Start second core with WiFi scanning task
    esp_rtos::start_second_core(
        cpu_ctrl,
        sw_ints.software_interrupt0,
        sw_ints.software_interrupt1,
        app_core_stack,
        move || {
            static EXECUTOR: static_cell::StaticCell<esp_rtos::embassy::Executor> =
                static_cell::StaticCell::new();
            let executor = EXECUTOR.init(esp_rtos::embassy::Executor::new());
            executor.run(|spawner| {
                spawner
                    .spawn(wifi_scanner_task(wifi_peripheral))
                    .expect("Failed to spawn WiFi scanner task");
            });
        },
    );

    info!("Multicore WiFi Scanner with Slint UI started successfully!");
    info!("- Core 0: Display and UI rendering (Slint software renderer)");
    info!("- Core 1: WiFi scanning and networking");

    // Core 0: Display setup and main loop
    info!("Initializing display hardware...");

    // Setup I2C for the TCA9554 IO expander
    let i2c = I2c::new(
        peripherals.I2C0,
        I2cConfig::default().with_frequency(Rate::from_khz(400)),
    )
    .unwrap()
    .with_sda(peripherals.GPIO47)
    .with_scl(peripherals.GPIO48);

    // Initialize the IO expander for controlling the display
    let mut expander = Tca9554::new(i2c);
    expander.write_output_reg(0b1111_0011).unwrap();
    expander.write_direction_reg(0b1111_0001).unwrap();

    let delay = Delay::new();
    info!("Initializing display...");

    // Set up the write_byte function for sending commands to the display
    let mut write_byte = |b: u8, is_cmd: bool| {
        const SCS_BIT: u8 = 0b0000_0010;
        const SCL_BIT: u8 = 0b0000_0100;
        const SDA_BIT: u8 = 0b0000_1000;

        let mut output = 0b1111_0001 & !SCS_BIT;
        expander.write_output_reg(output).unwrap();

        for bit in core::iter::once(!is_cmd).chain((0..8).map(|i| (b >> i) & 0b1 != 0).rev()) {
            let prev = output;
            if bit {
                output |= SDA_BIT;
            } else {
                output &= !SDA_BIT;
            }
            if prev != output {
                expander.write_output_reg(output).unwrap();
            }

            output &= !SCL_BIT;
            expander.write_output_reg(output).unwrap();

            output |= SCL_BIT;
            expander.write_output_reg(output).unwrap();
        }

        output &= !SCL_BIT;
        expander.write_output_reg(output).unwrap();

        output &= !SDA_BIT;
        expander.write_output_reg(output).unwrap();

        output |= SCS_BIT;
        expander.write_output_reg(output).unwrap();
    };

    // VSYNC must be high during initialization
    let vsync_pin = Output::new(peripherals.GPIO3, Level::High, OutputConfig::default());

    // Initialize the display by sending the initialization commands
    for &init in INIT_CMDS.iter() {
        match init {
            InitCmd::Cmd(cmd, args) => {
                write_byte(cmd, true);
                for &arg in args {
                    write_byte(arg, false);
                }
            }
            InitCmd::Delay(ms) => {
                delay.delay_millis(ms as _);
            }
        }
    }
    // vsync_pin is now available for Dpi usage

    // Set up DMA channel for LCD
    let tx_channel = peripherals.DMA_CH2;
    let lcd_cam = LcdCam::new(peripherals.LCD_CAM);

    // Configure the RGB display
    let config = Config::default()
        .with_clock_mode(ClockMode {
            polarity: Polarity::IdleLow,
            phase: Phase::ShiftLow,
        })
        .with_frequency(Rate::from_mhz(10))
        .with_format(Format {
            enable_2byte_mode: true,
            ..Default::default()
        })
        .with_timing(FrameTiming {
            horizontal_active_width: LCD_H_RES as usize,
            vertical_active_height: LCD_V_RES as usize,
            horizontal_total_width: 600,
            horizontal_blank_front_porch: 80,
            vertical_total_height: 600,
            vertical_blank_front_porch: 80,
            hsync_width: 10,
            vsync_width: 4,
            hsync_position: 10,
        })
        .with_vsync_idle_level(Level::High)
        .with_hsync_idle_level(Level::High)
        .with_de_idle_level(Level::Low)
        .with_disable_black_region(false);

    let mut dpi = Dpi::new(lcd_cam.lcd, tx_channel, config)
        .unwrap()
        .with_vsync(vsync_pin)
        .with_hsync(peripherals.GPIO46)
        .with_de(peripherals.GPIO17)
        .with_pclk(peripherals.GPIO9)
        .with_data0(peripherals.GPIO10)
        .with_data1(peripherals.GPIO11)
        .with_data2(peripherals.GPIO12)
        .with_data3(peripherals.GPIO13)
        .with_data4(peripherals.GPIO14)
        .with_data5(peripherals.GPIO21)
        .with_data6(peripherals.GPIO8)
        .with_data7(peripherals.GPIO18)
        .with_data8(peripherals.GPIO45)
        .with_data9(peripherals.GPIO38)
        .with_data10(peripherals.GPIO39)
        .with_data11(peripherals.GPIO40)
        .with_data12(peripherals.GPIO41)
        .with_data13(peripherals.GPIO42)
        .with_data14(peripherals.GPIO2)
        .with_data15(peripherals.GPIO1);

    info!("Display initialized, setting up frame buffers...");

    const FRAME_PIXELS: usize = (LCD_H_RES as usize) * (LCD_V_RES as usize);

    // Allocate a PSRAM-backed DMA buffer for the frame
    let buf_box: Box<[u8; FRAME_BYTES]> = Box::new([0; FRAME_BYTES]);
    let psram_buf: &'static mut [u8] = Box::leak(buf_box);
    let mut dma_tx: DmaTxBuf = unsafe {
        let descriptors = &mut *core::ptr::addr_of_mut!(TX_DESCRIPTORS);
        DmaTxBuf::new(descriptors, psram_buf).unwrap()
    };
    let pixel_box: Box<[Rgb565Pixel; FRAME_PIXELS]> = Box::new([Rgb565Pixel(0); FRAME_PIXELS]);
    let pixel_buf: &'static mut [Rgb565Pixel] = Box::leak(pixel_box);

    // Initialize pixel buffer and DMA buffer
    let dst = dma_tx.as_mut_slice();
    for (i, px) in pixel_buf.iter().enumerate() {
        let [lo, hi] = px.0.to_le_bytes();
        dst[2 * i] = lo;
        dst[2 * i + 1] = hi;
    }

    // Initial flush of the screen buffer
    match dpi.send(false, dma_tx) {
        Ok(xfer) => {
            let (_res, dpi2, tx2) = xfer.wait();
            dpi = dpi2;
            dma_tx = tx2;
        }
        Err((e, dpi2, tx2)) => {
            error!("Initial DMA send error: {:?}", e);
            dpi = dpi2;
            dma_tx = tx2;
        }
    }

    // Create the Slint window adapter
    let window = slint::platform::software_renderer::MinimalSoftwareWindow::new(
        slint::platform::software_renderer::RepaintBufferType::ReusedBuffer,
    );

    // Tell Slint the window dimensions match the DPI display resolution
    let size = slint::PhysicalSize::new(LCD_H_RES.into(), LCD_V_RES.into());
    window.set_size(size);

    // Create the Slint UI component
    let ui = WifiListWindow::new().unwrap();

    // Set up the connect callback
    ui.on_connect(|index| {
        info!("Connect requested to network index: {}", index);
        // Here you would implement actual WiFi connection logic
    });

    info!("Display initialized, starting Embassy display task");

    // Spawn display streaming task on main core (Core 0) - like Conway's game_logic_task
    spawner
        .spawn(display_streaming_task(dpi, dma_tx, pixel_buf, window, ui))
        .unwrap();

    info!("WiFi Scanner with Slint UI started successfully!");
    info!("- Core 0: Display streaming and UI rendering");
    info!("- Core 1: WiFi scanning and networking");

    // Main loop - should never exit
    loop {
        embassy_time::Timer::after(Duration::from_secs(10)).await;
        info!("Main loop heartbeat - Embassy system running");
    }
}
