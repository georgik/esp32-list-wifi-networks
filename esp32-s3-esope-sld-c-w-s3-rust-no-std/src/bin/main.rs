#![no_std]
#![no_main]

use esp_hal::peripherals::*;
use esp_hal::gpio::Level;
use esp_hal::delay::Delay;
use esp_println::logger::init_logger_from_env;
use esp_hal::time::Rate;
use esp_hal::lcd_cam::{
    LcdCam,
    lcd::{
        ClockMode, Phase, Polarity,
        dpi::{Config as DpiConfig, Dpi, Format, FrameTiming},
    },
};
use embedded_graphics::pixelcolor::Rgb565;

use esp_hal::clock::CpuClock;
use esp_hal::main;
use esp_hal::timer::timg::TimerGroup;
use log::info;

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}

extern crate alloc;

// use slint::SharedString;

#[main]
fn main() -> ! {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    init_logger_from_env();
    esp_alloc::heap_allocator!(size: 72 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let _init = esp_wifi::init(
        timg0.timer0,
        esp_hal::rng::Rng::new(peripherals.RNG),
        peripherals.RADIO_CLK,
    )
    .unwrap();


    info!("Initializing display...");
    // Initialize the parallel‐RGB display via DPI
    // let mut display = init_display(peripherals);
    // --- Initialize parallel‐RGB display via DPI inline ---
    // let mut delay = Delay::new(&peripherals.TIMG1);
    let lcd_cam = LcdCam::new(peripherals.LCD_CAM);

    let dpi_config = DpiConfig::default()
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
            horizontal_active_width: 480,
            vertical_active_height: 480,
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

    let mut display = Dpi::new(lcd_cam.lcd, peripherals.DMA_CH2, dpi_config).unwrap()
        .with_vsync(peripherals.GPIO6)
        .with_hsync(peripherals.GPIO15)
        .with_de(peripherals.GPIO5)
        .with_pclk(peripherals.GPIO4)
        // Blue bus
        .with_data0(peripherals.GPIO9)
        .with_data1(peripherals.GPIO17)
        .with_data2(peripherals.GPIO46)
        .with_data3(peripherals.GPIO16)
        .with_data4(peripherals.GPIO7)
        // Green bus
        .with_data5(peripherals.GPIO8)
        .with_data6(peripherals.GPIO21)
        .with_data7(peripherals.GPIO3)
        .with_data8(peripherals.GPIO11)
        .with_data9(peripherals.GPIO18)
        .with_data10(peripherals.GPIO10)
        // Red bus
        .with_data11(peripherals.GPIO14)
        .with_data12(peripherals.GPIO20)
        .with_data13(peripherals.GPIO13)
        .with_data14(peripherals.GPIO19)
        .with_data15(peripherals.GPIO12);

    // display.clear(Rgb565::BLACK).unwrap();

    // Initialize Slint runtime
    // slint::platform::init(Default::default()).unwrap();

    // let main_window = wifi_list::MainWindow::new().unwrap();
    // main_window.run().unwrap();

    info!("Entering main loop...");
    loop {}
}

// fn init_display<'d>(peripherals: Peripherals) -> Dpi<
//     'd,
//     LcdCam<'d, esp_hal::peripherals::LCD_CAM>,
//     esp_hal::peripherals::DMA,
//     Rgb565,
// > {
//     // Delay for init
//     let mut delay = Delay::new(&peripherals.TIMG1);
//
//     // Create the LCD controller
//     let lcd = LcdCam::new(peripherals.LCD_CAM);
//
//     // Build DPI config
//     let config = DpiConfig::default()
//         .with_clock_mode(ClockMode {
//             polarity: Polarity::IdleLow,
//             phase: Phase::ShiftLow,
//         })
//         .with_frequency(Rate::from_mhz(10))
//         .with_format(Format {
//             enable_2byte_mode: true,
//             ..Default::default()
//         })
//         .with_timing(FrameTiming {
//             horizontal_active_width: 480,
//             vertical_active_height: 480,
//             horizontal_total_width: 600,
//             horizontal_blank_front_porch: 80,
//             vertical_total_height: 600,
//             vertical_blank_front_porch: 80,
//             hsync_width: 10,
//             vsync_width: 4,
//             hsync_position: 10,
//         })
//         .with_vsync_idle_level(Level::High)
//         .with_hsync_idle_level(Level::High)
//         .with_de_idle_level(Level::Low)
//         .with_disable_black_region(false);
//
//     // Initialize the DPI interface
//     let mut dpi = Dpi::new(lcd, peripherals.DMA_CH2, config).unwrap()
//         .with_vsync(peripherals.GPIO6)
//         .with_hsync(peripherals.GPIO15)
//         .with_de(peripherals.GPIO5)
//         .with_pclk(peripherals.GPIO4)
//         // Blue bus
//         .with_data0(peripherals.GPIO9)
//         .with_data1(peripherals.GPIO17)
//         .with_data2(peripherals.GPIO46)
//         .with_data3(peripherals.GPIO16)
//         .with_data4(peripherals.GPIO7)
//         // Green bus
//         .with_data5(peripherals.GPIO8)
//         .with_data6(peripherals.GPIO21)
//         .with_data7(peripherals.GPIO3)
//         .with_data8(peripherals.GPIO11)
//         .with_data9(peripherals.GPIO18)
//         .with_data10(peripherals.GPIO10)
//         // Red bus
//         .with_data11(peripherals.GPIO14)
//         .with_data12(peripherals.GPIO20)
//         .with_data13(peripherals.GPIO13)
//         .with_data14(peripherals.GPIO19)
//         .with_data15(peripherals.GPIO12);
//
//     dpi
// }
