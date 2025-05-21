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
use esp_hal::gpio::{Output, OutputConfig};
use alloc::boxed::Box;
use embedded_graphics::{
    Drawable,
    pixelcolor::Rgb565,
    prelude::*,
    primitives::{PrimitiveStyle, Rectangle},
};
use embedded_graphics_framebuf::FrameBuf;
use embedded_graphics_framebuf::backends::FrameBufferBackend;

use esp_hal::clock::CpuClock;
use esp_hal::main;
use esp_hal::timer::timg::TimerGroup;
use esp_hal::i2c::master::I2c;
use eeprom24x::{Eeprom24x, SlaveAddr};
use log::info;

// DMA line‐buffer for parallel RGB (1 descriptor, up to 4095 bytes each)
use esp_hal::dma::{DmaDescriptor, DmaTxBuf};
#[link_section = ".dma"]
static mut LINE_DESCRIPTOR: [DmaDescriptor; 1] = [DmaDescriptor::EMPTY; 1];
static mut LINE_BYTES: [u8; 320 * 2] = [0; 320 * 2];

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}

extern crate alloc;

/// A wrapper around a boxed array that implements FrameBufferBackend.
pub struct HeapBuffer<C: PixelColor, const N: usize>(Box<[C; N]>);

impl<C: PixelColor, const N: usize> HeapBuffer<C, N> {
    pub fn new(data: Box<[C; N]>) -> Self {
        Self(data)
    }

    pub fn as_slice(&self) -> &[C] {
        self.0.as_ref()
    }
}

impl<C: PixelColor, const N: usize> core::ops::Deref for HeapBuffer<C, N> {
    type Target = [C; N];
    fn deref(&self) -> &Self::Target {
        &*self.0
    }
}

impl<C: PixelColor, const N: usize> core::ops::DerefMut for HeapBuffer<C, N> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut *self.0
    }
}

impl<C: PixelColor, const N: usize> FrameBufferBackend for HeapBuffer<C, N> {
    type Color = C;
    fn set(&mut self, index: usize, color: Self::Color) {
        self.0[index] = color;
    }
    fn get(&self, index: usize) -> Self::Color {
        self.0[index]
    }
    fn nr_elements(&self) -> usize {
        N
    }
}

#[main]
fn main() -> ! {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);
    // esp_alloc::psram_allocator!(peripherals.PSRAM, esp_hal::psram);

    init_logger_from_env();
    esp_alloc::heap_allocator!(size: 72 * 1024);

    // Read display dimensions from EEPROM
    let mut i2c_bus = I2c::new(
        peripherals.I2C0,
        esp_hal::i2c::master::Config::default()
    )
    .unwrap()
    .with_sda(peripherals.GPIO1)
    .with_scl(peripherals.GPIO41);
    let mut eeid = [0u8; 0x1c];
    let mut eeprom = Eeprom24x::new_24x01(i2c_bus, SlaveAddr::default());
    eeprom.read_data(0x00, &mut eeid).unwrap();
    let display_width = u16::from_be_bytes([eeid[8], eeid[9]]) as usize;
    let display_height = u16::from_be_bytes([eeid[10], eeid[11]]) as usize;
    info!("Display size from EEPROM: {}x{}", display_width, display_height);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let _init = esp_wifi::init(
        timg0.timer0,
        esp_hal::rng::Rng::new(peripherals.RNG),
        peripherals.RADIO_CLK,
    )
    .unwrap();


    info!("Initializing display...");

    // Panel‐enable / backlight
    let mut panel_enable = Output::new(peripherals.GPIO42, Level::Low, OutputConfig::default());
    // some boards require LOW→HIGH pulse, or just HIGH
    panel_enable.set_high();


    // Initialize the parallel‐RGB display via DPI
    // let mut display = init_display(peripherals);
    // --- Initialize parallel‐RGB display via DPI inline ---
    // let mut delay = Delay::new(&peripherals.TIMG1);
    let lcd_cam = LcdCam::new(peripherals.LCD_CAM);

    let pclk_hz = ((eeid[12] as u32) * 1_000_000 + (eeid[13] as u32) * 100_000).min(13_600_000);
    let h_res = display_width;
    let v_res = display_height;
    let hsync_pulse = eeid[17] as u32;
    let hsync_back  = u16::from_be_bytes([eeid[15], eeid[16]]) as u32;
    let hsync_front = u16::from_be_bytes([eeid[18], eeid[19]]) as u32;
    let vsync_pulse = eeid[22] as usize;
    let vsync_back  = u16::from_be_bytes([eeid[20], eeid[21]]);
    let vsync_front = u16::from_be_bytes([eeid[23], eeid[24]]) as u32;

    let dpi_config = DpiConfig::default()
        .with_clock_mode(ClockMode {
            polarity: Polarity::IdleLow,
            phase:    Phase::ShiftLow,
        })
        .with_frequency(Rate::from_hz(pclk_hz))
        .with_format(Format {
            enable_2byte_mode: true,
            ..Default::default()
        })
        .with_timing(FrameTiming {
            horizontal_active_width:   h_res,
            vertical_active_height:    v_res,
            horizontal_total_width:    h_res + hsync_back as usize + hsync_front as usize,
            horizontal_blank_front_porch: hsync_front as usize,
            vertical_total_height:     v_res + vsync_back as usize + vsync_front as usize,
            vertical_blank_front_porch:   vsync_front as usize,
            hsync_width:               hsync_pulse as usize,
            vsync_width:               vsync_pulse,
            // start of HSYNC pulse relative to start of line = back porch
            hsync_position:            hsync_back as usize,
        })
        .with_vsync_idle_level(Level::High)
        .with_hsync_idle_level(Level::High)
        .with_de_idle_level(Level::Low)
        .with_disable_black_region(false);

    let mut dpi = Dpi::new(lcd_cam.lcd, peripherals.DMA_CH2, dpi_config).unwrap()
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

    static mut LINE_BUF: [Rgb565; 320] = [Rgb565::BLUE; 320];

    info!("Entering main loop...");
    // Prepare DMA transaction for line streaming
    let mut dma_tx: DmaTxBuf =
        unsafe { DmaTxBuf::new(&mut LINE_DESCRIPTOR, &mut LINE_BYTES).unwrap() };

    loop {
        info!("Drawing frame...");
        for y in 0..display_height {
            // Simple test pattern: alternating red/blue scanlines
            let color = if y % 2 == 0 { Rgb565::RED } else { Rgb565::BLUE };
            // Fill the line buffer
            for pixel in unsafe { &mut LINE_BUF } {
                *pixel = color;
            }
            // fill LINE_BYTES from LINE_BUF
            for x in 0..display_width {
                let color: Rgb565 = unsafe { LINE_BUF[x] };
                let [lo, hi] = color.into_storage().to_le_bytes();
                unsafe {
                    LINE_BYTES[2 * x] = lo;
                    LINE_BYTES[2 * x + 1] = hi;
                }
            }
            // send this scanline via DPI+DMA
            match dpi.send(false, dma_tx) {
                Ok(xfer) => {
                    let (res, new_dpi, new_tx) = xfer.wait();
                    dpi = new_dpi;
                    dma_tx = new_tx;
                    if let Err(e) = res {
                        info!("DMA error: {:?}", e);
                    }
                }
                Err((e, new_dpi, new_tx)) => {
                    info!("DMA send error: {:?}", e);
                    dpi = new_dpi;
                    dma_tx = new_tx;
                }
            }
        }
    }

    // Main loop to draw the entire image
    // loop {
    //
    //     // Pack entire frame into PSRAM DMA buffer
    //     let dst = dma_tx.as_mut_slice();
    //     for (i, px) in frame_buf.data.iter().enumerate() {
    //         let [lo, hi] = px.into_storage().to_le_bytes();
    //         dst[2 * i] = lo;
    //         dst[2 * i + 1] = hi;
    //     }
    //
    //     // One-shot transfer
    //     match dpi.send(false, dma_tx) {
    //         Ok(xfer) => {
    //             let (res, dpi2, buf2) = xfer.wait();
    //             dpi = dpi2;
    //             dma_tx = buf2;
    //             if let Err(e) = res {
    //                 error!("DMA error: {:?}", e);
    //             }
    //         }
    //         Err((e, dpi2, buf2)) => {
    //             error!("DMA send error: {:?}", e);
    //             dpi = dpi2;
    //             dma_tx = buf2;
    //         }
    //     }
    // }
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
