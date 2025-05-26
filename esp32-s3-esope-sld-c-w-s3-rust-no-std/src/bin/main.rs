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
use log::{error, info};

// DMA line‐buffer for parallel RGB (1 descriptor, up to 4095 bytes each)
use esp_hal::dma::{DmaDescriptor, DmaTxBuf};
use esp_println::println;


#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    error!("Panic: {}", _info);
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
    println!("Starting up...");
    esp_alloc::psram_allocator!(peripherals.PSRAM, esp_hal::psram);
    println!("PSRAM allocator initialized");

    init_logger_from_env();
    // esp_alloc::heap_allocator!(size: 72 * 1024);

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

    // Full-screen DMA constants
    const CHUNK_SIZE: usize = 4095;
    const FRAME_BYTES: usize = 320 * 4 * 2;
    const NUM_DMA_DESC: usize = (FRAME_BYTES + CHUNK_SIZE - 1) / CHUNK_SIZE;

    #[link_section = ".dma"]
    static mut TX_DESCRIPTORS: [DmaDescriptor; NUM_DMA_DESC] =
        [DmaDescriptor::EMPTY; NUM_DMA_DESC];

    // Allocate PSRAM buffer for full frame
    let buf_box: Box<[u8; FRAME_BYTES]> = Box::new([0; FRAME_BYTES]);
    let psram_buf: &'static mut [u8] = Box::leak(buf_box);

    // Prepare full-frame DMA transaction
    let mut dma_tx: DmaTxBuf =
        unsafe { DmaTxBuf::new(&mut TX_DESCRIPTORS, psram_buf).unwrap() };

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

    // Log all infomration about display configuration
    info!("Display configuration:");
    info!("  Resolution: {}x{}", h_res, v_res);
    info!("  PCLK: {} Hz", pclk_hz);
    info!("  HSYNC pulse: {} pixels", hsync_pulse);
    info!("  HSYNC back porch: {} pixels", hsync_back);
    info!("  HSYNC front porch: {} pixels", hsync_front);
    info!("  VSYNC pulse: {} lines", vsync_pulse);
    info!("  VSYNC back porch: {} lines", vsync_back);
    info!("  VSYNC front porch: {} lines", vsync_front);


    let hsync_w = hsync_pulse as usize;
    let hsync_back_porch = hsync_back as usize;
    let hsync_front_porch = hsync_front as usize;
    let horizontal_total = h_res + hsync_w + hsync_back_porch + hsync_front_porch;

    let vsync_w = vsync_pulse as usize;
    let vsync_back_porch = vsync_back as usize;
    let vsync_front_porch = vsync_front as usize;
    let vertical_total = v_res + vsync_w + vsync_back_porch + vsync_front_porch;

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
            horizontal_total_width:    horizontal_total,
            horizontal_blank_front_porch: hsync_front_porch,
            vertical_total_height:     vertical_total,
            vertical_blank_front_porch:   vsync_front_porch,
            hsync_width:               hsync_w,
            vsync_width:               vsync_w,
            // start of HSYNC pulse relative to start of line = back porch + pulse width
            hsync_position:            hsync_back_porch + hsync_w,
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

    info!("Entering main loop...");
    loop {
        info!("Drawing 4 lines now...");
        // Fill each of the four lines with a distinct color
        // Line 0: Blue, Line 1: Red, Line 2: Green, Line 3: White
        static mut LINE_BUF: [Rgb565; 320 * 4] = [Rgb565::BLUE; 320 * 4];
        for i in 0..320 {
            unsafe {
                LINE_BUF[i]           = Rgb565::BLUE;
                LINE_BUF[320 + i]     = Rgb565::RED;
                LINE_BUF[2 * 320 + i] = Rgb565::GREEN;
                LINE_BUF[3 * 320 + i] = Rgb565::WHITE;
                // LINE_BUF[4 * 320 + i] = Rgb565::BLUE;
            }
        }
        // Pack 4-line buffer into PSRAM DMA buffer
        let dst = dma_tx.as_mut_slice();
        for (i, pixel) in unsafe { LINE_BUF.iter().enumerate() } {
            let [lo, hi] = pixel.into_storage().to_le_bytes();
            dst[2 * i]     = lo;
            dst[2 * i + 1] = hi;
        }

        // Send these lines via DPI+DMA
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
