#![no_std]
#![no_main]

use esp_hal::peripherals::*;
use esp_hal::dma::ExternalBurstConfig;

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
use alloc::vec::Vec;
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
use esp_hal::dma::{DmaDescriptor, DmaTxBuf, CHUNK_SIZE};
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
    const MAX_FRAME_BYTES: usize = 320 * 240 * 2;
    const MAX_NUM_DMA_DESC: usize = (MAX_FRAME_BYTES + CHUNK_SIZE - 1) / CHUNK_SIZE;

    #[link_section = ".dma"]
    static mut TX_DESCRIPTORS: [DmaDescriptor; MAX_NUM_DMA_DESC] =
        [DmaDescriptor::EMPTY; MAX_NUM_DMA_DESC];

    // Allocate PSRAM buffer for entire frame at runtime
    let frame_bytes = display_width * display_height * 2;
    let mut psram_box = Vec::with_capacity(frame_bytes);
    psram_box.resize(frame_bytes, 0);
    let psram_buf: &'static mut [u8] = Box::leak(psram_box.into_boxed_slice());

    // Verify PSRAM buffer allocation and alignment
    let buf_ptr = psram_buf.as_ptr() as usize;
    info!("PSRAM buffer allocated at address: 0x{:08X}", buf_ptr);
    info!("PSRAM buffer length: {}", psram_buf.len());
    info!("PSRAM buffer alignment modulo 32: {}", buf_ptr % 32);
    assert!(buf_ptr % 64 == 0, "PSRAM buffer must be 64-byte aligned for DMA");

    info!("Initializing display...");

    // Panel‐enable / backlight
    let mut panel_enable = Output::new(peripherals.GPIO42, Level::Low, OutputConfig::default());
    // some boards require LOW→HIGH pulse, or just HIGH
    panel_enable.set_high();

    // Backlight enable (PWM output pin)
    let mut backlight = Output::new(peripherals.GPIO39, Level::Low, OutputConfig::default());
    backlight.set_high();

    // Touch reset line
    let mut touch_reset = Output::new(peripherals.GPIO2, Level::High, OutputConfig::default());
    // Optionally pulse low-high here if required by the module.


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

    // Read idle/polarity flags from EEPROM (eeid[25])
    let flags = eeid[25];
    let hsync_idle_low  = (flags & 0x01) != 0;
    let vsync_idle_low  = (flags & 0x02) != 0;
    let de_idle_high    = (flags & 0x04) != 0;
    let pclk_active_neg = (flags & 0x20) != 0;

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

    // Check timing configuration and set minimal values, with warning if the value was too low
    if hsync_pulse < 4 {
        error!("HSYNC pulse width is too low: {} pixels, setting to minimum 4 pixels", hsync_pulse);
    }

    if hsync_back_porch < 43 {
        error!("HSYNC back porch is too low: {} pixels, setting to minimum 43 pixels", hsync_back);
    }

    if hsync_front_porch < 8 {
        error!("HSYNC front porch is too low: {} pixels, setting to minimum 8 pixels", hsync_front);
    }

    if vsync_pulse < 4 {
        error!("VSYNC pulse width is too low: {} lines, setting to minimum 4 lines", vsync_pulse);
    }

    if vsync_back_porch < 12 {
        error!("VSYNC back porch is too low: {} lines, setting to minimum 12 lines", vsync_back);
    }

    if vsync_front_porch < 8 {
        error!("VSYNC front porch is too low: {} lines, setting to minimum 8 lines", vsync_front);
    }

    let dpi_config = DpiConfig::default()
        .with_clock_mode(ClockMode {
            polarity: if pclk_active_neg { Polarity::IdleHigh } else { Polarity::IdleLow },
            phase:    if pclk_active_neg { Phase::ShiftHigh } else { Phase::ShiftLow },
        })
        .with_frequency(Rate::from_hz(pclk_hz))
        .with_format(Format {
            enable_2byte_mode: true,
            ..Default::default()
        })
        .with_timing(FrameTiming {
            horizontal_active_width:      320,
            horizontal_total_width:       320 + 4 + 43 + 79 + 8,  // =446
            horizontal_blank_front_porch: 79 + 8,    // was 47, add 32px
            vertical_active_height:       240,
            vertical_total_height:        240 + 4 + 12 + 16,  // increased blank front porch to 16
            vertical_blank_front_porch:    16,
            hsync_width:                  4,
            vsync_width:                  4,
            hsync_position:               43 + 4, // (= back_porch + pulse = 47)
        })
        // apply idle levels based on EEPROM flags
        .with_vsync_idle_level(if vsync_idle_low { Level::Low } else { Level::High })
        .with_hsync_idle_level(if hsync_idle_low { Level::Low } else { Level::High })
        .with_de_idle_level(if de_idle_high { Level::High } else { Level::Low })
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

    // Draw gradient across full frame directly into PSRAM buffer
    for y in 0..display_height {
        let r = ((y * 31) / (display_height - 1)) as u8;
        let g = ((y * 63) / (display_height - 1)) as u8;
        let b = r;
        let color = Rgb565::new(r, g, b);
        for x in 0..display_width {
            let idx = (y * display_width + x) * 2;
            let [lo, hi] = color.into_storage().to_le_bytes();
            psram_buf[idx] = lo;
            psram_buf[idx + 1] = hi;
        }
    }

    info!("Entering main loop...");

    // Configure a single DMA buffer over the whole PSRAM region with 64‑byte bursts
    let mut dma_tx: DmaTxBuf = unsafe {
        DmaTxBuf::new_with_config(
            &mut TX_DESCRIPTORS,
            psram_buf,
            ExternalBurstConfig::Size64,
        ).unwrap()
    };

    loop {
        info!("Drawing full frame now...");

        // Chunked full-frame transfer
        let safe_chunk_size = 320 * 240 *2 ;

        let len = safe_chunk_size.min(frame_bytes );
        dma_tx.set_length(len);
        match dpi.send(false, dma_tx) {
            Ok(xfer) => {
                let (res, new_dpi, new_dma_tx) = xfer.wait();
                dpi = new_dpi;
                dma_tx = new_dma_tx;
                if let Err(e) = res {
                    error!("DMA transfer error: {:?}", e);
                }
            }
            Err((e, new_dpi, new_dma_tx)) => {
                error!("DMA send error: {:?}", e);
                dpi = new_dpi;
                dma_tx = new_dma_tx;
            }
        }
    }
}
