//! Custom display implementation for ESP32-S3 LCD EV Board
//!
//! This module implements a custom display management system following the
//! slint-esp-workshop pattern, providing direct hardware control without
//! relying on MCU board support.

use log::info;
use embassy_time::Duration;
use alloc::boxed::Box;
use esp_hal::{
    gpio::{Level, OutputConfig, Output},
    i2c::master::Config as I2cConfig,
    time::Rate,
    dma::{DmaTxBuf, CHUNK_SIZE, DmaDescriptor},
    Blocking,
};
use esp_hal::delay::Delay;
use esp_hal::lcd_cam::{
    LcdCam,
    lcd::{
        ClockMode, Phase, Polarity,
        dpi::{Config as DpiConfig, Dpi, Format, FrameTiming},
    },
};
use slint::platform::software_renderer::{LineBufferProvider, Rgb565Pixel};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;
use esp_hal::i2c::master::I2c;

// Display configuration constants
const LCD_H_RES: u16 = 480;
const LCD_V_RES: u16 = 480;
const FRAME_BYTES: usize = (LCD_H_RES as usize * LCD_V_RES as usize) * 2;
const FRAME_PIXELS: usize = (LCD_H_RES as usize) * (LCD_V_RES as usize);
const NUM_DMA_DESC: usize = (FRAME_BYTES + CHUNK_SIZE - 1) / CHUNK_SIZE;

#[unsafe(link_section = ".dma")]
static mut TX_DESCRIPTORS: [DmaDescriptor; NUM_DMA_DESC] = [DmaDescriptor::EMPTY; NUM_DMA_DESC];

// Alias for Slint's Rgb565Pixel to match embedded-graphics
type EspRgb565Pixel = Rgb565Pixel;

// I2C expander (TCA9554) for display control
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

// Global display components container for safe access across embassy tasks
pub static DISPLAY_HARDWARE: Mutex<CriticalSectionRawMutex, Option<DisplayHardware<'static>>> = Mutex::new(None);

/// Display hardware components
pub struct DisplayHardware<'a> {
    pub dpi: Dpi<'a, esp_hal::Blocking>,
    pub dma_tx: DmaTxBuf,
    pub pixel_buf: &'static mut [EspRgb565Pixel],
    pub expander: Tca9554,
    pub delay: Delay,
}

/// Custom hardware draw buffer for Slint renderer
pub struct HardwareDrawBuffer<'a> {
    pub pixel_buf: &'a mut [EspRgb565Pixel],
}

impl<'a> HardwareDrawBuffer<'a> {
    pub fn new(pixel_buf: &'a mut [EspRgb565Pixel]) -> Self {
        Self { pixel_buf }
    }
}

impl LineBufferProvider for &mut HardwareDrawBuffer<'_> {
    type TargetPixel = slint::platform::software_renderer::Rgb565Pixel;

    fn process_line(
        &mut self,
        line: usize,
        range: core::ops::Range<usize>,
        render_fn: impl FnOnce(&mut [Self::TargetPixel]),
    ) {
        // Copy pixels from Slint buffer to our display buffer
        if line * LCD_H_RES as usize + range.len() <= self.pixel_buf.len() {
            let start_idx = line * LCD_H_RES as usize + range.start;
            let slice = &mut self.pixel_buf[start_idx..start_idx + range.len()];
            render_fn(slice);
        }
    }
}

/// Initialize display hardware and return components
pub fn init_display_hardware(
    peripherals: esp_hal::peripherals::Peripherals,
) -> Result<DisplayHardware<'static>, Box<dyn core::error::Error>> {
    info!("Initializing display hardware...");

    // Setup I2C for the TCA9554 IO expander
    let i2c = I2c::new(
        peripherals.I2C0,
        I2cConfig::default().with_frequency(Rate::from_khz(400)),
    )?
    .with_sda(peripherals.GPIO47)
    .with_scl(peripherals.GPIO48);

    // Initialize the IO expander for controlling the display
    let mut expander = Tca9554::new(i2c);
    expander.write_output_reg(0b1111_0011)?;
    expander.write_direction_reg(0b1111_0001)?;

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
    let mut vsync_pin = peripherals.GPIO3;
    let vsync_guard = Output::new(vsync_pin.reborrow(), Level::High, OutputConfig::default());

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
    drop(vsync_guard);

    // Set up DMA channel for LCD
    let tx_channel = peripherals.DMA_CH2;
    let lcd_cam = LcdCam::new(peripherals.LCD_CAM);

    // Configure the RGB display using the ESP32-S3 LCD EV board specific configuration
    let config = DpiConfig::default()
        .with_clock_mode(ClockMode { polarity: Polarity::IdleLow, phase: Phase::ShiftLow })
        .with_frequency(Rate::from_mhz(10))
        .with_format(Format { enable_2byte_mode: true, ..Default::default() })
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

    // Create DPI for display
    let mut vsync_pin = peripherals.GPIO3;
    let mut dpi = Dpi::new(lcd_cam.lcd, tx_channel, config)
        .unwrap()
        .with_vsync(vsync_pin.reborrow())
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

    // Allocate PSRAM-backed DMA buffer for the frame
    let buf_box: Box<[u8; FRAME_BYTES]> = Box::new([0; FRAME_BYTES]);
    let psram_buf: &'static mut [u8] = Box::leak(buf_box);
    let mut dma_tx: DmaTxBuf = unsafe {
        let descriptors = &mut *core::ptr::addr_of_mut!(TX_DESCRIPTORS);
        DmaTxBuf::new(descriptors, psram_buf)?
    };
    let mut pixel_box: Box<[EspRgb565Pixel; FRAME_PIXELS]> = Box::new([Rgb565Pixel(0); FRAME_PIXELS]);
    let pixel_buf: &'static mut [EspRgb565Pixel] = Box::leak(pixel_box);

    // Initialize pixel buffer and DMA buffer
    let dst = dma_tx.as_mut_slice();
    for (i, px) in pixel_buf.iter().enumerate() {
        let bytes = px.0.to_le_bytes();
        dst[2 * i] = bytes[0];
        dst[2 * i + 1] = bytes[1];
    }

    // Initial flush of the screen buffer
    let (dpi, dma_tx) = match dpi.send(false, dma_tx) {
        Ok(xfer) => {
            let (_res, dpi2, tx2) = xfer.wait();
            (dpi2, tx2)
        }
        Err((e, dpi2, tx2)) => {
            log::error!("Initial DMA send error: {:?}", e);
            (dpi2, tx2)
        }
    };

    // Store in global container
    let display_hardware = DisplayHardware {
        dpi,
        dma_tx,
        pixel_buf,
        expander,
        delay,
    };

    Ok(display_hardware)
}

/// Update display with current frame buffer content
pub fn update_display() -> Result<(), Box<dyn core::error::Error>> {
    let mut display_hardware_guard = DISPLAY_HARDWARE.lock(|hw| hw);
    if let Some(display_hardware) = display_hardware_guard.as_mut() {
        // Pack pixels into DMA buffer
        {
            let dst = display_hardware.dma_tx.as_mut_slice();
            for (i, px) in display_hardware.pixel_buf.iter().enumerate() {
                let bytes = px.0.to_le_bytes();
                dst[2 * i] = bytes[0];
                dst[2 * i + 1] = bytes[1];
            }
        }

        // DMA transfer of the full frame
        match display_hardware.dpi.send(false, display_hardware.dma_tx) {
            Ok(xfer) => {
                let (res, dpi_new, tx_new) = xfer.wait();
                *display_hardware_guard = Some(DisplayHardware {
                    dpi: dpi_new,
                    dma_tx: tx_new,
                    pixel_buf: display_hardware.pixel_buf,
                    expander: display_hardware.expander,
                    delay: display_hardware.delay,
                });
                if let Err(e) = res {
                    log::error!("DMA error: {:?}", e);
                }
            }
            Err((e, dpi_new, tx_new)) => {
                log::error!("DMA send error: {:?}", e);
                *display_hardware_guard = Some(DisplayHardware {
                    dpi: dpi_new,
                    dma_tx: tx_new,
                    pixel_buf: display_hardware.pixel_buf,
                    expander: display_hardware.expander,
                    delay: display_hardware.delay,
                });
            }
        }
    }
    Ok(())
}