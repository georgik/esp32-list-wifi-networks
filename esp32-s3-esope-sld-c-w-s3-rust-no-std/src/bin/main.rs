#![no_std]
#![no_main]

use esp_hal::peripherals::*;
use mipidsi::Builder as DisplayBuilder;
use mipidsi::models::ST7789;
use esp_hal::gpio::*;
use embedded_hal_bus::spi::ExclusiveDevice;

use esp_hal::clock::CpuClock;
use esp_hal::main;
use esp_hal::spi::master::Spi;
use esp_hal::time::{Duration, Instant};
use esp_hal::timer::timg::TimerGroup;
use esp_println::println;

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}

extern crate alloc;

use slint::SharedString;
// use esope_board::init_display;
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;

#[main]
fn main() -> ! {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(size: 72 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let _init = esp_wifi::init(
        timg0.timer0,
        esp_hal::rng::Rng::new(peripherals.RNG),
        peripherals.RADIO_CLK,
    )
    .unwrap();

    // Init board/display
    let mut display = init_display(
        peripherals.GPIO,
        peripherals.I2C0,
        peripherals.SPI2,
        peripherals.DMA,
        peripherals.PIN_CTRL,
        peripherals.LCD_CAM,
        peripherals.RMT,
        peripherals.TIMG1,
    );

    // Show some dummy screen before Slint UI is ready
    display.clear(Rgb565::BLACK).unwrap();

    // Initialize Slint runtime
    slint::platform::init(Default::default()).unwrap();

    let main_window = wifi_list::MainWindow::new().unwrap();
    main_window.run().unwrap();

    loop {}
}

fn init_display(
    gpio: GPIO,
    i2c0: I2C0,
    spi2: SPI2,
    dma: DMA,
    pin_ctrl: PIN_CTRL,
    lcd_cam: LCD_CAM,
    rmt: RMT,
    timg1: TIMG1,
) -> mipidsi::Display<
    mipidsi::models::ST7789,
    SPIInterfaceNoCS<
        Spi<SPI2, (Gpio6<Output<PushPull>>, Gpio7<Output<PushPull>>, Gpio8<Output<PushPull>>)>,
        Gpio10<Output<PushPull>>
    >,
    Gpio9<Output<PushPull>>
> {
    let dc = Output::new(peripherals.GPIO10, Level::Low, OutputConfig::default());
    let rst = Output::new(peripherals.GPIO9, Level::Low, OutputConfig::default());

    let spi = Spi::<Blocking>::new(
        spi2,
        esp_hal::spi::master::Config::default()
            .with_frequency(Rate::from_mhz(40))
            .with_mode(esp_hal::spi::Mode::_0),
    )
        .unwrap()
        .with_sck(peripherals.GPIO6)
        .with_mosi(peripherals.GPIO7)
        .with_dma(dma.ch0);
    let cs_output = Output::new(peripherals.GPIO8, Level::High, OutputConfig::default());
    let spi_delay = esp_hal::delay::Delay::new(&timg1);
    let spi_device = ExclusiveDevice::new(spi, cs_output, spi_delay).unwrap();

    let di = SPIInterfaceNoCS::new(spi_device, dc);

    let mut delay = esp_hal::delay::Delay::new(&timg1);

    let mut display = DisplayBuilder::new(ST7789, di)
        .with_display_size(240, 320)
        .with_invert_colors(mipidsi::ColorInversion::Inverted)
        .init(&mut delay, Some(rst))
        .unwrap();

    display.set_orientation(mipidsi::Orientation::Portrait).unwrap();
    display
}
