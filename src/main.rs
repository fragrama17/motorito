#![no_std]
#![no_main]

use core::fmt::Write;

use defmt::*;
use embassy_executor::Spawner;
use embassy_futures::join::{join4};
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::i2c;
use embassy_rp::i2c::{Blocking, I2c};
use embassy_rp::peripherals::I2C1;
use embassy_rp::pwm::{Config, Pwm, SetDutyCycle};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_time::Timer;
use embassy_sync::channel::{Channel};

use portable_atomic::{AtomicU32, Ordering};

use embedded_graphics::{
    pixelcolor::BinaryColor,
    prelude::*,
};
use embedded_graphics::mono_font::ascii::{FONT_8X13};
use embedded_graphics::mono_font::MonoTextStyleBuilder;
use embedded_graphics::text::{Baseline, Text};

use ssd1306::{I2CDisplayInterface, prelude::*, Ssd1306};
use ssd1306::mode::BufferedGraphicsMode;
use ssd1306::prelude::I2CInterface;
use ssd1306::rotation::DisplayRotation;
use ssd1306::size::{DisplaySize128x64};

use heapless::String;

use {defmt_rtt as _, panic_probe as _};

const MIN_DUTY_CYCLE: u8 = 20;
const MAX_DUTY_CYCLE: u8 = 80;
const BTN_DEBOUNCE_MS: u64 = 150;

// thread-safe acceleration delay shared variable
static ATOMIC_ACCELERATION_DELAY_MS: AtomicU32 = AtomicU32::new(50);

// Create a global channel with capacity 1 (latest value only)
static DELAY_CHANNEL: Channel<ThreadModeRawMutex, u32, 1> = Channel::new();

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let i2c_config = i2c::Config::default();

    let i2c = embassy_rp::i2c::I2c::new_blocking(
        p.I2C1,
        p.PIN_27,
        p.PIN_26,
        i2c_config,
    );

    let interface = I2CDisplayInterface::new(i2c);

    let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();

    display.init().unwrap();

    Text::with_baseline("Program\nStarted !", Point::new(32, 40),
                        MonoTextStyleBuilder::new()
                            .font(&FONT_8X13)
                            .text_color(BinaryColor::On)
                            .build(), Baseline::Bottom)
        .draw(&mut display)
        .unwrap();

    Text::with_baseline(delay_to_string(50).as_str(), Point::new(0, 0),
                        MonoTextStyleBuilder::new()
                            .font(&FONT_8X13)
                            .text_color(BinaryColor::On)
                            .build(), Baseline::Top)
        .draw(&mut display)
        .unwrap();

    display.flush().unwrap();

    let en = Output::new(p.PIN_17, Level::Low);

    let desired_freq_hz = 50_000;
    let clock_freq_hz = embassy_rp::clocks::clk_sys_freq();
    let divider = 16u8;
    let period = (clock_freq_hz / (desired_freq_hz * divider as u32)) as u16 - 1;

    let mut c = Config::default();
    c.top = period;
    c.divider = divider.into();

    info!("pwm channel pico clock [MHz]: {}, frequency [KHz]: {}, period: {}", clock_freq_hz / 1_000_000, desired_freq_hz / 1_000, period);

    let out1 = Pwm::new_output_a(p.PWM_SLICE0, p.PIN_16, c.clone());

    let increase_btn = Input::new(p.PIN_19, Pull::Down);
    let decrease_btn = Input::new(p.PIN_18, Pull::Down);

    info!("pwm channel successfully initialised !");

    let a = accelerate(en, out1);
    let up = button_control(increase_btn, true, "UP");
    let down = button_control(decrease_btn, false, "DOWN");
    let render = render(display);

    join4(a, up, down, render).await;
}

const SEGMENT: u32 = 10u32;

async fn render(mut display: Ssd1306<I2CInterface<I2c<'static, I2C1, Blocking>>, DisplaySize128x64, BufferedGraphicsMode<DisplaySize128x64>>) {
    let mut curr_delay: u32;
    loop {
        display.clear(BinaryColor::Off).unwrap();

        Text::with_baseline("Button Pressed !", Point::new(0, 48),
                            MonoTextStyleBuilder::new()
                                .font(&FONT_8X13)
                                .text_color(BinaryColor::On)
                                .build(), Baseline::Bottom)
            .draw(&mut display)
            .unwrap();

        curr_delay = DELAY_CHANNEL.receive().await;

        Text::with_baseline(delay_to_string(curr_delay).as_str(), Point::new(0, 0),
                            MonoTextStyleBuilder::new()
                                .font(&FONT_8X13)
                                .text_color(BinaryColor::On)
                                .build(), Baseline::Top)
            .draw(&mut display)
            .unwrap();

        display.flush().unwrap();
    }
}

async fn button_control(mut button: Input<'static>, up: bool, button_name: &str) {
    let mut c = 0;
    loop {
        button.wait_for_high().await;

        c += 1;

        info!("button {} pressed {} times", button_name, c);

        let curr = ATOMIC_ACCELERATION_DELAY_MS.load(Ordering::Relaxed);

        if curr == 10 && up { // do not set delay lower than 10ms
            button.wait_for_low().await;
            continue;
        }

        if up {
            ATOMIC_ACCELERATION_DELAY_MS.fetch_sub(SEGMENT, Ordering::Relaxed);
        } else {
            ATOMIC_ACCELERATION_DELAY_MS.fetch_add(SEGMENT, Ordering::Relaxed);
        }

        let new_value = ATOMIC_ACCELERATION_DELAY_MS.load(Ordering::Relaxed);
        info!("acceleration delay changed to {}", new_value);
        DELAY_CHANNEL.send(new_value).await;

        Timer::after_millis(BTN_DEBOUNCE_MS).await;

        button.wait_for_low().await;
    }
}

async fn accelerate(mut enable: Output<'static>, mut output_channel: Pwm<'static>) {
    enable.set_high();

    info!("motor enabled !");

    let mut duty_cycle = MIN_DUTY_CYCLE;
    loop {
        info!("accelerating...");

        output_channel.set_duty_cycle_percent(duty_cycle).unwrap();

        // ACCELERATE
        while duty_cycle < MAX_DUTY_CYCLE {
            duty_cycle += 1;

            output_channel.set_duty_cycle_percent(duty_cycle).unwrap();

            let curr = ATOMIC_ACCELERATION_DELAY_MS.load(Ordering::Relaxed);
            Timer::after_millis(curr as u64).await;
            info!("duty cycle: {}", duty_cycle);
        }

        Timer::after_secs(1).await;

        info!("decelerating...");

        // DECELERATE
        while duty_cycle > MIN_DUTY_CYCLE {
            duty_cycle -= 1;

            output_channel.set_duty_cycle_percent(duty_cycle).unwrap();

            let curr = ATOMIC_ACCELERATION_DELAY_MS.load(Ordering::Relaxed);
            Timer::after_millis(curr as u64).await;
            info!("duty cycle: {}", duty_cycle);
        }

        Timer::after_secs(1).await;

        output_channel.set_duty_cycle_fully_off().unwrap();
    }
}

pub fn delay_to_string(val: u32) -> String<50> {
    let mut buf = String::<50>::new();
    core::write!(buf, "Delay [ms]: {}", val).unwrap();
    buf
}

pub fn duty_cycle_to_string(val: u8) -> String<50> {
    let mut buf = String::<50>::new();
    core::write!(buf, "Duty Cycle: {}", val).unwrap();
    buf
}

