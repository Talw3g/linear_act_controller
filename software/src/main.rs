#![no_main]
#![no_std]

// Setup startup code and minimal runtime for uC
// (check https://docs.rs/cortex-m-rt/latest/cortex_m_rt/)
use cortex_m_rt::entry;

use stm32f0xx_hal::{adc, pac, prelude::*, pwm, serial};

use core::fmt::Write;

use core::panic::PanicInfo;
#[panic_handler]
fn panic(_pi: &PanicInfo) -> ! {
    cortex_m::asm::delay(10_000_000);
    // reboot in bootloader mode
    cortex_m::peripheral::SCB::sys_reset();
}

#[entry]
fn main() -> ! {
    let mut p = pac::Peripherals::take().unwrap();
    let mut _cp = cortex_m::peripheral::Peripherals::take().unwrap();

    // when booting from bootloader memory mapping will have
    // System Flash memory (bootloader code) mapped at 0x0000_0000
    // resulting in bootloader interrupt vectors to be used
    // The following line will force memory mapping to Main Flash memory
    // as intended by this code
    p.SYSCFG.cfgr1.modify(|_, w| w.mem_mode().main_flash());

    // configure clock frequency
    let mut rcc = p.RCC.configure().sysclk(48.mhz()).freeze(&mut p.FLASH);

    let gpioa = p.GPIOA.split(&mut rcc);
    let gpiob = p.GPIOB.split(&mut rcc);
    let _gpiof = p.GPIOF.split(&mut rcc);

    // -- GPIOs --
    let mut led = cortex_m::interrupt::free(|cs| gpioa.pa13.into_push_pull_output(cs));
    let mut hb_dir = cortex_m::interrupt::free(|cs| gpioa.pa6.into_push_pull_output(cs));
    let mut hb_nsleep = cortex_m::interrupt::free(|cs| gpiob.pb0.into_push_pull_output(cs));
    led.set_high().ok();
    hb_dir.set_high().ok();
    hb_nsleep.set_high().ok();

    // -- DEBUG SERIAL LINK --
    // configure debug serial link
    let mut s = cortex_m::interrupt::free(|cs| {
        let tx = gpioa.pa9.into_alternate_af1(cs);
        let rx = gpioa.pa10.into_alternate_af1(cs);
        serial::Serial::usart1(p.USART1, (tx, rx), 115200.bps(), &mut rcc)
    });
    write!(s, "BOOTING").ok();

    // -- ADCs --
    let mut adc = adc::Adc::new(p.ADC, &mut rcc);
    let mut ctrl_pot_pin = cortex_m::interrupt::free(|cs| gpioa.pa0.into_analog(cs));
    let mut pos_pot_pin = cortex_m::interrupt::free(|cs| gpioa.pa1.into_analog(cs));

    // -- PWMs --
    let pwm_pin = cortex_m::interrupt::free(|cs| gpioa.pa7.into_alternate_af1(cs));
    let mut pwm = pwm::tim3(p.TIM3, pwm_pin, &mut rcc, 10.khz());
    //
    // enable PWM output
    pwm.enable();

    loop {
        // toggle led
        led.toggle().ok();
        hb_dir.toggle().ok();

        // measure voltage on ADC
        let ctrl_pot: u16 = adc.read_abs_mv(&mut ctrl_pot_pin);
        let pos_pot: u16 = adc.read_abs_mv(&mut pos_pot_pin);
        let max_adc = adc.max_sample() as u32;

        // output measured voltage to serial
        write!(s, "ctrl/pos: {ctrl_pot}/{pos_pot}").ok();

        // set PWM according to ADC
        let max_pwm = pwm.get_max_duty() as u32;
        let duty = ((ctrl_pot as u32) * max_pwm) / max_adc;
        pwm.set_duty(duty as u16);

        // wait some time
        cortex_m::asm::delay(10_000_000);
    }
}
