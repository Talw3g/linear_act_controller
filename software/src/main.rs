#![no_main]
#![no_std]

// Setup startup code and minimal runtime for uC
// (check https://docs.rs/cortex-m-rt/latest/cortex_m_rt/)
use cortex_m_rt::entry;

use stm32f0xx_hal::{adc, pac, prelude::*, pwm, serial};

use heapless::spsc::Queue;

use core::fmt::Write;

use core::panic::PanicInfo;
#[panic_handler]
fn panic(_pi: &PanicInfo) -> ! {
    cortex_m::asm::delay(10_000_000);
    // reboot in bootloader mode
    cortex_m::peripheral::SCB::sys_reset();
}
const AVG_RANGE: usize = 50; // number of values for avg
const POS_MAX_MV: u16 = 295; // mv reading when actuator fully extended
const POS_MIN_MV: u16 = 1830; // mv reading when actuator fully retracted
const CTRL_MIN_MV: u16 = 0; // mv reading when pot at 0%
const CTRL_MAX_MV: u16 = 2834; // mv reading when pot at 100%
const POS_MAX_ERR: f32 = 5.0; // pos error (%) for full speed
const POS_MIN_ERR: f32 = 0.5; // authorized pos error (%)
const I_LIMIT: u16 = 2; // current limit (A)
const I_LIMIT_MV: u16 = I_LIMIT * 500; // current limit in mV (500mV = 1A)
const MIN_SPEED: f32 = 100.0;
const MAX_SPEED: f32 = 480.0;

#[entry]
fn main() -> ! {
    // -- CONFIG --

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
    // status led:
    let mut led = cortex_m::interrupt::free(|cs| gpioa.pa13.into_push_pull_output(cs));
    // halfbridge direction:
    let mut hb_dir = cortex_m::interrupt::free(|cs| gpioa.pa6.into_push_pull_output(cs));
    // halfbridge nsleep (needs to be pulled high to operate):
    let mut hb_nsleep = cortex_m::interrupt::free(|cs| gpiob.pb0.into_push_pull_output(cs));
    hb_nsleep.set_high().ok();
    let hb_nfault = cortex_m::interrupt::free(|cs| gpioa.pa8.into_floating_input(cs));

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
    // control potentiometer:
    let mut ctrl_pot_pin = cortex_m::interrupt::free(|cs| gpioa.pa0.into_analog(cs));
    // actuator position potentiometer:
    let mut pos_pot_pin = cortex_m::interrupt::free(|cs| gpioa.pa1.into_analog(cs));
    // halfbridge current sensor output:
    let mut visen_pin = cortex_m::interrupt::free(|cs| gpioa.pa2.into_analog(cs));

    // -- PWMs --
    // halfbridge enable control:
    let hb_enbl_pwm_pin = cortex_m::interrupt::free(|cs| gpioa.pa7.into_alternate_af1(cs));
    let mut hb_enbl_pwm = pwm::tim3(p.TIM3, hb_enbl_pwm_pin, &mut rcc, 100.khz());
    hb_enbl_pwm.enable();

    // -- QUEUES --
    let mut ctrl_pot_queue: Queue<u16, AVG_RANGE> = Queue::new();
    let mut pos_pot_queue: Queue<u16, AVG_RANGE> = Queue::new();
    let mut visen_queue: Queue<u16, AVG_RANGE> = Queue::new();

    let mut i: u16 = 0;
    loop {
        i += 1;
        // measure voltages on ADC
        let visen: u16 = adc.read_abs_mv(&mut visen_pin);
        visen_queue.enqueue(visen).ok();
        let ctrl_pot: u16 = adc.read_abs_mv(&mut ctrl_pot_pin);
        ctrl_pot_queue.enqueue(ctrl_pot).ok();
        let pos_pot: u16 = adc.read_abs_mv(&mut pos_pot_pin);
        pos_pot_queue.enqueue(pos_pot).ok();

        // warn user if halfbridge fault:
        if Ok(true) == hb_nfault.is_low() {
            writeln!(s, "Halfbridge fault detected").ok();
            if i.is_multiple_of(50) {
                led.toggle().ok();
                i = 0;
            }
            continue;
        }
        // software current limitation:
        if mean(&visen_queue) > I_LIMIT_MV {
            writeln!(s, "Triggered current limit: {visen}").ok();
            hb_enbl_pwm.set_duty(0);
            if i.is_multiple_of(150) {
                led.toggle().ok();
                i = 0;
            }
            continue;
        }

        // Wait for queues to be full:
        if !(ctrl_pot_queue.is_full() & pos_pot_queue.is_full()) {
            continue;
        }

        let _ctrl = mean(&ctrl_pot_queue);
        let _pos = mean(&pos_pot_queue);
        let ctrl: f32 =
            (_ctrl as f32 - CTRL_MIN_MV as f32) / (CTRL_MAX_MV as f32 - CTRL_MIN_MV as f32) * 100.0;
        let pos: f32 =
            (_pos as f32 - POS_MIN_MV as f32) / (POS_MAX_MV as f32 - POS_MIN_MV as f32) * 100.0;
        let pos_err = pos - ctrl;
        let speed = ((pos_err.abs() * MAX_SPEED / POS_MAX_ERR).clamp(MIN_SPEED, MAX_SPEED)) as u16;

        if pos_err.abs() <= POS_MIN_ERR {
            // target reached
            hb_enbl_pwm.set_duty(0);
            led.set_low().ok();
        } else {
            if pos_err < 0.0 {
                // below target
                hb_dir.set_high().ok();
            } else {
                // above target
                hb_dir.set_low().ok();
            }
            hb_enbl_pwm.set_duty(speed);
            led.set_high().ok();
        }

        let _ = ctrl_pot_queue.dequeue();
        let _ = pos_pot_queue.dequeue();
        if i > 65_000 {
            i = 0;
        }
        // wait some time
        cortex_m::asm::delay(10_000);
    }
}

fn sum(queue: &Queue<u16, AVG_RANGE>) -> u32 {
    let mut s: u32 = 0;
    for val in queue.iter() {
        s += *val as u32;
    }
    s
}

fn mean(queue: &Queue<u16, AVG_RANGE>) -> u16 {
    (sum(queue) / (queue.len() as u32)) as u16
}
