//! Blinks an LED

// #![deny(warnings)]
#![no_std]
#![no_main]
#![feature(maybe_uninit)]

#[macro_use]
extern crate cortex_m;
#[macro_use]
extern crate cortex_m_rt as rt;
extern crate stm32l4xx_hal as hal;
#[macro_use(block)]
extern crate nb;

use crate::hal::prelude::*;
use crate::hal::delay::Delay;
use crate::hal::timer::Timer;
use crate::hal::serial::Serial;
use crate::hal::i2c::I2c;
use crate::rt::ExceptionFrame;
use crate::rt::entry;
use embedded_hal::{
    digital::OutputPin,
    serial::Write
};
use core::panic::PanicInfo;
use core::sync::atomic::{self, Ordering};
use micromath::F32Ext;

mod gyro;
mod nunchuck;
mod accel;

use crate::nunchuck::Nunchuck;
use crate::gyro::Gyro;
use crate::accel::Accel;

static mut ESTOP_PIN: Option<&mut dyn OutputPin> = None;


const TICKRATE: u32 = 50u32; //Hz
const TICK_INTERVAL: f32 = 1f32 / (TICKRATE as f32);
const BETA: f32 = 1.0f32;
const KP: f32 = 5f32;
const KD: f32 = 2.0f32;
const MAX_DIFFERENTIAL: f32 = 50f32;
const MIN_DIFFERENTIAL: f32 = 10f32;
const DISABLE_TIMEOUT: f32 = 0.5f32;
const TILT_ADJUST_RATE: f32 = 2.0; //Degrees per second

#[derive(Debug, PartialEq)]
enum DriveState {
    Disabled,
    WaitingNegStart,
    WaitingPosStart,
    Driving
}

#[entry]
fn main() -> ! {

    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = hal::stm32::Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain(); // .constrain();
    let mut rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(32.mhz()).pclk1(32.mhz()).freeze(&mut flash.acr);

    // Wait for things to stabilize
    let mut delay = Delay::new(cp.SYST, clocks);
    delay.delay_ms(200u32);

    let mut gpiob = dp.GPIOB.split(&mut rcc.ahb2);
    let mut gpioa = dp.GPIOA.split(&mut rcc.ahb2);

    let mut estop_pin = gpiob.pb0.into_push_pull_output_with_state(&mut gpiob.moder, &mut gpiob.otyper, hal::gpio::State::High);
    
    //TODO: Kinda urgent, write a safe wrapper around interrupt/panic handlers. We're trusting that estop_pin remains
    // in scope for the rest of eternety
    unsafe {
        ESTOP_PIN = Some(
            core::mem::transmute::<&'_ mut dyn OutputPin, &'static mut dyn OutputPin>(&mut estop_pin)
        );
    }

    let mut itm = cp.ITM;
    let stim = &mut itm.stim[0];

    //I2C Setup
    let mut scl = gpioa.pa9.into_open_drain_output(&mut gpioa.moder, &mut gpioa.otyper);
    scl.internal_pull_up(&mut gpioa.pupdr, true);
    let scl = scl.into_af4(&mut gpioa.moder, &mut gpioa.afrh);
    let mut sda = gpioa.pa10.into_open_drain_output(&mut gpioa.moder, &mut gpioa.otyper);
    sda.internal_pull_up(&mut gpioa.pupdr, true);
    let sda = sda.into_af4(&mut gpioa.moder, &mut gpioa.afrh);
    let mut i2c = I2c::i2c1(dp.I2C1, (scl, sda), 400.khz(), clocks, &mut rcc.apb1r1);


    //Uart Setup
    let tx = gpiob.pb6.into_af7(&mut gpiob.moder, &mut gpiob.afrl);
    let rx = gpiob.pb7.into_af7(&mut gpiob.moder, &mut gpiob.afrl);
    let serial = Serial::usart1(dp.USART1, (tx, rx), 9_600.bps(), clocks, &mut rcc.apb2);
    let (mut tx, mut _rx) = serial.split();

    let mut gyro = Gyro::new(&mut i2c, gyro::DataRate::DR_200, gyro::Bandwidth::BW_3, true, true, true).unwrap();
    gyro.set_hpf_cutoff_and_mode(&mut i2c, 6u8, gyro::HighPassMode::NormReset).unwrap();
    gyro.enable_high_pass_filter(&mut i2c, true).unwrap();

    let mut accel = Accel::new(&mut i2c, 0u8, 7u8, true, true, true).unwrap();

    let mut nck = Nunchuck::init(&mut i2c, &mut delay).unwrap();

    // Allow time for gyro HPF to stabilize
    delay.delay_ms(100u32);

    let accel_data = accel.read(&mut i2c).unwrap();

    let mut angle = (accel_data.accel[1] as f32).atan2(accel_data.accel[2] as f32);

    let mut drive_state = DriveState::Disabled;
    let mut button_release_time = DISABLE_TIMEOUT;

    let mut target_angle = 0f32;

    let mut timer = Timer::tim7(dp.TIM7, TICKRATE.hz(), clocks, &mut rcc.apb1r1);

    loop {
        block!(timer.wait()).unwrap();

        let nck_data = nck.read(&mut i2c, &mut delay).unwrap();
        let gyro_data = gyro.read(&mut i2c).unwrap();
        let accel_data = accel.read(&mut i2c).unwrap();

        let angular_rate = ((gyro_data.gyro[0] as f32) * 245f32) / 32768f32;
        let instantaneous_angle = (-(accel_data.accel[1] as f32)).atan2(-(accel_data.accel[2] as f32)) * (180f32 / core::f32::consts::PI);

        angle += angular_rate * TICK_INTERVAL;
        angle = angle * (1.0f32 - BETA*TICK_INTERVAL) + (instantaneous_angle * BETA*TICK_INTERVAL);

        let thrust = -(KP*(angle-target_angle) + KD*angular_rate);

        let joy_x = -((nck_data.joy[0] as f32) - 128f32) / 96f32; //-1 to +1 ish
        let joy_y = ((nck_data.joy[1] as f32) - 128f32) / 96f32; //-1 to +1 ish
        let drive_enable_btn = nck_data.buttons & 0x01 == 0x01;
        let tilt_adjust_btn = nck_data.buttons & 0x02 == 0x02;

        let abs_thrust_percent = if thrust > 0f32 {
            if thrust > 127f32 {
                1.0f32
            } else {
                thrust / 127f32
            }
        } else {
            if thrust < -127f32 {
                1.0f32
            } else {
                thrust / -127f32
            }
        };

        let differential = joy_x * (MAX_DIFFERENTIAL * (1.0 - abs_thrust_percent) + MIN_DIFFERENTIAL * (abs_thrust_percent));

        let mut left_drive = thrust + differential;
        let mut right_drive = thrust - differential;


        if left_drive < -127f32 { left_drive = -127f32; }
        if left_drive > 127f32 { left_drive = 127f32; }

        if right_drive < -127f32 { right_drive = -127f32; }
        if right_drive > 127f32 { right_drive = 127f32; }


        if !drive_enable_btn {
            if button_release_time >= DISABLE_TIMEOUT {
                if drive_state != DriveState::Disabled {
                    target_angle = 0f32;
                    motor_drive(&mut tx, Motor::Left, 0i8);
                    motor_drive(&mut tx, Motor::Right, 0i8);
                    drive_state = DriveState::Disabled;
                }
            } else {
                button_release_time += TICK_INTERVAL;
            }
        } else {
            button_release_time = 0f32;
        }

        match drive_state {
            DriveState::Disabled => {
                if drive_enable_btn {
                    if angle < 0.0f32 {
                        drive_state = DriveState::WaitingNegStart;
                    } else {
                        drive_state = DriveState::WaitingPosStart;
                    }
                }
            },
            DriveState::WaitingNegStart => {
                if angle >= 0.0 {
                    drive_state = DriveState::Driving;
                }
            },
            DriveState::WaitingPosStart => {
                if angle <= 0.0 {
                    drive_state = DriveState::Driving;
                }
            },
            DriveState::Driving => {
                motor_drive(&mut tx, Motor::Left, left_drive as i8);
                motor_drive(&mut tx, Motor::Right, right_drive as i8);

                if tilt_adjust_btn {
                    target_angle += joy_y * TILT_ADJUST_RATE * TICK_INTERVAL;
                }
            }
        }

        //iprintln!(stim, "ds={:?}, brs={}", drive_state, button_release_time);

    }
}

const MOTOR_ADDRESS: u8 = 128;

enum Motor {
    Left,
    Right
}

fn motor_drive<W: Write<u8>>(writer: &mut W, motor: Motor, speed: i8) {
    let mut command = match motor {
        Motor::Left => 0x00,
        Motor::Right => 0x04
    };
    let speed: u8 = if speed < 0 {
        command += 1;
        if speed == -128 {
            127
        }else{
            (-speed) as u8
        }
    } else {
        speed as u8
    };

    let checksum: u8 = (MOTOR_ADDRESS + command + speed) & 0x7f;
    block!(writer.write(MOTOR_ADDRESS)).map_err(|_| ()).expect("Motor write failure");
    block!(writer.write(command)).map_err(|_| ()).expect("Motor write failure");
    block!(writer.write(speed)).map_err(|_| ()).expect("Motor write failure");
    block!(writer.write(checksum)).map_err(|_| ()).expect("Motor write failure");

}

#[exception]
fn HardFault(ef: &ExceptionFrame) -> ! {
    panic!("{:#?}", ef);
}

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    cortex_m::interrupt::disable();

    unsafe {
        if let Some(estop) = &mut ESTOP_PIN {
            estop.set_low();
        }
    };

    let itm = unsafe { &mut *cortex_m::peripheral::ITM::ptr() };
    let stim = &mut itm.stim[0];

    iprintln!(stim, "{}", info);

    loop {
        // add some side effect to prevent this from turning into a UDF instruction
        // see rust-lang/rust#28728 for details
        atomic::compiler_fence(Ordering::SeqCst)
    }
}