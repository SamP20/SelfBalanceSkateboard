use embedded_hal::blocking::{
    i2c::{Read, Write, WriteRead},
};

use core::{fmt::Debug, marker::PhantomData};

const GYRO_ADDR: u8 = 0xD6;
const GYRO_WHO_AM_I: u8 = 0x0F;
const GYRO_CTRL1: u8 = 0x20;
const GYRO_CTRL2: u8 = 0x21;
const GYRO_CTRL3: u8 = 0x22;
const GYRO_CTRL4: u8 = 0x23;
const GYRO_CTRL5: u8 = 0x24;
const GYRO_REFERENCE: u8 = 0x25;
const GYRO_OUT_TEMP: u8 = 0x26;
const GYRO_STATUS: u8 = 0x27;
const GYRO_OUT_X_L: u8 = 0x28;
const GYRO_OUT_X_H: u8 = 0x29;
const GYRO_OUT_Y_L: u8 = 0x2A;
const GYRO_OUT_Y_H: u8 = 0x2B;
const GYRO_OUT_Z_L: u8 = 0x2C;
const GYRO_OUT_Z_H: u8 = 0x2D;
const GYRO_FIFO_CTRL: u8 = 0x2E;
const GYRO_FIFO_SRC: u8 = 0x2F;
const GYRO_IG_CFG: u8 = 0x30;
const GYRO_IG_SRC: u8 = 0x31;
const GYRO_IG_THS_XH: u8 = 0x32;
const GYRO_IG_THS_XL: u8 = 0x33;
const GYRO_IG_THS_YH: u8 = 0x34;
const GYRO_IG_THS_YL: u8 = 0x35;
const GYRO_IG_THS_ZH: u8 = 0x36;
const GYRO_IG_THS_ZL: u8 = 0x37;
const GYRO_IG_DURATION : u8 =0x38;
const GYRO_LOW_ODR: u8 = 0x39;

#[derive(Debug, Copy, Clone)]
pub struct Data {
    pub gyro: [i16; 3]
}


pub struct Gyro<I2C, ERROR> {
    _i2c: PhantomData<*const I2C>,
    _error: PhantomData<*const ERROR>,
}

pub enum DataRate {
    DR_12_5,
    DR_25,
    DR_50,
    DR_100,
    DR_200,
    DR_400,
    DR_800,
}

pub enum Bandwidth {
    BW_0,
    BW_1,
    BW_2,
    BW_3
}

pub enum HighPassMode {
    NormReset,
    Ref,
    Norm,
    Autoreset
}

impl<I2C: Read<Error = ERROR> + Write<Error = ERROR> + WriteRead<Error = ERROR>, ERROR: Debug> Gyro<I2C, ERROR> {
    pub fn new(i2c: &mut I2C, data_rate: DataRate, bandwidth: Bandwidth, xen: bool, yen: bool, zen: bool) -> Result<Self, ERROR>
    {
        let mut ctrl1 = match data_rate {
            DataRate::DR_12_5 | DataRate::DR_100 => 0x00,
            DataRate::DR_25 | DataRate::DR_200 => 0x40,
            DataRate::DR_50 | DataRate::DR_400 => 0x80,
            DataRate::DR_800 => 0xC0,
        };

        let low_odr = match data_rate {
            DataRate::DR_12_5 | DataRate::DR_25 | DataRate::DR_50 => 0x01,
            DataRate::DR_100 | DataRate::DR_200 | DataRate::DR_400 | DataRate::DR_800 => 0x00,
        };

        match bandwidth {
            Bandwidth::BW_1 => ctrl1 |= 0x10,
            Bandwidth::BW_2 => ctrl1 |= 0x20,
            Bandwidth::BW_3 => ctrl1 |= 0x30,
            Bandwidth::BW_0 => (),
        }

        if xen { ctrl1 |= 0x01; }
        if yen { ctrl1 |= 0x02; }
        if zen { ctrl1 |= 0x04; }

        // Enable
        ctrl1 |= 0x08;

        i2c.write(GYRO_ADDR, &[GYRO_LOW_ODR, low_odr])?;
        i2c.write(GYRO_ADDR, &[GYRO_CTRL1, ctrl1])?;

        Ok(Gyro {
            _i2c: PhantomData,
            _error: PhantomData,
        })
    }

    pub fn enable_high_pass_filter(&mut self, i2c: &mut I2C, enable: bool) -> Result<(), ERROR> {
        if enable {
            i2c.write(GYRO_ADDR, &[GYRO_CTRL5, 0x10])?;
        }else{
            i2c.write(GYRO_ADDR, &[GYRO_CTRL5, 0x00])?;
        }

        Ok(())
    }

    pub fn set_hpf_cutoff_and_mode(&mut self, i2c: &mut I2C, cutoff: u8, mode: HighPassMode) -> Result<(), ERROR> {
        let mut ctrl2 = match mode {
            HighPassMode::NormReset => 0x10,
            HighPassMode::Ref => 0x20,
            HighPassMode::Norm => 0x40,
            HighPassMode::Autoreset => 0x80,
        };

        ctrl2 |= cutoff & 0x0f;
        i2c.write(GYRO_ADDR, &[GYRO_CTRL2, ctrl2])?;

        Ok(())
    }

    pub fn read(
        &mut self,
        i2c: &mut I2C,
    ) -> Result<Data, ERROR>
    where
        I2C: Write<Error = ERROR> + Read<Error = ERROR>,
    {
        let mut buffer = [0u8; 6];
        i2c.write_read(GYRO_ADDR, &[GYRO_OUT_X_L | 0x80], &mut buffer)?;

        Ok(Data {
            gyro: [
                ((buffer[0] as u16) | ((buffer[1] as u16) << 8)) as i16,
                ((buffer[2] as u16) | ((buffer[3] as u16) << 8)) as i16,
                ((buffer[4] as u16) | ((buffer[5] as u16) << 8)) as i16
            ]
        })
    }
}