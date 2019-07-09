use embedded_hal::blocking::{
    i2c::{Read, Write, WriteRead},
};

use core::{fmt::Debug, marker::PhantomData};

const ACCEL_ADDR: u8 = 0x3A;

const ACCEL_TEMP_OUT_L: u8 = 0x05;
const ACCEL_TEMP_OUT_H: u8 = 0x06;
const ACCEL_STATUS_M: u8 = 0x07;
const ACCEL_OUT_X_L_M: u8 = 0x08;
const ACCEL_OUT_X_H_M: u8 = 0x09;
const ACCEL_OUT_Y_L_M: u8 = 0x0A;
const ACCEL_OUT_Y_H_M: u8 = 0x0B;
const ACCEL_OUT_Z_L_M: u8 = 0x0C;
const ACCEL_OUT_Z_H_M: u8 = 0x0D;
const ACCEL_WHO_AM_I: u8 = 0x0F;
const ACCEL_NT_CTRL_M: u8 = 0x12;
const ACCEL_INT_SRC_M: u8 = 0x13;
const ACCEL_INT_THS_L_M: u8 = 0x14;
const ACCEL_INT_THS_H_M: u8 = 0x15;
const ACCEL_OFFSET_X_L_M: u8 = 0x16;
const ACCEL_OFFSET_X_H_M: u8 = 0x17;
const ACCEL_OFFSET_Y_L_M: u8 = 0x18;
const ACCEL_OFFSET_Y_H_M: u8 = 0x19;
const ACCEL_OFFSET_Z_L_M: u8 = 0x1A;
const ACCEL_OFFSET_Z_H_M: u8 = 0x1B;
const ACCEL_REFERENCE_X: u8 = 0x1C;
const ACCEL_REFERENCE_Y: u8 = 0x1D;
const ACCEL_REFERENCE_Z: u8 = 0x1E;
const ACCEL_CTRL0: u8 = 0x1F;
const ACCEL_CTRL1: u8 = 0x20;
const ACCEL_CTRL2: u8 = 0x21;
const ACCEL_CTRL3: u8 = 0x22;
const ACCEL_CTRL4: u8 = 0x23;
const ACCEL_CTRL5: u8 = 0x24;
const ACCEL_CTRL6: u8 = 0x25;
const ACCEL_CTRL7: u8 = 0x26;
const ACCEL_STATUS_A: u8 = 0x27;
const ACCEL_OUT_X_L_A: u8 = 0x28;
const ACCEL_OUT_X_H_A: u8 = 0x29;
const ACCEL_OUT_Y_L_A: u8 = 0x2A;
const ACCEL_OUT_Y_H_A: u8 = 0x2B;
const ACCEL_OUT_Z_L_A: u8 = 0x2C;
const ACCEL_OUT_Z_H_A: u8 = 0x2D;
const ACCEL_FIFO_CTRL: u8 = 0x2E;
const ACCEL_FIFO_SRC: u8 = 0x2F;
const ACCEL_IG_CFG1: u8 = 0x30;
const ACCEL_IG_SRC1: u8 = 0x31;
const ACCEL_IG_THS1: u8 = 0x32;
const ACCEL_IG_DUR1: u8 = 0x33;
const ACCEL_IG_CFG2: u8 = 0x34;
const ACCEL_IG_SRC2: u8 = 0x35;
const ACCEL_IG_THS2: u8 = 0x36;
const ACCEL_IG_DUR2: u8 = 0x37;
const ACCEL_CLICK_CFG: u8 = 0x38;
const ACCEL_CLICK_SRC: u8 = 0x39;
const ACCEL_CLICK_THS: u8 = 0x3A;
const ACCEL_TIME_LIMIT: u8 = 0x3B;
const ACCEL_TIME_LATENCY: u8 = 0x3C;
const ACCEL_TIME_WINDOW: u8 = 0x3D;
const ACCEL_ACT_THS: u8 = 0x3E;
const ACCEL_ACT_DUR: u8 = 0x3F;

pub struct Accel<I2C, ERROR> {
    _i2c: PhantomData<*const I2C>,
    _error: PhantomData<*const ERROR>,
}

#[derive(Debug, Copy, Clone)]
pub struct Data {
    pub accel: [i16; 3]
}

impl<I2C: Read<Error = ERROR> + Write<Error = ERROR> + WriteRead<Error = ERROR>, ERROR: Debug> Accel<I2C, ERROR> {
    pub fn new(i2c: &mut I2C, scale: u8, data_rate: u8, xen: bool, yen: bool, zen: bool) -> Result<Self, ERROR>
    {
        let ctrl2 = (scale << 3) & 0x38;
        let mut ctrl1 = data_rate << 4;
        if xen { ctrl1 |= 0x01; }
        if yen { ctrl1 |= 0x02; }
        if zen { ctrl1 |= 0x04; }

        i2c.write(ACCEL_ADDR, &[ACCEL_CTRL2, ctrl2])?;
        i2c.write(ACCEL_ADDR, &[ACCEL_CTRL1, ctrl1])?;

        Ok(Accel {
            _i2c: PhantomData,
            _error: PhantomData
        })
    }

    pub fn read(
        &mut self,
        i2c: &mut I2C,
    ) -> Result<Data, ERROR>
    where
        I2C: Write<Error = ERROR> + Read<Error = ERROR>,
    {
        let mut buffer = [0u8; 6];
        i2c.write_read(ACCEL_ADDR, &[ACCEL_OUT_X_L_A | 0x80], &mut buffer)?;

        Ok(Data {
            accel: [
                ((buffer[0] as u16) | ((buffer[1] as u16) << 8)) as i16,
                ((buffer[2] as u16) | ((buffer[3] as u16) << 8)) as i16,
                ((buffer[4] as u16) | ((buffer[5] as u16) << 8)) as i16
            ]
        })
    }
}