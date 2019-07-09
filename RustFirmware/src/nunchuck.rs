use embedded_hal::blocking::{
    delay::DelayMs,
    i2c::{Read, Write},
};

use core::{fmt::Debug, marker::PhantomData};

const NUNCHUCK_ADDR: u8 = 0xA4;

#[derive(Debug)]
pub enum DataError<I2CERROR: Debug> {
    I2CError(I2CERROR),
    InvalidData,
}

#[derive(Debug, Copy, Clone)]
pub struct Data {
    pub buttons: u8,
    pub joy: [u8; 2],
    pub accel: [u16; 3],
}

pub struct Nunchuck<I2C, ERROR> {
    _i2c: PhantomData<*const I2C>,
    _error: PhantomData<*const ERROR>,
}

impl<I2C: Read<Error = ERROR> + Write<Error = ERROR>, ERROR: Debug> Nunchuck<I2C, ERROR> {
    pub fn init<DELAY>(i2c: &mut I2C, timer: &mut DELAY) -> Result<Self, ERROR>
    where
        DELAY: DelayMs<u32>,
    {
        timer.delay_ms(20u32);
        i2c.write(NUNCHUCK_ADDR, &[0xF0, 0x55, 0xFB, 0x00])?;

        timer.delay_ms(20u32);

        Ok(Nunchuck {
            _i2c: PhantomData,
            _error: PhantomData,
        })
    }

    pub fn read<DELAY>(
        &mut self,
        i2c: &mut I2C,
        timer: &mut DELAY,
    ) -> Result<Data, DataError<ERROR>>
    where
        I2C: Write<Error = ERROR> + Read<Error = ERROR>,
        DELAY: DelayMs<u32>,
    {
        let mut buffer = [0u8; 6];
        i2c.write(NUNCHUCK_ADDR, &[0x00])
            .map_err(|e| DataError::I2CError(e))?;
        timer.delay_ms(3u32);
        i2c.read(NUNCHUCK_ADDR, &mut buffer)
            .map_err(|e| DataError::I2CError(e))?;

        if (buffer[1] == 0x00
            && buffer[2] == 0x00
            && buffer[3] == 0x00
            && buffer[4] == 0x00
            && buffer[5] == 0x00)
            || (buffer[1] == 0xff
                && buffer[2] == 0xff
                && buffer[3] == 0xff
                && buffer[4] == 0xff
                && buffer[5] == 0xff)
        {
            return Err(DataError::InvalidData);
        }

        Ok(Data {
            buttons: (!buffer[5] & 0x03),
            joy: [buffer[0], buffer[1]],
            accel: [
                ((buffer[2] as u16) << 2) | (((buffer[5] as u16) >> 2) & 0x03),
                ((buffer[3] as u16) << 2) | (((buffer[5] as u16) >> 4) & 0x03),
                ((buffer[4] as u16) << 2) | (((buffer[5] as u16) >> 6) & 0x03),
            ],
        })
    }
}
