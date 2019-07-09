#include "imu.h"

#include "i2c_master.h"

void imu_init(void) {
    // FS = 0x00 (250dps)
    imu_write_reg(GYRO_ADDR, GYRO_CTRL4, 0x00);
    imu_write_reg(GYRO_ADDR, GYRO_LOW_ODR, 0x00);
    imu_write_reg(GYRO_ADDR, GYRO_CTRL1, 0x6F);

    // AFS = 0 (+/- 2 g full scale)
    imu_write_reg(ACCEL_ADDR, ACCEL_CTRL2, 0x00);
    // AODR = 0110 (100 Hz ODR); AZEN = AYEN = AXEN = 1 (all axes enabled)
    imu_write_reg(ACCEL_ADDR, ACCEL_CTRL1, 0x67);
}

void imu_write_reg(uint8_t addr, uint8_t reg, uint8_t value) {
    uint8_t data[] = {reg, value};
    i2c_transmit(addr, data, 2);
}

uint8_t imu_read_reg(uint8_t addr, uint8_t reg) {
    i2c_transmit(addr, &reg, 1);
    uint8_t value;
    i2c_receive(addr, &value, 1);
    return value;
}

void imu_read_data(struct IMU* data) {
    uint8_t values[6];

    uint8_t reg = GYRO_OUT_X_L | 0x80; //Set high bit for auto address increment
    i2c_transmit(GYRO_ADDR, &reg, 1);
    i2c_receive(GYRO_ADDR, &values, 6);
    data->gyro.x = (int16_t)(values[0] | (values[1]<<8));
    data->gyro.y = (int16_t)(values[2] | (values[3]<<8));
    data->gyro.z = (int16_t)(values[4] | (values[5]<<8));

    reg = ACCEL_OUT_X_L_A | 0x80; //Set high bit for auto address increment
    i2c_transmit(ACCEL_ADDR, &reg, 1);
    i2c_receive(ACCEL_ADDR, &values, 6);
    data->accel.x = (int16_t)(values[0] | (values[1]<<8));
    data->accel.y = (int16_t)(values[2] | (values[3]<<8));
    data->accel.z = (int16_t)(values[4] | (values[5]<<8));
}
