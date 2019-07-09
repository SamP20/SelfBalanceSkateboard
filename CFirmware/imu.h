#ifndef IMU_H
#define IMU_H

#include <stdint.h>

//#define GYRO_ADDR 0x6A //SA0=0
#define GYRO_ADDR 0x6B //SA0=1

//#define ACCEL_ADDR 0x1E //SA0=0
#define ACCEL_ADDR 0x1D //SA0=1

#define GYRO_WHO_AM_I 0x0F
#define GYRO_CTRL1 0x20
#define GYRO_CTRL2 0x21
#define GYRO_CTRL3 0x22
#define GYRO_CTRL4 0x23
#define GYRO_CTRL5 0x24
#define GYRO_REFERENCE 0x25
#define GYRO_OUT_TEMP 0x26
#define GYRO_STATUS 0x27
#define GYRO_OUT_X_L 0x28
#define GYRO_OUT_X_H 0x29
#define GYRO_OUT_Y_L 0x2A
#define GYRO_OUT_Y_H 0x2B
#define GYRO_OUT_Z_L 0x2C
#define GYRO_OUT_Z_H 0x2D
#define GYRO_FIFO_CTRL 0x2E
#define GYRO_FIFO_SRC 0x2F
#define GYRO_IG_CFG 0x30
#define GYRO_IG_SRC 0x31
#define GYRO_IG_THS_XH 0x32
#define GYRO_IG_THS_XL 0x33
#define GYRO_IG_THS_YH 0x34
#define GYRO_IG_THS_YL 0x35
#define GYRO_IG_THS_ZH 0x36
#define GYRO_IG_THS_ZL 0x37
#define GYRO_IG_DURATION 0x38
#define GYRO_LOW_ODR 0x39

#define ACCEL_TEMP_OUT_L 0x05
#define ACCEL_TEMP_OUT_H 0x06
#define ACCEL_STATUS_M 0x07
#define ACCEL_OUT_X_L_M 0x08
#define ACCEL_OUT_X_H_M 0x09
#define ACCEL_OUT_Y_L_M 0x0A
#define ACCEL_OUT_Y_H_M 0x0B
#define ACCEL_OUT_Z_L_M 0x0C
#define ACCEL_OUT_Z_H_M 0x0D
#define ACCEL_WHO_AM_I 0x0F
#define ACCEL_NT_CTRL_M 0x12
#define ACCEL_INT_SRC_M 0x13
#define ACCEL_INT_THS_L_M 0x14
#define ACCEL_INT_THS_H_M 0x15
#define ACCEL_OFFSET_X_L_M 0x16
#define ACCEL_OFFSET_X_H_M 0x17
#define ACCEL_OFFSET_Y_L_M 0x18
#define ACCEL_OFFSET_Y_H_M 0x19
#define ACCEL_OFFSET_Z_L_M 0x1A
#define ACCEL_OFFSET_Z_H_M 0x1B
#define ACCEL_REFERENCE_X 0x1C
#define ACCEL_REFERENCE_Y 0x1D
#define ACCEL_REFERENCE_Z 0x1E
#define ACCEL_CTRL0 0x1F
#define ACCEL_CTRL1 0x20
#define ACCEL_CTRL2 0x21
#define ACCEL_CTRL3 0x22
#define ACCEL_CTRL4 0x23
#define ACCEL_CTRL5 0x24
#define ACCEL_CTRL6 0x25
#define ACCEL_CTRL7 0x26
#define ACCEL_STATUS_A 0x27
#define ACCEL_OUT_X_L_A 0x28
#define ACCEL_OUT_X_H_A 0x29
#define ACCEL_OUT_Y_L_A 0x2A
#define ACCEL_OUT_Y_H_A 0x2B
#define ACCEL_OUT_Z_L_A 0x2C
#define ACCEL_OUT_Z_H_A 0x2D
#define ACCEL_FIFO_CTRL 0x2E
#define ACCEL_FIFO_SRC 0x2F
#define ACCEL_IG_CFG1 0x30
#define ACCEL_IG_SRC1 0x31
#define ACCEL_IG_THS1 0x32
#define ACCEL_IG_DUR1 0x33
#define ACCEL_IG_CFG2 0x34
#define ACCEL_IG_SRC2 0x35
#define ACCEL_IG_THS2 0x36
#define ACCEL_IG_DUR2 0x37
#define ACCEL_CLICK_CFG 0x38
#define ACCEL_CLICK_SRC 0x39
#define ACCEL_CLICK_THS 0x3A
#define ACCEL_TIME_LIMIT 0x3B
#define ACCEL_TIME_LATENCY 0x3C
#define ACCEL_TIME_WINDOW 0x3D
#define ACCEL_ACT_THS 0x3E
#define ACCEL_ACT_DUR 0x3F

struct Vector {
    int16_t x,y,z;
};

struct IMU {
    struct Vector gyro;
    struct Vector accel;
    struct Vector mag;
};

void imu_init(void);
void imu_write_reg(uint8_t addr, uint8_t reg, uint8_t value);
uint8_t imu_read_reg(uint8_t addr, uint8_t reg);
void imu_read_data(struct IMU* data);


#endif
