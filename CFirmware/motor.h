#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

#define MOTOR_ADDR 128

void motor_drive(uint8_t motor, int8_t speed);

#endif
