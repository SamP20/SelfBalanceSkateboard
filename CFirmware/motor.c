#include "motor.h"

#include "serial.h"

void motor_drive(uint8_t motor, int8_t speed) {
    uint8_t command = motor==0?0:4;
    if(speed < 0) {
        command += 1;
        if(speed == -128) {
            speed = 127; // Slight bodge. Range should be -127<=x<=127
        }else{
            speed = -speed;
        }
    }
    uint8_t checksum = (MOTOR_ADDR+command+speed) & 0x7f;
    fputc(MOTOR_ADDR, &uartfile);
    fputc(command, &uartfile);
    fputc(speed, &uartfile);
    fputc(checksum, &uartfile);
}
