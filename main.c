/* ledblink.c, an LED blinking program */
#include "serial.h"
#include "imu.h"
#include "nunchuck.h"
#include "motor.h"

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <math.h>

void setup_timer()
{
    // 1/64 prescaler
    TCCR0B = (1<<CS01) | (1<<CS00);
    // Enable CTC mode (reset to 0 when OCR0A reached)
    TCCR0A |= (1 << WGM01);
    // 1kHz assuming 16MHz clock
    OCR0A = 249;
    // Enable interrupt
    TIMSK0 |= (1<<OCIE0A);
    sei();
}

volatile uint8_t do_tick;

#define DRIVE_STATE_START 0
#define DRIVE_STATE_NEG_START 1
#define DRIVE_STATE_POS_START 2
#define DRIVE_STATE_DRIVING 3
uint8_t drive_state = DRIVE_STATE_START;

#define START_AVG 100;
#define RAD_TO_DEG 57.295779513
uint16_t startup = START_AVG;
//Exponential average gyro zero rate over 20 seconds
const double alpha = 0.00001;
//Filter in acceleration angle over 4 seconds
const double beta = 0.02;
double gyro_zero = 0.0f;
double gyro_rate;
double rotation_speed;
double accel_angle;

double angle = 0.0f;
struct IMU imu_data;
struct NCK nck_data;

double kp = 5;
double kd = 150.0;

double target_angle = 0.0f;
double thrust;

int main()
{
    _delay_ms(100);
    uart_init(UART_RX_EN | UART_TX_EN);
    stdout = stdin = &uartfile;
    i2c_init();
    imu_init();
    //nck_init();
    // Allow time for gyro to settle
    _delay_ms(500);

    // Debug LED
    DDRC |= (1<<PC0);
    PORTC ^= (1<<PC0);

    // Start 1KHz interrupt timer.
    setup_timer();

    while(1)
    {
        //printf("Hello world\n");

        //nck_read_data(&nck_data);
        //imu_read_data(&imu_data);
        //printf("G(x: % 6d, y: % 6d, z: % 6d) ", imu_data.gyro.x, imu_data.gyro.y, imu_data.gyro.z);
        //printf("A(x: % 6d, y: % 6d, z: % 6d) ", imu_data.accel.x, imu_data.accel.y, imu_data.accel.z);
        //printf("M(x: % 6d, y: % 6d, z: % 6d) ", imu_data.mag.x, imu_data.mag.y, imu_data.mag.z);
        //printf("NS(x: % 6d, y: % 6d) ", nck_data.sx, nck_data.sy);
        //printf("NA(x: % 6d, y: % 6d, z: % 6d) ", nck_data.ax, nck_data.ay, nck_data.az);
        //printf("\n");
        if(do_tick)
        {
            imu_read_data(&imu_data);
            gyro_rate = ((double)imu_data.gyro.x *2.5)/32768.0; // Convert to degrees/tick

            // Average a bunch of readings at the start to calibrate the gyro
            if(startup>0)
            {
                startup--;
                gyro_zero += gyro_rate/(double)START_AVG;
            }

            rotation_speed = gyro_rate - gyro_zero;
            angle += rotation_speed;
            while(angle<-180.0) angle+=360.0;
            while(angle>=180.0) angle-=360.0;
            // Calculate accelerometer angle from y/z components
            accel_angle = atan2(-imu_data.accel.y, -imu_data.accel.z)*RAD_TO_DEG;
            // Blend accelerometer and gyro angles.
            angle = angle*(1.0-beta) + accel_angle*beta;

            // PD controller
            thrust = -(kp*(angle-target_angle) + kd*rotation_speed);

            // Clamp
            if(thrust > 127.0) thrust=127.0;
            if(thrust < -127.0) thrust=-127.0;

            //printf("%6d\n", (int)thrust);
            // Prevent driving when board is resting on the ground.
            if(drive_state == DRIVE_STATE_START)
            {
                if(angle < 0.0)
                {
                    drive_state = DRIVE_STATE_NEG_START;
                }
                else
                {
                    drive_state = DRIVE_STATE_POS_START;
                }
            }
            else if(drive_state == DRIVE_STATE_NEG_START)
            {
                if(angle >= 0.0)
                {
                    drive_state = DRIVE_STATE_DRIVING;
                }
            }
            else if(drive_state == DRIVE_STATE_POS_START)
            {
                if(angle <= 0.0)
                {
                    drive_state = DRIVE_STATE_DRIVING;
                }
            }
            else if(drive_state == DRIVE_STATE_DRIVING)
            {
                motor_drive(0, thrust);
                motor_drive(1, thrust);
            }
        }
    }
    return 0;
}

uint16_t loop_count = 0;
uint16_t print_count = 0;

ISR(TIMER0_COMPA_vect)
{
    loop_count++;
    if(loop_count==10)
    {
        loop_count=0;
        do_tick = 1;
    }
}
