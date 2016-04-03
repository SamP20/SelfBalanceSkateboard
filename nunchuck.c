#import "nunchuck.h"

#import "i2c_master.h"
#include <util/delay.h>

void nck_init(void) {
    uint8_t data[] = {0xF0, 0x55, 0xFB, 0x00};
    if(i2c_transmit(NCK_ADDR, &data, 4)) {
        printf("setup error\n");
    }
    //_delay_ms(1);
    //uint8_t data2[] = {0xFB, 0x00};
    //twi_send(NCK_ADDR, &data2, 2, TWI_NOREPEAT);
}

void nck_read_data(struct NCK* data) {
    uint8_t raw[6];
    if(i2c_receive(NCK_ADDR, &raw, 6)) {
        printf("receive error\n");
    }

    _delay_ms(20);
    uint8_t send[] = {0x00};
    //for(uint8_t i=0; i<3; i++) {
        if(i2c_transmit(NCK_ADDR, &send, 1)) {
            printf("transmit error\n");
        }
    //}

    data->buttons = (raw[5] & 0x03);
    data->sx = raw[0];
    data->sy = raw[1];
    data->ax = (raw[2]<<2) | ((raw[5]>>2) & 0x03);
    data->ay = (raw[3]<<2) | ((raw[5]>>4) & 0x03);
    data->az = (raw[4]<<2) | ((raw[5]>>6) & 0x03);
}
