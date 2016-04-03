#ifndef NUNCHUCK_H
#define NUNCHUCK_H

#include <stdint.h>

#define NCK_ADDR 0x52

struct NCK {
    uint8_t buttons;
    uint8_t sx, sy;
    uint16_t ax, ay, az;
};

void nck_init(void);
void nck_read_data(struct NCK* data);

#endif
