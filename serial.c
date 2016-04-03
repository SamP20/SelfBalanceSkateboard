#include "serial.h"
#include <avr/interrupt.h>

FILE uartfile = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);

volatile uint8_t uart_tx_buf[UART_BUF_LEN];
volatile uint8_t uart_rx_buf[UART_BUF_LEN];
uint8_t uart_tx_write_ptr = 0;
uint8_t uart_tx_read_ptr = 0;
volatile uint8_t uart_tx_len = 0;
uint8_t uart_rx_write_ptr = 0;
uint8_t uart_rx_read_ptr = 0;
volatile uint8_t uart_rx_len = 0;

void uart_init(uint8_t enabled) {
    UBRR0H = (BAUDRATE>>8); // set baud rate
    UBRR0L = BAUDRATE;
    UCSR0B = enabled | (1<<RXCIE0); // enable receiver and transmitter
    UCSR0C = (1<<UCSZ00)|(1<<UCSZ01);   // 8bit data format

}

void uart_write(uint8_t data) {
    while(!(UCSR0A & (1<<UDRE0)));
    UDR0 = data;
}

uint8_t uart_available(void) {
    //return (UCSR0A & (1<<RXC0))!=0;
    return uart_rx_len;
}

uint8_t uart_read(void) {
    while(!(UCSR0A & (1<<RXC0)));                   // wait while data is being received
    return UDR0;                                   // return 8-bit data
}

int uart_putchar(char var, FILE *stream) {
    while(uart_tx_len>=UART_BUF_LEN);
    uart_tx_buf[uart_tx_write_ptr++] = (uint8_t)var;
    uart_tx_write_ptr &= UART_BUF_MASK;
    uart_tx_len++;
    UCSR0B |= (1<<UDRIE0); // Enable TX empty interrupt
    //uart_write((uint8_t)var);
    return 0;
}

int uart_getchar(FILE *stream) {
    while(uart_rx_len==0);
    uint8_t val = uart_rx_buf[uart_rx_read_ptr++];
    uart_rx_read_ptr &= UART_BUF_MASK;
    uart_rx_len--;
    return val;
    //return uart_read();
}

ISR(USART_TX_vect) {
}

ISR(USART_RX_vect) {
    if(uart_rx_len<=UART_BUF_LEN) {
        uart_rx_buf[uart_rx_write_ptr++] = UDR0;
        uart_rx_write_ptr &= UART_BUF_MASK;
        uart_rx_len++;
    }else{
        // Buffer overun. Nothing we do is good. Just drop the bytes
        uint8_t tmp = UDR0;
    }
}

ISR(USART_UDRE_vect) {
    if(uart_tx_len>0) {
        UDR0 = uart_tx_buf[uart_tx_read_ptr++];
        uart_tx_read_ptr &= UART_BUF_MASK;
        uart_tx_len--;
    }else{
        UCSR0B &= ~(1<<UDRIE0); // No more data to send. Disable empty interrupt
    }
}
