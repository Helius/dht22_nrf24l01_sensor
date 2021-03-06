#ifndef _uart_h
#define _uart_h 1

#include <avr/io.h>
#include <stdio.h>

#define USART_BAUDRATE 19200
#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)

void uart_init(void);

int uart_putchar(char c, FILE *stream);
int uart_getchar(FILE *sttream);

extern FILE uart_stream;

#endif /* defined _uart_h */
