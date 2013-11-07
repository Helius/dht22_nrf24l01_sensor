
#include "uart.h"

#define BAUDRATE 9600                //The baudrate that we want to use
#define BAUD_PRESCALLER (((F_CPU / (BAUDRATE * 16UL))) - 1)     //The formula that does all the required maths

FILE uart_stream = FDEV_SETUP_STREAM (
	uart_putchar,
	uart_getchar,
	_FDEV_SETUP_RW);

void uart_init(void) {

	UBRRH = (uint8_t)(BAUD_PRESCALLER>>8);
	UBRRL = (uint8_t)(BAUD_PRESCALLER);
	UCSRB = (0<<RXEN)|(1<<TXEN); //!!!!!
	UCSRC = (1<<UCSZ0)|(1<<UCSZ1);
}

int uart_putchar(char c, FILE *stream) {
	while(!(UCSRA & (1<<UDRE)));
	UDR = c;
	return 0;
}

int uart_getchar(FILE *sttream) {
	while(!(UCSRA & (1<<RXC)));
	return UDR;
}
