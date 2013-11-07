#include <stdlib.h>
#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include "uart.h"

//#include "uart.h"

#define SETBIT(reg,bit) (reg|=1<<bit)
#define CLRBIT(reg,bit) (reg&=~(1<<bit))



void gpio_init() {
//	DDRD |= 1<<PD1;
	DDRD |= 1<<PD0;
//	DDRD = 0xff;
}

void interrupt_init() {
	/*EIMSK |= (1 << INT0);     // Turns on INT0
	SETBIT(PORTD,WAKEUP);     // enable pull-up
	CLRBIT(DDRD,WAKEUP);      // set as input*/
}

void go_sleep () {
	sleep_enable();
	sei();
	sleep_cpu();
	sleep_disable();
	cli();
}

int main (void) {

	gpio_init();
	stdout = &uart_stream;
	stdin = &uart_stream;
	uart_init();
	timer1_pwm_init();	
//	timer1_pwm_set_percent(50);
//	printf("Hi, how are you?\n\r");
//	interrupt_init();
//	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	
	while (1) {
		PORTD^=1;
		printf("Hi, how are you?\n\r");
		_delay_ms(100);
	}
	
	return 0;
}
