#include <stdlib.h>
#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>

#include "uart.h"

#define SETBIT(reg,bit) (reg|=1<<bit)
#define CLRBIT(reg,bit) (reg&=~(1<<bit))


#define LED    5       //PORTB
#define LAMP0  3       //PORTD
#define LAMP1  4       //PORTD
#define WAKEUP 2       //PORTD

//#define LED_ENABLE


void gpio_init (void) {
#ifdef LED_ENABLE
	SETBIT(DDRB,LED);
	CLRBIT(PORTB,LED);
#endif
	SETBIT(DDRD,LAMP0);
	SETBIT(DDRD,LAMP1);
	
	CLRBIT(PORTD,LAMP0);
	CLRBIT(PORTD,LAMP1);
}

void interrupt_init() {
	//EICRA |= (1 << ISC00);    // set INT0 to trigger on ANY logic change
	EIMSK |= (1 << INT0);     // Turns on INT0
	SETBIT(PORTD,WAKEUP);     // enable pull-up
	CLRBIT(DDRD,WAKEUP);      // set as input
}

void set_lamp (uint16_t level) {
		if (rand()%255 < level) {
#ifdef LED_ENABLE
			SETBIT(PORTB,LED);
#endif
			CLRBIT(PORTD,LAMP0);
		} else {
#ifdef LED_ENABLE
			CLRBIT(PORTB,LED);
#endif
			SETBIT(PORTD,LAMP0);
		}
		if (rand()%255 < level) {
			CLRBIT(PORTD,LAMP1);
		} else {
			SETBIT(PORTD,LAMP1);
		}
}

void start_ignition () {
	uint16_t ignition_level = 0;
	uint16_t light_timer = 0;
	while(ignition_level++ < 250) {
		set_lamp(ignition_level);
//		printf ("level: %d\n\r", ignition_level);
		_delay_ms (50);	
		if (ignition_level > 100) {
			ignition_level+=5;
		}
	}
	while(light_timer++ < 500) {
		set_lamp(ignition_level);
//		printf ("timer: %d\n\r", light_timer);
		_delay_ms (50);	
	}
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
	adc_init();
	interrupt_init();
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	stdout = &uart_stream;
	stdin = &uart_stream;
	printf("Hi, how are you?\n\r");
	uint16_t first_adc_value = adc_get(3);
	printf("adc on 0 channel: %d\n\r", first_adc_value);
	srand (first_adc_value);
	while (1) {
		start_ignition();
		printf ("will sleep, by-by\n\r");
	#ifdef LED_ENABLE
		CLRBIT(PORTB,LED); // turn off led
	#endif
		CLRBIT(PORTD,LAMP0);
		CLRBIT(PORTD,LAMP1);
		CLRBIT(DDRD,LAMP0);
		CLRBIT(DDRD,LAMP1);
		ADCSRA = 0;// turn off ADC
		go_sleep();
		gpio_init();
		adc_init();
	}
	return 0;
}
