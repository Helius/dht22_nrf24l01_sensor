#include <stdlib.h>
#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include "uart.h"
#include "dht.h"

#define DHTpin 3
#define DHTpower 4

#define SETBIT(reg,bit) (reg|=1<<bit)
#define CLRBIT(reg,bit) (reg&=~(1<<bit))
#define TSTBIT(reg,bit) (reg&(1<<bit))



void gpio_init() {
		DDRB |= 1<<PB5;
		DDRD |= 1<<DHTpower;
		PORTD|= 1<<DHTpower;
		//SETBIT(PORTD,DHTpin);
		//CLRBIT(DDRD,DHTpin);
		CLRBIT(PORTD,DHTpin);
		SETBIT(DDRD,5);
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

#define DHT_PIN(reg) BIT(C, 0, reg)

static inline int16_t dhtproc(dht_request_t req, uint16_t arg){
	switch(req){
		case DHT_READ_PIN:
		{
			CLRBIT(DDRD,3);
	//		SETBIT(PORTD,5);
			int val = bit_is_set(PIND,3);
	//		_delay_us(1);
	//		CLRBIT(PORTD,5);

			return val;
		}
			break;
		case DHT_WRITE_PIN:
			if (arg) {
				//SETBIT(PORTD,DHTpin);
				CLRBIT(DDRD,DHTpin);
			}	else {
				SETBIT(DDRD,DHTpin);
			}
			return 0;
			break;
		case DHT_DELAY_MS:
			while(arg--) {
				_delay_ms(1);
			}
			break;
		case DHT_DELAY_US:
			while(arg--) {
				_delay_us(1);
			}
			break;
		default:
			return -1;
	}
	return 0;
}




int main (void) {
	dht_t dht;
	gpio_init();
	stdout = &uart_stream;
	stdin = &uart_stream;
	uart_init();
	printf("Hi, how are you?\n\r");
	DHT_Init(&dht, dhtproc);
	while(1){
		_delay_ms(3000);
		PORTB^=(1<<5);
		if(DHT_Read22(&dht) == DHTLIB_OK){
			printf("data: T:%d, H:%d\r\n", (int)dht.temperature, (int)dht.humidity);
		} else {
			printf("-\r\n");
		}
	}


//	interrupt_init();
//	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	
	return 0;
}
