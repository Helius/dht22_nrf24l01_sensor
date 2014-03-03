#include <stdlib.h>
#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include "uart.h"
#include "dht.h"
#include "nrf24L01_plus/nrf24.h"

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

void prepare_sleep() {
	// disable ADC
	ADCSRA = 0;  
	// turn off various modules
	PRR = 0xFF; 
	// turn off brown-out enable in software
	MCUCR = (1<<BODS)|(1<<BODSE);
	MCUCR = 1<<BODS; 
	// clear various "reset" flags
	MCUSR = 0;     
	// allow changes, disable reset
	WDTCSR = (1<<WDCE) | (1<<WDE);
	// set interrupt mode and an interval 
	WDTCSR = (1<<WDIE) | (1<<WDP3) | (1<<WDP0);    // set WDIE, and 8 seconds delay
	wdt_reset();  // pat the dog
}

void go_sleep () {
	nrf24_powerDown();
	prepare_sleep();
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
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
			int val = bit_is_set(PIND,3);

			return val;
		}
			break;
		case DHT_WRITE_PIN:
			if (arg) {
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

uint8_t temp;
uint8_t q = 0;
uint8_t data_array[8];
uint8_t tx_address[5] = {0xE7,0xE7,0xE7,0xE7,0xE7};
uint8_t rx_address[5] = {0xD7,0xD7,0xD7,0xD7,0xD7};
uint8_t Vbat = 0;
uint8_t wakeupCnt __attribute__ ((section (".noinit")));

int main (void) {
	
	if (wakeupCnt++ < 3) {
		go_sleep();
	}
	wakeupCnt = 0;

	SETBIT(PORTB,PB5);
	dht_t dht;
	gpio_init();
	stdout = &uart_stream;
	stdin = &uart_stream;
	uart_init();

//	printf("Hi, how are you %d?\n\r", wakeupCnt);

	DHT_Init(&dht, dhtproc);

	/* init hardware pins */
	nrf24_init();
	/* Channel #2 , payload length: 4 */
	nrf24_config(2,8);
	/* Set the device addresses */
	nrf24_tx_address(tx_address);
	nrf24_rx_address(rx_address);    

	while(1){
		if(DHT_Read22(&dht) == DHTLIB_OK){
			printf("data: T:%f, H:%f\r\n", dht.temperature, dht.humidity);
		} else {
			//printf("-\r\n");
		}
		
		data_array[0] = 0x01;         // kind: DHT22 = 1
		data_array[1] = 0x02;         // UID
		data_array[2] = dht._bits[0]; // humidity_0
		data_array[3] = dht._bits[1]; // humidiry_1
		data_array[4] = dht._bits[2]; // tempr_0
		data_array[5] = dht._bits[3]; // tempr_1
		data_array[6] = Vbat;
		data_array[7] = temp;

		/* Automatically goes to TX mode */
		nrf24_send(data_array);

		/* Wait for transmission to end */
		while(nrf24_isSending());

		/* Make analysis on last tranmission attempt */
		temp = nrf24_lastMessageStatus();

		if(temp == NRF24_TRANSMISSON_OK) {
			//printf("> Tranmission went OK\r\n");
		}
		else if(temp == NRF24_MESSAGE_LOST)
		{
			//printf("> Message is lost ...\r\n");
		}
		CLRBIT(PORTB,PB5);

		/* Retranmission count indicates the tranmission quality */
		temp = nrf24_retransmissionCount();
		//printf("> Retranmission count: %d\r\n",temp);
		if (temp > 0) {
			SETBIT(PORTB,PB5);
			_delay_ms(500);
			CLRBIT(PORTB,PB5);
		}
		go_sleep();
	}
	return 0;
}
