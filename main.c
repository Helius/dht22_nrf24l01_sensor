#include <stdlib.h>
#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include "uart.h"
#include "dht.h"
#include <nrf24L01_plus/nrf24.h>

#define DHTpin 3
#define DHTpower 4
#define NRF24_POWER 0

#define SETBIT(reg,bit) (reg|=1<<bit)
#define CLRBIT(reg,bit) (reg&=~(1<<bit))
#define TSTBIT(reg,bit) (reg&(1<<bit))

#define SET_LED SETBIT(PORTB,PB5) // led
#define CLR_LED CLRBIT(PORTB,PB5) // led

#define DEBUG_ENABLE 0


void gpio_init() {
	//SETBIT(PORTB, NRF24_POWER);
	//SETBIT(DDRB, NRF24_POWER);
	DDRB = 0;
	PORTB = 0;
	DDRC = 0;
	PORTC = 0;
	DDRD = 0;
	PORTD = 0;
	
	SETBIT(DDRD,  DHTpower);
	SETBIT(PORTD, DHTpower);

	CLRBIT(PORTD, DHTpin);
	
	SETBIT(DDRB,PB5); // led
	
	DIDR0 = 0xFF; // disable digital buffer on ADC channel
}

void adc_init() {

	CLRBIT(PRR,PRADC); // enable adc power
	DIDR0 = 0; // enable digital buffer on ADC pin

	CLRBIT(PORTC,4); // connect GND to meagurment circuit
	SETBIT(DDRC,4);
	
	ADMUX = (1<<REFS1) | (1<<REFS0) | (1<<ADLAR) | 5; // use 5 channel 
	ADCSRA = (1<<ADEN) | (0<<ADIE) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);
}

// Now it consumes about 160 uA in sleep mode
void prepare_sleep() {
	
	CLRBIT(DDRC,4); // GND for battery check circuit
	CLRBIT(PORTC,4);
	
	//CLRBIT(DDRD,  DHTpower);
	//CLRBIT(PORTD, DHTpower);

	// disable digital buffer on ADC pin
	DIDR0 = 0xFF;
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
	WDTCSR = (1<<WDIE) | (1<<WDP3) | (0<<WDP0); // set WDIE, and 4 seconds delay (8 max)
	//WDTCSR = (1<<WDIE) | (1<<WDP2) | (1<<WDP1) | (1<<WDP0);  // set WDIE, and 2 seconds delay for debug
	wdt_reset();  // pat the dog
}

void adc_do_convertion () {
	if (ADCSRA & (1<<ADEN))
		ADCSRA |= (1<<ADSC);
}

uint8_t battery_level __attribute__ ((section (".noinit")));
void update_battery_level() {
	if (ADCSRA & (1<<ADEN)) {
		while (!(ADCSRA & (1<<ADIF)));
		battery_level = ADCH;
	}
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
			CLRBIT(DDRD,DHTpin);
			int val = bit_is_set(PIND,DHTpin);

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
uint8_t data_array[8];
uint8_t tx_address[5] = {0xE7,0xE7,0xE7,0xE7,0xE7};
uint8_t rx_address[5] = {0xD7,0xD7,0xD7,0xD7,0xD7};
uint8_t Vbat = 0;
uint8_t battery_check_count __attribute__ ((section (".noinit")));
uint8_t wakeupCnt __attribute__ ((section (".noinit")));
uint8_t radioLostCnt __attribute__ ((section (".noinit")));

#define SLEEP_CNT 10

int main (void) {
	wakeupCnt++;	
	// за один цикл до просыпания (4с включаем питалово датчику, иначе он глючит и выдает пустые показания)
	// поэтому цикл сделал 4с не максимальный 8с
	if (wakeupCnt == (SLEEP_CNT-1)) {
		SETBIT(DDRD,  DHTpower);
		SETBIT(PORTD, DHTpower);
	}
	
	if (wakeupCnt < SLEEP_CNT) {
		go_sleep();
	}
	wakeupCnt = 0;

	dht_t dht;
	gpio_init();
	SET_LED;

	if (battery_check_count == 0) {
		adc_init();
		battery_check_count = 10;
	} else {
		battery_check_count--;
	}
	

#if DEBUG_ENABLE
	// enable USART
	CLRBIT(PRR,PRUSART0);
	stdout = &uart_stream;
	stdin = &uart_stream;
	uart_init();
	printf("Hi, how are you %d?\n\r", wakeupCnt);
#endif


	DHT_Init(&dht, dhtproc);

	/* init hardware pins */
	nrf24_init();
	/* Channel #2 , payload length: 4 */
	nrf24_config(2,8);
	/* Set the device addresses */
	nrf24_tx_address(tx_address);
	nrf24_rx_address(rx_address);    

	while (1) {
		if(DHT_Read22(&dht) == DHTLIB_OK){
			//printf("dht read ok\r\n", dht._bits[0], dht._bits[1], dht._bits[2], dht._bits[3]);
		} else {
			//printf("dht read fail, retry\r\n");
			DHT_Read22(&dht);
		}
		CLRBIT(DDRD,  DHTpower);
		CLRBIT(PORTD, DHTpower);
		
		data_array[0] = 0x01;         // kind: DHT22 = 1
		data_array[1] = 0x02;         // UID
		data_array[2] = dht._bits[0]; // humidity_0
		data_array[3] = dht._bits[1]; // humidiry_1
		data_array[4] = dht._bits[2]; // tempr_0
		data_array[5] = dht._bits[3]; // tempr_1
		data_array[6] = battery_level;// напряжение батареек  (x 100мВ)
		data_array[7] = radioLostCnt; // number of trying to send

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
		CLR_LED;

		/* Retranmission count indicates the tranmission quality */
		radioLostCnt = nrf24_retransmissionCount();
		//printf("> Retranmission count: %d\r\n",temp);
		adc_do_convertion();
		if (radioLostCnt > 0) {
			_delay_ms(80);
			SET_LED;
			_delay_ms(80);
			CLR_LED;
		}
		update_battery_level();
		go_sleep();
	}
	return 0;
}
