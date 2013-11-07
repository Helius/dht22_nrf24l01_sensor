#include <avr/io.h>

/*
	Atmega 328 at 16MHz, PWM for servo, period 20mc, max duration 1-2 mc
*/

void timer1_pwm_init() {
	DDRB   |= 1 << PB2;                 // set OC1B on PB2 to output 

	TCCR1A  = (1<<WGM11) | (1<<WGM10);  // fast pwm mode where TOP is OCR1A
	TCCR1A |= (1<<COM1A1) | (0<<COM1A0);// OC1A clear on match, set at BOTTOM 
	TCCR1A |= (1<<COM1B1) | (0<<COM1B0);// OC1A clear on match, set at BOTTOM 
	
	TCCR1B  = (1<<WGM13) | (1<<WGM12);  // fast pwm mode where TOP is OCR1A
	TCCR1B |= (1<<CS10);                // set prescaller /8 (start timer)

	OCR1A = 0x0250; // set TOP (period)
	OCR1B = 0x3; // set durations
	//OCR1B = 0x0100; // min value
	//OCR1B = 0x1550; // max value
}

void timer1_pwm_stop() {
	TCCR1B = 0;
}

void timer1_pwm_set_percent (uint8_t value) {
	if (value > 100) {
		value=100;
	}
	if (value < 0) {
		value = 0;
	}
	OCR1B = 0x100 + ((0x1550-0x100)*value)/100;
}

