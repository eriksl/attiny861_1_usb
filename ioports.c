#include <stdint.h>
#include <avr/io.h>

#include "ioports.h"

const adcport_t adc_ports[ADC_PORTS] = 
{
	{							// pa7 pin 11 adc input 1
		{ 0, 0, 0 },			// Vref = Vcc
		{ 0, 1, 1, 0, 0, 0 },	// mux = adc6 = 000110
	},
	{							// pa6 pin 12 adc input 2, tmp36
		{ 0, 1, 0 },			// Vref = internal 1.1V reference
		{ 1, 0, 1, 0, 0, 0 },	// mux = adc5 = 000101
	},
	{							// internal temp sensor
		{ 0, 1, 0, },			// Vref = internal 1.1V reference
		{ 1, 1, 1, 1, 1, 1 }	// mux = adc11 = 111111
	}
};

const ioport_t input_ports[INPUT_PORTS] =
{
	{ &PORTA, &PINA, &DDRA, 4, &PCMSK0, PCINT4,  PCIE1 },	// a4	input 1
	{ &PORTA, &PINA, &DDRA, 5, &PCMSK0, PCINT5,  PCIE1 },	// a5	input 2
	{ &PORTB, &PINB, &DDRB, 6, &PCMSK1, PCINT14, PCIE1 },	// b6	input 3
};

const ioport_t output_ports[OUTPUT_PORTS] =
{
	{ &PORTB, &PINB, &DDRB, 0 },	// b0	output 1
	{ &PORTB, &PINB, &DDRB, 2 }		// b2	output 2
};

const ioport_t usb_ports[USB_PORTS] =
{
	{ &PORTA, &PINA, &DDRA, 2 },	// a2	d-
	{ &PORTA, &PINA, &DDRA, 1 }		// a1	d+
};

const ioport_t internal_output_ports[INTERNAL_OUTPUT_PORTS] =
{
	{ &PORTA, &PINA, &DDRA, 3 },	// a3	input sense led
	{ &PORTA, &PINA, &DDRA, 0 },	// a0	command sense led
};

const pwmport_t pwm_ports[PWM_PORTS] =
{
	{ &PORTB, &DDRB, 1, &TCCR1A, COM1A1, COM1A0, FOC1A, PWM1A, &TC1H, &OCR1A },	// b1	pwm output 1
	{ &PORTB, &DDRB, 3, &TCCR1A, COM1B1, COM1B0, FOC1B, PWM1B, &TC1H, &OCR1B },	// b3	pwm output 2
};
