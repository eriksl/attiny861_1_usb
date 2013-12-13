#include <stdint.h>
#include <avr/io.h>

#include "ioports.h"

const adcport_t temp_ports[TEMP_PORTS] = 
{
	{							// internal temp sensor
		{ 1, 1, 1, 1, 1, 1 }	// mux = adc11 = 111111
	},
#if (BOARD == 0)
	{							// pa6 pin 12 adc input 2, tmp36
		{ 1, 0, 1, 0, 0, 0 },	// mux = adc5 = 000101
	},
#endif
#if (BOARD == 1)
    {                           // pa4 pin 14 adc input 3
        { 1, 1, 0, 0, 0, 0 },   // mux = adc3
    },
#endif
};

const adcport_t analog_ports[ANALOG_PORTS] = 
{
#if (BOARD == 0 || BOARD == 1)
	{							// pa7 pin 11 adc input 1
		{ 0, 1, 1, 0, 0, 0 },	// mux = adc6 = 000110
	},
#endif
};

const ioport_t input_ports[INPUT_PORTS] =
{
#if (BOARD == 0)
	{ &PORTA, &PINA, &DDRA, 4, &PCMSK0, PCINT4,  PCIE1 },	// a4	input 1
	{ &PORTA, &PINA, &DDRA, 5, &PCMSK0, PCINT5,  PCIE1 },	// a5	input 2
	{ &PORTB, &PINB, &DDRB, 6, &PCMSK1, PCINT14, PCIE1 },	// b6	input 3
#endif
#if (BOARD == 1)
	{ &PORTB, &PINB, &DDRB, 6, &PCMSK1, PCINT14, PCIE1 },	// b6	input 3
	{ &PORTA, &PINA, &DDRA, 6, &PCMSK0, PCINT6,  PCIE1 },	// a6	input 2
#endif
};

const ioport_t output_ports[OUTPUT_PORTS] =
{
#if (BOARD == 0)
	{ &PORTB, &PINB, &DDRB, 2 }		// b2	output 2
#endif
#if (BOARD == 1)
	{ &PORTB, &PINB, &DDRB, 0 },	// b0	output 1
#endif
#if (BOARD == 2)
	{ &PORTA, &PINA, &DDRA, 2, &PCMSK0, PCINT2, PCIE1 },    // a2
	{ &PORTA, &PINA, &DDRA, 7, &PCMSK0, PCINT7, PCIE1 },    // a7
	{ &PORTA, &PINA, &DDRA, 6, &PCMSK0, PCINT6, PCIE1 },    // a6
	{ &PORTA, &PINA, &DDRA, 5, &PCMSK0, PCINT5, PCIE1 },    // a5
#endif
};

const ioport_t usb_ports[USB_PORTS] =
{
#if (BOARD == 0)
	{ &PORTA, &PINA, &DDRA, 2 },	// a2	d-
	{ &PORTA, &PINA, &DDRA, 1 }		// a1	d+
#endif
#if (BOARD == 1)
	{ &PORTA, &PINA, &DDRA, 2 },	// a2	d-
	{ &PORTA, &PINA, &DDRA, 0 }		// a0	d+
#endif
#if (BOARD == 2)
    { &PORTB, &PINB, &DDRB, 6 },    // b6   d-
    { &PORTB, &PINB, &DDRB, 3 }     // b3   d+
#endif
};

const ioport_t internal_output_ports[INTERNAL_OUTPUT_PORTS] =
{
#if (BOARD == 0)
	{ &PORTA, &PINA, &DDRA, 3 },	// a3	input sense
	{ &PORTA, &PINA, &DDRA, 0 },	// a0	command sense
#endif
#if (BOARD == 1)
	{ &PORTA, &PINA, &DDRA, 3 },	// a3
	{ &PORTB, &PINB, &DDRB, 4 },	// b4
#endif
#if (BOARD == 2)
	{ &PORTA, &PINA, &DDRA, 0 },    // a0
	{ &PORTA, &PINA, &DDRA, 1 },    // a1
#endif
};

const pwmport_t pwm_ports[PWM_PORTS] =
{
#if ((BOARD == 0) || (BOARD == 1))
	{ &PORTB, &DDRB, 1, &TCCR1A, COM1A1, COM1A0, FOC1A, PWM1A, &TC1H, &OCR1A },	// b1	pwm output 1
	{ &PORTB, &DDRB, 3, &TCCR1A, COM1B1, COM1B0, FOC1B, PWM1B, &TC1H, &OCR1B },	// b3	pwm output 2
#endif
};
