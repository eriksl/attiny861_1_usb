#include <stdint.h>
#include <string.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>

#include "usbdrv.h"

#include "ioports.h"
#include "adc.h"
#include "timer0.h"
#include "pwm_timer1.h"

typedef enum
{
	pwm_mode_none				= 0,
	pwm_mode_fade_in			= 1,
	pwm_mode_fade_out			= 2,
	pwm_mode_fade_in_out_cont	= 3,
	pwm_mode_fade_out_in_cont	= 4
} pwm_mode_t;

typedef struct
{
	uint8_t		duty;
	pwm_mode_t	pwm_mode:8;
} pwm_meta_t;

typedef struct
{
	uint32_t	counter;
	uint8_t		state;
} counter_meta_t;

static	const		ioport_t		*ioport;
static				pwm_meta_t		softpwm_meta[OUTPUT_PORTS];
static				pwm_meta_t		pwm_meta[PWM_PORTS];
static				pwm_meta_t		*pwm_slot;
static				counter_meta_t	counter_meta[INPUT_PORTS];

static	uint8_t		usb_running;
static	uint8_t		slot, duty, next_duty;
static	uint8_t		timer0_value, timer0_debug_1, timer0_debug_2;

static uint8_t		receive_buffer[32];
static uint8_t		receive_buffer_to_fetch	= 0;
static uint8_t		receive_buffer_length	= 0;
static uint8_t		receive_buffer_complete	= 0;

static uint8_t		send_buffer[32];
static uint8_t		send_buffer_sent		= 0;
static uint8_t		send_buffer_length		= 0;

static void put_word(uint16_t from, uint8_t *to)
{
	to += 2;

	*(--to) = from & 0xff;
	from >>= 8;
	*(--to) = from & 0xff;
}

static void put_long(uint32_t from, uint8_t *to)
{
	to += 4;

	*(--to) = from & 0xff;
	from >>= 8;
	*(--to) = from & 0xff;
	from >>= 8;
	*(--to) = from & 0xff;
	from >>= 8;
	*(--to) = from & 0xff;
}

ISR(TIMER0_COMPA_vect) // timer 0 softpwm overflow
{
	pwm_slot	= &softpwm_meta[0];
	ioport		= &output_ports[0];

	for(slot = OUTPUT_PORTS; slot > 0; slot--)
	{
		if(pwm_slot->duty == 0)				// pwm duty == 0, port is off, set it off
			*ioport->port &= ~_BV(ioport->bit);
		else									// else set the port on
			*ioport->port |=  _BV(ioport->bit);

		pwm_slot++;
		ioport++;
	}
}

ISR(TIMER0_COMPB_vect) // timer 0 softpwm trigger
{
	timer0_value = timer0_get_counter();

	if(timer0_value < 253)
		timer0_value += 1;
	else
		timer0_value = 255;

	next_duty = 255;

	pwm_slot	= &softpwm_meta[0];
	ioport		= &output_ports[0];

	for(slot = OUTPUT_PORTS; slot > 0; slot--)
	{
		if(pwm_slot->duty <= timer0_value)
			*ioport->port &= ~_BV(ioport->bit);
		else
			if(pwm_slot->duty < next_duty)
				next_duty = pwm_slot->duty;

		pwm_slot++;
		ioport++;
	}

	if(next_duty == 255)
	{
		next_duty = 255;

		pwm_slot = &softpwm_meta[0];

		for(slot = OUTPUT_PORTS; slot > 0; slot--)
		{
			if((pwm_slot->duty != 0) && (pwm_slot->duty < next_duty))
				next_duty = pwm_slot->duty;

			pwm_slot++;
		}

		if(next_duty == 255)
			timer0_stop();
	}

	timer0_set_trigger(next_duty);
}

ISR(PCINT_vect, ISR_NOBLOCK)
{
	static			uint8_t			pc_dirty, pc_slot;
	static const	ioport_t		*pc_ioport;
	static			counter_meta_t	*pc_counter_slot;

	if(!usb_running)
		return;

	for(pc_slot = 0, pc_dirty = 0; pc_slot < INPUT_PORTS; pc_slot++)
	{
		pc_ioport		= &input_ports[pc_slot];
		pc_counter_slot	= &counter_meta[pc_slot];

		if((*pc_ioport->pin & _BV(pc_ioport->bit)) ^ pc_counter_slot->state)
		{
			pc_counter_slot->counter++;
			pc_dirty = 1;
		}

		pc_counter_slot->state = *pc_ioport->pin & _BV(pc_ioport->bit);
	}

	if(pc_dirty)
		*internal_output_ports[1].port |= _BV(internal_output_ports[1].bit);
}

static void update_static_softpwm_ports(void)
{
	static pwm_meta_t		*usp_pwm_slot	= &softpwm_meta[0];
	static const ioport_t	*usp_ioport		= &output_ports[0];
	static uint8_t			usp_slot;

	for(usp_slot = OUTPUT_PORTS; usp_slot > 0; usp_slot--)
	{
		if(usp_pwm_slot->duty == 0)
			*usp_ioport->port &= ~_BV(usp_ioport->bit);
		else if(usp_pwm_slot->duty == 255)
			*usp_ioport->port |= _BV(usp_ioport->bit);

		usp_pwm_slot++;
		usp_ioport++;
	}
}

static inline void process_pwmmode(void)
{
	static pwm_meta_t	*pm_pwm_slot;
	static uint8_t		pm_dirty, pm_slot, pm_duty, pm_diff;
	static uint16_t		pm_duty16, pm_diff16;

	pm_dirty = 0;
	pm_pwm_slot = &softpwm_meta[0];

	for(pm_slot = OUTPUT_PORTS; pm_slot > 0; pm_slot--)
	{
		pm_duty	= pm_pwm_slot->duty;
		pm_diff	= pm_duty / 10;

		if(pm_diff < 3)
			pm_diff = 3;

		switch(pm_pwm_slot->pwm_mode)
		{
			case(pwm_mode_fade_in):
			case(pwm_mode_fade_in_out_cont):
			{
				if(pm_duty < (255 - pm_diff))
					pm_duty += pm_diff;
				else
				{
					pm_duty = 255;

					if(pm_pwm_slot->pwm_mode == pwm_mode_fade_in)
						pm_pwm_slot->pwm_mode = pwm_mode_none;
					else
						pm_pwm_slot->pwm_mode = pwm_mode_fade_out_in_cont;
				}

				pm_pwm_slot->duty = pm_duty;

				pm_dirty = 1;

				break;
			}

			case(pwm_mode_fade_out):
			case(pwm_mode_fade_out_in_cont):
			{
				if(pm_duty > pm_diff)
					pm_duty -= pm_diff;
				else
				{
					pm_duty = 0;

					if(pm_pwm_slot->pwm_mode == pwm_mode_fade_out)
						pm_pwm_slot->pwm_mode = pwm_mode_none;
					else
						pm_pwm_slot->pwm_mode = pwm_mode_fade_in_out_cont;
				}

				pm_pwm_slot->duty = pm_duty;

				pm_dirty = 1;

				break;
			}
		}

		pm_pwm_slot++;
	}

	if(pm_dirty)
	{
		update_static_softpwm_ports();
		timer0_start();
	}

	pm_pwm_slot = &pwm_meta[0];

	for(pm_slot = 0; pm_slot < PWM_PORTS; pm_slot++)
	{
		pm_duty16	= pwm_timer1_get_pwm(pm_slot);
		pm_diff16	= pm_duty16 / 8;

		if(pm_diff16 < 8)
			pm_diff16 = 8;

		switch(pm_pwm_slot->pwm_mode)
		{
			case(pwm_mode_fade_in):
			case(pwm_mode_fade_in_out_cont):
			{
				if(pm_duty16 < (1020 - pm_diff16))
					pm_duty16 += pm_diff16;
				else
				{
					pm_duty16 = 1020;

					if(pm_pwm_slot->pwm_mode == pwm_mode_fade_in)
						pm_pwm_slot->pwm_mode = pwm_mode_none;
					else
						pm_pwm_slot->pwm_mode = pwm_mode_fade_out_in_cont;
				}

				pwm_timer1_set_pwm(pm_slot, pm_duty16);

				break;
			}

			case(pwm_mode_fade_out):
			case(pwm_mode_fade_out_in_cont):
			{
				if(pm_duty16 > pm_diff16)
					pm_duty16 -= pm_diff16;
				else
				{
					pm_duty16 = 0;

					if(pm_pwm_slot->pwm_mode == pwm_mode_fade_out)
						pm_pwm_slot->pwm_mode = pwm_mode_none;
					else
						pm_pwm_slot->pwm_mode = pwm_mode_fade_in_out_cont;
				}

				pwm_timer1_set_pwm(pm_slot, pm_duty16);

				break;
			}
		}

		pm_pwm_slot++;
	}

}

#if (USE_CRYSTAL == 0)
static void calibrateOscillator(void)
{
	uint8_t		step = 128;
	uint8_t		trialValue = 0, optimumValue;
	uint16_t	x, optimumDev, targetValue = (unsigned)(1499 * (double)F_CPU / 10.5e6 + 0.5);

	/* do a binary search: */

	do
	{
		OSCCAL = trialValue + step;
		x = usbMeasureFrameLength();	// proportional to current real frequency
		if(x < targetValue)				// frequency still too low
			trialValue += step;
		step >>= 1;
	}
	while(step > 0);

	/* We have a precision of +/- 1 for optimum OSCCAL here */
	/* now do a neighborhood search for optimum value */

	optimumValue	= trialValue;
	optimumDev		= x; // this is certainly far away from optimum

	for(OSCCAL = trialValue - 1; OSCCAL <= trialValue + 1; OSCCAL++)
	{
		x = usbMeasureFrameLength() - targetValue;

		if(x < 0)
			x = -x;
		if(x < optimumDev)
		{
			optimumDev = x;
			optimumValue = OSCCAL;
		}
	}

	OSCCAL = optimumValue;
}
#endif

void usbEventResetReady(void)
{
#if (USE_CRYSTAL == 0)
	cli(); // usbMeasureFrameLength() counts CPU cycles, so disable interrupts.
	calibrateOscillator();
	sei();
#endif
}

usbMsgLen_t usbFunctionSetup(uchar data[8])
{
	usbRequest_t *rq = (usbRequest_t *)data;

	if((rq->bmRequestType & USBRQ_TYPE_MASK) != USBRQ_TYPE_VENDOR)
		return(0);

	if((rq->bmRequestType & USBRQ_RCPT_MASK) != USBRQ_RCPT_ENDPOINT)
		return(0);

	if((rq->bmRequestType & USBRQ_DIR_MASK) == USBRQ_DIR_HOST_TO_DEVICE)
	{
		receive_buffer_to_fetch = rq->wLength.word;
		receive_buffer_length	= 0;
		receive_buffer_complete	= 0;
	}
	else
		send_buffer_sent		= 0;

	return(USB_NO_MSG);
}

uint8_t usbFunctionRead(uint8_t *data, uint8_t length)
{
	if((send_buffer_sent + length) > send_buffer_length)
		length = (send_buffer_length - send_buffer_sent);

	if(length > 0)
	{
		memcpy(data, send_buffer + send_buffer_sent, length);
		send_buffer_sent += length;
	}
	else
	{
		send_buffer_sent	= 0;
		send_buffer_length	= 0;
	}

	return(length);
}

uint8_t usbFunctionWrite(uint8_t *data, uint8_t length)
{
	if((receive_buffer_length + length) > sizeof(receive_buffer))
	{
		receive_buffer_to_fetch = sizeof(receive_buffer);
		length = sizeof(receive_buffer) - receive_buffer_length;
	}

	memcpy(receive_buffer + receive_buffer_length, data, length);
	receive_buffer_length += length;

	if(receive_buffer_to_fetch >= receive_buffer_length)
	{
		receive_buffer_to_fetch = 0;
		receive_buffer_complete	= 1;
		return(1);
	}

	return(0);
}

static void build_reply(uint8_t volatile *output_length, volatile uint8_t *output,
		uint8_t command, uint8_t error_code, uint8_t reply_length, const uint8_t *reply_string)
{
	uint8_t checksum;
	uint8_t ix;

	output[0] = 3 + reply_length;
	output[1] = error_code;
	output[2] = command;

	for(ix = 0; ix < reply_length; ix++)
		output[3 + ix] = reply_string[ix];

	for(ix = 1, checksum = 0; ix < (3 + reply_length); ix++)
		checksum += output[ix];

	output[3 + reply_length] = checksum;
	*output_length = 3 + reply_length + 1;
}

static void extended_command(uint8_t buffer_size, volatile uint8_t input_buffer_length, const volatile uint8_t *input_buffer,
						uint8_t volatile *output_buffer_length, volatile uint8_t *output_buffer)
{
	uint8_t command = input_buffer[1];

	if(command < 5)
	{
		struct
		{
			uint8_t	amount;
			uint8_t	data[4];
		} control_info;

		switch(input_buffer[1])
		{
			case(0x00):	// get digital inputs
			{
				control_info.amount = INPUT_PORTS;
				put_long(0x3fffffff, &control_info.data[0]);
				break;
			}

			case(0x01):	// get analog inputs
			{
				control_info.amount = ADC_PORTS;
				put_word(0x0000, &control_info.data[0]);
				put_word(0x03ff, &control_info.data[2]);
				break;
			}

			case(0x02):	// get digital outputs
			{
				control_info.amount = OUTPUT_PORTS;
				put_word(0x0000, &control_info.data[0]);
				put_word(0x00ff, &control_info.data[2]);
				break;
			}

			case(0x03):	// get pwm outputs
			{
				control_info.amount = PWM_PORTS;
				put_word(0x0000, &control_info.data[0]);
				put_word(0x03ff, &control_info.data[2]);
				break;
			}

			default:
			{
				return(build_reply(output_buffer_length, output_buffer, input_buffer[0], 7, 0, 0));
			}
		}

		return(build_reply(output_buffer_length, output_buffer, input_buffer[0], 0, sizeof(control_info), (uint8_t *)&control_info));
	}

	return(build_reply(output_buffer_length, output_buffer, input_buffer[0], 7, 0, 0));
}

static void process_input(uint8_t buffer_size, volatile uint8_t input_buffer_length, const volatile uint8_t *input_buffer,
						uint8_t volatile *output_buffer_length, volatile uint8_t *output_buffer)
{
	uint8_t input;
	uint8_t	command;
	uint8_t	io;

	*internal_output_ports[0].port |= _BV(internal_output_ports[0].bit);

	if(input_buffer_length < 1)
		return(build_reply(output_buffer_length, output_buffer, 0, 1, 0, 0));

	input	= input_buffer[0];
	command	= input & 0xf8;
	io		= input & 0x07;

	switch(command)
	{
		case(0x00):	// short / no-io
		{
			switch(io)
			{
				case(0x00):	// identify
				{
					struct
					{
						uint8_t id1, id2;
						uint8_t model, version, revision;
						uint8_t name[16];
					} reply =
					{
						0x4a, 0xfb,
						0x06, 0x01, 0x01,
						"attiny861a",
					};

					return(build_reply(output_buffer_length, output_buffer, input, 0, sizeof(reply), (uint8_t *)&reply));
				}

				case(0x01):	// 0x02 read ADC
				{
					uint16_t value;

					value = ADCW;

					if(ADCSRA & _BV(ADSC))	// conversion not ready
						return(build_reply(output_buffer_length, output_buffer, input, 5, 0, 0));

					adc_stop();

					uint8_t replystring[2];

					put_word(value, replystring);

					return(build_reply(output_buffer_length, output_buffer, input, 0, sizeof(replystring), replystring));
				}

				case(0x02): // 0x02 DEBUG read timer0 counter
				{
					uint8_t value = timer0_get_counter();
					return(build_reply(output_buffer_length, output_buffer, input, 0, 1, &value));
				}

				case(0x03): // 0x03 DEBUG read timer1 counter
				{
					uint16_t value = pwm_timer1_get_counter();
					uint8_t replystring[2];

					put_word(value, replystring);

					return(build_reply(output_buffer_length, output_buffer, input, 0, sizeof(replystring), replystring));
				}

				case(0x04): // 0x04 DEBUG read timer1 max
				{
					uint16_t value = pwm_timer1_get_max();
					uint8_t replystring[2];

					put_word(value, replystring);

					return(build_reply(output_buffer_length, output_buffer, input, 0, sizeof(replystring), replystring));
				}

				case(0x05): // 0x05 read timer1 prescaler
				{
					uint8_t value = pwm_timer1_status();

					return(build_reply(output_buffer_length, output_buffer, input, 0, sizeof(value), &value));
				}

				case(0x06): // 0x06 DEBUG read timer0 entry / exit counter values
				{
					uint8_t value[2];

					value[0] = timer0_debug_1;
					value[1] = timer0_debug_2;

					return(build_reply(output_buffer_length, output_buffer, input, 0, sizeof(value), value));
				}

				case(0x07): // extended command
				{
					return(extended_command(buffer_size, input_buffer_length, input_buffer, output_buffer_length, output_buffer));
				}

				default:
				{
					return(build_reply(output_buffer_length, output_buffer, input, 7, 0, 0));
				}
			}

			break;
		}

		case(0x10):	// 0x10 read counter
		case(0x20): // 0x20 read / reset counter
		{
			if(io >= INPUT_PORTS)
				return(build_reply(output_buffer_length, output_buffer, input, 3, 0, 0));

			uint8_t		replystring[4];
			uint32_t	counter = counter_meta[io].counter;

			if(command == 0x20)
				counter_meta[io].counter = 0;

			put_long(counter, replystring);

			return(build_reply(output_buffer_length, output_buffer, input, 0, sizeof(replystring), replystring));
		}

		case(0x30):	//	read input
		{
			uint8_t value;

			if(io >= INPUT_PORTS)
				return(build_reply(output_buffer_length, output_buffer, input, 3, 0, 0));

			ioport	= &input_ports[io];
			value	= *ioport->pin & _BV(ioport->bit) ? 0xff : 0x00;

			return(build_reply(output_buffer_length, output_buffer, input, 0, 1, &value));
		}

		case(0x40):	//	write output / softpwm
		{
			if(input_buffer_length < 2)
				return(build_reply(output_buffer_length, output_buffer, input, 4, 0, 0));

			if(io >= OUTPUT_PORTS)
				return(build_reply(output_buffer_length, output_buffer, input, 3, 0, 0));

			softpwm_meta[io].duty = input_buffer[1];
			update_static_softpwm_ports();
			timer0_start();
			duty = softpwm_meta[io].duty;

			return(build_reply(output_buffer_length, output_buffer, input, 0, sizeof(duty), &duty));
		}

		case(0x50):	// read output / softpwm
		{
			if(io >= OUTPUT_PORTS)
				return(build_reply(output_buffer_length, output_buffer, input, 3, 0, 0));

			duty = softpwm_meta[io].duty;

			return(build_reply(output_buffer_length, output_buffer, input, 0, sizeof(duty), &duty));
		}

		case(0x60): // write softpwm mode
		{
			if(input_buffer_length < 2)
				return(build_reply(output_buffer_length, output_buffer, input, 4, 0, 0));

			if(io >= OUTPUT_PORTS)
				return(build_reply(output_buffer_length, output_buffer, input, 3, 0, 0));

			uint8_t mode = input_buffer[1];

			if(mode > 3)
				return(build_reply(output_buffer_length, output_buffer, input, 3, 0, 0));

			softpwm_meta[io].pwm_mode = mode;

			return(build_reply(output_buffer_length, output_buffer, input, 0, sizeof(mode), &mode));
		}

		case(0x70):	// read softpwm mode
		{
			if(io >= OUTPUT_PORTS)
				return(build_reply(output_buffer_length, output_buffer, input, 3, 0, 0));

			uint8_t mode;

			mode = softpwm_meta[io].pwm_mode;

			return(build_reply(output_buffer_length, output_buffer, input, 0, sizeof(mode), &mode));
		}

		case(0x80): // write pwm
		{
			if(input_buffer_length < 3)
				return(build_reply(output_buffer_length, output_buffer, input, 4, 0, 0));

			if(io >= PWM_PORTS)
				return(build_reply(output_buffer_length, output_buffer, input, 3, 0, 0));

			uint16_t value;

			value = input_buffer[1];
			value <<= 8;
			value |= input_buffer[2];

			pwm_timer1_set_pwm(io, value);

			return(build_reply(output_buffer_length, output_buffer, input, 0, 0, 0));
		}

		case(0x90): // read pwm
		{
			if(io >= PWM_PORTS)
				return(build_reply(output_buffer_length, output_buffer, input, 3, 0, 0));

			uint16_t value = pwm_timer1_get_pwm(io);
			uint8_t reply[2];

			put_word(value, reply);

			return(build_reply(output_buffer_length, output_buffer, input, 0, sizeof(reply), reply));
		}

		case(0xa0): // write pwm mode
		{
			if(input_buffer_length < 2)
				return(build_reply(output_buffer_length, output_buffer, input, 4, 0, 0));

			if(io >= PWM_PORTS)
				return(build_reply(output_buffer_length, output_buffer, input, 3, 0, 0));

			uint8_t mode = input_buffer[1];

			if(mode > 3)
				return(build_reply(output_buffer_length, output_buffer, input, 3, 0, 0));

			pwm_meta[io].pwm_mode = mode;

			return(build_reply(output_buffer_length, output_buffer, input, 0, sizeof(mode), &mode));
		}

		case(0xb0):	// read pwm mode
		{
			if(io >= PWM_PORTS)
				return(build_reply(output_buffer_length, output_buffer, input, 3, 0, 0));

			uint8_t mode;

			mode = pwm_meta[io].pwm_mode;

			return(build_reply(output_buffer_length, output_buffer, input, 0, sizeof(mode), &mode));
		}

		case(0xc0):	// start adc conversion
		{
			if(io >= ADC_PORTS)
				return(build_reply(output_buffer_length, output_buffer, input, 3, 0, 0));

			adc_start(io);
			return(build_reply(output_buffer_length, output_buffer, input, 0, 0, 0));
		}

#if 0
		case(0xf0):	// twi stats
		{
			uint8_t		replystring[2];
			uint16_t	stats;

			switch(io)
			{
				case(0x00):	//	disable
				{
					usi_twi_enable_stats(0);
					return(build_reply(output_buffer_length, output_buffer, input, 0, 0, 0));
				}

				case(0x01):	//	enable
				{
					usi_twi_enable_stats(1);
					return(build_reply(output_buffer_length, output_buffer, input, 0, 0, 0));
				}

				case(0x02):	//	read start conditions
				{
					stats = usi_twi_stats_start_conditions();
					break;
				}

				case(0x03):	//	read stop conditions
				{
					stats = usi_twi_stats_stop_conditions();
					break;
				}

				case(0x04):	//	read error conditions
				{
					stats = usi_twi_stats_error_conditions();
					break;
				}

				case(0x05):	//	read overflow conditions
				{
					stats = usi_twi_stats_overflow_conditions();
					break;
				}

				case(0x06):	//	read local frames
				{
					stats = usi_twi_stats_local_frames();
					break;
				}

				case(0x07):	//	read idle calls
				{
					stats = usi_twi_stats_idle_calls();
					break;
				}
			}

			put_word(stats, replystring);

			return(build_reply(output_buffer_length, output_buffer, input, 0, sizeof(replystring), replystring));
		}
#endif

		default:
		{
			return(build_reply(output_buffer_length, output_buffer, input, 2, 0, 0));
		}
	}

	return(build_reply(output_buffer_length, output_buffer, input, 2, 0, 0));
}

int main(void)
{
	cli();

	PRR =		(0 << 7)		|
				(0 << 6)		|	// reserved
				(0 << 5)		|
				(0 << 4)		|
				(0 << PRTIM1)	|	// timer1
				(0 << PRTIM0)	|	// timer0
				(1 << PRUSI)	|	// usi
				(0 << PRADC);		// adc / analog comperator

	DDRA	= 0;
	DDRB	= 0;
	GIMSK	= 0;
	PCMSK0	= 0;
	PCMSK1	= 0;

	for(slot = 0; slot < INPUT_PORTS; slot++)
	{
		counter_meta_t	*counter_slot;

		ioport					= &input_ports[slot];
		*ioport->port			&= ~_BV(ioport->bit);
		*ioport->ddr			&= ~_BV(ioport->bit);
		*ioport->port			|=  _BV(ioport->bit);
		*ioport->pcmskreg		|=  _BV(ioport->pcmskbit);
		GIMSK					|=  _BV(ioport->gimskbit);

		counter_slot			= &counter_meta[slot];
		counter_slot->state		= *ioport->pin & _BV(ioport->bit);
		counter_slot->counter	= 0;
	}

	for(slot = 0; slot < OUTPUT_PORTS; slot++)
	{
		ioport			= &output_ports[slot];
		*ioport->ddr	|= _BV(ioport->bit);
		*ioport->port	&= ~_BV(ioport->bit);

		softpwm_meta[slot].duty		= 0;
		softpwm_meta[slot].pwm_mode	= pwm_mode_none;
	}

	for(slot = 0; slot < INTERNAL_OUTPUT_PORTS; slot++)
	{
		ioport			= &internal_output_ports[slot];
		*ioport->ddr	|= _BV(ioport->bit);
		*ioport->port	&= ~_BV(ioport->bit);
	}

	for(slot = 0; slot < PWM_PORTS; slot++)
	{
		*pwm_ports[slot].ddr 		|= _BV(pwm_ports[slot].bit);
		*pwm_ports[slot].port		&= ~_BV(pwm_ports[slot].bit);
		pwm_meta[slot].pwm_mode		= pwm_mode_none;
	}

	for(duty = 0; duty < 3; duty++)
	{
		for(slot = 0; slot < INTERNAL_OUTPUT_PORTS; slot++)
		{
			ioport = &internal_output_ports[slot];
			*ioport->port |= _BV(ioport->bit);
		}

		for(slot = 0; slot < INTERNAL_OUTPUT_PORTS; slot++)
		{
			ioport = &internal_output_ports[slot];
			*ioport->port &= ~_BV(ioport->bit);
		}

		for(slot = INTERNAL_OUTPUT_PORTS; slot > 0; slot--)
		{
			ioport = &internal_output_ports[slot - 1];
			*ioport->port |= _BV(ioport->bit);
		}

		for(slot = INTERNAL_OUTPUT_PORTS; slot > 0; slot--)
		{
			ioport = &internal_output_ports[slot - 1];
			*ioport->port &= ~_BV(ioport->bit);
		}
	}

	adc_init();

	// 8 mhz / 256 / 256 = 122 Hz
	timer0_init(TIMER0_PRESCALER_256);
	timer0_set_max(0xff);

	// 8 mhz / 32 / 1024 = 244 Hz
	pwm_timer1_init(PWM_TIMER1_PRESCALER_32);
	pwm_timer1_set_max(0x3ff);
	pwm_timer1_start();

	usbInit();
	usbDeviceDisconnect();
	_delay_ms(250);
	usbDeviceConnect();

	sei();

	for(;;)
	{
		static uint8_t ext_sof_count1 = 0, ext_sof_count2 = 0;

		usbPoll();

		if(usbSofCount > 16)
		{
			process_pwmmode();
			usb_running = 1;
			ext_sof_count1++;
			usbSofCount = 0;
		}

		if(ext_sof_count1 > 4)
		{
			*internal_output_ports[0].port &= ~_BV(internal_output_ports[0].bit);
			*internal_output_ports[1].port &= ~_BV(internal_output_ports[1].bit);
			ext_sof_count2++;
			ext_sof_count1 = 0;
		}

		if(ext_sof_count2 > 8)
		{
			*internal_output_ports[2].port ^=  _BV(internal_output_ports[2].bit);
			ext_sof_count2 = 0;
		}

		if(receive_buffer_complete)
			process_input(sizeof(send_buffer), receive_buffer_length, receive_buffer, &send_buffer_length, send_buffer);
	}

	return(0);
}
