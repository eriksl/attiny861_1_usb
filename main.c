#include <stdint.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <string.h>

#include "usbdrv.h"

#include "clock.h"
#include "eeprom.h"
#include "ioports.h"
#include "adc.h"
#include "timer0.h"
#include "pwm_timer1.h"
#include "watchdog.h"

enum
{
	adc_warmup_init = 8,
	command_led_timeout = 8,
	if_active_led_timeout = 2,
	buffer_size = 32
};

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
	pwm_mode_t	pwm_mode:8;
} pwm_meta_t;

typedef struct
{
	uint32_t	counter;
	uint8_t		state;
} counter_meta_t;

static	uint8_t	const *input_buffer;
static	uint8_t	input_buffer_length;
static	uint8_t	*output_buffer;
static	uint8_t	*output_buffer_length;

static	pwm_meta_t		softpwm_meta[OUTPUT_PORTS];
static	pwm_meta_t		pwm_meta[PWM_PORTS];
static	counter_meta_t	counter_meta[INPUT_PORTS];

static	uint8_t	duty;
static	uint8_t	init_counter = 0;
static	uint8_t	if_sense_led, input_sense_led;

static	uint8_t	input_byte;
static	uint8_t	input_command;
static	uint8_t	input_io;

static	int16_t	temp_cal_multiplier;
static	int16_t	temp_cal_offset;

static	uint8_t		adc_warmup;
static	uint16_t	adc_samples;
static	uint32_t	adc_value;

typedef struct 
{
	uint16_t	multiplier;
	uint16_t	offset;
} eeprom_temp_cal_t;

typedef struct
{
	eeprom_temp_cal_t temp_cal[8];
} eeprom_t;

static eeprom_t *eeprom = (eeprom_t *)0;

static uint16_t get_word(const uint8_t *from)
{
	uint16_t result;

	result = from[0];
	result <<= 8;
	result |= from[1];

	return(result);
}

#if 0
static uint32_t get_long(const uint8_t *from)
{
	uint32_t result;

	result = from[0];
	result <<= 8;
	result |= from[1];
	result <<= 8;
	result |= from[2];
	result <<= 8;
	result |= from[3];

	return(result);
}
#endif

static	uint8_t		receive_buffer[buffer_size];
static	uint8_t		receive_buffer_to_fetch	= 0;
static	uint8_t		receive_buffer_length	= 0;
static	uint8_t		receive_buffer_complete	= 0;

static	uint8_t		send_buffer[buffer_size];
static	uint8_t		send_buffer_sent		= 0;
static	uint8_t		send_buffer_length		= 0;

static void put_word(uint16_t from, uint8_t *to)
{
	to[0] = (from >> 8) & 0xff;
	to[1] = (from >> 0) & 0xff;
}

static void put_long(uint32_t from, uint8_t *to)
{
	to[0] = (from >> 24) & 0xff;
	to[1] = (from >> 16) & 0xff;
	to[2] = (from >>  8) & 0xff;
	to[3] = (from >>  0) & 0xff;
}

static inline void process_pwmmode(void)
{
	static uint8_t			pm_slot, pm_duty, pm_diff;
	static uint16_t			pm_duty16, pm_diff16;

	for(pm_slot = 0; pm_slot < OUTPUT_PORTS; pm_slot++)
	{
		if(pm_slot == 0)
			pm_duty = timer0_get_compa();
		else
			pm_duty = timer0_get_compb();

		pm_diff	= pm_duty / 10;

		if(pm_diff < 3)
			pm_diff = 3;

		switch(softpwm_meta[pm_slot].pwm_mode)
		{
			case(pwm_mode_none):
			{
				break;
			}
			case(pwm_mode_fade_in):
			case(pwm_mode_fade_in_out_cont):
			{
				if(pm_duty < (255 - pm_diff))
					pm_duty += pm_diff;
				else
				{
					pm_duty = 255;

					if(softpwm_meta[pm_slot].pwm_mode == pwm_mode_fade_in)
						softpwm_meta[pm_slot].pwm_mode = pwm_mode_none;
					else
						softpwm_meta[pm_slot].pwm_mode = pwm_mode_fade_out_in_cont;
				}

				if(pm_slot == 0)
					timer0_set_compa(pm_duty);
				else
					timer0_set_compb(pm_duty);

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

					if(softpwm_meta[pm_slot].pwm_mode == pwm_mode_fade_out)
						softpwm_meta[pm_slot].pwm_mode = pwm_mode_none;
					else
						softpwm_meta[pm_slot].pwm_mode = pwm_mode_fade_in_out_cont;
				}

				if(pm_slot == 0)
					timer0_set_compa(pm_duty);
				else
					timer0_set_compb(pm_duty);

				break;
			}
		}
	}

	for(pm_slot = 0; pm_slot < PWM_PORTS; pm_slot++)
	{
		pm_duty16	= pwm_timer1_get_pwm(pm_slot);
		pm_diff16	= pm_duty16 / 8;

		if(pm_diff16 < 8)
			pm_diff16 = 8;

		switch(pwm_meta[pm_slot].pwm_mode)
		{
			case(pwm_mode_none):
			{
				break;
			}

			case(pwm_mode_fade_in):
			case(pwm_mode_fade_in_out_cont):
			{
				if(pm_duty16 < (1020 - pm_diff16))
					pm_duty16 += pm_diff16;
				else
				{
					pm_duty16 = 1020;

					if(pwm_meta[pm_slot].pwm_mode == pwm_mode_fade_in)
						pwm_meta[pm_slot].pwm_mode = pwm_mode_none;
					else
						pwm_meta[pm_slot].pwm_mode = pwm_mode_fade_out_in_cont;
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

					if(pwm_meta[pm_slot].pwm_mode == pwm_mode_fade_out)
						pwm_meta[pm_slot].pwm_mode = pwm_mode_none;
					else
						pwm_meta[pm_slot].pwm_mode = pwm_mode_fade_in_out_cont;
				}

				pwm_timer1_set_pwm(pm_slot, pm_duty16);

				break;
			}
		}
	}
}

ISR(PCINT_vect)
{
	static uint8_t pc_dirty, pc_slot;

	if(init_counter < 16)	// discard spurious first few interrupts
		return;

	for(pc_slot = 0, pc_dirty = 0; pc_slot < INPUT_PORTS; pc_slot++)
	{
		if((*input_ports[pc_slot].pin & _BV(input_ports[pc_slot].bit)) ^ counter_meta[pc_slot].state)
		{
			counter_meta[pc_slot].counter++;
			pc_dirty = 1;
		}

		counter_meta[pc_slot].state = *input_ports[pc_slot].pin & _BV(input_ports[pc_slot].bit);
	}

	sei();

	if(pc_dirty)
	{
		*internal_output_ports[0].port |= _BV(internal_output_ports[0].bit);
		input_sense_led = command_led_timeout;
	}
}

ISR(TIMER0_OVF_vect) // timer 0 softpwm overflow (default normal mode) (244 Hz)
{
	static uint8_t pwm_divisor = 0;

	sei();

	if(timer0_get_compa() == 0)
		*output_ports[0].port &= ~_BV(output_ports[0].bit);
	else
		*output_ports[0].port |=  _BV(output_ports[0].bit);

	if(timer0_get_compb() == 0)
		*output_ports[1].port &= ~_BV(output_ports[1].bit);
	else
		*output_ports[1].port |=  _BV(output_ports[1].bit);

	if(init_counter < 255)
		init_counter++;

	if(++pwm_divisor > 4)
	{
		process_pwmmode();
		pwm_divisor = 0;
	}

	if(if_sense_led == 1)
		*internal_output_ports[1].port &= ~_BV(internal_output_ports[1].bit);

	if(if_sense_led > 0)
		if_sense_led--;

	if(input_sense_led == 1)
		*internal_output_ports[0].port &= ~_BV(internal_output_ports[0].bit);

	if(input_sense_led > 0)
		input_sense_led--;
}

ISR(TIMER0_COMPA_vect) // timer 0 softpwm port 1 trigger
{
	sei();

	if(timer0_get_compa() != 0xff)
		*output_ports[0].port &= ~_BV(output_ports[0].bit);
}

ISR(TIMER0_COMPB_vect) // timer 0 softpwm port 2 trigger
{
	sei();

	if(timer0_get_compb() != 0xff)
		*output_ports[1].port &= ~_BV(output_ports[1].bit);
}

#if (USE_CRYSTAL == 0)
static void calibrateOscillator(void)
{
	uint8_t		step = 128;
	uint8_t		trialValue = 0, optimumValue;
	uint16_t	x, optimumDev, targetValue = (unsigned)(1499 * (double)F_CPU / 10.5e6 + 0.5);

	/* do a binary search: */

	cli();

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

	sei();
}
#else
static void calibrateOscillator(void)
{
}
#endif

void usbEventResetReady(void)
{
	calibrateOscillator();
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
#if 0
	static uint8_t previous_data_token = 0xff;

	if(usbCurrentDataToken == previous_data_token)
		return(1);

	previous_data_token = usbCurrentDataToken;
#endif

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

static void reply(uint8_t error_code, uint8_t reply_length, const uint8_t *reply_string)
{
	uint8_t checksum;
	uint8_t ix;

	if((reply_length + 4) > buffer_size)
		return;

	output_buffer[0] = 3 + reply_length;
	output_buffer[1] = error_code;
	output_buffer[2] = input_byte;

	for(ix = 0; ix < reply_length; ix++)
		output_buffer[3 + ix] = reply_string[ix];

	for(ix = 1, checksum = 0; ix < (3 + reply_length); ix++)
		checksum += output_buffer[ix];

	output_buffer[3 + reply_length] = checksum;
	*output_buffer_length = 3 + reply_length + 1;
}

static void reply_char(uint8_t value)
{
	reply(0, sizeof(value), &value);
}

static void reply_short(uint16_t value)
{
	uint8_t reply_string[sizeof(value)];

	put_word(value, reply_string);

	reply(0, sizeof(reply_string), reply_string);
}

static void reply_long(uint32_t value)
{
	uint8_t reply_string[sizeof(value)];

	put_long(value, reply_string);

	reply(0, sizeof(reply_string), reply_string);
}

static void reply_error(uint8_t error_code)
{
	reply(error_code, 0, 0);
}

static void extended_command()
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
			return(reply(0, sizeof(control_info), (uint8_t *)&control_info));
		}

		case(0x01):	// get analog inputs
		{
			control_info.amount = ANALOG_PORTS;
			put_word(0x0000, &control_info.data[0]);
			put_word(0x03ff, &control_info.data[2]);
			return(reply(0, sizeof(control_info), (uint8_t *)&control_info));
		}

		case(0x02):	// get digital outputs
		{
			control_info.amount = OUTPUT_PORTS;
			put_word(0x0000, &control_info.data[0]);
			put_word(0x00ff, &control_info.data[2]);
			return(reply(0, sizeof(control_info), (uint8_t *)&control_info));
		}

		case(0x03):	// get pwm outputs
		{
			control_info.amount = PWM_PORTS;
			put_word(0x0000, &control_info.data[0]);
			put_word(0x03ff, &control_info.data[2]);
			return(reply(0, sizeof(control_info), (uint8_t *)&control_info));
		}

		case(0x04):	// get temperature sensors
		{
			control_info.amount = TEMP_PORTS;
			put_long(0x00, &control_info.data[0]);
			return(reply(0, sizeof(control_info), (uint8_t *)&control_info));
		}
	}

	return(reply_error(7));
}

static void process_input(uint8_t buffer_size, uint8_t if_input_buffer_length, const uint8_t *if_input_buffer,
						uint8_t *if_output_buffer_length, uint8_t *if_output_buffer)
{
	*internal_output_ports[1].port |= _BV(internal_output_ports[1].bit);
	if_sense_led = command_led_timeout;

	input_buffer_length		= if_input_buffer_length;
	input_buffer			= if_input_buffer;

	output_buffer_length	= if_output_buffer_length;
	output_buffer			= if_output_buffer;

	if(input_buffer_length < 1)
		return(reply_error(1));

	input_byte		= input_buffer[0];
	input_command	= input_byte & 0xf8;
	input_io		= input_byte & 0x07;

	watchdog_reset();

	switch(input_command)
	{
		case(0x00):	// short / no-io
		{
			switch(input_io)
			{
				case(0x00):	// identify
				{
					struct
					{
						uint8_t id1, id2;
						uint8_t model, version, revision;
						uint8_t name[16];
					} id =
					{
						0x4a, 0xfb,
						0x06, 0x01, 0x05,
						"attiny861a",
					};

					return(reply(0, sizeof(id), (uint8_t *)&id));
				}

				case(0x01):	// read analog input
				{
					uint8_t reply_string[6];

					put_word(adc_samples, &reply_string[0]);
					put_long(adc_value, &reply_string[2]);
					adc_warmup	= adc_warmup_init;
					adc_samples = 0;
					adc_value	= 0;

					return(reply(0, sizeof(reply_string), reply_string));
				}

				case(0x02):	// read temperature sensor
				{
					int32_t	value;
					int32_t	samples;

					uint8_t reply_string[8];

					value	= (int32_t)adc_value;
					samples	= (int32_t)adc_samples;

					value	*= 10;
					value	*= temp_cal_multiplier;
					value	/= samples;
					value	/= 1000;
					value	+= temp_cal_offset;

					put_word(value, &reply_string[0]);
					put_word(adc_samples, &reply_string[2]);
					put_long(adc_value, &reply_string[4]);

					adc_warmup	= adc_warmup_init;
					adc_samples = 0;
					adc_value	= 0;

					return(reply(0, sizeof(reply_string), reply_string));
				}

				case(0x06): // test watchdog
				{
					for(;;)
						(void)0;
				}

				case(0x07): // extended command
				{
					return(extended_command());
				}

				default:
				{
					return(reply_error(7));
				}
			}

			break;
		}

		case(0x10):	// 0x10 read counter
		case(0x20): // 0x20 read / reset counter
		{
			if(input_io >= INPUT_PORTS)
				return(reply_error(3));

			uint32_t counter = counter_meta[input_io].counter;

			if(input_command == 0x20)
				counter_meta[input_io].counter = 0;

			return(reply_long(counter));
		}

		case(0x30):	//	read input
		{
			uint8_t value;

			if(input_io >= INPUT_PORTS)
				return(reply_error(3));

			value = *input_ports[input_io].pin & _BV(input_ports[input_io].bit) ? 0x00 : 0x01;

			return(reply_char(value));
		}

		case(0x40):	//	write output / softpwm
		{
			uint8_t value;

			if(input_buffer_length < 2)
				return(reply_error(4));

			if(input_io >= OUTPUT_PORTS)
				return(reply_error(3));

			value = input_buffer[1];

			if(input_io == 0)
				timer0_set_compa(value);
			else
				timer0_set_compb(value);

			return(reply_char(value));
		}

		case(0x50):	// read output / softpwm
		{
			uint8_t value;

			if(input_io >= OUTPUT_PORTS)
				return(reply_error(3));

			if(input_io == 0)
				value = timer0_get_compa();
			else
				value = timer0_get_compb();

			return(reply_char(value));
		}

		case(0x60): // write softpwm mode
		{
			if(input_buffer_length < 2)
				return(reply_error(4));

			if(input_io >= OUTPUT_PORTS)
				return(reply_error(3));

			if(input_buffer[1] > 3)
				return(reply_error(3));

			softpwm_meta[input_io].pwm_mode = input_buffer[1];

			return(reply_char(input_buffer[1]));
		}

		case(0x70):	// read softpwm mode
		{
			if(input_io >= OUTPUT_PORTS)
				return(reply_error(3));

			return(reply_char(softpwm_meta[input_io].pwm_mode));
		}

		case(0x80): // write pwm
		{
			uint16_t value;

			if(input_buffer_length < 3)
				return(reply_error(4));

			if(input_io >= PWM_PORTS)
				return(reply_error(3));

			value = get_word(&input_buffer[1]);

			pwm_timer1_set_pwm(input_io, value);

			return(reply_short(value));
		}

		case(0x90): // read pwm
		{
			if(input_io >= PWM_PORTS)
				return(reply_error(3));

			return(reply_short(pwm_timer1_get_pwm(input_io)));
		}

		case(0xa0): // write pwm mode
		{
			if(input_buffer_length < 2)
				return(reply_error(4));

			if(input_io >= PWM_PORTS)
				return(reply_error(3));

			if(input_buffer[1] > 3)
				return(reply_error(3));

			pwm_meta[input_io].pwm_mode = input_buffer[1];

			return(reply_char(input_buffer[1]));
		}

		case(0xb0):	// read pwm mode
		{
			if(input_io >= PWM_PORTS)
				return(reply_error(3));

			return(reply_char(pwm_meta[input_io].pwm_mode));
		}

		case(0xc0):	// select adc
		{
			if(input_io >= ANALOG_PORTS)
				return(reply_error(3));

			adc_select(&analog_ports[input_io]);
			adc_warmup	= adc_warmup_init;
			adc_samples = 0;
			adc_value	= 0;

			return(reply_char(input_io));
		}

		case(0xc0):	// read adc
		{
			uint16_t value;
			uint8_t replystring[2];

			adc_select(&temp_ports[input_io]);
			adc_warmup	= adc_warmup_init;
			adc_samples = 0;
			adc_value	= 0;

			value = adc_read(io);

			return(reply_char(input_io));
		}

		case(0xe0):	// set temperature sensor calibration values
		{
			int32_t value;

			if(input_io >= TEMP_PORTS)
				return(reply_error(3));

			if(input_buffer_length < 5)
				return(reply_error(4));

			value = get_word(&input_buffer[1]);
			eeprom_write_uint16(&eeprom->temp_cal[input_io].multiplier, value);

			if(io == (ANALOG_PORTS + 0)) // TMP36
			{
				calc_value	= (int32_t)value;
				calc_value *= 1000;
				calc_value /= 961;
				calc_value -= (500 - 5);
				value       = (uint16_t)calc_value;
			}

			if(io == (ANALOG_PORTS + 1)) // internal temp ref
			{
				calc_value	 = (int32_t)value;
				calc_value	 = calc_value - 280;
				calc_value	*= 1000;
				calc_value	/= 110;
				value		 = (uint16_t)calc_value;
			}

			put_word(value, replystring);

			return(reply(0, sizeof(replystring), replystring));
		}

		default:
		{
			return(reply_error(2));
		}
	}

	return(reply_error(2));
}

void if_idle(void) // gets called ~1000 Hz
{
	static uint16_t led_divisor = 0;

	if(led_divisor > 1000) // 1000 Hz / 1000 = 1 Hz
	{
		led_divisor = 0;

		*internal_output_ports[1].port |= _BV(internal_output_ports[1].bit);
		if_sense_led  = if_active_led_timeout;
	}

	led_divisor++;

	if(adc_warmup > 0)
	{
		adc_warmup--;
		adc_start();
	}
	else
	{
		if(adc_ready() && (adc_samples < 1024))
		{
			adc_samples++;
			adc_value += adc_read();
			adc_start();
		}
	}

	watchdog_reset();
}

int main(void)
{
	uint8_t slot;

	watchdog_stop();

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
		*input_ports[slot].ddr			|=  _BV(input_ports[slot].bit);	// output
		*input_ports[slot].port			&= ~_BV(input_ports[slot].bit); // low
		*input_ports[slot].ddr			&= ~_BV(input_ports[slot].bit);	// input
		*input_ports[slot].port			&= ~_BV(input_ports[slot].bit); // disable pullup
		*input_ports[slot].pcmskreg		|=  _BV(input_ports[slot].pcmskbit);
		GIMSK							|=  _BV(input_ports[slot].gimskbit);

		counter_meta[slot].state		= *input_ports[slot].pin & _BV(input_ports[slot].bit);
		counter_meta[slot].counter		= 0;
	}

	for(slot = 0; slot < OUTPUT_PORTS; slot++)
	{
		*output_ports[slot].ddr		|=  _BV(output_ports[slot].bit);
		*output_ports[slot].port	&= ~_BV(output_ports[slot].bit);
		softpwm_meta[slot].pwm_mode	= pwm_mode_none;
	}

	for(slot = 0; slot < USB_PORTS; slot++)
	{
		*usb_ports[slot].port	&= ~_BV(usb_ports[slot].bit);
		*usb_ports[slot].ddr	&= ~_BV(usb_ports[slot].bit);
		*usb_ports[slot].port	&= ~_BV(usb_ports[slot].bit);
	}

	for(slot = 0; slot < INTERNAL_OUTPUT_PORTS; slot++)
	{
		*internal_output_ports[slot].ddr	|= _BV(internal_output_ports[slot].bit);
		*internal_output_ports[slot].port	&= ~_BV(internal_output_ports[slot].bit);
	}

	for(slot = 0; slot < PWM_PORTS; slot++)
	{
		*pwm_ports[slot].ddr 		|=  _BV(pwm_ports[slot].bit);
		*pwm_ports[slot].port		&= ~_BV(pwm_ports[slot].bit);
		pwm_meta[slot].pwm_mode		= pwm_mode_none;
	}

	adc_init();

	// 18 mhz / 256 / 256 = 274 Hz
	timer0_init(TIMER0_PRESCALER_256);
	timer0_set_compa(0x00);
	timer0_set_compb(0x00);
	timer0_start();

	// 18 mhz / 64 / 1024 = 274 Hz
	pwm_timer1_init(PWM_TIMER1_PRESCALER_32);
	pwm_timer1_set_max(0x3ff);
	pwm_timer1_start();

	usbInit();
	usbDeviceDisconnect();

	for(duty = 0; duty < 3; duty++)
	{
		for(slot = 0; slot < INTERNAL_OUTPUT_PORTS; slot++)
		{
			*internal_output_ports[slot].port |= _BV(internal_output_ports[slot].bit);
			_delay_ms(25);
		}

		for(slot = 0; slot < INTERNAL_OUTPUT_PORTS; slot++)
		{
			*internal_output_ports[slot].port &= ~_BV(internal_output_ports[slot].bit);
			_delay_ms(25);
		}

		for(slot = 0; slot < INTERNAL_OUTPUT_PORTS; slot++)
		{
			*internal_output_ports[slot].port |= _BV(internal_output_ports[slot].bit);
			_delay_ms(25);
		}

		for(slot = 0; slot < INTERNAL_OUTPUT_PORTS; slot++)
		{
			*internal_output_ports[slot].port &= ~_BV(internal_output_ports[slot].bit);
			_delay_ms(25);
		}
	}

	adc_warmup	= adc_warmup_init;
	adc_samples	= 0;
	adc_value	= 0;

	watchdog_setup(WATCHDOG_PRESCALER_1024K);	//	8.0 seconds timeout
	watchdog_start();

	usbDeviceConnect();

	sei();

	for(;;)
	{
		usbPoll();

		if(receive_buffer_complete)
		{
			process_input(sizeof(send_buffer), receive_buffer_length, receive_buffer, &send_buffer_length, send_buffer);
			receive_buffer_complete	= 0;
		}

		if(usbSofCount > 0) // 1ms = 1000 Hz
		{
			usbSofCount = 0;
			if_idle();
		}
	}
}
