#if !defined(_TIMER0_H_)
#define _TIMER0_H_

#include <stdint.h>
#include <avr/io.h>

enum
{
	TIMER0_PRESCALER_OFF	= 0,
	TIMER0_PRESCALER_1		= 1,
	TIMER0_PRESCALER_8		= 2,
	TIMER0_PRESCALER_64		= 3,
	TIMER0_PRESCALER_256	= 4,
	TIMER0_PRESCALER_1024	= 5
};

		void		timer0_init(uint8_t scaler);
static	void		timer0_reset_counter(void);
static	uint8_t		timer0_get_counter(void);
static	void		timer0_set_max(uint8_t);		// OCR0A
static	void		timer0_set_trigger(uint8_t);	// OCR0B
		void		timer0_start(void);
		void		timer0_stop(void);
		uint8_t		timer0_status(void);

static inline void timer0_reset_counter(void)
{
	TCNT0L = 0;
}

static inline uint8_t timer0_get_counter(void)
{
	return(TCNT0L);
}

static inline void timer0_set_max(uint8_t max_value)
{
	OCR0A = max_value;
}

static inline void timer0_set_trigger(uint8_t trigger_value)
{
	OCR0B = trigger_value;
}

#endif
