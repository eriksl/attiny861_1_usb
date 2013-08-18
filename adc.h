#if !defined(_ADC_H_)
#define _ADC_H_

#include <stdint.h>
#include <avr/io.h>

void adc_init(void);
uint16_t adc_read(uint8_t source);

#endif
