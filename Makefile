# BOARD = 0		production
# BOARD = 1		testboard 1 (rc oscillator)
# BOARD = 2		testboard 2 (crystal)

BOARD =	2

ifeq ($(BOARD), 0)
	USE_CRYSTAL	= 1
	USE_PLL		= 0
	MCUSPEED	= 18000000
	LFUSE		= 0xff
	HFUSE		= 0xd5
endif
ifeq ($(BOARD), 1)
	USE_CRYSTAL	= 0
	USE_PLL		= 0
	MCUSPEED	= 12800000
	LFUSE		= 0xe2
	HFUSE		= 0xd5
endif
ifeq ($(BOARD), 2)
	USE_CRYSTAL	= 1
	USE_PLL		= 0
	MCUSPEED	= 18000000
	LFUSE		= 0xff
	HFUSE		= 0xd5
endif

MCU			=		attiny861
PROGRAMMER	=		dragon_isp
PRGFLAGS	=		-b 0 -P usb

PROGRAM		=		main
OBJFILES	=		adc.o ioports.o timer0.o pwm_timer1.o watchdog.o eeprom.o clock.o v-usb/usbdrv/usbdrv.o v-usb/usbdrv/usbdrvasm.o $(PROGRAM).o
HEADERS		=		adc.h ioports.h timer0.h pwm_timer1.h watchdog.h eeprom.h clock.h usbconfig.h v-usb/usbdrv/usbdrv.h v-usb/usbdrv/usbportability.h
HEXFILE		=		$(PROGRAM).hex
ELFFILE		=		$(PROGRAM).elf
PROGRAMMED	=		.programmed
CFLAGS		=		-I$(CURDIR) -I$(CURDIR)/v-usb/usbdrv \
					--std=c99 -Wall -Winline -Os -mmcu=$(MCU) -DF_CPU=$(MCUSPEED) -DUSE_CRYSTAL=$(USE_CRYSTAL) -DUSE_PLL=$(USE_PLL) -DBOARD=$(BOARD) \
					-fpack-struct -funit-at-a-time -fno-keep-static-consts -frename-registers
LDFLAGS		=		-Wall -mmcu=$(MCU)

.PHONY:				all clean hex
.SUFFIXES:
.SUFFIXES:			.c .o .elf .hex
.PRECIOUS:			.c .h

all:				$(PROGRAMMED)
hex:				$(HEXFILE)

$(PROGRAM).o:		$(PROGRAM).c $(HEADERS)

%.o:				%.c
					@echo "CC $< -> $@"
					@avr-gcc -c $(CFLAGS) $< -o $@

%.o:				%.S
					@echo "AS $< -> $@"
					@avr-gcc -x assembler-with-cpp -c $(CFLAGS) $< -o $@

%.s:				%.c
					@echo "CC (ASM) $< -> $@"
					@avr-gcc -S $(CFLAGS) $< -o $@

adc.o:				adc.h
ioports.o:			ioports.h
timer0.o:			timer0.h
watchdog.o:			watchdog.h
clock.o:			clock.h
eeprom.o:			eeprom.h

v-usb/usbdrv/usbdrv.o:		usbconfig.h v-usb/usbdrv/usbdrv.h v-usb/usbdrv/usbportability.h
v-usb/usbdrv/usbdrvasm.o:	usbconfig.h v-usb/usbdrv/usbdrv.h v-usb/usbdrv/usbportability.h
main.o:						usbconfig.h v-usb/usbdrv/usbdrv.h v-usb/usbdrv/usbportability.h

$(ELFFILE):			$(OBJFILES)
					@echo "LD $(OBJFILES) -> $@"
					@avr-gcc $(LDFLAGS) $(OBJFILES) -o $@

$(HEXFILE):			$(ELFFILE)
					@echo "OBJCOPY $< -> $@"
					@avr-objcopy -j .text -j .data -O ihex $< $@
					@sh -c 'avr-size $< | (read header; read text data bss junk; echo "SIZE: flash: $$[text + data] ram: $$[data + bss]")'

$(PROGRAMMED):		$(HEXFILE)
					@echo "AVRDUDE $^"
					@avrdude -vv -c $(PROGRAMMER) -p $(MCU) $(PRGFLAGS) -U flash:w:$^ -U lfuse:w:$(LFUSE):m -U hfuse:w:$(HFUSE):m

clean:			
					@echo "RM $(OBJFILES) $(ELFFILE) $(HEXFILE) $(PROGRAMMED)"
					@-rm $(OBJFILES) $(ELFFILE) $(HEXFILE) 2> /dev/null || true
