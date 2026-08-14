#if defined (__AVR__)
	#include <avr/sleep.h>
	#include <avr/wdt.h>
	#include <avr/power.h>
	#include <avr/interrupt.h>
#elif defined (__arm__)

#else
	#error "Processor architecture is not supported."
#endif

#include "LowPower.h"

#if defined (__AVR__)
#if defined (__AVR_ATmega328P__) || defined (__AVR_ATmega168P__)
#ifndef sleep_bod_disable
#define sleep_bod_disable() 										\
do { 																\
  unsigned char tempreg; 													\
  __asm__ __volatile__("in %[tempreg], %[mcucr]" "\n\t" 			\
                       "ori %[tempreg], %[bods_bodse]" "\n\t" 		\
                       "out %[mcucr], %[tempreg]" "\n\t" 			\
                       "andi %[tempreg], %[not_bodse]" "\n\t" 		\
                       "out %[mcucr], %[tempreg]" 					\
                       : [tempreg] "=&d" (tempreg) 					\
                       : [mcucr] "I" _SFR_IO_ADDR(MCUCR), 			\
                         [bods_bodse] "i" (_BV(BODS) | _BV(BODSE)), \
                         [not_bodse] "i" (~_BV(BODSE))); 			\
} while (0)
#endif
#endif

#define	lowPowerBodOn(mode)	\
do { 						\
      set_sleep_mode(mode); \
      cli();				\
      sleep_enable();		\
      sei();				\
      sleep_cpu();			\
      sleep_disable();		\
      sei();				\
} while (0);

#if defined (__AVR_ATmega328P__) || defined (__AVR_ATmega168P__)
#define	lowPowerBodOff(mode)\
do { 						\
      set_sleep_mode(mode); \
      cli();				\
      sleep_enable();		\
			sleep_bod_disable(); \
      sei();				\
      sleep_cpu();			\
      sleep_disable();		\
      sei();				\
} while (0);
#endif

#if defined __AVR_ATmega32U4__
	#ifndef PRTIM4
		#define PRTIM4 4
	#endif
	#ifndef power_timer4_disable
		#define power_timer4_disable()	(PRR1 |= (uint8_t)(1 << PRTIM4))
	#endif
	#ifndef power_timer4_enable
		#define power_timer4_enable()		(PRR1 &= (uint8_t)~(1 << PRTIM4))
	#endif
#endif

#if defined (__AVR_ATmega328P__) || defined (__AVR_ATmega168__) || defined (__AVR_ATmega168P__) || defined (__AVR_ATmega88__)
void	LowPowerClass::idle(period_t period, adc_t adc, timer2_t timer2,
							timer1_t timer1, timer0_t timer0,
							spi_t spi, usart0_t usart0,	twi_t twi)
{
	unsigned char clockSource = 0;

	if (timer2 == TIMER2_OFF)
	{
        clockSource = TCCR2B;
		TCCR2B &= ~(1 << CS22);
		TCCR2B &= ~(1 << CS21);
		TCCR2B &= ~(1 << CS20);
		power_timer2_disable();
	}

	if (adc == ADC_OFF)
	{
		ADCSRA &= ~(1 << ADEN);
		power_adc_disable();
	}

	if (timer1 == TIMER1_OFF)	power_timer1_disable();
	if (timer0 == TIMER0_OFF)	power_timer0_disable();
	if (spi == SPI_OFF)			power_spi_disable();
	if (usart0 == USART0_OFF)	power_usart0_disable();
	if (twi == TWI_OFF)			power_twi_disable();

	if (period != SLEEP_FOREVER)
	{
		wdt_enable(period);
		WDTCSR |= (1 << WDIE);
	}

	lowPowerBodOn(SLEEP_MODE_IDLE);

	if (adc == ADC_OFF)
	{
		power_adc_enable();
		ADCSRA |= (1 << ADEN);
	}

	if (timer2 == TIMER2_OFF)
	{
        TCCR2B = clockSource;
		power_timer2_enable();
	}

	if (timer1 == TIMER1_OFF)	power_timer1_enable();
	if (timer0 == TIMER0_OFF)	power_timer0_enable();
	if (spi == SPI_OFF)			power_spi_enable();
	if (usart0 == USART0_OFF)	power_usart0_enable();
	if (twi == TWI_OFF)			power_twi_enable();
}
#endif

#if defined __AVR_ATmega32U4__
void	LowPowerClass::idle(period_t period, adc_t adc,
							timer4_t timer4, timer3_t timer3,
							timer1_t timer1, timer0_t timer0,
							spi_t spi, usart1_t usart1,	twi_t twi, usb_t usb)
{
	if (adc == ADC_OFF)
	{
		ADCSRA &= ~(1 << ADEN);
		power_adc_disable();
	}

	if (timer4 == TIMER4_OFF)	power_timer4_disable();
	if (timer3 == TIMER3_OFF)	power_timer3_disable();
	if (timer1 == TIMER1_OFF)	power_timer1_disable();
	if (timer0 == TIMER0_OFF)	power_timer0_disable();
	if (spi == SPI_OFF)			power_spi_disable();
	if (usart1 == USART1_OFF)	power_usart1_disable();
	if (twi == TWI_OFF)			power_twi_disable();
	if (usb == USB_OFF)			power_usb_disable();

	if (period != SLEEP_FOREVER)
	{
		wdt_enable(period);
		WDTCSR |= (1 << WDIE);
	}

	lowPowerBodOn(SLEEP_MODE_IDLE);

	if (adc == ADC_OFF)
	{
		power_adc_enable();
		ADCSRA |= (1 << ADEN);
	}

	if (timer4 == TIMER4_OFF)	power_timer4_enable();
	if (timer3 == TIMER3_OFF)	power_timer3_enable();
	if (timer1 == TIMER1_OFF)	power_timer1_enable();
	if (timer0 == TIMER0_OFF)	power_timer0_enable();
	if (spi == SPI_OFF)			power_spi_enable();
	if (usart1 == USART1_OFF)	power_usart1_enable();
	if (twi == TWI_OFF)			power_twi_enable();
	if (usb == USB_OFF)			power_usb_enable();
}
#endif

#if defined (__AVR_ATmega644P__) || defined (__AVR_ATmega1284P__)
void	LowPowerClass::idle(period_t period, adc_t adc, timer2_t timer2,
							timer1_t timer1, timer0_t timer0, spi_t spi,
							usart1_t usart1, usart0_t usart0, twi_t twi)
{
	unsigned char clockSource = 0;

	if (timer2 == TIMER2_OFF)
	{
        clockSource = TCCR2B;
		TCCR2B &= ~(1 << CS22);
		TCCR2B &= ~(1 << CS21);
		TCCR2B &= ~(1 << CS20);
		power_timer2_disable();
	}

	if (adc == ADC_OFF)
	{
		ADCSRA &= ~(1 << ADEN);
		power_adc_disable();
	}

	if (timer1 == TIMER1_OFF)	power_timer1_disable();
	if (timer0 == TIMER0_OFF)	power_timer0_disable();
	if (spi == SPI_OFF)		    power_spi_disable();
	if (usart1 == USART1_OFF)	power_usart1_disable();
	if (usart0 == USART0_OFF)	power_usart0_disable();
	if (twi == TWI_OFF)			power_twi_disable();

	if (period != SLEEP_FOREVER)
	{
		wdt_enable(period);
		WDTCSR |= (1 << WDIE);
	}

	lowPowerBodOn(SLEEP_MODE_IDLE);

	if (adc == ADC_OFF)
	{
		power_adc_enable();
		ADCSRA |= (1 << ADEN);
	}

	if (timer2 == TIMER2_OFF)
	{
        TCCR2B = clockSource;
		power_timer2_enable();
	}

	if (timer1 == TIMER1_OFF)	power_timer1_enable();
	if (timer0 == TIMER0_OFF)	power_timer0_enable();
	if (spi == SPI_OFF)			power_spi_enable();
	if (usart1 == USART1_OFF)	power_usart1_enable();
	if (usart0 == USART0_OFF)	power_usart0_enable();
	if (twi == TWI_OFF)			power_twi_enable();
}
#endif

#if defined (__AVR_ATmega2560__) || defined (__AVR_ATmega1280__)
void	LowPowerClass::idle(period_t period, adc_t adc, timer5_t timer5,
					        timer4_t timer4, timer3_t timer3, timer2_t timer2,
							timer1_t timer1, timer0_t timer0, spi_t spi,
							usart3_t usart3, usart2_t usart2, usart1_t usart1,
			                usart0_t usart0, twi_t twi)
{
	unsigned char clockSource = 0;

	if (timer2 == TIMER2_OFF)
	{
        clockSource = TCCR2B;
		TCCR2B &= ~(1 << CS22);
		TCCR2B &= ~(1 << CS21);
		TCCR2B &= ~(1 << CS20);
		power_timer2_disable();
	}

	if (adc == ADC_OFF)
	{
		ADCSRA &= ~(1 << ADEN);
		power_adc_disable();
	}

	if (timer5 == TIMER5_OFF)	power_timer5_disable();
	if (timer4 == TIMER4_OFF)	power_timer4_disable();
	if (timer3 == TIMER3_OFF)	power_timer3_disable();
	if (timer1 == TIMER1_OFF)	power_timer1_disable();
	if (timer0 == TIMER0_OFF)	power_timer0_disable();
	if (spi == SPI_OFF)		    power_spi_disable();
	if (usart3 == USART3_OFF)	power_usart3_disable();
	if (usart2 == USART2_OFF)	power_usart2_disable();
	if (usart1 == USART1_OFF)	power_usart1_disable();
	if (usart0 == USART0_OFF)	power_usart0_disable();
	if (twi == TWI_OFF)			power_twi_disable();

	if (period != SLEEP_FOREVER)
	{
		wdt_enable(period);
		WDTCSR |= (1 << WDIE);
	}

	lowPowerBodOn(SLEEP_MODE_IDLE);

	if (adc == ADC_OFF)
	{
		power_adc_enable();
		ADCSRA |= (1 << ADEN);
	}

	if (timer2 == TIMER2_OFF)
	{
        TCCR2B = clockSource;
		power_timer2_enable();
	}

	if (timer5 == TIMER5_OFF)	power_timer5_enable();
	if (timer4 == TIMER4_OFF)	power_timer4_enable();
	if (timer3 == TIMER3_OFF)	power_timer3_enable();
	if (timer1 == TIMER1_OFF)	power_timer1_enable();
	if (timer0 == TIMER0_OFF)	power_timer0_enable();
	if (spi == SPI_OFF)			power_spi_enable();
	if (usart3 == USART3_OFF)	power_usart3_enable();
	if (usart2 == USART2_OFF)	power_usart2_enable();
	if (usart1 == USART1_OFF)	power_usart1_enable();
	if (usart0 == USART0_OFF)	power_usart0_enable();
	if (twi == TWI_OFF)			power_twi_enable();
}
#endif

#if defined (__AVR_ATmega256RFR2__)
void	LowPowerClass::idle(period_t period, adc_t adc, timer5_t timer5,
					                timer4_t timer4, timer3_t timer3, timer2_t timer2,
													timer1_t timer1, timer0_t timer0, spi_t spi,
													usart1_t usart1,
			                    usart0_t usart0, twi_t twi)
{
	unsigned char clockSource = 0;

	if (timer2 == TIMER2_OFF)
	{
        clockSource = TCCR2B;
		TCCR2B &= ~(1 << CS22);
		TCCR2B &= ~(1 << CS21);
		TCCR2B &= ~(1 << CS20);
		power_timer2_disable();
	}

	if (adc == ADC_OFF)
	{
		ADCSRA &= ~(1 << ADEN);
		power_adc_disable();
	}

	if (timer5 == TIMER5_OFF)	power_timer5_disable();
	if (timer4 == TIMER4_OFF)	power_timer4_disable();
	if (timer3 == TIMER3_OFF)	power_timer3_disable();
	if (timer1 == TIMER1_OFF)	power_timer1_disable();
	if (timer0 == TIMER0_OFF)	power_timer0_disable();
	if (spi == SPI_OFF)			  power_spi_disable();
	if (usart1 == USART1_OFF)	power_usart1_disable();
	if (usart0 == USART0_OFF)	power_usart0_disable();
	if (twi == TWI_OFF)			  power_twi_disable();

	if (period != SLEEP_FOREVER)
	{
		wdt_enable(period);
		WDTCSR |= (1 << WDIE);
	}

	lowPowerBodOn(SLEEP_MODE_IDLE);

	if (adc == ADC_OFF)
	{
		power_adc_enable();
		ADCSRA |= (1 << ADEN);
	}

	if (timer2 == TIMER2_OFF)
	{
        TCCR2B = clockSource;
		power_timer2_enable();
	}

	if (timer5 == TIMER5_OFF)	power_timer5_enable();
	if (timer4 == TIMER4_OFF)	power_timer4_enable();
	if (timer3 == TIMER3_OFF)	power_timer3_enable();
	if (timer1 == TIMER1_OFF)	power_timer1_enable();
	if (timer0 == TIMER0_OFF)	power_timer0_enable();
	if (spi == SPI_OFF)			  power_spi_enable();
	if (usart1 == USART1_OFF)	power_usart1_enable();
	if (usart0 == USART0_OFF)	power_usart0_enable();
	if (twi == TWI_OFF)			  power_twi_enable();
}
#endif

void	LowPowerClass::adcNoiseReduction(period_t period, adc_t adc,
										 timer2_t timer2)
{
	unsigned char clockSource = 0;

	#if !defined(__AVR_ATmega32U4__)
	if (timer2 == TIMER2_OFF)
	{
        clockSource = TCCR2B;
		TCCR2B &= ~(1 << CS22);
		TCCR2B &= ~(1 << CS21);
		TCCR2B &= ~(1 << CS20);
	}
	#endif

	if (adc == ADC_OFF)	ADCSRA &= ~(1 << ADEN);

	if (period != SLEEP_FOREVER)
	{
		wdt_enable(period);
		WDTCSR |= (1 << WDIE);
	}

	lowPowerBodOn(SLEEP_MODE_ADC);

	if (adc == ADC_OFF) ADCSRA |= (1 << ADEN);

	#if !defined(__AVR_ATmega32U4__)
	if (timer2 == TIMER2_OFF)
	{
        TCCR2B = clockSource;
	}
	#endif
}

void	LowPowerClass::powerDown(period_t period, adc_t adc, bod_t bod)
{
	if (adc == ADC_OFF)	ADCSRA &= ~(1 << ADEN);

	if (period != SLEEP_FOREVER)
	{
		wdt_enable(period);
		WDTCSR |= (1 << WDIE);
	}
	if (bod == BOD_OFF)
	{
		#if defined (__AVR_ATmega328P__) || defined (__AVR_ATmega168P__)
			lowPowerBodOff(SLEEP_MODE_PWR_DOWN);
		#else
			lowPowerBodOn(SLEEP_MODE_PWR_DOWN);
		#endif
	}
	else
	{
		lowPowerBodOn(SLEEP_MODE_PWR_DOWN);
	}

	if (adc == ADC_OFF) ADCSRA |= (1 << ADEN);
}

void	LowPowerClass::powerSave(period_t period, adc_t adc, bod_t bod,
							     timer2_t timer2)
{
	unsigned char clockSource = 0;

	#if !defined(__AVR_ATmega32U4__)
	if (timer2 == TIMER2_OFF)
	{
        clockSource = TCCR2B;
		TCCR2B &= ~(1 << CS22);
		TCCR2B &= ~(1 << CS21);
		TCCR2B &= ~(1 << CS20);
	}
	#endif

	if (adc == ADC_OFF)	ADCSRA &= ~(1 << ADEN);

	if (period != SLEEP_FOREVER)
	{
		wdt_enable(period);
		WDTCSR |= (1 << WDIE);
	}

	if (bod == BOD_OFF)
	{
		#if defined (__AVR_ATmega328P__) || defined (__AVR_ATmega168P__)
			lowPowerBodOff(SLEEP_MODE_PWR_SAVE);
		#else
			lowPowerBodOn(SLEEP_MODE_PWR_SAVE);
		#endif
	}
	else
	{
		lowPowerBodOn(SLEEP_MODE_PWR_SAVE);
	}

	if (adc == ADC_OFF) ADCSRA |= (1 << ADEN);

	#if !defined(__AVR_ATmega32U4__)
	if (timer2 == TIMER2_OFF)
	{
        TCCR2B = clockSource;
	}
	#endif
}

void	LowPowerClass::powerStandby(period_t period, adc_t adc, bod_t bod)
{
	if (adc == ADC_OFF)	ADCSRA &= ~(1 << ADEN);

	if (period != SLEEP_FOREVER)
	{
		wdt_enable(period);
		WDTCSR |= (1 << WDIE);
	}

	if (bod == BOD_OFF)
	{
		#if defined (__AVR_ATmega328P__) || defined (__AVR_ATmega168P__)
			lowPowerBodOff(SLEEP_MODE_STANDBY);
		#else
			lowPowerBodOn(SLEEP_MODE_STANDBY);
		#endif
	}
	else
	{
		lowPowerBodOn(SLEEP_MODE_STANDBY);
	}

	if (adc == ADC_OFF) ADCSRA |= (1 << ADEN);
}

void	LowPowerClass::powerExtStandby(period_t period, adc_t adc, bod_t bod,
									   timer2_t timer2)
{
	unsigned char clockSource = 0;

	#if !defined(__AVR_ATmega32U4__)
	if (timer2 == TIMER2_OFF)
	{
        clockSource = TCCR2B;
		TCCR2B &= ~(1 << CS22);
		TCCR2B &= ~(1 << CS21);
		TCCR2B &= ~(1 << CS20);
	}
	#endif

	if (adc == ADC_OFF)	ADCSRA &= ~(1 << ADEN);

	if (period != SLEEP_FOREVER)
	{
		wdt_enable(period);
		WDTCSR |= (1 << WDIE);
	}

	#if defined (__AVR_ATmega88__) || defined (__AVR_ATmega168__)
	#else
		if (bod == BOD_OFF)
		{
			#if defined (__AVR_ATmega328P__) || defined (__AVR_ATmega168P__)
				lowPowerBodOff(SLEEP_MODE_EXT_STANDBY);
			#else
				lowPowerBodOn(SLEEP_MODE_EXT_STANDBY);
			#endif
		}
		else
		{
			lowPowerBodOn(SLEEP_MODE_EXT_STANDBY);
		}
	#endif

	if (adc == ADC_OFF) ADCSRA |= (1 << ADEN);

	#if !defined(__AVR_ATmega32U4__)
	if (timer2 == TIMER2_OFF)
	{
        TCCR2B = clockSource;
	}
	#endif
}

ISR (WDT_vect)
{
	wdt_disable();
}

#elif defined (__arm__)
#if defined (__SAMD21G18A__)
void	LowPowerClass::idle(idle_t idleMode)
{
	SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
	PM->SLEEP.reg = idleMode;
	__DSB();
	__WFI();
}

void	LowPowerClass::standby()
{
	SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
	__DSB();
	__WFI();
}

#else
	#error "Please ensure chosen MCU is ATSAMD21G18A."
#endif
#else
	#error "Processor architecture is not supported."
#endif

LowPowerClass LowPower;
