/*
 * Timer.c
 *
 * Created: 26.01.2021 16:27:08
 *  Author: Niklas Theis
 */ 
#include <stdint.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/common.h>

#include "Timer.h"

ISR(TIMER1_COMPA_vect)
{
	if(IRCompAFlag != 0)
	{
		*IRCompAFlag = 1;
	}
}

ISR(TIMER1_COMPB_vect)
{
	if(IRCompBFlag != 0)
	{
		*IRCompBFlag = 1;
	}
}

Timer_Error_t Timer_init(uint16_t compareAValue, uint16_t compareBValue, Timer_Settings_t timerSettings)
{
	ClockSignal = timerSettings.ClockSignal;
	
	TCCR1A = 0b00000000;
	TCCR1B = timerSettings.CTCMode << WGM12 | timerSettings.ClockSignal;
	
	OCR1A = compareAValue;
	OCR1B = compareBValue;
	
	TCNT1 = 0;
	
	InitFlag = 1;
	return NoError;
}

Timer_Error_t Timer_start()
{
	if (!InitFlag || ClockSignal == CSTimerStop)
	{
		return ErrorNoCSSet;
	}
	TCCR1B |= ClockSignal;
	TCNT1 = 0;
	return NoError;
}

Timer_Error_t Timer_stop()
{
	TCCR1B &= ~(CSTimerStop);
	return NoError;
}

Timer_Error_t Timer_reset()
{
	TCNT1 = 0;
	return NoError;
}

Timer_Error_t Timer_addInterrupt(enum Timer_Interrupt_t interrupt, volatile uint8_t* irFlag)
{
	TIMSK |= 1 << interrupt;
	SREG |= 1 << SREG_I;
	IRCompAFlag = irFlag;
	TCNT1 = 0;
	return NoError;
	
}

Timer_Error_t Timer_removeInterrupt(enum Timer_Interrupt_t interrupt)
{
	TIMSK &= ~(1 << interrupt);
	return NoError;
}

Timer_Error_t Timer_calculateTimerSettings_s(uint16_t* compareAValue, Timer_ClockSignal_t* clockSignal, uint8_t seconds)
{
	uint16_t prescaler[5] = {1, 8, 64, 256, 1024};
	Timer_ClockSignal_t clockSignals[5] = {CSSystemClock, CSSystemClockDiv8, CSSystemClockDiv64, CSSystemClockDiv256, CSSystemClockDiv1024};
	uint8_t i = 0;
	while ((F_CPU / prescaler[i]) * seconds >= 65536)
	{
		if(i <= 4)
		{
			i++;
		}
		else
		{
			//Error
		}
	}
	*compareAValue = (F_CPU / prescaler[i]) * seconds;
	*clockSignal = clockSignals[i];
	
	return NoError;
}

Timer_Error_t Timer_calculateTimerSettings_ms(uint16_t* compareAValue, Timer_ClockSignal_t* clockSignal, uint16_t ms)
{
	uint16_t prescaler[5] = {1, 8, 64, 256, 1024};
	Timer_ClockSignal_t clockSignals[5] = {CSSystemClock, CSSystemClockDiv8, CSSystemClockDiv64, CSSystemClockDiv256, CSSystemClockDiv1024};
	uint8_t i = 0;
	while ((F_CPU / 1000 / prescaler[i]) * ms >= 65536)
	{
		if(i <= 4)
		{
			i++;
		}
		else
		{
			//Error
		}
	}
	*compareAValue = (F_CPU / 1000 / prescaler[i]) * ms;
	*clockSignal = clockSignals[i];

	return NoError;
}
