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

enum Timer_PWMCOM1A_t {NoPWM, PWNNormal, PWMInverted};
typedef enum Timer_PWMCOM1A_t Timer_PWMCOM1A_t;

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

Timer_Error_t Timer_init(Timer_Settings_t timerSettings)
{
	ClockSignal = timerSettings.ClockSignal;
	
	TCCR1A = 0b00000000;
	TCCR1B = timerSettings.CTCMode << WGM12 | timerSettings.ClockSignal;
	
	OCR1A = timerSettings.CompareAValue;
	OCR1B = timerSettings.CompareBValue;
	
	TCNT1 = 0;
	
	InitFlag = 1;
	return Timer_NoError;
}

Timer_Error_t Timer_start()
{
	if (!InitFlag || ClockSignal == CSTimerStop)
	{
		return Timer_ErrorNoCSSet;
	}
	TCCR1B |= ClockSignal;
	TCNT1 = 0;
	return Timer_NoError;
}

Timer_Error_t Timer_stop()
{
	TCCR1B &= ~(CSTimerStop);
	return Timer_NoError;
}

Timer_Error_t Timer_reset()
{
	TCNT1 = 0;
	return Timer_NoError;
}

Timer_Error_t Timer_addInterrupt(Timer_Interrupt_t inter, volatile uint8_t* irFlag)
{
	TIMSK |= 1 << inter;
	SREG |= 1 << SREG_I;
	IRCompAFlag = irFlag;
	TCNT1 = 0;
	return Timer_NoError;
	
}

Timer_Error_t Timer_removeInterrupt(Timer_Interrupt_t inter)
{
	TIMSK &= ~(1 << inter);
	return Timer_NoError;
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
			return Timer_Error;
		}
	}
	*compareAValue = (F_CPU / prescaler[i]) * seconds;
	*clockSignal = clockSignals[i];
	
	return Timer_NoError;
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
			return Timer_Error;
		}
	}
	*compareAValue = (F_CPU / 1000 / prescaler[i]) * ms;
	*clockSignal = clockSignals[i];

	return Timer_NoError;
}
