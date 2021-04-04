/*
 * Timer.h
 *
 * Created: 26.01.2021 16:27:30
 *  Author: Niklas Theis
 */ 

#pragma once

enum Timer_Error_t {Timer_NoError, Timer_Error, Timer_ErrorNoCSSet, Timer_Error_NotImplemented};
typedef enum Timer_Error_t Timer_Error_t;

enum Timer_ClockSignal_t {CSTimerStop = 0b000, CSSystemClock = 0b001, CSSystemClockDiv8 = 0b010, CSSystemClockDiv64 = 0b011, CSSystemClockDiv256 = 0b100, CSSystemClockDiv1024 = 0b101, CSExtClockT1Fall = 0b110, CSExtClockT1Rise = 0b111};
typedef enum Timer_ClockSignal_t Timer_ClockSignal_t;

enum Timer_CTCMode_t {NOCTC= 0b00, CTCTopOCR1A = 0b01, CTCTopICR1 = 0b10};
typedef enum Timer_CTCMode_t Timer_CTCMode_t;

enum Timer_Interrupt_t {InterruptInputCapture = 5, InterruptCompareA = 4, InterruptCompareB = 3, InterruptOverflow = 2};
typedef enum Timer_Interrupt_t Timer_Interrupt_t;

struct Timer_Settings_t
{
	uint16_t CompareAValue;
	uint16_t CompareBValue;
	Timer_ClockSignal_t ClockSignal;
	Timer_CTCMode_t CTCMode;
};
typedef struct Timer_Settings_t Timer_Settings_t;

Timer_ClockSignal_t ClockSignal;
uint8_t InitFlag;
volatile uint8_t* IRCompAFlag;
uint8_t* IRCompBFlag;

Timer_Error_t Timer_init(Timer_Settings_t timerSettings);
Timer_Error_t Timer_start();
Timer_Error_t Timer_stop();

Timer_Error_t Timer_addInterrupt(Timer_Interrupt_t inter, volatile uint8_t* irFlag);
Timer_Error_t Timer_removeInterrupt(Timer_Interrupt_t inter);

Timer_Error_t Timer_calculateTimerSettings_s(uint16_t* compareAValue, Timer_ClockSignal_t* clockSignal, uint8_t seconds);
Timer_Error_t Timer_calculateTimerSettings_ms(uint16_t* compareAValue, Timer_ClockSignal_t* clockSignal, uint16_t ms);