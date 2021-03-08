/*
 * Timer.h
 *
 * Created: 26.01.2021 16:27:30
 *  Author: Niklas Theis
 */ 

#pragma once

enum Timer_Error_t {NoError, Error, ErrorNoCSSet};
typedef enum Timer_Error_t Timer_Error_t;

enum Timer_ClockSignal_t {CSTimerStop = 0b000, CSSystemClock = 0b001, CSSystemClockDiv8 = 0b010, CSSystemClockDiv64 = 0b011, CSSystemClockDiv256 = 0b100, CSSystemClockDiv1024 = 0b101, CSExtClockT1Fall = 0b110, CSExtClockT1Rise = 0b111};
typedef enum Timer_ClockSignal_t Timer_ClockSignal_t;

enum Timer_CTCMode_t {CTCTopOCR1A = 0b01, CTCTopICR1 = 0b10};
typedef enum Timer_CTCMode_t Timer_CTCMode_t;

enum Timer_PWMCOM1A_t {NoPWM, PWNNormal, PWMInverted};
typedef enum Timer_PWMCOM1A_t Timer_PWMCOM1A_t;

enum Timer_Interrupt_t {InterruptInputCapture = 5, InterruptCompareA = 4, InterruptCompareB = 3, InterruptOverflow = 2};
typedef enum Timer_Interrupt_t Timer_Interrupt_t;

struct Timer_Settings_t
{
	Timer_ClockSignal_t ClockSignal;
	Timer_CTCMode_t CTCMode;
	Timer_PWMCOM1A_t PWNCOM1A;
};
typedef struct Timer_Settings_t Timer_Settings_t;

Timer_ClockSignal_t ClockSignal;
uint8_t InitFlag;
volatile uint8_t* IRCompAFlag;
uint8_t* IRCompBFlag;

Timer_Error_t Timer_init(uint16_t compareAValue, uint16_t compareBValue, Timer_Settings_t timerSettings);
Timer_Error_t Timer_start();
Timer_Error_t Timer_stop();

Timer_Error_t Timer_addInterrupt(enum Timer_Interrupt_t interrupt, volatile uint8_t* irFlag);
Timer_Error_t Timer_removeInterrupt(enum Timer_Interrupt_t interrupt);

Timer_Error_t Timer_calculateTimerSettings_s(uint16_t* compareAValue, Timer_ClockSignal_t* clockSignal, uint8_t seconds);
Timer_Error_t Timer_calculateTimerSettings_ms(uint16_t* compareAValue, Timer_ClockSignal_t* clockSignal, uint16_t ms);