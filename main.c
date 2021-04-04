/*
 * Projekt.c
 *
 * Created: 26.01.2021 16:19:56
 * Author : Niklas Theis
 */ 

#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>
#include <string.h>
#include "CO2Output.h"
#include "Timer.h"
#include "utils.h"
#include "CO2Sensor.h"

#define setpin(port,bitnummer) port |= (1<<bitnummer)
#define clrpin(port,bitnummer) port &= ~(1<<bitnummer)

#define LCDPORT PORTB
#define LCDDDR DDRB

#define BUTTONUP ((~PIND & (1 << 0)) >> 0)
#define BUTTONDOWN ((~PIND & (1 << 1)) >> 1)

#define LCD_RS_PIN 0
#define LCD_RW_PIN 1
#define LCD_E_PIN 2

uint8_t buttonUp = 0;
uint8_t buttonUp_before = 0;
uint8_t buttonDown = 0;
uint8_t buttonDown_before = 0;

uint8_t timerInterruptFlag = 0;
uint8_t timerLEDOutput = 0;

//struct SensorData_t sensorData;

void CallbackDataReady(void)
{
	return;
}

int main(void)
{
	LCDDDR = 0xff;
	DDRD = 0b01110100;
	DDRA = 0xff;
	struct SensorData_t sensorData;
	struct SensorConfigData_t Sensorconfig =
	{
		.altitude_in_m_u16 = 0,
		.meas_interval_in_sec_u16 = 2,
		.ambient_pressure_in_mbar_u16 = 0,
		.temp_offset_u16 = 0
	};
	
	LCD_Settings_t lcdSettings =
	{
		.Cursor = CursorOff,
		.Port = &LCDPORT,
		.PortDDR = &LCDDDR,
		.PortPIN = &PINB
	};
	
	struct Timer_Settings_t timerSettings =
	{
		.CompareBValue = 0,
		.CTCMode = CTCTopOCR1A
	};
	
	CO2Output_Init(&sensorData, &lcdSettings, NoAlign);
	
	Timer_calculateTimerSettings_ms(&(timerSettings.CompareAValue), &(timerSettings.ClockSignal), 3000);
	
	Timer_init(timerSettings);
	Timer_addInterrupt(InterruptCompareA, &timerInterruptFlag);

		
	CO2_InitSensor(&sensorData, &Sensorconfig);
	//Config
	CO2_ConfigSensor();
	CO2_StartMeasurement(0);
	CO2_UpdateSensorParameterData();

	while(1)
	{
		if (timerInterruptFlag)
		{
			CO2_GetDataReadyStatus();		
			if(sensorData.new_data_available_u16)
			{
				CO2_GetMeasurementData();
				CO2Output_UpdateData();
				
			}
			
			timerInterruptFlag = 0;
		}
		buttonUp = BUTTONUP;
		buttonDown = BUTTONDOWN;
		if ((buttonUp != buttonUp_before) && buttonUp == 1 && buttonDown == 0)
		{
			CO2Output_MoveUp();
		}
		buttonUp_before = buttonUp;
		if ((buttonDown != buttonDown_before) && buttonDown == 1 && buttonUp == 0)
		{
			CO2Output_MoveDown();
		}
		buttonDown_before = buttonDown;
	}
}