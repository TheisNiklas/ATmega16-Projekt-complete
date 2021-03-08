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
	CO2Output_Init(&sensorData, &LCDPORT, CursorOff, NoAlign);
	
	struct Timer_Settings_t timerSettings =
	{
		.CTCMode = CTCTopOCR1A,
		.PWNCOM1A = NoPWM,
	};
	uint16_t CompA = 0;
	Timer_calculateTimerSettings_ms(&CompA, &(timerSettings.ClockSignal), 3000);
	
	Timer_init(CompA, 0, timerSettings);
	Timer_addInterrupt(InterruptCompareA, &timerInterruptFlag);

		
	//TODO: InitSensor with Pointer
	CO2_InitSensor(&sensorData, &Sensorconfig);
	//Config
	CO2_ConfigSensor();
	CO2_StartMeasurement(0);
	CO2_UpdateSensorParameterData();

	while(1)
	{
		if (timerInterruptFlag)
		{
			//TODO: Werte abholen CO2_StartMeasurement();
			CO2_GetDataReadyStatus();		
			if(sensorData.new_data_available_u16)
			{
				CO2_GetMeasurementData();
				CO2Output_UpdateData();
				
			}
			
			
			if (timerLEDOutput)
			{
				PORTD &= 0b11111011;
				timerLEDOutput = 0;
			}
			else
			{
				PORTD |= 0b00000100;
				timerLEDOutput = 1;
			}
			timerInterruptFlag = 0;
		}
		buttonUp = BUTTONUP;
		buttonDown = BUTTONDOWN;
		if ((buttonUp != buttonUp_before) && buttonUp == 1 && buttonDown == 0)
		{
			CO2Output_MoveUp();
			/*LCD_Clear();
			float deb = 230.5f;
			char deb1[40];
			
			ConvertFloatToCharArray(deb1, deb);
			
			LCD_Write2Lines(deb1, NULL);*/
		}
		buttonUp_before = buttonUp;
		if ((buttonDown != buttonDown_before) && buttonDown == 1 && buttonUp == 0)
		{
			CO2Output_MoveDown();
		}
		buttonDown_before = buttonDown;
	}
}