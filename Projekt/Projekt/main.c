/*
 * Projekt.c
 *
 * Created: 15.01.2024
 * 
 * Author : Yamk
 */  

#define F_CPU 8000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "LCD_Programm.h"
#include "SPI_interface.h"



void vTask1( void *pvParameters );
void vTask2( void *pvParameters );

SemaphoreHandle_t xMutex;


float f_values[2];


/*-----------------------------------------------------------*/

int main( void )
{	
	char buffer[16];
	DDRC = 0xFF;
	LCD_Init();
	SPI_Init();	
	
	xTaskCreate( vTask1, "Read_Task", 1000, NULL, 1, NULL );
	xTaskCreate( vTask2, "Write_Task", 1000, NULL, 1, NULL );
	xMutex = xSemaphoreCreateMutex();
	
	
	
	// Who am i
	LCD_Print("Who am I? ");
	LCD_Cursor_Position(0,2);
	LCD_Print(utoa(SPI_MasterReceive(WHO_AM_I),buffer,2));
	_delay_ms(3000);
	LCD_Init();
	
	vTaskStartScheduler();

	
	while(1)
	{	
		// if all is well then the PC will never reach here
		PORTC = 0x55;
		_delay_ms(1500);
		PORTC = 0xAA;
		_delay_ms(1500);

	}
	return 0;
}
/*-----------------------------------------------------------*/

void vTask1( void *pvParameters )
{
	TickType_t xLastWakeTime = xTaskGetTickCount();
	uint8_t i = 0; 


	while(1)
	{
		if(xSemaphoreTake(xMutex, portMAX_DELAY) == 1)
		{
			f_values[0] = read_Pressure();
			f_values[1] = read_Temperature();
		
	
			PORTC &= ~(1<<i);
			i++;
	
			if (i==8)
			{
				i = 0;
			}
			xSemaphoreGive(xMutex);
		}
		
		//vTaskDelay(pdMS_TO_TICKS(100UL));
		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS( 100UL ));

	}
}

/*-----------------------------------------------------------*/

void vTask2( void *pvParameters )
{
		
	TickType_t xLastWakeTime = xTaskGetTickCount();
	char buffer[16];
	
	
	while(1)
	{
		
		if(xSemaphoreTake(xMutex,portMAX_DELAY) == 1)
		{
			PORTC = 0xFF;
			LCD_Cursor_Position(0,1);
		
		
		
			// Pressure
			LCD_Print("p=");
			LCD_Print(ftoa(f_values[0],1,buffer));
			LCD_Print(" hPa");
		
			// Temperature
			LCD_Cursor_Position(0,2);
			LCD_Print("T=");
			LCD_Print(ftoa(f_values[1],1,buffer));
			LCD_Print("C ");
			
			// NHN
			LCD_Print("h=");
			LCD_Print(ftoa(NHN(f_values[0]),1,buffer));
			LCD_Print("m");
			xSemaphoreGive(xMutex);
		}
		//vTaskDelay(pdMS_TO_TICKS(100UL));
		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS( 100UL ));
		
	}

}


