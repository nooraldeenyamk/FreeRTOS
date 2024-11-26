/*
 * SPI_interface.c
 *
 * Created: 19.11.2023 10:15:10
 *  Author: noors
 */ 

#define  F_CPU 8000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "SPI_interface.h"
#include "LCD_Programm.h"



void SPI_Init(void)
{
	DDRB |= (1<<PINB2) | (1<<PINB1) | (1<<PINB0);		/* Make MOSI, SCK, CS as Output pin */
	DDRB &= ~(1<<PINB3);								/* Make MISO pin as input pin */
	PORTB |= CS;										/* Make high on CS pin */
	
													    /* Enable SPI in master mode with F_osc/16 */
	SPCR |= (1<<SPE) | (1<<MSTR) | (1<<CPOL) | (1<<SPR0)| (1<<CPHA);
	SPCR &= ~(1<<SPR1);									/* F_osc/16 */
	SPCR &= ~(1<<DORD);									/* Data Order to MSB first first*/
	SPSR &= ~(1<<SPI2X);								/* Disable speed doubler */
}


void SPI_MasterTransmit(uint8_t cReg, uint8_t cData)
{
	PORTB &= ~CS;		/* enable CS pin */
	/* Start transmission with register-address*/
	SPDR= cReg | SPI_WRITE;
	
	/* Wait for transmission complete*/
	while(!(SPSR & (1<<SPIF)));
	
	/* Resume transmission with register-data*/
	SPDR=cData;
	
	/* Wait for transmission complete*/
	while(!(SPSR & (1<<SPIF)));
	PORTB |= CS;      /* disable CS pin */
}


uint8_t SPI_MasterReceive(uint8_t cReg)
{
	uint8_t ret;
	
	PORTB &= ~CS; /* enable CS pin */
	/* Start transmission*/
	SPDR= cReg | SPI_READ;
	
	/* Wait for transmission complete*/
	while(!(SPSR & (1<<SPIF)));
		
	/* Resume Transmission with some Data*/
	SPDR=0xFF;
	
	/* Wait for transmission complete*/
	while(!(SPSR & (1<<SPIF)));
	
	ret= SPDR;
	PORTB |= CS; /* disable CS pin */
	
	return ret; //Return the received data
}

float read_Pressure()
{
	uint8_t values[3];
	
	SPI_MasterTransmit(CTRL_REG1,CONFIG);
	values[0] = SPI_MasterReceive(PRESS_OUT_XL);
	values[1] = SPI_MasterReceive(PRESS_OUT_L);
	values[2] = SPI_MasterReceive(PRESS_OUT_H);
	
	return ((float)(((uint32_t)values[2]<< 16) + ((uint16_t)values[1]<<8) + (values[0]))/4096.0f);
}

float read_Temperature()
{
	uint8_t values[2];
	
	SPI_MasterTransmit(CTRL_REG1,CONFIG);
	values[0] = SPI_MasterReceive(TEMP_OUT_L);
	values[1] = SPI_MasterReceive(TEMP_OUT_H);
	
	return ((float)(((uint16_t)values[1]<<8) + (values[0]))/100.0f);
}

float NHN(float pressure)
{
	return ((T_h/0.0065f) * (1.0f -(pow((float)(pressure/p0),(1.0f/5.225f)))));
}


