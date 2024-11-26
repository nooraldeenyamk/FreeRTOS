/*
 * LCD_Programm.c
 *
 * Created: 19.11.2023 10:14:25
 *  Author: noors
 */ 

#include <avr/io.h>
#include "LCD_Programm.h"
#include <util/delay.h>
#include <stdio.h>



void LCD_cmd(unsigned char cmd )
{
	
	PORTB &= ~(RS);
	PORTB &= ~(RW);
	PORTB |= E;
	PORTL= cmd;
	_delay_us(1);
	PORTB &= ~(E);
}

void LCD_Init(void)
{
	DDRB |= (RS | RW | E);
	DDRL = 0xFF;
	LCD_cmd(0b00111000)  ; // 2 Line   	PORTL  = (D3 | D4 | D5) ;
	_delay_us(42);
	LCD_cmd(0b00001110)  ; // Display on, Cursor on, Blinking off   PORTL  = (D1 | D2 | D3);
	_delay_us(42);
	//LCD_cmd(0b00010110)  ; // Shifts the cursor position to the right. (AC is incremented by one.) , Display is not shifted.  = (D1 | D2 | D4) ;
	//_delay_us(40);
	LCD_cmd(0b00000001);
	_delay_ms(2);
	

}

void LCD_Cursor_Position(uint8_t position, uint8_t line)  // 1. Line and 0th position: 0x80  , 2. Line and 0th position 0xC0
{
	if (line ==1)
	{
		LCD_cmd(0x80 + position);
		_delay_us(42);

	}
	else if (line ==2)
	{
		LCD_cmd(0xC0 + position);
		_delay_us(42);
	}
}


void LCD_Print(const char *data)
{
	
	PORTB |= RS;
	PORTB &= ~(RW);
	
	while (*data)
	{
		PORTB |= E;
		_delay_us(1);
		PORTL = *data;
		PORTB &= ~(E);
		data++;
		_delay_us(42);
	}
	
}

void LCD_Shift(void)
{
	LCD_cmd(0b00011010);
	_delay_us(42);
}

void LCD_float(float voltage_value)
{
	char buffer[16]; // Characters holder
	sprintf(buffer,"%.2f", voltage_value); // to convert from float to string , 2 two decimal places
	
	LCD_Cursor_Position(0,1);
	LCD_Print(buffer);
}

void LCD_Pause(void)
{
	_delay_ms(10);
}
void LCD_busy(void)
{
	PORTB &= ~(RS);
	PORTB |= (RW); // busy flag check
	while(1)
	{
		
		PORTB |= (E);
		if (((PORTL & (0b10000000))== 0))
		{
			_delay_us(1);
			PORTB &= ~(E);
			break;
		}
		_delay_us(1);
		PORTB &= ~(E);
	}
	
}

//..........................................ADC...........................................//
void ADC_init(void)
{
	ADMUX |=  (1<<REFS0);
	ADMUX &= ~(1<<REFS1); // external Voltage
	
	ADMUX  &= ~((1<<MUX0)|(1<<MUX1)|(1<<MUX2)|(1<<MUX3)|(1<<MUX4));
	ADCSRB &= ~(1<<MUX5); // ADC0
	
	ADCSRA &= ~(1<<ADPS0);
	ADCSRA |=  (1<<ADPS1)|(1<<ADPS2); // Prescaler = 64
	
	ADCSRA |= (1<<ADEN); // ADC on
}

int itoa_s(int value, char *buf)
{
	int index = 0;
	int i = value % 10;
	if (value >= 10) {
		index += itoa_s(value / 10, buf);
	}
	buf[index] = i+0x30;
	index++;
	return index;
}

char *itoa_2(int value, char *buf)
{
	int len = itoa_s(value, buf);
	buf[len] = '\0';
	return buf;
}

char *ftoa(float value, int decimals, char *buf)
{
	int index = 0;
	// Handle negative values
	if (value < 0) {
		buf[index] = '-';
		index++;
		value = -value;
	}
	
	// Rounding
	float rounding = 0.5;
	for (int d = 0; d < decimals; rounding /= 10.0, d++);
	value += rounding;

	// Integer part
	index += itoa_s((int)(value), buf+index);
	buf[index++] = '.';

	// Remove everything except the decimals
	value = value - (int)(value);

	// Convert decmial part to integer
	int ival = 1;
	for (int d = 0; d < decimals; ival *= 10, d++);
	ival *= value;

	// Add decimal part to string
	index += itoa_s(ival, buf+index);
	buf[index] = '\0';
	return buf;
}