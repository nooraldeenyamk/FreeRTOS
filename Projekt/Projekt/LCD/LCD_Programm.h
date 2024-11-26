/*
 * LCD_Programm.h
 *
 * Created: 19.11.2023 10:14:09
 *  Author: noors
 */ 


#ifndef LCD_PROGRAMM_H_
#define LCD_PROGRAMM_H_


#ifndef F_CPU
#define F_CPU 8000000
#endif



#define RS (1 << PB5)
#define RW (1 << PB6)
#define E  (1 << PB7)
#define D0 (1 << PL0)
#define D1 (1 << PL1)
#define D2 (1 << PL2)
#define D3 (1 << PL3)
#define D4 (1 << PL4)
#define D5 (1 << PL5)
#define D6 (1 << PL6)
#define D7 (1 << PL7)

//................LCD...................//

void LCD_Init(void);
void LCD_cmd(unsigned char cmd);
void LCD_Cursor_Position(uint8_t position, uint8_t line);
void LCD_Print(const char *text);
void LCD_Shift(void);
void LCD_float(float voltage_value);
void LCD_Pause(void);
void LCD_busy(void);
int itoa_s(int value, char *buf);
char *itoa_2(int value, char *buf);
char *ftoa(float value, int decimals, char *buf);
//..................ADC..................//
void ADC_init(void);




#endif /* LCD_PROGRAMM_H_ */