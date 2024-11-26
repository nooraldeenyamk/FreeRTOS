/*
 * SPI_interface.h
 *
 * Created: 19.11.2023 10:15:03
 *  Author: noors
 */ 


#ifndef SPI_INTERFACE_H_
#define SPI_INTERFACE_H_

#ifndef F_CPU
#define F_CPU 8000000
#endif

/*..................Macros..................*/
#define CS			 (1<< PINB0)
#define SPI_WRITE	 (0x00)
#define SPI_READ	 (0x80)
#define WHO_AM_I	 (0x0F)  
#define CTRL_REG1	 (0x10)
#define CONFIG       (0x2C)  // DR=10Hz , LPF enable (ORD/20).  
#define PRESS_OUT_XL (0x28)
#define PRESS_OUT_L  (0x29)
#define PRESS_OUT_H  (0x2A)
#define TEMP_OUT_L	 (0x2B)
#define TEMP_OUT_H	 (0x2C)
#define p0			 (1013.25f)
#define T_h			 (294.15f)


/*..................Functions..................*/
void SPI_Init(void);
void SPI_MasterTransmit(uint8_t cReg, uint8_t cData);
uint8_t SPI_MasterReceive(uint8_t cReg);
float read_Pressure();
float read_Temperature();
float NHN(float pressure);


#endif /* SPI_INTERFACE_H_ */