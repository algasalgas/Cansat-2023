/*
 * MS5611.h
 *
 *  Created on: Feb 15, 2023
 *      Author: aldar
 */

#ifndef INC_MS5611_H_
#define INC_MS5611_H_

#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "main.h"

extern SPI_HandleTypeDef hspi1;
#define MS5611_SPI			hspi1
#define	MS5611_CS_GPIO		GPIOA
#define	MS5611_CS_PIN		GPIO_PIN_15
#define MS5611_RESET 0x1E
#define MS5611_PROM 0xA0
#define MS5611_PRES 0x40
#define MS5611_TEMP 0x50

#define MS5611_OSR_256 0
#define MS5611_OSR_512 2
#define MS5611_OSR_1024 4
#define MS5611_OSR_2048 6
#define MS5611_OSR_4096 8

void MSwriteRegister(uint8_t subAddress, uint8_t data);
void MSreadRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest);
void MS_SPI_Read(uint8_t *pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead);
void MS_SPI_Write (uint8_t *pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite);
uint8_t MSSPIx_WriteRead(uint8_t Byte);
void readProm();
void reset();
uint32_t readADC(uint8_t addr);
int32_t meas();

#endif /* INC_MS5611_H_ */
