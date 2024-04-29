/*
 * MS5611.c
 *
 *  Created on: Feb 15, 2023
 *      Author: aldar
 */
#include "MS5611.h"
#include "main.h"
static uint8_t _buffer[21];

static uint16_t coef[8];
#define MS5611_RESET 0x1E
#define MS5611_PROM 0xA0
#define MS5611_PRES 0x40
#define MS5611_TEMP 0x50

#define MS5611_OSR_256 0
#define MS5611_OSR_512 2
#define MS5611_OSR_1024 4
#define MS5611_OSR_2048 6
#define MS5611_OSR_4096 8

uint32_t readADC(uint8_t addr){
	uint32_t val = 0;
	uint8_t data[3];
	MSwriteRegister(addr+MS5611_OSR_4096, 1);
	osDelay(10);
	//writeRegister(0x00, 1); // начало для барометра
	MSreadRegisters(0x00, 3, data);
	val += (int32_t)data[0] << 16;
	val += (int32_t)data[1] << 8;
	val += (int32_t)data[2];
	return val;
}
void reset(){
	MSwriteRegister(MS5611_RESET, 1);
	osDelay(4);
	readProm();
}
void readProm(){
	for (uint8_t i = 0; i < 8; i++) {
	  uint8_t dataprom[2];
	  MSreadRegisters(MS5611_PROM + 2 * i, 2, dataprom);
	  printf("Data calib: %d, %d\n", dataprom[0], dataprom[1]);
	  coef[i] = ((int16_t)dataprom[0] << 8) | (int16_t)dataprom[1];
	  printf("Data calib: %d\n", coef[i]);
	}
}
int32_t meas(){
	uint32_t D1, D2;
	D1 = readADC(MS5611_PRES);
	D2 = readADC(MS5611_TEMP);
	uint32_t dT, dTC6, Tref;
	  int32_t Temp;
	  Tref = (uint32_t)coef[5] << 8;
	  if (D2 < Tref) {
	    dT = Tref - D2;
	    dTC6 = ((uint64_t)dT * (uint64_t)coef[6]) >> 23;
	    Temp = 2000 - dTC6;
	  } else {
	    dT = D2 - Tref;
	    dTC6 = ((uint64_t)dT * (uint64_t)coef[6]) >> 23;
	    Temp = 2000 + dTC6;
	  }

	  // Convert pressure measurement to mbar
	  uint64_t offT1, TCOdT;
	  uint64_t sensT1, TCSdT;
	  int64_t off, sens;
	  int64_t P;
	  offT1 = (uint64_t)coef[2] << 16;
	  TCOdT = ((uint64_t)coef[4] * (uint64_t)dT) >> 7;
	  sensT1 = (uint64_t)coef[1] << 15;
	  TCSdT = ((uint64_t)coef[3] * (uint64_t)dT) >> 8;
	  if (D2 < Tref) {
	    off = offT1 - TCOdT;
	    sens = sensT1 - TCSdT;
	  } else {
	    off = offT1 + TCOdT;
	    sens = sensT1 + TCSdT;
	  }

	// Second order temperature compensation

	  uint32_t T2, off2, sens2;
	  if (Temp < 2000) {
	    T2 = (uint64_t)dT * (uint64_t)dT >> 31;
	    off2 = 5 * (Temp - 2000) * (Temp - 2000) >> 1;
	    sens2 = off2 >> 1;

	    if (Temp < -1500) {
	      uint32_t dtdt2;
	      dtdt2 = (Temp + 1500) * (Temp + 1500);
	      off2 += 7 * dtdt2;
	      sens2 += 11 * dtdt2 >> 1;
	    }
	    Temp -= T2;
	    off -= off2;
	    sens -= sens2;
	  }

	  P = (D1 * sens / 2097152 - off) / 32768;
	  //P = (D1 * sens) >> 21;
	  //P = (P - off) >> 15;

	  //temp = Temp;
	  //bar = (int32_t)P;
	  return (int32_t)P;
}
static inline void MS_Activate()
{
	HAL_GPIO_WritePin(MS5611_CS_GPIO, MS5611_CS_PIN, GPIO_PIN_RESET);
}

static inline void MS_Deactivate()
{
	HAL_GPIO_WritePin(MS5611_CS_GPIO, MS5611_CS_PIN, GPIO_PIN_SET);
}

uint8_t MSSPIx_WriteRead(uint8_t Byte)
{
	uint8_t receivedbyte = 0;
	if(HAL_SPI_TransmitReceive(&MS5611_SPI,(uint8_t*) &Byte,(uint8_t*) &receivedbyte,1,10000)!=HAL_OK)
	{
		return -1;
	}
	else
	{
	}
	return receivedbyte;
}

void MS_SPI_Write (uint8_t *pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite)
{
	MS_Activate();
	MSSPIx_WriteRead(WriteAddr);
	while(NumByteToWrite>=0x01)
	{
		MSSPIx_WriteRead(*pBuffer);
		NumByteToWrite--;
		pBuffer++;
	}
	MS_Deactivate();
}

void MS_SPI_Read(uint8_t *pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead)
{
	MS_Activate();
	uint8_t data = ReadAddr;
	HAL_SPI_Transmit(&MS5611_SPI, &data, 1, 1000);
	HAL_SPI_Receive(&MS5611_SPI, pBuffer, NumByteToRead, 1000);
	MS_Deactivate();
}

/* writes a byte to MPU9250 register given a register address and data */
void MSwriteRegister(uint8_t subAddress, uint8_t data)
{
	MS_SPI_Write(&data, subAddress, 1);
	HAL_Delay(10);
}

/* reads registers from MPU9250 given a starting register address, number of bytes, and a pointer to store data */
void MSreadRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest){
	MS_SPI_Read(dest, subAddress, count);
}


