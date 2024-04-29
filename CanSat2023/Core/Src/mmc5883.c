/*
 * mmc5883.c
 *
 *  Created on: Apr 26, 2023
 *      Author: aldar
 */
#include "mmc5883.h"
int16_t lx = 0;
int16_t ly = 0;
int16_t lz = 0;
void MMC5883_init(){
	writeRegMMC(MMC5883MA_INTERNAL_CONTROL_1, 0x00);
	writeRegMMC(MMC5883MA_INTERNAL_CONTROL_2, 0x02);
	writeRegMMC(MMC5883MA_INTERNAL_CONTROL_0, 0x01);
}
void MMCgetMag(int16_t* mag){
    uint8_t out2[6];

    readRegMMC(MMC5883_OUT,out2+0,1);
    readRegMMC(MMC5883_OUT+1,out2+1,1);
    readRegMMC(MMC5883_OUT+2,out2+2,1);
    readRegMMC(MMC5883_OUT+3,out2+3,1);
    readRegMMC(MMC5883_OUT+4,out2+4,1);
    readRegMMC(MMC5883_OUT+5,out2+5,1);

    mag[0] = ((int16_t)(out2[1] << 8) | (int16_t)out2[0]);
    mag[1] = ((int16_t)(out2[3] << 8) | (int16_t)out2[2]);
    mag[2] = ((int16_t)(out2[5] << 8) | (int16_t)out2[4]);
}
/*
void writeRegMMC(uint8_t address, uint8_t data){
	HAL_I2C_Master_Transmit(&hi2c1, MMC5883, &address, 1, 100);
	HAL_I2C_Master_Transmit(&hi2c1, MMC5883, &data, 1, 100);
}
*/
int16_t MMCgetMAGX(){
	return lx;
}
int16_t MMCgetMAGY(){
	return ly;
}
int16_t MMCgetMAGZ(){
	return lz;
}
void writeRegMMC(uint8_t address, uint8_t data){
	/*
	HAL_I2C_Master_Transmit(&hi2c1, L3G4200D, address, 1, 100);
	HAL_I2C_Master_Transmit(&hi2c1, L3G4200D, data, 1, 100);
	*/
	uint8_t WriteData[2] = {0};
	WriteData[0] = address;
	WriteData[1] = data;

	HAL_I2C_Master_Transmit(&hi2c1, MMC5883, WriteData, 2, 100);
}
/*
void readRegMMC(uint8_t address, uint8_t *data, uint8_t size){
	HAL_I2C_Master_Transmit(&hi2c1, MMC5883, &address, 1,  100);
	HAL_I2C_Master_Receive(&hi2c1, MMC5883, data, size, 100);
}
*/
void readRegMMC(uint8_t address, uint8_t *data, uint8_t size){
	/*
	HAL_I2C_Master_Transmit(&hi2c1, L3G4200D, &address, 1,  100);
	HAL_I2C_Master_Receive(&hi2c1, L3G4200D, data, size, 100);
	*/
	HAL_I2C_Mem_Read(&hi2c1, MMC5883, address, size, data,
					I2C_MEMADD_SIZE_8BIT, 100);
}

