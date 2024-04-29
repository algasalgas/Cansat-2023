/*
 * L3G4200D.c
 *
 *  Created on: Apr 26, 2023
 *      Author: aldar
 */
#include "L3G4200D.h"
void L3G4200D_init(){
	osDelay(10);
	writeRegL3G(L3G4200D_CTRL_REG2, 0b00000000);
	osDelay(10);
	writeRegL3G(L3G4200D_CTRL_REG3, 0b00000000);
	osDelay(10);
	writeRegL3G(L3G4200D_CTRL_REG4, 0b00000000);
	osDelay(10);
	writeRegL3G(L3G4200D_CTRL_REG5, 0b00000000);
	osDelay(10);
	writeRegL3G(L3G4200D_CTRL_REG1, 0b00001111);
		osDelay(10);
	//writeRegL3G(L3G4200D_CTRL_REG1, 0x3F);
}
int16_t x;
int16_t y;
int16_t z;
//uint8_t out1[6] = {};
char asd[128] = {};
void L3GgetGyro(){
	uint8_t out1[6] = {};
	uint8_t reg1 = 0x00;
	    reg1 |= 0x0F; // Enable all axis and setup normal mode
	    reg1 |= (0b1010 << 4); // Set output data rate & bandwidh

	//readRegL3G(L3G4200D_OUT_X_L | (1 << 7), out1, 6);
	/*
    readRegL3G(L3G4200D_OUT_X_L | 0x80,out1,1);
    readRegL3G(L3G4200D_OUT_X_H | 0x80,out1+1,1);
    readRegL3G(L3G4200D_OUT_Y_L | 0x80,out1+2,1);
    readRegL3G(L3G4200D_OUT_Y_H | 0x80,out1+3,1);
    readRegL3G(L3G4200D_OUT_Z_L | 0x80,out1+4,1);
    readRegL3G(L3G4200D_OUT_Z_H | 0x80,out1+5,1);
    */
	readRegL3G(L3G4200D_OUT_X_L,out1+0,1);
    readRegL3G(L3G4200D_OUT_X_H,out1+1,1);
    readRegL3G(L3G4200D_OUT_Y_L,out1+2,1);
    readRegL3G(L3G4200D_OUT_Y_H,out1+3,1);
    readRegL3G(L3G4200D_OUT_Z_L,out1+4,1);
    readRegL3G(L3G4200D_OUT_Z_H,out1+5,1);

    x = ((int16_t)(out1[1] << 8) | (int16_t)out1[0]);
    y = ((int16_t)(out1[3] << 8) | (int16_t)out1[2]);
    z = ((int16_t)(out1[5] << 8) | (int16_t)out1[4]);

    //sprintf(asd, "\n\n\n %d %d %d %d %d %d %d %d %d\n\n\n", out1[0], out1[1], out1[2], out1[3], out1[4], out1[5], x, y, z);
    //CDC_Transmit_FS(asd, 256);

}
int16_t L3GgetGyroX(){
	return x;
}
int16_t L3GgetGyroY(){
	return y;
}
int16_t L3GgetGyroZ(){
	return z;
}

void writeRegL3G(uint8_t address, uint8_t data){
	/*
	HAL_I2C_Master_Transmit(&hi2c1, L3G4200D, address, 1, 100);
	HAL_I2C_Master_Transmit(&hi2c1, L3G4200D, data, 1, 100);
	*/
	uint8_t WriteData[2] = {0};
	WriteData[0] = address;
	WriteData[1] = data;

	HAL_I2C_Master_Transmit(&hi2c1, L3G4200D, WriteData, 2, 100);
}
void readRegL3G(uint8_t address, uint8_t *data, uint8_t size){
	/*
	HAL_I2C_Master_Transmit(&hi2c1, L3G4200D, &address, 1,  100);
	HAL_I2C_Master_Receive(&hi2c1, L3G4200D, data, size, 100);
	*/
	HAL_I2C_Mem_Read(&hi2c1, L3G4200D, address, size, data,
					I2C_MEMADD_SIZE_8BIT, 100);
}
uint8_t L3Gid(){
	uint8_t out[1];
	readRegL3G(0x0F, out, 1);
	return out[0];
}


