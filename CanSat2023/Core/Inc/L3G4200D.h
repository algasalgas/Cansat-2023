/*
 * L3G4200D.h
 *
 *  Created on: Apr 26, 2023
 *      Author: aldar
 */

#ifndef INC_L3G4200D_H_
#define INC_L3G4200D_H_
#include "main.h"
extern I2C_HandleTypeDef hi2c1;

#define L3G4200D 0x69 << 1
#define L3G4200D_WHO_AM_I      (0x0F)

#define L3G4200D_CTRL_REG1     (0x20)
#define L3G4200D_CTRL_REG2     (0x21)
#define L3G4200D_CTRL_REG3     (0x22)
#define L3G4200D_CTRL_REG4     (0x23)
#define L3G4200D_CTRL_REG5     (0x24)
#define L3G4200D_REFERENCE     (0x25)
#define L3G4200D_OUT_TEMP      (0x26)
#define L3G4200D_STATUS_REG    (0x27)

#define L3G4200D_OUT_X_L       (0x28)
#define L3G4200D_OUT_X_H       (0x29)
#define L3G4200D_OUT_Y_L       (0x2A)
#define L3G4200D_OUT_Y_H       (0x2B)
#define L3G4200D_OUT_Z_L       (0x2C)
#define L3G4200D_OUT_Z_H       (0x2D)

void L3G4200D_init();
void L3GgetGyro();
int16_t L3GgetGyroX();
int16_t L3GgetGyroY();
int16_t L3GgetGyroZ();
void writeRegL3G(uint8_t address, uint8_t data);
void readRegL3G(uint8_t address, uint8_t *data, uint8_t size);
uint8_t L3Gid();

#endif /* INC_L3G4200D_H_ */
