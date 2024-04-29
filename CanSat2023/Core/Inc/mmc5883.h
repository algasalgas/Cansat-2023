/*
 * mmc5883.h
 *
 *  Created on: Apr 26, 2023
 *      Author: aldar
 */

#ifndef INC_MMC5883_H_
#define INC_MMC5883_H_

#define MMC5883 0x30 << 1
#define MMC5883_PRODUCT_ID 0x2F
#define MMC5883_OUT 0x00
#define MMC5883_XOUT 0x00
#define MMC5883_XOUT_LOW 0x00
#define MMC5883_XOUT_HIGH 0x01
#define MMC5883_YOUT 0x02
#define MMC5883_YOUT_LOW 0x02
#define MMC5883_YOUT_HIGH 0x03
#define MMC5883_ZOUT 0x04
#define MMC5883_ZOUT_LOW 0x04
#define MMC5883_ZOUT_HIGH 0x05
#define MMC5883MA_STATUS 0x07
#define MMC5883MA_INTERNAL_CONTROL_0 0x08
#define MMC5883MA_INTERNAL_CONTROL_1 0x09
#define MMC5883MA_INTERNAL_CONTROL_2 0x0A
#define MMC5883MA_PRODUCT_ID 0x2F

#define MMC5883MA_DYNAMIC_RANGE 16
#define MMC5883MA_RESOLUTION 65536
#include "main.h"
extern I2C_HandleTypeDef hi2c1;
void MMC5883_init();
void MMCgetMag();
int16_t MMCgetMAGX();
int16_t MMCgetMAGY();
int16_t MMCgetMAGZ();
void writeRegMMC(uint8_t address, uint8_t data);
void readRegMMC(uint8_t address, uint8_t *data, uint8_t size);

#endif /* INC_MMC5883_H_ */
