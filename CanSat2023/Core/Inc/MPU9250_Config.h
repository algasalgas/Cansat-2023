/*
 * MPU9250_Config.h
 *
 *  Created on: Feb 28, 2019
 *      Author: Desert
 */
#include "main.h"
#ifndef UTIL_MPU9250_CONFIG_H_
#define UTIL_MPU9250_CONFIG_H_
extern I2C_HandleTypeDef hi2c1;
extern SPI_HandleTypeDef hspi1;
#define MPU9250_SPI			hspi1
#define	MPU9250_CS_GPIO		GPIOA
#define	MPU9250_CS_PIN		GPIO_PIN_4

#endif /* UTIL_MPU9250_CONFIG_H_ */
