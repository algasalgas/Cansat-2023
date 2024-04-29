################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/ADXL345.c \
../Core/Src/File_Handling.c \
../Core/Src/L3G4200D.c \
../Core/Src/MPU9250.c \
../Core/Src/MS5611.c \
../Core/Src/ds18b20.c \
../Core/Src/freertos.c \
../Core/Src/gpio.c \
../Core/Src/lora_sx1276.c \
../Core/Src/main.c \
../Core/Src/mmc5883.c \
../Core/Src/mpu9255.c \
../Core/Src/onewire.c \
../Core/Src/stm32f4xx_hal_msp.c \
../Core/Src/stm32f4xx_hal_timebase_tim.c \
../Core/Src/stm32f4xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32f4xx.c \
../Core/Src/tim.c \
../Core/Src/w25qxx.c 

OBJS += \
./Core/Src/ADXL345.o \
./Core/Src/File_Handling.o \
./Core/Src/L3G4200D.o \
./Core/Src/MPU9250.o \
./Core/Src/MS5611.o \
./Core/Src/ds18b20.o \
./Core/Src/freertos.o \
./Core/Src/gpio.o \
./Core/Src/lora_sx1276.o \
./Core/Src/main.o \
./Core/Src/mmc5883.o \
./Core/Src/mpu9255.o \
./Core/Src/onewire.o \
./Core/Src/stm32f4xx_hal_msp.o \
./Core/Src/stm32f4xx_hal_timebase_tim.o \
./Core/Src/stm32f4xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32f4xx.o \
./Core/Src/tim.o \
./Core/Src/w25qxx.o 

C_DEPS += \
./Core/Src/ADXL345.d \
./Core/Src/File_Handling.d \
./Core/Src/L3G4200D.d \
./Core/Src/MPU9250.d \
./Core/Src/MS5611.d \
./Core/Src/ds18b20.d \
./Core/Src/freertos.d \
./Core/Src/gpio.d \
./Core/Src/lora_sx1276.d \
./Core/Src/main.d \
./Core/Src/mmc5883.d \
./Core/Src/mpu9255.d \
./Core/Src/onewire.d \
./Core/Src/stm32f4xx_hal_msp.d \
./Core/Src/stm32f4xx_hal_timebase_tim.d \
./Core/Src/stm32f4xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32f4xx.d \
./Core/Src/tim.d \
./Core/Src/w25qxx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F401xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FatFs/src -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/ADXL345.d ./Core/Src/ADXL345.o ./Core/Src/ADXL345.su ./Core/Src/File_Handling.d ./Core/Src/File_Handling.o ./Core/Src/File_Handling.su ./Core/Src/L3G4200D.d ./Core/Src/L3G4200D.o ./Core/Src/L3G4200D.su ./Core/Src/MPU9250.d ./Core/Src/MPU9250.o ./Core/Src/MPU9250.su ./Core/Src/MS5611.d ./Core/Src/MS5611.o ./Core/Src/MS5611.su ./Core/Src/ds18b20.d ./Core/Src/ds18b20.o ./Core/Src/ds18b20.su ./Core/Src/freertos.d ./Core/Src/freertos.o ./Core/Src/freertos.su ./Core/Src/gpio.d ./Core/Src/gpio.o ./Core/Src/gpio.su ./Core/Src/lora_sx1276.d ./Core/Src/lora_sx1276.o ./Core/Src/lora_sx1276.su ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/mmc5883.d ./Core/Src/mmc5883.o ./Core/Src/mmc5883.su ./Core/Src/mpu9255.d ./Core/Src/mpu9255.o ./Core/Src/mpu9255.su ./Core/Src/onewire.d ./Core/Src/onewire.o ./Core/Src/onewire.su ./Core/Src/stm32f4xx_hal_msp.d ./Core/Src/stm32f4xx_hal_msp.o ./Core/Src/stm32f4xx_hal_msp.su ./Core/Src/stm32f4xx_hal_timebase_tim.d ./Core/Src/stm32f4xx_hal_timebase_tim.o ./Core/Src/stm32f4xx_hal_timebase_tim.su ./Core/Src/stm32f4xx_it.d ./Core/Src/stm32f4xx_it.o ./Core/Src/stm32f4xx_it.su ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32f4xx.d ./Core/Src/system_stm32f4xx.o ./Core/Src/system_stm32f4xx.su ./Core/Src/tim.d ./Core/Src/tim.o ./Core/Src/tim.su ./Core/Src/w25qxx.d ./Core/Src/w25qxx.o ./Core/Src/w25qxx.su

.PHONY: clean-Core-2f-Src

