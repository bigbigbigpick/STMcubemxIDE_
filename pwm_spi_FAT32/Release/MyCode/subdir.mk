################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../MyCode/SDCard.c \
../MyCode/file_app.c \
../MyCode/spi_sca100T.c 

OBJS += \
./MyCode/SDCard.o \
./MyCode/file_app.o \
./MyCode/spi_sca100T.o 

C_DEPS += \
./MyCode/SDCard.d \
./MyCode/file_app.d \
./MyCode/spi_sca100T.d 


# Each subdirectory must supply rules for building sources it contributes
MyCode/SDCard.o: ../MyCode/SDCard.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -I"D:/ST/workspace/pwm_spi_FAT32/MyCode" -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"MyCode/SDCard.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
MyCode/file_app.o: ../MyCode/file_app.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -I"D:/ST/workspace/pwm_spi_FAT32/MyCode" -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"MyCode/file_app.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
MyCode/spi_sca100T.o: ../MyCode/spi_sca100T.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -I"D:/ST/workspace/pwm_spi_FAT32/MyCode" -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"MyCode/spi_sca100T.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

