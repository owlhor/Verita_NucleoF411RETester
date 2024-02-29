################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/LCDrv_f4_spi/ili9341.c \
../Core/Src/LCDrv_f4_spi/lcd_io_spi.c 

C_DEPS += \
./Core/Src/LCDrv_f4_spi/ili9341.d \
./Core/Src/LCDrv_f4_spi/lcd_io_spi.d 

OBJS += \
./Core/Src/LCDrv_f4_spi/ili9341.o \
./Core/Src/LCDrv_f4_spi/lcd_io_spi.o 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/LCDrv_f4_spi/%.o Core/Src/LCDrv_f4_spi/%.su Core/Src/LCDrv_f4_spi/%.cyclo: ../Core/Src/LCDrv_f4_spi/%.c Core/Src/LCDrv_f4_spi/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-LCDrv_f4_spi

clean-Core-2f-Src-2f-LCDrv_f4_spi:
	-$(RM) ./Core/Src/LCDrv_f4_spi/ili9341.cyclo ./Core/Src/LCDrv_f4_spi/ili9341.d ./Core/Src/LCDrv_f4_spi/ili9341.o ./Core/Src/LCDrv_f4_spi/ili9341.su ./Core/Src/LCDrv_f4_spi/lcd_io_spi.cyclo ./Core/Src/LCDrv_f4_spi/lcd_io_spi.d ./Core/Src/LCDrv_f4_spi/lcd_io_spi.o ./Core/Src/LCDrv_f4_spi/lcd_io_spi.su

.PHONY: clean-Core-2f-Src-2f-LCDrv_f4_spi

