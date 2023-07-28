################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/LCDrv_f4_spi/Fonts/font12.c \
../Core/Src/LCDrv_f4_spi/Fonts/font16.c \
../Core/Src/LCDrv_f4_spi/Fonts/font20.c \
../Core/Src/LCDrv_f4_spi/Fonts/font24.c \
../Core/Src/LCDrv_f4_spi/Fonts/font8.c 

OBJS += \
./Core/Src/LCDrv_f4_spi/Fonts/font12.o \
./Core/Src/LCDrv_f4_spi/Fonts/font16.o \
./Core/Src/LCDrv_f4_spi/Fonts/font20.o \
./Core/Src/LCDrv_f4_spi/Fonts/font24.o \
./Core/Src/LCDrv_f4_spi/Fonts/font8.o 

C_DEPS += \
./Core/Src/LCDrv_f4_spi/Fonts/font12.d \
./Core/Src/LCDrv_f4_spi/Fonts/font16.d \
./Core/Src/LCDrv_f4_spi/Fonts/font20.d \
./Core/Src/LCDrv_f4_spi/Fonts/font24.d \
./Core/Src/LCDrv_f4_spi/Fonts/font8.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/LCDrv_f4_spi/Fonts/%.o Core/Src/LCDrv_f4_spi/Fonts/%.su Core/Src/LCDrv_f4_spi/Fonts/%.cyclo: ../Core/Src/LCDrv_f4_spi/Fonts/%.c Core/Src/LCDrv_f4_spi/Fonts/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F411xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-LCDrv_f4_spi-2f-Fonts

clean-Core-2f-Src-2f-LCDrv_f4_spi-2f-Fonts:
	-$(RM) ./Core/Src/LCDrv_f4_spi/Fonts/font12.cyclo ./Core/Src/LCDrv_f4_spi/Fonts/font12.d ./Core/Src/LCDrv_f4_spi/Fonts/font12.o ./Core/Src/LCDrv_f4_spi/Fonts/font12.su ./Core/Src/LCDrv_f4_spi/Fonts/font16.cyclo ./Core/Src/LCDrv_f4_spi/Fonts/font16.d ./Core/Src/LCDrv_f4_spi/Fonts/font16.o ./Core/Src/LCDrv_f4_spi/Fonts/font16.su ./Core/Src/LCDrv_f4_spi/Fonts/font20.cyclo ./Core/Src/LCDrv_f4_spi/Fonts/font20.d ./Core/Src/LCDrv_f4_spi/Fonts/font20.o ./Core/Src/LCDrv_f4_spi/Fonts/font20.su ./Core/Src/LCDrv_f4_spi/Fonts/font24.cyclo ./Core/Src/LCDrv_f4_spi/Fonts/font24.d ./Core/Src/LCDrv_f4_spi/Fonts/font24.o ./Core/Src/LCDrv_f4_spi/Fonts/font24.su ./Core/Src/LCDrv_f4_spi/Fonts/font8.cyclo ./Core/Src/LCDrv_f4_spi/Fonts/font8.d ./Core/Src/LCDrv_f4_spi/Fonts/font8.o ./Core/Src/LCDrv_f4_spi/Fonts/font8.su

.PHONY: clean-Core-2f-Src-2f-LCDrv_f4_spi-2f-Fonts

