################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/adc.c \
../Src/comp.c \
../Src/dac.c \
../Src/gpio.c \
../Src/i2c.c \
../Src/main.c \
../Src/pid.c \
../Src/ssd1306.c \
../Src/ssd1306_fonts.c \
../Src/ssd1306_tests.c \
../Src/stm32g0xx_hal_msp.c \
../Src/stm32g0xx_it.c \
../Src/syscalls.c \
../Src/sysmem.c \
../Src/system_stm32g0xx.c \
../Src/tim.c 

OBJS += \
./Src/adc.o \
./Src/comp.o \
./Src/dac.o \
./Src/gpio.o \
./Src/i2c.o \
./Src/main.o \
./Src/pid.o \
./Src/ssd1306.o \
./Src/ssd1306_fonts.o \
./Src/ssd1306_tests.o \
./Src/stm32g0xx_hal_msp.o \
./Src/stm32g0xx_it.o \
./Src/syscalls.o \
./Src/sysmem.o \
./Src/system_stm32g0xx.o \
./Src/tim.o 

C_DEPS += \
./Src/adc.d \
./Src/comp.d \
./Src/dac.d \
./Src/gpio.d \
./Src/i2c.d \
./Src/main.d \
./Src/pid.d \
./Src/ssd1306.d \
./Src/ssd1306_fonts.d \
./Src/ssd1306_tests.d \
./Src/stm32g0xx_hal_msp.d \
./Src/stm32g0xx_it.d \
./Src/syscalls.d \
./Src/sysmem.d \
./Src/system_stm32g0xx.d \
./Src/tim.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o Src/%.su Src/%.cyclo: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G071xx -c -I../Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/STM32G0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/adc.cyclo ./Src/adc.d ./Src/adc.o ./Src/adc.su ./Src/comp.cyclo ./Src/comp.d ./Src/comp.o ./Src/comp.su ./Src/dac.cyclo ./Src/dac.d ./Src/dac.o ./Src/dac.su ./Src/gpio.cyclo ./Src/gpio.d ./Src/gpio.o ./Src/gpio.su ./Src/i2c.cyclo ./Src/i2c.d ./Src/i2c.o ./Src/i2c.su ./Src/main.cyclo ./Src/main.d ./Src/main.o ./Src/main.su ./Src/pid.cyclo ./Src/pid.d ./Src/pid.o ./Src/pid.su ./Src/ssd1306.cyclo ./Src/ssd1306.d ./Src/ssd1306.o ./Src/ssd1306.su ./Src/ssd1306_fonts.cyclo ./Src/ssd1306_fonts.d ./Src/ssd1306_fonts.o ./Src/ssd1306_fonts.su ./Src/ssd1306_tests.cyclo ./Src/ssd1306_tests.d ./Src/ssd1306_tests.o ./Src/ssd1306_tests.su ./Src/stm32g0xx_hal_msp.cyclo ./Src/stm32g0xx_hal_msp.d ./Src/stm32g0xx_hal_msp.o ./Src/stm32g0xx_hal_msp.su ./Src/stm32g0xx_it.cyclo ./Src/stm32g0xx_it.d ./Src/stm32g0xx_it.o ./Src/stm32g0xx_it.su ./Src/syscalls.cyclo ./Src/syscalls.d ./Src/syscalls.o ./Src/syscalls.su ./Src/sysmem.cyclo ./Src/sysmem.d ./Src/sysmem.o ./Src/sysmem.su ./Src/system_stm32g0xx.cyclo ./Src/system_stm32g0xx.d ./Src/system_stm32g0xx.o ./Src/system_stm32g0xx.su ./Src/tim.cyclo ./Src/tim.d ./Src/tim.o ./Src/tim.su

.PHONY: clean-Src

