################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/Src/Stm32f407xx_gpio_driver.c 

OBJS += \
./drivers/Src/Stm32f407xx_gpio_driver.o 

C_DEPS += \
./drivers/Src/Stm32f407xx_gpio_driver.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/Src/Stm32f407xx_gpio_driver.o: ../drivers/Src/Stm32f407xx_gpio_driver.c drivers/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -c -I"/Users/enesayanoglu/Desktop/stm32_mcu_1/Stm32f4xx_drivers/drivers/Inc" -I../Inc -I"/Users/enesayanoglu/Desktop/stm32_mcu_1/Stm32f4xx_drivers/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"drivers/Src/Stm32f407xx_gpio_driver.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

