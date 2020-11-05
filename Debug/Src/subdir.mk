################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/302_USART_TxRx_testing.c 

OBJS += \
./Src/302_USART_TxRx_testing.o 

C_DEPS += \
./Src/302_USART_TxRx_testing.d 


# Each subdirectory must supply rules for building sources it contributes
Src/302_USART_TxRx_testing.o: ../Src/302_USART_TxRx_testing.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -DDEBUG -c -I"D:/Desktop/Dev/Windows_Dev/Embedded/STM32CubeIDE/MCU-1/stm32f407xx_drivers/Drivers/Inc" -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/302_USART_TxRx_testing.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

