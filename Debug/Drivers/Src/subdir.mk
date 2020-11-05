################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Src/stm32f407xxgpio_drivers.c \
../Drivers/Src/stm32f407xxi2c_drivers.c \
../Drivers/Src/stm32f407xxrcc_drivers.c \
../Drivers/Src/stm32f407xxspi_drivers.c \
../Drivers/Src/stm32f407xxusart_drivers.c 

OBJS += \
./Drivers/Src/stm32f407xxgpio_drivers.o \
./Drivers/Src/stm32f407xxi2c_drivers.o \
./Drivers/Src/stm32f407xxrcc_drivers.o \
./Drivers/Src/stm32f407xxspi_drivers.o \
./Drivers/Src/stm32f407xxusart_drivers.o 

C_DEPS += \
./Drivers/Src/stm32f407xxgpio_drivers.d \
./Drivers/Src/stm32f407xxi2c_drivers.d \
./Drivers/Src/stm32f407xxrcc_drivers.d \
./Drivers/Src/stm32f407xxspi_drivers.d \
./Drivers/Src/stm32f407xxusart_drivers.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Src/stm32f407xxgpio_drivers.o: ../Drivers/Src/stm32f407xxgpio_drivers.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -DDEBUG -c -I"D:/Desktop/Dev/Windows_Dev/Embedded/STM32CubeIDE/MCU-1/stm32f407xx_drivers/Drivers/Inc" -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/Src/stm32f407xxgpio_drivers.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Drivers/Src/stm32f407xxi2c_drivers.o: ../Drivers/Src/stm32f407xxi2c_drivers.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -DDEBUG -c -I"D:/Desktop/Dev/Windows_Dev/Embedded/STM32CubeIDE/MCU-1/stm32f407xx_drivers/Drivers/Inc" -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/Src/stm32f407xxi2c_drivers.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Drivers/Src/stm32f407xxrcc_drivers.o: ../Drivers/Src/stm32f407xxrcc_drivers.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -DDEBUG -c -I"D:/Desktop/Dev/Windows_Dev/Embedded/STM32CubeIDE/MCU-1/stm32f407xx_drivers/Drivers/Inc" -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/Src/stm32f407xxrcc_drivers.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Drivers/Src/stm32f407xxspi_drivers.o: ../Drivers/Src/stm32f407xxspi_drivers.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -DDEBUG -c -I"D:/Desktop/Dev/Windows_Dev/Embedded/STM32CubeIDE/MCU-1/stm32f407xx_drivers/Drivers/Inc" -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/Src/stm32f407xxspi_drivers.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Drivers/Src/stm32f407xxusart_drivers.o: ../Drivers/Src/stm32f407xxusart_drivers.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F407G_DISC1 -DSTM32F4 -DSTM32F407VGTx -DDEBUG -c -I"D:/Desktop/Dev/Windows_Dev/Embedded/STM32CubeIDE/MCU-1/stm32f407xx_drivers/Drivers/Inc" -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/Src/stm32f407xxusart_drivers.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

