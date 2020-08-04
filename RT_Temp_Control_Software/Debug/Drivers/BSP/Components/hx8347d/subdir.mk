################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/Components/hx8347d/hx8347d.c \
../Drivers/BSP/Components/hx8347d/hx8347d_reg.c 

OBJS += \
./Drivers/BSP/Components/hx8347d/hx8347d.o \
./Drivers/BSP/Components/hx8347d/hx8347d_reg.o 

C_DEPS += \
./Drivers/BSP/Components/hx8347d/hx8347d.d \
./Drivers/BSP/Components/hx8347d/hx8347d_reg.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/Components/hx8347d/hx8347d.o: ../Drivers/BSP/Components/hx8347d/hx8347d.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32G431xx -DUSE_HAL_DRIVER -DDEBUG -c -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Include -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Core/Inc -I../Drivers/BSP/STM32G4xx_Nucleo -I../Drivers/STM32G4xx_HAL_Driver/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/BSP/Components/hx8347d/hx8347d.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Drivers/BSP/Components/hx8347d/hx8347d_reg.o: ../Drivers/BSP/Components/hx8347d/hx8347d_reg.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32G431xx -DUSE_HAL_DRIVER -DDEBUG -c -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Include -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Core/Inc -I../Drivers/BSP/STM32G4xx_Nucleo -I../Drivers/STM32G4xx_HAL_Driver/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/BSP/Components/hx8347d/hx8347d_reg.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

