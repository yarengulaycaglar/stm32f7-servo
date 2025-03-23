################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/CAN/Src/can.c 

OBJS += \
./Drivers/CAN/Src/can.o 

C_DEPS += \
./Drivers/CAN/Src/can.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/CAN/Src/%.o Drivers/CAN/Src/%.su Drivers/CAN/Src/%.cyclo: ../Drivers/CAN/Src/%.c Drivers/CAN/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-CAN-2f-Src

clean-Drivers-2f-CAN-2f-Src:
	-$(RM) ./Drivers/CAN/Src/can.cyclo ./Drivers/CAN/Src/can.d ./Drivers/CAN/Src/can.o ./Drivers/CAN/Src/can.su

.PHONY: clean-Drivers-2f-CAN-2f-Src

