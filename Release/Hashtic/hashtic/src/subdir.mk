################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Hashtic/hashtic/src/hash.c 

OBJS += \
./Hashtic/hashtic/src/hash.o 

C_DEPS += \
./Hashtic/hashtic/src/hash.d 


# Each subdirectory must supply rules for building sources it contributes
Hashtic/hashtic/src/%.o Hashtic/hashtic/src/%.su Hashtic/hashtic/src/%.cyclo: ../Hashtic/hashtic/src/%.c Hashtic/hashtic/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32L452xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/tenet/Downloads/distance/speed_encoder/Hashtic/hashtic/inc" -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Hashtic-2f-hashtic-2f-src

clean-Hashtic-2f-hashtic-2f-src:
	-$(RM) ./Hashtic/hashtic/src/hash.cyclo ./Hashtic/hashtic/src/hash.d ./Hashtic/hashtic/src/hash.o ./Hashtic/hashtic/src/hash.su

.PHONY: clean-Hashtic-2f-hashtic-2f-src

