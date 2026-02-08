################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Core/Seeed_Arduino_SSCMA/src/Seeed_Arduino_SSCMA.cpp 

OBJS += \
./Core/Seeed_Arduino_SSCMA/src/Seeed_Arduino_SSCMA.o 

CPP_DEPS += \
./Core/Seeed_Arduino_SSCMA/src/Seeed_Arduino_SSCMA.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Seeed_Arduino_SSCMA/src/%.o Core/Seeed_Arduino_SSCMA/src/%.su Core/Seeed_Arduino_SSCMA/src/%.cyclo: ../Core/Seeed_Arduino_SSCMA/src/%.cpp Core/Seeed_Arduino_SSCMA/src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../Core/Inc -I../core/Seeed_Arduino_SSCMA/src -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I../STDIO -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Seeed_Arduino_SSCMA-2f-src

clean-Core-2f-Seeed_Arduino_SSCMA-2f-src:
	-$(RM) ./Core/Seeed_Arduino_SSCMA/src/Seeed_Arduino_SSCMA.cyclo ./Core/Seeed_Arduino_SSCMA/src/Seeed_Arduino_SSCMA.d ./Core/Seeed_Arduino_SSCMA/src/Seeed_Arduino_SSCMA.o ./Core/Seeed_Arduino_SSCMA/src/Seeed_Arduino_SSCMA.su

.PHONY: clean-Core-2f-Seeed_Arduino_SSCMA-2f-src

