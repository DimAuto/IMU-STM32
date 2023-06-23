################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/Fusion/Fusion.c \
../Core/Src/Fusion/FusionAhrs.c \
../Core/Src/Fusion/FusionCompass.c \
../Core/Src/Fusion/FusionOffset.c 

OBJS += \
./Core/Src/Fusion/Fusion.o \
./Core/Src/Fusion/FusionAhrs.o \
./Core/Src/Fusion/FusionCompass.o \
./Core/Src/Fusion/FusionOffset.o 

C_DEPS += \
./Core/Src/Fusion/Fusion.d \
./Core/Src/Fusion/FusionAhrs.d \
./Core/Src/Fusion/FusionCompass.d \
./Core/Src/Fusion/FusionOffset.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/Fusion/%.o Core/Src/Fusion/%.su Core/Src/Fusion/%.cyclo: ../Core/Src/Fusion/%.c Core/Src/Fusion/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L4S5xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-Fusion

clean-Core-2f-Src-2f-Fusion:
	-$(RM) ./Core/Src/Fusion/Fusion.cyclo ./Core/Src/Fusion/Fusion.d ./Core/Src/Fusion/Fusion.o ./Core/Src/Fusion/Fusion.su ./Core/Src/Fusion/FusionAhrs.cyclo ./Core/Src/Fusion/FusionAhrs.d ./Core/Src/Fusion/FusionAhrs.o ./Core/Src/Fusion/FusionAhrs.su ./Core/Src/Fusion/FusionCompass.cyclo ./Core/Src/Fusion/FusionCompass.d ./Core/Src/Fusion/FusionCompass.o ./Core/Src/Fusion/FusionCompass.su ./Core/Src/Fusion/FusionOffset.cyclo ./Core/Src/Fusion/FusionOffset.d ./Core/Src/Fusion/FusionOffset.o ./Core/Src/Fusion/FusionOffset.su

.PHONY: clean-Core-2f-Src-2f-Fusion

