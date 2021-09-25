################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/IIS2DLPC/iis2dlpc_reg.c 

OBJS += \
./Drivers/IIS2DLPC/iis2dlpc_reg.o 

C_DEPS += \
./Drivers/IIS2DLPC/iis2dlpc_reg.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/IIS2DLPC/%.o: ../Drivers/IIS2DLPC/%.c Drivers/IIS2DLPC/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L072xx -c -I../Core/Inc -I../Drivers/STM32L0xx_HAL_Driver/Inc -I../Drivers/STM32L0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L0xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/mzeml/embedded/bdm/Drivers/IIS2DLPC" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

