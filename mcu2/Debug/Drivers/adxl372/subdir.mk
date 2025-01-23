################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/adxl372/adxl372.c 

OBJS += \
./Drivers/adxl372/adxl372.o 

C_DEPS += \
./Drivers/adxl372/adxl372.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/adxl372/%.o Drivers/adxl372/%.su Drivers/adxl372/%.cyclo: ../Drivers/adxl372/%.c Drivers/adxl372/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L496xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I"D:/Cube/workspace/adxl372_proj/mcu2/Drivers/adxl372" -I/Drivers/adxl372 -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-adxl372

clean-Drivers-2f-adxl372:
	-$(RM) ./Drivers/adxl372/adxl372.cyclo ./Drivers/adxl372/adxl372.d ./Drivers/adxl372/adxl372.o ./Drivers/adxl372/adxl372.su

.PHONY: clean-Drivers-2f-adxl372

