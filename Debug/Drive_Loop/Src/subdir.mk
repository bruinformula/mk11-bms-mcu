################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drive_Loop/Src/currLimiting.c \
../Drive_Loop/Src/datalogging.c \
../Drive_Loop/Src/gui_test.c \
../Drive_Loop/Src/prchg.c \
../Drive_Loop/Src/safety_handler.c 

OBJS += \
./Drive_Loop/Src/currLimiting.o \
./Drive_Loop/Src/datalogging.o \
./Drive_Loop/Src/gui_test.o \
./Drive_Loop/Src/prchg.o \
./Drive_Loop/Src/safety_handler.o 

C_DEPS += \
./Drive_Loop/Src/currLimiting.d \
./Drive_Loop/Src/datalogging.d \
./Drive_Loop/Src/gui_test.d \
./Drive_Loop/Src/prchg.d \
./Drive_Loop/Src/safety_handler.d 


# Each subdirectory must supply rules for building sources it contributes
Drive_Loop/Src/%.o Drive_Loop/Src/%.su Drive_Loop/Src/%.cyclo: ../Drive_Loop/Src/%.c Drive_Loop/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G474xx -c -I"C:/Users/colin/STM32CubeIDE/workspace_1.19.0/mk11-bms-mcu/Charging/Inc" -I"C:/Users/colin/STM32CubeIDE/workspace_1.19.0/mk11-bms-mcu/Core/Inc" -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/colin/STM32CubeIDE/workspace_1.19.0/mk11-bms-mcu/ADBMS6830/program/inc" -I"C:/Users/colin/STM32CubeIDE/workspace_1.19.0/mk11-bms-mcu/ADBMS6830/lib/inc" -I../Core/Inc -I"C:/Users/colin/STM32CubeIDE/workspace_1.19.0/mk11-bms-mcu/Calculations/Inc" -I"C:/Users/colin/STM32CubeIDE/workspace_1.19.0/mk11-bms-mcu/Drive_Loop/Inc" -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -Werror -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drive_Loop-2f-Src

clean-Drive_Loop-2f-Src:
	-$(RM) ./Drive_Loop/Src/currLimiting.cyclo ./Drive_Loop/Src/currLimiting.d ./Drive_Loop/Src/currLimiting.o ./Drive_Loop/Src/currLimiting.su ./Drive_Loop/Src/datalogging.cyclo ./Drive_Loop/Src/datalogging.d ./Drive_Loop/Src/datalogging.o ./Drive_Loop/Src/datalogging.su ./Drive_Loop/Src/gui_test.cyclo ./Drive_Loop/Src/gui_test.d ./Drive_Loop/Src/gui_test.o ./Drive_Loop/Src/gui_test.su ./Drive_Loop/Src/prchg.cyclo ./Drive_Loop/Src/prchg.d ./Drive_Loop/Src/prchg.o ./Drive_Loop/Src/prchg.su ./Drive_Loop/Src/safety_handler.cyclo ./Drive_Loop/Src/safety_handler.d ./Drive_Loop/Src/safety_handler.o ./Drive_Loop/Src/safety_handler.su

.PHONY: clean-Drive_Loop-2f-Src

