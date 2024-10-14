################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/SEGGER/SEGGER/Syscalls/SEGGER_RTT_Syscalls_GCC.c \
../Middlewares/Third_Party/SEGGER/SEGGER/Syscalls/SEGGER_RTT_Syscalls_IAR.c \
../Middlewares/Third_Party/SEGGER/SEGGER/Syscalls/SEGGER_RTT_Syscalls_KEIL.c \
../Middlewares/Third_Party/SEGGER/SEGGER/Syscalls/SEGGER_RTT_Syscalls_SES.c 

OBJS += \
./Middlewares/Third_Party/SEGGER/SEGGER/Syscalls/SEGGER_RTT_Syscalls_GCC.o \
./Middlewares/Third_Party/SEGGER/SEGGER/Syscalls/SEGGER_RTT_Syscalls_IAR.o \
./Middlewares/Third_Party/SEGGER/SEGGER/Syscalls/SEGGER_RTT_Syscalls_KEIL.o \
./Middlewares/Third_Party/SEGGER/SEGGER/Syscalls/SEGGER_RTT_Syscalls_SES.o 

C_DEPS += \
./Middlewares/Third_Party/SEGGER/SEGGER/Syscalls/SEGGER_RTT_Syscalls_GCC.d \
./Middlewares/Third_Party/SEGGER/SEGGER/Syscalls/SEGGER_RTT_Syscalls_IAR.d \
./Middlewares/Third_Party/SEGGER/SEGGER/Syscalls/SEGGER_RTT_Syscalls_KEIL.d \
./Middlewares/Third_Party/SEGGER/SEGGER/Syscalls/SEGGER_RTT_Syscalls_SES.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/SEGGER/SEGGER/Syscalls/%.o Middlewares/Third_Party/SEGGER/SEGGER/Syscalls/%.su Middlewares/Third_Party/SEGGER/SEGGER/Syscalls/%.cyclo: ../Middlewares/Third_Party/SEGGER/SEGGER/Syscalls/%.c Middlewares/Third_Party/SEGGER/SEGGER/Syscalls/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I"C:/Users/Elfatek/STM32CubeIDE/workspace_1.13.2/freeRTOS_SEGGER_alma_DMU_CanBUS_v2/Middlewares/Third_Party/SEGGER/Config" -I"C:/Users/Elfatek/STM32CubeIDE/workspace_1.13.2/freeRTOS_SEGGER_alma_DMU_CanBUS_v2/Middlewares/Third_Party/SEGGER/OS" -I"C:/Users/Elfatek/STM32CubeIDE/workspace_1.13.2/freeRTOS_SEGGER_alma_DMU_CanBUS_v2/Middlewares/Third_Party/SEGGER/SEGGER" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-Third_Party-2f-SEGGER-2f-SEGGER-2f-Syscalls

clean-Middlewares-2f-Third_Party-2f-SEGGER-2f-SEGGER-2f-Syscalls:
	-$(RM) ./Middlewares/Third_Party/SEGGER/SEGGER/Syscalls/SEGGER_RTT_Syscalls_GCC.cyclo ./Middlewares/Third_Party/SEGGER/SEGGER/Syscalls/SEGGER_RTT_Syscalls_GCC.d ./Middlewares/Third_Party/SEGGER/SEGGER/Syscalls/SEGGER_RTT_Syscalls_GCC.o ./Middlewares/Third_Party/SEGGER/SEGGER/Syscalls/SEGGER_RTT_Syscalls_GCC.su ./Middlewares/Third_Party/SEGGER/SEGGER/Syscalls/SEGGER_RTT_Syscalls_IAR.cyclo ./Middlewares/Third_Party/SEGGER/SEGGER/Syscalls/SEGGER_RTT_Syscalls_IAR.d ./Middlewares/Third_Party/SEGGER/SEGGER/Syscalls/SEGGER_RTT_Syscalls_IAR.o ./Middlewares/Third_Party/SEGGER/SEGGER/Syscalls/SEGGER_RTT_Syscalls_IAR.su ./Middlewares/Third_Party/SEGGER/SEGGER/Syscalls/SEGGER_RTT_Syscalls_KEIL.cyclo ./Middlewares/Third_Party/SEGGER/SEGGER/Syscalls/SEGGER_RTT_Syscalls_KEIL.d ./Middlewares/Third_Party/SEGGER/SEGGER/Syscalls/SEGGER_RTT_Syscalls_KEIL.o ./Middlewares/Third_Party/SEGGER/SEGGER/Syscalls/SEGGER_RTT_Syscalls_KEIL.su ./Middlewares/Third_Party/SEGGER/SEGGER/Syscalls/SEGGER_RTT_Syscalls_SES.cyclo ./Middlewares/Third_Party/SEGGER/SEGGER/Syscalls/SEGGER_RTT_Syscalls_SES.d ./Middlewares/Third_Party/SEGGER/SEGGER/Syscalls/SEGGER_RTT_Syscalls_SES.o ./Middlewares/Third_Party/SEGGER/SEGGER/Syscalls/SEGGER_RTT_Syscalls_SES.su

.PHONY: clean-Middlewares-2f-Third_Party-2f-SEGGER-2f-SEGGER-2f-Syscalls

