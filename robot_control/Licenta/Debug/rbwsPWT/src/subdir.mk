################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../rbwsPWT/src/rbwsPWT.c 

C_DEPS += \
./rbwsPWT/src/rbwsPWT.d 

OBJS += \
./rbwsPWT/src/rbwsPWT.o 


# Each subdirectory must supply rules for building sources it contributes
rbwsPWT/src/%.o: ../rbwsPWT/src/%.c rbwsPWT/src/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -std=gnu99 -DSDK_DEBUGCONSOLE_UART=1 -DCPU_MKE16Z64VLF4 -DCPU_MKE16Z64VLF4_cm0plus -DSDK_OS_FREE_RTOS -DSERIAL_PORT_TYPE_UART=1 -DFREEDOM -DCR_INTEGER_PRINTF -DPRINTF_FLOAT_ENABLE=1 -D__MCUXPRESSO -D__USE_CMSIS -DDEBUG -D__NEWLIB__ -I"C:\Users\PLE1CLJ\Licenta\Code\Licenta - FINAL\Licenta\freertos\freertos-kernel\include" -I"C:\Users\PLE1CLJ\Licenta\Code\Licenta - FINAL\Licenta\freertos\freertos-kernel\portable\GCC\ARM_CM0" -I"C:\Users\PLE1CLJ\Licenta\Code\Licenta - FINAL\Licenta\drivers" -I"C:\Users\PLE1CLJ\Licenta\Code\Licenta - FINAL\Licenta\device" -I"C:\Users\PLE1CLJ\Licenta\Code\Licenta - FINAL\Licenta\CMSIS" -I"C:\Users\PLE1CLJ\Licenta\Code\Licenta - FINAL\Licenta\component\uart" -I"C:\Users\PLE1CLJ\Licenta\Code\Licenta - FINAL\Licenta\component\serial_manager" -I"C:\Users\PLE1CLJ\Licenta\Code\Licenta - FINAL\Licenta\utilities" -I"C:\Users\PLE1CLJ\Licenta\Code\Licenta - FINAL\Licenta\component\lists" -I"C:\Users\PLE1CLJ\Licenta\Code\Licenta - FINAL\Licenta\board" -I"C:\Users\PLE1CLJ\Licenta\Code\Licenta - FINAL\Licenta\source" -I"C:\Users\PLE1CLJ\Licenta\Code\Licenta - FINAL\Licenta\drivers\freertos" -I"C:\Users\PLE1CLJ\Licenta\Code\Licenta - FINAL\Licenta\rbwsPWT\inc" -I"C:\Users\PLE1CLJ\Licenta\Code\Licenta - FINAL\Licenta\rbwsKalmanFilter\inc" -I"C:\Users\PLE1CLJ\Licenta\Code\Licenta - FINAL\Licenta\rbwsFreeRTOS\inc" -I"C:\Users\PLE1CLJ\Licenta\Code\Licenta - FINAL\Licenta\rbwsInertialMeasurementUnit\inc" -I"C:\Users\PLE1CLJ\Licenta\Code\Licenta - FINAL\Licenta\rbwsLidarA1M8\inc" -I"C:\Users\PLE1CLJ\Licenta\Code\Licenta - FINAL\Licenta\rbwsMotorControl\inc" -Os -fno-common -g3 -Wall -c -ffunction-sections -fdata-sections -ffreestanding -fno-builtin -fmacro-prefix-map="$(<D)/"= -mcpu=cortex-m0plus -mthumb -D__NEWLIB__ -fstack-usage -specs=nano.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-rbwsPWT-2f-src

clean-rbwsPWT-2f-src:
	-$(RM) ./rbwsPWT/src/rbwsPWT.d ./rbwsPWT/src/rbwsPWT.o

.PHONY: clean-rbwsPWT-2f-src

