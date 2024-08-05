################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../rbwsLiDAR/src/rbwsLiDAR.c 

C_DEPS += \
./rbwsLiDAR/src/rbwsLiDAR.d 

OBJS += \
./rbwsLiDAR/src/rbwsLiDAR.o 


# Each subdirectory must supply rules for building sources it contributes
rbwsLiDAR/src/%.o: ../rbwsLiDAR/src/%.c rbwsLiDAR/src/subdir.mk
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -std=gnu99 -DCPU_MKE16Z64VLF4 -DCPU_MKE16Z64VLF4_cm0plus -DSDK_OS_FREE_RTOS -DSERIAL_PORT_TYPE_UART=1 -DSDK_DEBUGCONSOLE=0 -DFREEDOM -DCR_INTEGER_PRINTF -DPRINTF_FLOAT_ENABLE=1 -D__MCUXPRESSO -D__USE_CMSIS -DDEBUG -D__NEWLIB__ -I"C:\Users\PLE1CLJ\Licenta\Code\Licenta - FINAL\Licenta\freertos\freertos-kernel\include" -I"C:\Users\PLE1CLJ\Licenta\Code\Licenta - FINAL\Licenta\freertos\freertos-kernel\portable\GCC\ARM_CM0" -I"C:\Users\PLE1CLJ\Licenta\Code\Licenta - FINAL\Licenta\drivers" -I"C:\Users\PLE1CLJ\Licenta\Code\Licenta - FINAL\Licenta\device" -I"C:\Users\PLE1CLJ\Licenta\Code\Licenta - FINAL\Licenta\CMSIS" -I"C:\Users\PLE1CLJ\Licenta\Code\Licenta - FINAL\Licenta\component\uart" -I"C:\Users\PLE1CLJ\Licenta\Code\Licenta - FINAL\Licenta\component\serial_manager" -I"C:\Users\PLE1CLJ\Licenta\Code\Licenta - FINAL\Licenta\utilities" -I"C:\Users\PLE1CLJ\Licenta\Code\Licenta - FINAL\Licenta\component\lists" -I"C:\Users\PLE1CLJ\Licenta\Code\Licenta - FINAL\Licenta\board" -I"C:\Users\PLE1CLJ\Licenta\Code\Licenta - FINAL\Licenta\source" -I"C:\Users\PLE1CLJ\Licenta\Code\Licenta - FINAL\Licenta\drivers\freertos" -I"C:\Users\PLE1CLJ\Licenta\Code\Licenta - FINAL\Licenta\rbwsPWT\inc" -I"C:\Users\PLE1CLJ\Licenta\Code\Licenta - FINAL\Licenta\rbwsFreeRTOS\inc" -I"C:\Users\PLE1CLJ\Licenta\Code\Licenta - FINAL\Licenta\rbwsInertialMeasurementUnit\inc" -I"C:\Users\PLE1CLJ\Licenta\Code\Licenta - FINAL\Licenta\rbwsLiDAR\inc" -I"C:\Users\PLE1CLJ\Licenta\Code\Licenta - FINAL\Licenta\rbwsLidarA1M8\inc" -I"C:\Users\PLE1CLJ\Licenta\Code\Licenta - FINAL\Licenta\rbwsMotorControl\inc" -I"C:\Users\PLE1CLJ\Licenta\Code\Licenta - FINAL\Licenta\rbwsKalmanFilter\inc" -O0 -fno-common -g3 -Wall -c -ffunction-sections -fdata-sections -ffreestanding -fno-builtin -fmerge-constants -fmacro-prefix-map="$(<D)/"= -mcpu=cortex-m0plus -mthumb -D__NEWLIB__ -fstack-usage -specs=nano.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


clean: clean-rbwsLiDAR-2f-src

clean-rbwsLiDAR-2f-src:
	-$(RM) ./rbwsLiDAR/src/rbwsLiDAR.d ./rbwsLiDAR/src/rbwsLiDAR.o

.PHONY: clean-rbwsLiDAR-2f-src

