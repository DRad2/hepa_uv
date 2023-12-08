################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Core/Startup/startup_stm32g491retx.s 

S_DEPS += \
./Core/Startup/startup_stm32g491retx.d 

OBJS += \
./Core/Startup/startup_stm32g491retx.o 


# Each subdirectory must supply rules for building sources it contributes
Core/Startup/%.o: ../Core/Startup/%.s Core/Startup/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -DDEBUG -c -I"G:/Shared drives/Opentrons/Hardware/1 - Hardware Engineering/1 - NPI Programs/12 - OT3/3 - Electrical/15 - HEPA_UV/Test Code/DC Ballast" -I"G:/Shared drives/Opentrons/Hardware/1 - Hardware Engineering/1 - NPI Programs/12 - OT3/3 - Electrical/15 - HEPA_UV/Test Code/DC Ballast/Drivers/STM32G4xx_HAL_Driver/Inc" -I"G:/Shared drives/Opentrons/Hardware/1 - Hardware Engineering/1 - NPI Programs/12 - OT3/3 - Electrical/15 - HEPA_UV/Test Code/DC Ballast/Drivers/STM32G4xx_HAL_Driver/Inc/Legacy" -I"G:/Shared drives/Opentrons/Hardware/1 - Hardware Engineering/1 - NPI Programs/12 - OT3/3 - Electrical/15 - HEPA_UV/Test Code/DC Ballast/Drivers/CMSIS/Device/ST/STM32G4xx/Include" -I"G:/Shared drives/Opentrons/Hardware/1 - Hardware Engineering/1 - NPI Programs/12 - OT3/3 - Electrical/15 - HEPA_UV/Test Code/DC Ballast/Drivers/CMSIS/Include" -I"G:/Shared drives/Opentrons/Hardware/1 - Hardware Engineering/1 - NPI Programs/12 - OT3/3 - Electrical/15 - HEPA_UV/Test Code/DC Ballast/Core/Inc" -I"G:/Shared drives/Opentrons/Hardware/1 - Hardware Engineering/1 - NPI Programs/12 - OT3/3 - Electrical/15 - HEPA_UV/Test Code/DC Ballast/Drivers/STM32G4xx_HAL_Driver/Src" -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

clean: clean-Core-2f-Startup

clean-Core-2f-Startup:
	-$(RM) ./Core/Startup/startup_stm32g491retx.d ./Core/Startup/startup_stm32g491retx.o

.PHONY: clean-Core-2f-Startup

