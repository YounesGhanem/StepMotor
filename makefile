##########################################################################################################################

# ------------------------------------------------
# Generic Makefile (based on gcc)
#
# ChangeLog :
#	2017-02-10 - Several enhancements + project update mode
#   2015-07-22 - first version
# ------------------------------------------------

######################################
# target
######################################
TARGET = START


######################################
# building variables
######################################
# debug build?
DEBUG = 1
# optimization
OPT = -O0


#######################################
# paths
#######################################
# Build path
BUILD_DIR = build

######################################
# source
######################################
# C sources
C_SOURCES =  \
      Drivers/BSP/Components/L6470/L6470.c      \
      Drivers/BSP/STM32F4xx-Nucleo/stm32f4xx_nucleo.c \
      Drivers/BSP/X-NUCLEO-IHM02A1/xnucleoihm02a1.c\
      Drivers/BSP/X-NUCLEO-IHM02A1/NUCLEO-F401RE/xnucleoihm02a1_interface.c   \
      Drivers/CMSIS/Device/ST/STM32F4xx/Source/Templates/system_stm32f4xx.c   \
      Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c      \
      Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc.c  \
      Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc_ex.c     \
      Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c\
      Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c\
      Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c\
      Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c\
      Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c\
      Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c\
      Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c\
      Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c\
      Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c\
      Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c\
      Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c\
      Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_spi.c\
      Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c\
      Projects/STM32F401RE-Nucleo/Examples/MotionControl/MicrosteppingMotor/Src/example.c\
      Projects/STM32F401RE-Nucleo/Examples/MotionControl/MicrosteppingMotor/Src/example_usart.c\
      Projects/STM32F401RE-Nucleo/Examples/MotionControl/MicrosteppingMotor/Src/main.c\
      Projects/STM32F401RE-Nucleo/Examples/MotionControl/MicrosteppingMotor/Src/params.c\
      Projects/STM32F401RE-Nucleo/Examples/MotionControl/MicrosteppingMotor/Src/stm32f4xx_hal_msp.c\
      Projects/STM32F401RE-Nucleo/Examples/MotionControl/MicrosteppingMotor/Src/stm32f4xx_it.c
# ASM sources
ASM_SOURCES =  \
Drivers/CMSIS/Device/ST/STM32F4xx/Source/Templates/gcc/startup_stm32f401xe.s

# ASM sources
# ASM sources
ASMM_SOURCES = 


#######################################
# binaries
#######################################
PREFIX = arm-none-eabi-
# The gcc compiler bin path can be either defined in make command via GCC_PATH variable (> make GCC_PATH=xxx)
# either it can be added to the PATH environment variable.
ifdef GCC_PATH
CC = $(GCC_PATH)/$(PREFIX)gcc
AS = $(GCC_PATH)/$(PREFIX)gcc -x assembler-with-cpp
CP = $(GCC_PATH)/$(PREFIX)objcopy
SZ = $(GCC_PATH)/$(PREFIX)size
else
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size
endif
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S
 
######################################
# CFLAGS
#######################################
# cpu
CPU = -mcpu=cortex-m4

# fpu
FPU = -mfpu=fpv4-sp-d16

# float-abi
FLOAT-ABI = -mfloat-abi=hard

# mcu
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

# macros for gcc
# AS defines
AS_DEFS = 

# C defines
C_DEFS =  \
-DARM_MATH_CM4 \
-DUSE_HAL_DRIVER \
-DSTM32F401xE
# -DSTM32L476xx

# AS includes
AS_INCLUDES =  \
-IInc

# C includes
C_INCLUDES =  \
-IDrivers/BSP/Components      \
-IDrivers/BSP/Components/Common     \
-IDrivers/BSP/STM32F4xx-Nucleo\
-IDrivers/BSP/X-NUCLEO-IHM02A1\NUCLEO-F401RE\
-IDrivers/CMSIS\Device/ST/STM32F4xx\Source\Templates\
-IDrivers/STM32F4xx_HAL_Driver\Src\
-IProjects/STM32F401RE-Nucleo/Examples/MotionControl/MicrosteppingMotor\Src\
-IDrivers/BSP/X-NUCLEO-IHM02A1/NUCLEO-F401RE\
-IDrivers/CMSIS/Device/ST/STM32F4xx/Include\
-IDrivers/STM32F4xx_HAL_Driver/Inc\
-IProjects/STM32F401RE-Nucleo/Examples/MotionControl/MicrosteppingMotor/Inc\
-IDrivers/CMSIS/Include\
-IDrivers/CMSIS/DSP/Include\
-IDrivers/BSP/X-NUCLEO-IHM02A1\
-IDrivers/BSP/Components/L6470


# compile gcc flags
ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

CFLAGS += $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
endif


# Generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"


#######################################
# LDFLAGS
#######################################
# link script
LDSCRIPT = Projects\STM32F401RE-Nucleo\Examples\MotionControl\MicrosteppingMotor\SW4STM32\X-CUBE-SPN2_F401\STM32F401RETx_FLASH.ld

# libraries
LIBS = -lc -lm -lnosys
LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--gc-sections

# default action: build all
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin

#######################################
# build the application
#######################################
# list of objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASMM_SOURCES:.S=.o)))
vpath %.S $(sort $(dir $(ASMM_SOURCES)))

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR) 
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@
$(BUILD_DIR)/%.o: %.S Makefile | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@
	
$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@	

$(BUILD_DIR):
	mkdir $@		

#######################################
# clean up
#######################################
clean:
	-rm -fR $(BUILD_DIR)

#######################################
# dependencies
#######################################
-include $(wildcard $(BUILD_DIR)/*.d)

# *** EOF ***
