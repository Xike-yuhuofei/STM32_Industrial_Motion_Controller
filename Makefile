#####################################
# Makefile for STM32F407 Industrial Motion Controller
#####################################

# Target
TARGET = STM32_Industrial_Motion_Controller

# Build directory
BUILD_DIR = build

# Compilation mode selection
MODE ?= industrial

# Mode-specific sources and target suffix  
ifeq ($(MODE), industrial)
    TARGET_SUFFIX = _industrial
    MODE_C_SOURCES = \
        firmware/src/core/industrial_motion_controller.c \
        firmware/src/app/motion/advanced_interpolation.c \
        firmware/src/core/synchronous_control.c \
        firmware/src/app/communication/industrial_communication.c \
        firmware/src/app/diagnostics/fault_diagnosis.c \
        firmware/src/demos/industrial_motion_demo.c
    $(info Building Industrial Motion Control Mode)
else ifeq ($(MODE), gcode)
    TARGET_SUFFIX = _gcode
    MODE_C_SOURCES = \
        firmware/src/app/gcode/gcode_parser.c \
        firmware/src/app/gcode/gcode_executor.c \
        tests/unit/test_gcode_parser.c
    $(info Building G-Code Parser Test Mode)
else ifeq ($(MODE), advanced)
    TARGET_SUFFIX = _advanced
    MODE_C_SOURCES = \
        firmware/src/app/motion/advanced_motion_control.c \
        tests/unit/test_advanced_motion.c
    $(info Building Advanced Motion Control Test Mode)
else ifeq ($(MODE), basic)
    TARGET_SUFFIX = _basic
    MODE_C_SOURCES = \
        firmware/src/app/motion/motion_control.c
    $(info Building Basic Mode)
else ifeq ($(MODE), simple)
    TARGET_SUFFIX = _simple
    MODE_C_SOURCES = \
        firmware/src/app/motion/motion_control.c
    $(info Building Simple Test Mode)
else
    $(error Invalid MODE: $(MODE). Valid options: industrial, gcode, advanced, basic, simple)
endif

# Update target name with suffix
TARGET_FULL = $(TARGET)$(TARGET_SUFFIX)

# Toolchain
PREFIX = arm-none-eabi-
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size

# MCU
CPU = -mcpu=cortex-m4
FPU = -mfpu=fpv4-sp-d16
FLOAT-ABI = -mfloat-abi=hard
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

# Macros for gcc
AS_DEFS = 
C_DEFS = -DUSE_HAL_DRIVER -DSTM32F407xx

# AS includes
AS_INCLUDES = 

# C includes
C_INCLUDES = \
-Ifirmware/include \
-Ifirmware/src/platform \
-Ifirmware/drivers/STM32F4xx_HAL_Driver/Inc \
-Ifirmware/drivers/STM32F4xx_HAL_Driver/Inc/Legacy \
-Ifirmware/drivers/CMSIS/Device/ST/STM32F4xx/Include \
-Ifirmware/drivers/CMSIS/Include

# compile gcc flags
ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections
CFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections

ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
endif

# Generate dependency information
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"

# optimization
OPT = -Og

# libraries
LIBS = -lc -lm -lnosys 
LIBDIR = 
LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(BUILD_DIR)/$(TARGET_FULL).map,--cref -Wl,--gc-sections

# Common C sources (platform and HAL)
COMMON_C_SOURCES = \
firmware/src/platform/main.c \
firmware/src/platform/stm32f4xx_hal_msp.c \
firmware/src/platform/stm32f4xx_it.c \
firmware/src/platform/system_stm32f4xx.c \
firmware/src/platform/syscalls.c \
firmware/src/platform/sysmem.c \
firmware/src/app/motion/motion_control.c \
firmware/src/app/motion/position_control.c \
firmware/src/demos/position_control_demo.c \
firmware/src/app/communication/communication.c \
firmware/drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c \
firmware/drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c \
firmware/drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c \
firmware/drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c \
firmware/drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c \
firmware/drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c \
firmware/drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c \
firmware/drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c \
firmware/drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c \
firmware/drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c \
firmware/drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c \
firmware/drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c \
firmware/drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_exti.c \
firmware/drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c \
firmware/drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c \
firmware/drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_sram.c \
firmware/drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_fsmc.c

# Complete C sources
C_SOURCES = $(COMMON_C_SOURCES) $(MODE_C_SOURCES)

# ASM sources
ASM_SOURCES = firmware/startup/startup_stm32f407igtx.s

# linker script
LDSCRIPT = firmware/linker/STM32F407IGTX_FLASH.ld

# objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))

# ASM objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

# default action: build all
all: $(BUILD_DIR)/$(TARGET_FULL).elf $(BUILD_DIR)/$(TARGET_FULL).hex $(BUILD_DIR)/$(TARGET_FULL).bin

# Build rules
$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET_FULL).elf: $(OBJECTS) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(CP) -O ihex $< $@
	
$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(CP) -O binary -S $< $@

$(BUILD_DIR):
	mkdir $@

# clean up
clean:
	-rm -fR $(BUILD_DIR)

# program - use the mode-specific binary
flash: $(BUILD_DIR)/$(TARGET_FULL).bin
	@echo "Flashing $(BUILD_DIR)/$(TARGET_FULL).bin to STM32F407..."
	@if command -v st-flash >/dev/null 2>&1; then \
		st-flash write $< 0x08000000; \
	elif command -v STM32_Programmer_CLI >/dev/null 2>&1; then \
		STM32_Programmer_CLI -c port=SWD -w $< 0x08000000 -v -rst; \
	else \
		echo "Error: No flashing tool found. Please install st-link tools or STM32CubeProgrammer."; \
		echo "Available options:"; \
		echo "  1. Install st-link: brew install stlink"; \
		echo "  2. Install STM32CubeProgrammer from ST website"; \
	fi

# Show available modes
modes:
	@echo "Available compilation modes:"
	@echo "  industrial - Industrial motion control (default)"
	@echo "  gcode      - G-Code Parser Test"
	@echo "  advanced   - Advanced Motion Control Test"
	@echo "  basic      - Basic stepper motor control"
	@echo "  simple     - Simple test program"
	@echo ""
	@echo "Usage examples:"
	@echo "  make                    # Build industrial motion control (default)"
	@echo "  make MODE=gcode         # Build G-code parser test"
	@echo "  make MODE=advanced      # Build advanced motion control test"
	@echo "  make flash MODE=gcode   # Build and flash G-code test"

# Test target
test: 
	@echo "Running tests..."
	@python3 tools/scripts/build.py -m test -t industrial

# dependencies
-include $(wildcard $(BUILD_DIR)/*.d)

.PHONY: all clean flash modes test