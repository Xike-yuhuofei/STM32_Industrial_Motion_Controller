#!/bin/bash

# STM32F407 Industrial Motion Control Build Script
# 使用arm-none-eabi-gcc工具链编译项目

echo "========================================"
echo "  STM32F407 Industrial Motion Control"
echo "  Manual Build Script"
echo "========================================"

# 编译参数
MCU_FLAGS="-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard"
DEFINES="-DUSE_HAL_DRIVER -DSTM32F407xx"
INCLUDES="-ICore/Inc -IDrivers/STM32F4xx_HAL_Driver/Inc -IDrivers/STM32F4xx_HAL_Driver/Inc/Legacy -IDrivers/CMSIS/Device/ST/STM32F4xx/Include -IDrivers/CMSIS/Include"
CFLAGS="$MCU_FLAGS $DEFINES $INCLUDES -Og -Wall -fdata-sections -ffunction-sections"
LDFLAGS="$MCU_FLAGS -specs=nano.specs -TSTM32F407IGTX_FLASH.ld -lc -lm -lnosys -Wl,-Map=build/ATK_StepMotor_industrial.map,--cref -Wl,--gc-sections"

# 创建build目录
mkdir -p build

echo "[INFO] 编译核心源文件..."

# 编译核心源文件
CORE_SOURCES=(
    "Core/Src/main.c"
    "Core/Src/motion_control.c"
    "Core/Src/advanced_motion_control.c"
    "Core/Src/position_control.c"
    "Core/Src/position_control_demo.c"
    "Core/Src/communication.c"
    "Core/Src/stm32f4xx_it.c"
    "Core/Src/system_stm32f4xx.c"
    "Core/Src/syscalls.c"
    "Core/Src/sysmem.c"
)

# 工业运动控制模式源文件
INDUSTRIAL_SOURCES=(
    "Core/Src/industrial_motion_controller.c"
    "Core/Src/advanced_interpolation.c"
    "Core/Src/synchronous_control.c"
    "Core/Src/industrial_communication.c"
    "Core/Src/fault_diagnosis.c"
    "Core/Src/industrial_motion_demo.c"
    "Core/Src/test_advanced_motion.c"
)

# HAL库源文件
HAL_SOURCES=(
    "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c"
    "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c"
    "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c"
    "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c"
    "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c"
    "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c"
    "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c"
    "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c"
    "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c"
    "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c"
    "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c"
    "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c"
    "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_exti.c"
    "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c"
    "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c"
    "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_sram.c"
    "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_fsmc.c"
)

# 编译所有源文件
OBJECTS=""
compile_file() {
    local src_file=$1
    local obj_file="build/$(basename ${src_file%.c}.o)"
    
    echo "  编译: $src_file"
    arm-none-eabi-gcc -c $CFLAGS "$src_file" -o "$obj_file"
    
    if [ $? -eq 0 ]; then
        OBJECTS="$OBJECTS $obj_file"
    else
        echo "[ERROR] 编译失败: $src_file"
        exit 1
    fi
}

# 编译所有源文件
for src in "${CORE_SOURCES[@]}" "${INDUSTRIAL_SOURCES[@]}" "${HAL_SOURCES[@]}"; do
    if [ -f "$src" ]; then
        compile_file "$src"
    else
        echo "[WARNING] 文件不存在: $src"
    fi
done

echo "[INFO] 编译汇编启动文件..."
arm-none-eabi-gcc -c $CFLAGS "Core/Startup/startup_stm32f407igtx.s" -o "build/startup_stm32f407igtx.o"
OBJECTS="$OBJECTS build/startup_stm32f407igtx.o"

echo "[INFO] 链接生成ELF文件..."
arm-none-eabi-gcc $OBJECTS $LDFLAGS -o "build/ATK_StepMotor_industrial.elf"

if [ $? -eq 0 ]; then
    echo "[INFO] 生成二进制文件..."
    arm-none-eabi-objcopy -O ihex "build/ATK_StepMotor_industrial.elf" "build/ATK_StepMotor_industrial.hex"
    arm-none-eabi-objcopy -O binary -S "build/ATK_StepMotor_industrial.elf" "build/ATK_StepMotor_industrial.bin"
    
    echo "[INFO] 显示内存使用情况..."
    arm-none-eabi-size "build/ATK_StepMotor_industrial.elf"
    
    echo "[SUCCESS] 编译完成!"
    echo "  ELF: build/ATK_StepMotor_industrial.elf"
    echo "  HEX: build/ATK_StepMotor_industrial.hex"
    echo "  BIN: build/ATK_StepMotor_industrial.bin"
else
    echo "[ERROR] 链接失败!"
    exit 1
fi