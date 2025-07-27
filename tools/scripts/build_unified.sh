#!/bin/bash

# STM32F407工业运动控制系统统一构建脚本
# 根据config/build.conf自动选择编译模式

echo "========================================"
echo "  STM32F407 工业运动控制系统构建工具"
echo "  Unified Build System v3.0"
echo "========================================"

# 检查配置文件
CONFIG_FILE="config/build.conf"
if [ ! -f "$CONFIG_FILE" ]; then
    echo "[ERROR] 配置文件 $CONFIG_FILE 不存在!"
    echo "正在创建默认配置..."
    mkdir -p config
    cat > "$CONFIG_FILE" << EOF
# 项目构建配置文件
BUILD_MODE=0
TEST_MODE=0
FEATURE_ADVANCED_MOTION=1
FEATURE_GCODE_PARSER=1
FEATURE_FAULT_DIAGNOSIS=1
FEATURE_COMMUNICATION=1
FEATURE_SERVO_DEMO=0
DEBUG_ENABLED=0
PERFORMANCE_MONITORING=0
EOF
    echo "✅ 已创建默认配置文件"
fi

# 读取配置
source "$CONFIG_FILE"

echo "[INFO] 当前配置:"
echo "  构建模式: $BUILD_MODE (0=生产, 1=开发, 2=调试)"
echo "  测试模式: $TEST_MODE"
echo "  高级运动控制: $FEATURE_ADVANCED_MOTION"
echo "  G-code解析: $FEATURE_GCODE_PARSER"
echo "  故障诊断: $FEATURE_FAULT_DIAGNOSIS"
echo "  通信模块: $FEATURE_COMMUNICATION"

# 设置编译参数
MCU_FLAGS="-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard"
BASE_DEFINES="-DUSE_HAL_DRIVER -DSTM32F407xx"
CONFIG_DEFINES="-DBUILD_MODE=$BUILD_MODE -DCURRENT_TEST_MODE=$TEST_MODE"

# 功能模块定义
if [ "$FEATURE_ADVANCED_MOTION" -eq 1 ]; then
    CONFIG_DEFINES="$CONFIG_DEFINES -DFEATURE_ADVANCED_MOTION=1"
else
    CONFIG_DEFINES="$CONFIG_DEFINES -DFEATURE_ADVANCED_MOTION=0"
fi

if [ "$FEATURE_GCODE_PARSER" -eq 1 ]; then
    CONFIG_DEFINES="$CONFIG_DEFINES -DFEATURE_GCODE_PARSER=1"
else
    CONFIG_DEFINES="$CONFIG_DEFINES -DFEATURE_GCODE_PARSER=0"
fi

if [ "$FEATURE_FAULT_DIAGNOSIS" -eq 1 ]; then
    CONFIG_DEFINES="$CONFIG_DEFINES -DFEATURE_FAULT_DIAGNOSIS=1"
else
    CONFIG_DEFINES="$CONFIG_DEFINES -DFEATURE_FAULT_DIAGNOSIS=0"
fi

if [ "$FEATURE_COMMUNICATION" -eq 1 ]; then
    CONFIG_DEFINES="$CONFIG_DEFINES -DFEATURE_COMMUNICATION=1"
else
    CONFIG_DEFINES="$CONFIG_DEFINES -DFEATURE_COMMUNICATION=0"
fi

DEFINES="$BASE_DEFINES $CONFIG_DEFINES"
INCLUDES="-Iconfig -ICore/Inc -IDrivers/STM32F4xx_HAL_Driver/Inc -IDrivers/STM32F4xx_HAL_Driver/Inc/Legacy -IDrivers/CMSIS/Device/ST/STM32F4xx/Include -IDrivers/CMSIS/Include"

# 根据构建模式设置优化级别
case $BUILD_MODE in
    0) # 生产模式
        OPT_LEVEL="-O2"
        BUILD_NAME="production"
        ;;
    1) # 开发模式
        OPT_LEVEL="-Og"
        BUILD_NAME="development"
        DEFINES="$DEFINES -DDEBUG=1"
        ;;
    2) # 调试模式
        OPT_LEVEL="-O0"
        BUILD_NAME="debug"
        DEFINES="$DEFINES -DDEBUG=1 -DDEBUG_LEVEL=4"
        ;;
    *)
        echo "[ERROR] 无效的构建模式: $BUILD_MODE"
        exit 1
        ;;
esac

CFLAGS="$MCU_FLAGS $DEFINES $INCLUDES $OPT_LEVEL -Wall -fdata-sections -ffunction-sections"
LDFLAGS="$MCU_FLAGS -specs=nano.specs -TSTM32F407IGTX_FLASH.ld -lc -lm -lnosys -Wl,-Map=build/ATK_StepMotor_${BUILD_NAME}.map,--cref -Wl,--gc-sections"

# 创建构建目录
mkdir -p build

echo "[INFO] 开始编译 ($BUILD_NAME 模式)..."

# 核心源文件
CORE_SOURCES=(
    "Core/Src/main.c"
    "Core/Src/stm32f4xx_it.c"
    "Core/Src/system_stm32f4xx.c"
    "Core/Src/syscalls.c"
    "Core/Src/sysmem.c"
    "Core/Src/error_handler.c"
)

# 功能模块源文件
FEATURE_SOURCES=()

if [ "$FEATURE_ADVANCED_MOTION" -eq 1 ]; then
    FEATURE_SOURCES+=(
        "Core/Src/motion_control.c"
        "Core/Src/advanced_motion_control.c"
        "Core/Src/position_control.c"
        "Core/Src/advanced_interpolation.c"
        "Core/Src/test_advanced_motion.c"
    )
fi

if [ "$FEATURE_GCODE_PARSER" -eq 1 ]; then
    FEATURE_SOURCES+=(
        "Core/Src/gcode_parser.c"
        "Core/Src/gcode_executor.c"
    )
fi

if [ "$FEATURE_FAULT_DIAGNOSIS" -eq 1 ]; then
    FEATURE_SOURCES+=(
        "Core/Src/fault_diagnosis.c"
    )
fi

if [ "$FEATURE_COMMUNICATION" -eq 1 ]; then
    FEATURE_SOURCES+=(
        "Core/Src/communication.c"
        "Core/Src/industrial_communication.c"
    )
fi

# 工业模式额外源文件
if [ "$BUILD_MODE" -ne 2 ]; then
    FEATURE_SOURCES+=(
        "Core/Src/industrial_motion_controller.c"
        "Core/Src/synchronous_control.c"
        "Core/Src/industrial_motion_demo.c"
    )
fi

# HAL库源文件
HAL_SOURCES=(
    "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c"
    "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c"
    "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c"
    "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c"
    "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c"
    "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c"
    "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c"
    "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c"
    "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_exti.c"
    "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c"
    "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c"
)

# 编译函数
OBJECTS=""
compile_file() {
    local src_file=$1
    local obj_file="build/$(basename ${src_file%.c}.o)"
    
    if [ -f "$src_file" ]; then
        echo "  编译: $src_file"
        arm-none-eabi-gcc -c $CFLAGS "$src_file" -o "$obj_file" 2>&1
        
        if [ $? -eq 0 ]; then
            OBJECTS="$OBJECTS $obj_file"
            return 0
        else
            echo "[ERROR] 编译失败: $src_file"
            return 1
        fi
    else
        echo "[WARNING] 文件不存在: $src_file"
        return 0
    fi
}

# 编译所有源文件
SUCCESS=true
echo "[INFO] 编译核心文件..."
for src in "${CORE_SOURCES[@]}"; do
    if ! compile_file "$src"; then
        SUCCESS=false
        break
    fi
done

if [ "$SUCCESS" = true ]; then
    echo "[INFO] 编译功能模块..."
    for src in "${FEATURE_SOURCES[@]}"; do
        if ! compile_file "$src"; then
            SUCCESS=false
            break
        fi
    done
fi

if [ "$SUCCESS" = true ]; then
    echo "[INFO] 编译HAL库..."
    for src in "${HAL_SOURCES[@]}"; do
        if ! compile_file "$src"; then
            SUCCESS=false
            break
        fi
    done
fi

# 编译启动文件
if [ "$SUCCESS" = true ]; then
    echo "[INFO] 编译启动文件..."
    arm-none-eabi-gcc -c $CFLAGS "Core/Startup/startup_stm32f407igtx.s" -o "build/startup_stm32f407igtx.o"
    OBJECTS="$OBJECTS build/startup_stm32f407igtx.o"
fi

# 链接
if [ "$SUCCESS" = true ]; then
    echo "[INFO] 链接生成ELF文件..."
    arm-none-eabi-gcc $OBJECTS $LDFLAGS -o "build/ATK_StepMotor_${BUILD_NAME}.elf" 2>&1
    
    if [ $? -eq 0 ]; then
        echo "[INFO] 生成二进制文件..."
        arm-none-eabi-objcopy -O ihex "build/ATK_StepMotor_${BUILD_NAME}.elf" "build/ATK_StepMotor_${BUILD_NAME}.hex"
        arm-none-eabi-objcopy -O binary -S "build/ATK_StepMotor_${BUILD_NAME}.elf" "build/ATK_StepMotor_${BUILD_NAME}.bin"
        
        echo "[INFO] 显示内存使用情况..."
        arm-none-eabi-size "build/ATK_StepMotor_${BUILD_NAME}.elf"
        
        echo "[SUCCESS] 编译完成!"
        echo "  ELF: build/ATK_StepMotor_${BUILD_NAME}.elf"
        echo "  HEX: build/ATK_StepMotor_${BUILD_NAME}.hex"
        echo "  BIN: build/ATK_StepMotor_${BUILD_NAME}.bin"
        
        # 显示文件大小
        ls -lh build/ATK_StepMotor_${BUILD_NAME}.*
        
        # 生成构建信息文件
        cat > "build/build_info_${BUILD_NAME}.txt" << EOF
构建信息
========
构建时间: $(date)
构建模式: $BUILD_NAME ($BUILD_MODE)
测试模式: $TEST_MODE
功能模块:
  - 高级运动控制: $FEATURE_ADVANCED_MOTION
  - G-code解析器: $FEATURE_GCODE_PARSER  
  - 故障诊断: $FEATURE_FAULT_DIAGNOSIS
  - 通信模块: $FEATURE_COMMUNICATION
编译选项: $CFLAGS
链接选项: $LDFLAGS
EOF
        echo "✅ 构建信息已保存到 build/build_info_${BUILD_NAME}.txt"
        
    else
        echo "[ERROR] 链接失败!"
        exit 1
    fi
else
    echo "[ERROR] 编译失败!"
    exit 1
fi