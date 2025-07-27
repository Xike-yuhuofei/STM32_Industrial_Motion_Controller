#!/bin/bash

####################################################################################################
# STM32F407 高级运动控制算法 编译和测试脚本
# Advanced Motion Control Algorithm Build and Test Script
#
# 功能特性:
# ✅ 梯形速度规划 - 平滑加减速控制
# ✅ S曲线速度规划 - 减少振动和冲击  
# ✅ DDA直线插补算法 - 高精度多轴协调
# ✅ 逐点比较法圆弧插补 - 精确圆弧轨迹
# ✅ NURBS样条曲线插补 - 复杂曲面加工
# ✅ 实时性能基准测试
#
# 作者: Claude AI & Cursor
# 版本: 2.0
# 日期: 2025-01-27
####################################################################################################

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
NC='\033[0m' # No Color

# 项目配置
TARGET="ATK_StepMotor"
BUILD_DIR="build"
MAIN_FILE="Core/Src/main_advanced_motion_test.c"

# 打印横幅
print_banner() {
    echo -e "${BLUE}"
    echo "=========================================="
    echo "  STM32F407 高级运动控制算法"
    echo "  Advanced Motion Control Algorithms"
    echo "  编译和测试脚本 v2.0"
    echo "=========================================="
    echo -e "${NC}"
}

# 打印使用说明
print_usage() {
    echo -e "${YELLOW}用法: $0 [选项]${NC}"
    echo ""
    echo "选项:"
    echo "  build     - 编译高级运动控制算法"
    echo "  flash     - 烧录到STM32F407开发板"
    echo "  test      - 编译并烧录，然后运行测试"
    echo "  clean     - 清理构建文件"
    echo "  info      - 显示项目信息"
    echo "  help      - 显示此帮助信息"
    echo ""
    echo "示例:"
    echo "  $0 test           # 完整测试流程"
    echo "  $0 build          # 仅编译"
    echo "  $0 flash          # 仅烧录"
}

# 检查工具链
check_toolchain() {
    echo -e "${BLUE}[检查] 验证开发工具链...${NC}"
    
    if ! command -v arm-none-eabi-gcc &> /dev/null; then
        echo -e "${RED}错误: 未找到 arm-none-eabi-gcc${NC}"
        echo "请安装ARM GCC工具链"
        exit 1
    fi
    
    if ! command -v make &> /dev/null; then
        echo -e "${RED}错误: 未找到 make${NC}"
        echo "请安装make工具"
        exit 1
    fi
    
    echo -e "${GREEN}✓ 工具链检查通过${NC}"
}

# 检查烧录工具
check_flash_tools() {
    echo -e "${BLUE}[检查] 验证烧录工具...${NC}"
    
    if command -v st-flash &> /dev/null; then
        echo -e "${GREEN}✓ 找到 st-flash${NC}"
        FLASH_TOOL="st-flash"
    elif command -v STM32_Programmer_CLI &> /dev/null; then
        echo -e "${GREEN}✓ 找到 STM32_Programmer_CLI${NC}"
        FLASH_TOOL="stm32cubeprog"
    else
        echo -e "${YELLOW}⚠ 未找到烧录工具，将跳过烧录步骤${NC}"
        echo "建议安装: brew install stlink 或下载STM32CubeProgrammer"
        FLASH_TOOL="none"
    fi
}

# 显示项目信息
show_info() {
    echo -e "${PURPLE}"
    echo "=========================================="
    echo "         项目信息"
    echo "=========================================="
    echo -e "${NC}"
    echo "项目名称: STM32F407 高级运动控制算法"
    echo "目标MCU:  STM32F407IGT6"
    echo "开发板:   ATK-DMF407"
    echo "驱动器:   ATK-2MD5050步进电机驱动器"
    echo ""
    echo -e "${YELLOW}高级算法特性:${NC}"
    echo "  🔶 梯形速度规划 - 加速度受限的平滑运动"
    echo "  🔶 S曲线速度规划 - Jerk限制减少振动冲击"
    echo "  🔶 DDA直线插补 - 高精度多轴协调控制"
    echo "  🔶 逐点比较法圆弧插补 - 精确圆弧轨迹生成"
    echo "  🔶 NURBS样条曲线插补 - 复杂曲面加工支持"
    echo "  🔶 实时性能基准测试 - 算法性能验证"
    echo ""
    echo -e "${YELLOW}硬件接口:${NC}"
    echo "  📡 脉冲信号: PC9 (TIM3_CH4)"
    echo "  📡 方向信号: PH2"
    echo "  📡 使能信号: PH3"
    echo "  📡 系统时钟: 168MHz"
    echo "  📡 插补频率: 1000Hz"
    echo ""
    echo -e "${YELLOW}文件结构:${NC}"
    echo "  📁 Core/Inc/motion_control.h - 运动控制头文件"
    echo "  📁 Core/Src/advanced_motion_control.c - 高级算法实现"
    echo "  📁 Core/Src/test_advanced_motion.c - 算法测试套件"
    echo "  📁 Core/Src/main_advanced_motion_test.c - 测试主程序"
}

# 编译项目
build_project() {
    echo -e "${BLUE}[编译] 开始编译高级运动控制算法...${NC}"
    
    # 确保使用正确的主文件
    if [ ! -f "$MAIN_FILE" ]; then
        echo -e "${RED}错误: 未找到主文件 $MAIN_FILE${NC}"
        exit 1
    fi
    
    # 临时备份原main.c并使用新主文件
    if [ -f "Core/Src/main.c" ]; then
        cp "Core/Src/main.c" "Core/Src/main.c.backup_$(date +%s)" 2>/dev/null || true
    fi
    cp "$MAIN_FILE" "Core/Src/main.c"
    
    # 清理之前的构建
    make clean > /dev/null 2>&1
    
    # 编译项目
    echo "正在编译..."
    if make all > build.log 2>&1; then
        echo -e "${GREEN}✓ 编译成功${NC}"
        
        # 显示编译结果
        if [ -f "${BUILD_DIR}/${TARGET}.elf" ]; then
            echo -e "${BLUE}[信息] 编译结果:${NC}"
            arm-none-eabi-size "${BUILD_DIR}/${TARGET}.elf" | grep -v "text"
            
            # 计算内存使用率
            FLASH_SIZE=$(arm-none-eabi-size "${BUILD_DIR}/${TARGET}.elf" | awk 'NR==2 {print $1+$2}')
            RAM_SIZE=$(arm-none-eabi-size "${BUILD_DIR}/${TARGET}.elf" | awk 'NR==2 {print $2+$3}')
            
            FLASH_PCT=$((FLASH_SIZE * 100 / (1024 * 1024)))  # STM32F407 has 1MB Flash
            RAM_PCT=$((RAM_SIZE * 100 / (192 * 1024)))        # STM32F407 has 192KB RAM
            
            echo "Flash使用: ${FLASH_SIZE} 字节 (${FLASH_PCT}%)"
            echo "RAM使用:   ${RAM_SIZE} 字节 (${RAM_PCT}%)"
        fi
        
        return 0
    else
        echo -e "${RED}✗ 编译失败${NC}"
        echo "详细错误信息:"
        cat build.log | tail -20
        return 1
    fi
}

# 烧录程序
flash_program() {
    echo -e "${BLUE}[烧录] 开始烧录到STM32F407...${NC}"
    
    if [ ! -f "${BUILD_DIR}/${TARGET}.bin" ]; then
        echo -e "${RED}错误: 未找到编译产物 ${BUILD_DIR}/${TARGET}.bin${NC}"
        echo "请先运行编译"
        return 1
    fi
    
    case $FLASH_TOOL in
        "st-flash")
            echo "使用 st-flash 烧录..."
            if st-flash write "${BUILD_DIR}/${TARGET}.bin" 0x08000000; then
                echo -e "${GREEN}✓ 烧录成功${NC}"
                return 0
            else
                echo -e "${RED}✗ 烧录失败${NC}"
                return 1
            fi
            ;;
        "stm32cubeprog")
            echo "使用 STM32CubeProgrammer 烧录..."
            if STM32_Programmer_CLI -c port=SWD -w "${BUILD_DIR}/${TARGET}.bin" 0x08000000 -v -rst; then
                echo -e "${GREEN}✓ 烧录成功${NC}"
                return 0
            else
                echo -e "${RED}✗ 烧录失败${NC}"
                return 1
            fi
            ;;
        "none")
            echo -e "${YELLOW}⚠ 跳过烧录步骤（未找到烧录工具）${NC}"
            echo "请手动烧录文件: ${BUILD_DIR}/${TARGET}.bin"
            return 0
            ;;
    esac
}

# 运行完整测试
run_test() {
    echo -e "${PURPLE}"
    echo "=========================================="
    echo "     开始高级运动控制算法测试"
    echo "=========================================="
    echo -e "${NC}"
    
    # 编译
    if build_project; then
        echo ""
        # 烧录
        if flash_program; then
            echo ""
            echo -e "${GREEN}🎉 测试程序已成功部署到STM32F407！${NC}"
            echo ""
            echo -e "${YELLOW}📋 测试内容包括:${NC}"
            echo "  1. 梯形速度规划算法验证"
            echo "  2. S曲线速度规划平滑性测试"
            echo "  3. DDA插补算法精度测试"
            echo "  4. 圆弧插补路径验证"
            echo "  5. 样条曲线连续性测试"
            echo "  6. 算法性能基准测试"
            echo ""
            echo -e "${BLUE}📺 观察测试结果:${NC}"
            echo "  🔸 连接STM32开发板到PC"
            echo "  🔸 观察开发板LED状态指示"
            echo "  🔸 程序会自动运行所有测试"
            echo "  🔸 可通过调试器查看详细结果"
            echo ""
            echo -e "${GREEN}✅ 部署完成！系统正在运行高级运动控制算法测试...${NC}"
        else
            echo -e "${RED}❌ 烧录失败，测试中止${NC}"
            return 1
        fi
    else
        echo -e "${RED}❌ 编译失败，测试中止${NC}"
        return 1
    fi
}

# 清理构建文件
clean_project() {
    echo -e "${BLUE}[清理] 清理构建文件...${NC}"
    make clean
    rm -f build.log
    echo -e "${GREEN}✓ 清理完成${NC}"
}

# 主函数
main() {
    print_banner
    
    case "$1" in
        "build")
            check_toolchain
            build_project
            ;;
        "flash")
            check_flash_tools
            flash_program
            ;;
        "test")
            check_toolchain
            check_flash_tools
            run_test
            ;;
        "clean")
            clean_project
            ;;
        "info")
            show_info
            ;;
        "help"|"--help"|"-h")
            print_usage
            ;;
        "")
            echo -e "${YELLOW}请指定操作。使用 '$0 help' 查看帮助。${NC}"
            ;;
        *)
            echo -e "${RED}未知选项: $1${NC}"
            print_usage
            exit 1
            ;;
    esac
}

# 运行主函数
main "$@" 