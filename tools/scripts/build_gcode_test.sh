#!/bin/bash

# STM32F407 G代码解析器编译和测试脚本
# Author: STM32 Development Team
# Date: 2024-01-20

set -e  # 遇到错误立即退出

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 打印带颜色的信息
print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# 检查工具链
check_toolchain() {
    print_info "检查ARM工具链..."
    
    if ! command -v arm-none-eabi-gcc &> /dev/null; then
        print_error "ARM GCC工具链未找到！"
        print_info "请安装ARM GCC工具链："
        echo "  macOS: brew install arm-none-eabi-gcc"
        echo "  Ubuntu: sudo apt-get install gcc-arm-none-eabi"
        exit 1
    fi
    
    print_success "ARM GCC工具链已找到"
    arm-none-eabi-gcc --version | head -1
}

# 检查项目文件
check_project_files() {
    print_info "检查项目文件..."
    
    required_files=(
        "Makefile"
        "Core/Inc/gcode_parser.h"
        "Core/Src/gcode_parser.c"
        "Core/Inc/gcode_executor.h"
        "Core/Src/gcode_executor.c"
        "Core/Src/test_gcode_parser.c"
        "Core/Src/main_gcode_test.c"
    )
    
    for file in "${required_files[@]}"; do
        if [[ ! -f "$file" ]]; then
            print_error "缺少必要文件: $file"
            exit 1
        fi
    done
    
    print_success "所有必要文件检查完成"
}

# 清理构建目录
clean_build() {
    print_info "清理构建目录..."
    make clean
    print_success "构建目录已清理"
}

# 编译项目
build_project() {
    print_info "开始编译G代码解析器..."
    
    # 设置编译主文件为测试程序
    if [[ -f "Core/Src/main.c.backup" ]]; then
        print_info "发现备份文件，恢复原始main.c"
        mv Core/Src/main.c.backup Core/Src/main.c
    fi
    
    # 备份原始main.c并使用测试主程序
    if [[ -f "Core/Src/main.c" ]] && [[ ! -f "Core/Src/main.c.backup" ]]; then
        print_info "备份原始main.c文件"
        cp Core/Src/main.c Core/Src/main.c.backup
    fi
    
    # 使用测试主程序
    if [[ -f "Core/Src/main_gcode_test.c" ]]; then
        print_info "使用G代码测试主程序"
        cp Core/Src/main_gcode_test.c Core/Src/main.c
    fi
    
    # 开始编译
    make all
    
    if [[ $? -eq 0 ]]; then
        print_success "编译成功完成！"
        
        # 显示编译结果
        if [[ -f "build/ATK_StepMotor.elf" ]]; then
            print_info "编译产物信息："
            arm-none-eabi-size build/ATK_StepMotor.elf
            
            print_info "文件列表："
            ls -la build/ATK_StepMotor.*
        fi
    else
        print_error "编译失败！"
        exit 1
    fi
}

# 烧录程序
flash_program() {
    print_info "准备烧录程序到STM32F407..."
    
    if [[ ! -f "build/ATK_StepMotor.bin" ]]; then
        print_error "未找到编译文件，请先编译项目"
        exit 1
    fi
    
    # 检查烧录工具
    if command -v st-flash &> /dev/null; then
        print_info "使用st-flash烧录..."
        st-flash write build/ATK_StepMotor.bin 0x08000000
        print_success "烧录完成！"
    elif command -v STM32_Programmer_CLI &> /dev/null; then
        print_info "使用STM32CubeProgrammer烧录..."
        STM32_Programmer_CLI -c port=SWD -w build/ATK_StepMotor.bin 0x08000000 -v -rst
        print_success "烧录完成！"
    else
        print_warning "未找到烧录工具，跳过烧录步骤"
        print_info "可用的烧录选项："
        echo "  1. 安装st-link: brew install stlink"
        echo "  2. 安装STM32CubeProgrammer"
        echo "  3. 手动烧录 build/ATK_StepMotor.bin"
    fi
}

# 显示使用说明
show_usage() {
    echo "G代码解析器测试系统编译脚本"
    echo ""
    echo "用法: $0 [选项]"
    echo ""
    echo "选项:"
    echo "  build     - 编译项目"
    echo "  flash     - 烧录程序"
    echo "  test      - 编译并烧录"
    echo "  clean     - 清理构建文件"
    echo "  check     - 检查环境和文件"
    echo "  help      - 显示此帮助信息"
    echo ""
    echo "示例:"
    echo "  $0 test     # 完整测试（编译+烧录）"
    echo "  $0 build    # 仅编译"
    echo "  $0 flash    # 仅烧录"
}

# 显示测试指南
show_test_guide() {
    print_success "=== G代码解析器测试指南 ==="
    echo ""
    echo "1. 硬件连接："
    echo "   - STM32F407开发板连接到电脑"
    echo "   - 步进电机连接到ATK-2MD5050驱动器"
    echo "   - 驱动器信号线连接到开发板（PC9/PH2/PH3）"
    echo ""
    echo "2. 串口测试："
    echo "   - 打开串口调试助手（115200bps）"
    echo "   - 观察系统启动信息和测试结果"
    echo ""
    echo "3. G代码测试指令："
    echo "   G21          ; 设置公制单位"
    echo "   G90          ; 绝对坐标模式"
    echo "   G28          ; 回零"
    echo "   G01 X10 F500 ; 移动到X=10，速度500mm/min"
    echo "   G00 X0       ; 快速回到原点"
    echo "   M30          ; 程序结束"
    echo ""
    echo "4. 状态监控："
    echo "   - 观察步进电机运动"
    echo "   - 检查串口响应消息"
    echo "   - 验证坐标位置报告"
    echo ""
    print_info "详细文档请参考: README_GCode_Parser.md"
}

# 主程序
main() {
    echo "=========================================="
    echo "  STM32F407 G代码解析器测试系统"
    echo "  ATK-DMF407 + ATK-2MD5050步进电机"
    echo "=========================================="
    echo ""
    
    case "${1:-help}" in
        "build")
            check_toolchain
            check_project_files
            build_project
            show_test_guide
            ;;
        "flash")
            check_toolchain
            flash_program
            show_test_guide
            ;;
        "test")
            check_toolchain
            check_project_files
            clean_build
            build_project
            flash_program
            show_test_guide
            ;;
        "clean")
            clean_build
            ;;
        "check")
            check_toolchain
            check_project_files
            print_success "环境检查完成，一切正常！"
            ;;
        "help"|*)
            show_usage
            ;;
    esac
}

# 运行主程序
main "$@" 