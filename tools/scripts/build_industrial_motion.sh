#!/bin/bash

##############################################################################
# 工业运动控制卡构建脚本
# Build Script for Industrial Motion Control Card
# 版本: 2.0
# 日期: 2025-01-27
##############################################################################

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# 脚本配置
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_NAME="工业运动控制卡"
VERSION="2.0.0"
BUILD_DATE=$(date '+%Y-%m-%d %H:%M:%S')

# 构建配置
DEFAULT_MODE="industrial"
AVAILABLE_MODES=("industrial" "gcode" "advanced" "basic" "ui" "simple")
CLEAN_BUILD=false
FLASH_AFTER_BUILD=false
VERBOSE=false

##############################################################################
# 函数定义
##############################################################################

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

print_header() {
    echo -e "${PURPLE}$1${NC}"
}

# 显示帮助信息
show_help() {
    cat << EOF
${PROJECT_NAME} 构建脚本 v${VERSION}

用法: $0 [选项] [模式]

选项:
    -h, --help          显示此帮助信息
    -c, --clean         清理构建目录后重新构建
    -f, --flash         构建完成后自动烧录
    -v, --verbose       显示详细构建信息
    -l, --list          列出可用的构建模式

构建模式:
    industrial          工业运动控制卡模式 (默认)
    gcode              G代码解析测试模式
    advanced           高级运动控制测试模式
    basic              基础模式
    ui                 用户界面测试模式
    simple             简单测试模式

示例:
    $0                  # 使用默认模式构建
    $0 industrial       # 构建工业运动控制卡模式
    $0 -c -f advanced   # 清理构建并烧录高级模式
    $0 --clean gcode    # 清理构建G代码模式

EOF
}

# 列出可用模式
list_modes() {
    print_header "可用的构建模式:"
    for mode in "${AVAILABLE_MODES[@]}"; do
        if [ "$mode" = "$DEFAULT_MODE" ]; then
            echo -e "  ${GREEN}$mode${NC} (默认)"
        else
            echo -e "  $mode"
        fi
    done
}

# 检查模式是否有效
validate_mode() {
    local mode=$1
    for valid_mode in "${AVAILABLE_MODES[@]}"; do
        if [ "$mode" = "$valid_mode" ]; then
            return 0
        fi
    done
    return 1
}

# 检查构建环境
check_environment() {
    print_info "检查构建环境..."
    
    # 检查工具链
    if ! command -v arm-none-eabi-gcc >/dev/null 2>&1; then
        print_error "ARM工具链未找到，请安装 arm-none-eabi-gcc"
        print_info "在macOS上可以使用: brew install --cask gcc-arm-embedded"
        exit 1
    fi
    
    # 检查make
    if ! command -v make >/dev/null 2>&1; then
        print_error "make工具未找到"
        exit 1
    fi
    
    # 显示工具链版本
    local gcc_version=$(arm-none-eabi-gcc --version | head -n1)
    print_success "ARM GCC: $gcc_version"
    
    # 检查项目文件
    if [ ! -f "Makefile" ]; then
        print_error "Makefile不存在"
        exit 1
    fi
    
    print_success "构建环境检查完成"
}

# 显示项目信息
show_project_info() {
    print_header "==============================================="
    print_header "        ${PROJECT_NAME} 构建系统"
    print_header "==============================================="
    echo -e "${CYAN}项目版本:${NC} $VERSION"
    echo -e "${CYAN}构建时间:${NC} $BUILD_DATE"
    echo -e "${CYAN}构建模式:${NC} $BUILD_MODE"
    echo -e "${CYAN}项目目录:${NC} $SCRIPT_DIR"
    print_header "==============================================="
}

# 显示模式特性
show_mode_features() {
    local mode=$1
    print_info "模式特性说明:"
    
    case $mode in
        "industrial")
            echo "  • FreeRTOS实时操作系统"
            echo "  • 高级插补算法 (直线、圆弧、螺旋、NURBS)"
            echo "  • 同步控制算法 (电子齿轮、电子凸轮、交叉耦合)"
            echo "  • 工业通讯协议 (EtherCAT、Modbus、CAN)"
            echo "  • 故障诊断系统 (振动分析、预测性维护)"
            echo "  • 位置控制算法 (PID、前馈、自适应控制)"
            echo "  • G代码解析和执行"
            ;;
        "gcode")
            echo "  • G代码解析器"
            echo "  • G代码执行引擎"
            echo "  • 测试用例"
            ;;
        "advanced")
            echo "  • 高级运动控制算法"
            echo "  • 位置控制系统"
            echo "  • 测试程序"
            ;;
        "basic")
            echo "  • 基础系统功能"
            echo "  • 最小化配置"
            ;;
        "ui")
            echo "  • LCD显示驱动"
            echo "  • 触摸屏驱动"
            echo "  • 用户界面"
            ;;
        "simple")
            echo "  • 简单测试程序"
            echo "  • 快速验证"
            ;;
    esac
}

# 清理构建
clean_build() {
    print_info "清理构建目录..."
    if make clean; then
        print_success "构建目录清理完成"
    else
        print_error "清理构建目录失败"
        exit 1
    fi
}

# 执行构建
perform_build() {
    local mode=$1
    
    print_info "开始构建模式: $mode"
    
    # 构建命令
    local build_cmd="make MODE=$mode"
    if [ "$VERBOSE" = true ]; then
        build_cmd="$build_cmd V=1"
    fi
    
    print_info "执行命令: $build_cmd"
    
    # 记录构建开始时间
    local start_time=$(date +%s)
    
    # 执行构建
    if [ "$VERBOSE" = true ]; then
        $build_cmd
    else
        $build_cmd 2>&1 | while read line; do
            # 过滤并显示重要信息
            if [[ $line == *"Building"* ]] || [[ $line == *"Compiling"* ]] || [[ $line == *"Linking"* ]] || [[ $line == *"error:"* ]] || [[ $line == *"Error"* ]]; then
                echo "$line"
            fi
        done
    fi
    
    local build_result=${PIPESTATUS[0]}
    local end_time=$(date +%s)
    local build_duration=$((end_time - start_time))
    
    if [ $build_result -eq 0 ]; then
        print_success "构建完成! 用时: ${build_duration}秒"
        
        # 显示构建结果信息
        show_build_results "$mode"
    else
        print_error "构建失败!"
        exit 1
    fi
}

# 显示构建结果
show_build_results() {
    local mode=$1
    local build_dir="build"
    local target_name="ATK_StepMotor_${mode}"
    
    print_header "构建结果:"
    
    if [ -f "${build_dir}/${target_name}.elf" ]; then
        echo -e "${CYAN}ELF文件:${NC} ${build_dir}/${target_name}.elf"
        
        # 显示内存使用情况
        if command -v arm-none-eabi-size >/dev/null 2>&1; then
            print_info "内存使用情况:"
            arm-none-eabi-size "${build_dir}/${target_name}.elf"
        fi
    fi
    
    if [ -f "${build_dir}/${target_name}.hex" ]; then
        echo -e "${CYAN}HEX文件:${NC} ${build_dir}/${target_name}.hex"
    fi
    
    if [ -f "${build_dir}/${target_name}.bin" ]; then
        echo -e "${CYAN}BIN文件:${NC} ${build_dir}/${target_name}.bin"
        
        # 显示文件大小
        local file_size=$(ls -lh "${build_dir}/${target_name}.bin" | awk '{print $5}')
        echo -e "${CYAN}文件大小:${NC} $file_size"
    fi
}

# 烧录固件
flash_firmware() {
    local mode=$1
    local bin_file="build/ATK_StepMotor_${mode}.bin"
    
    if [ ! -f "$bin_file" ]; then
        print_error "二进制文件不存在: $bin_file"
        exit 1
    fi
    
    print_info "开始烧录固件: $bin_file"
    
    if make flash MODE="$mode"; then
        print_success "固件烧录完成!"
    else
        print_error "固件烧录失败!"
        exit 1
    fi
}

##############################################################################
# 主程序
##############################################################################

# 解析命令行参数
BUILD_MODE="$DEFAULT_MODE"

while [[ $# -gt 0 ]]; do
    case $1 in
        -h|--help)
            show_help
            exit 0
            ;;
        -c|--clean)
            CLEAN_BUILD=true
            shift
            ;;
        -f|--flash)
            FLASH_AFTER_BUILD=true
            shift
            ;;
        -v|--verbose)
            VERBOSE=true
            shift
            ;;
        -l|--list)
            list_modes
            exit 0
            ;;
        -*)
            print_error "未知选项: $1"
            show_help
            exit 1
            ;;
        *)
            if validate_mode "$1"; then
                BUILD_MODE="$1"
            else
                print_error "无效的构建模式: $1"
                list_modes
                exit 1
            fi
            shift
            ;;
    esac
done

# 主流程
main() {
    # 显示项目信息
    show_project_info
    
    # 显示模式特性
    show_mode_features "$BUILD_MODE"
    echo
    
    # 检查环境
    check_environment
    echo
    
    # 清理构建 (如果需要)
    if [ "$CLEAN_BUILD" = true ]; then
        clean_build
        echo
    fi
    
    # 执行构建
    perform_build "$BUILD_MODE"
    echo
    
    # 烧录固件 (如果需要)
    if [ "$FLASH_AFTER_BUILD" = true ]; then
        flash_firmware "$BUILD_MODE"
    fi
    
    print_success "所有操作完成!"
}

# 运行主程序
main "$@" 