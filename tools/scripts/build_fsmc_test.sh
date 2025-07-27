#!/bin/bash

# FSMC测试编译脚本
# 测试FSMC接口的基本功能

set -e

echo "编译FSMC接口测试程序..."
echo "=========================="

# 清理之前的编译文件
echo "清理编译文件..."
make clean

# 设置编译目标
export TARGET=main_fsmc_test

# 开始编译
echo "开始编译 $TARGET..."
make TARGET=$TARGET

# 检查编译结果
if [ -f "build/ATK_StepMotor_${TARGET}.elf" ]; then
    echo ""
    echo "✅ 编译成功！"
    echo "目标文件: build/ATK_StepMotor_${TARGET}.elf"
    echo "二进制文件: build/ATK_StepMotor_${TARGET}.bin"
    echo "十六进制文件: build/ATK_StepMotor_${TARGET}.hex"
    
    # 显示程序大小
    echo ""
    echo "程序大小信息:"
    arm-none-eabi-size build/ATK_StepMotor_${TARGET}.elf
    
    echo ""
    echo "测试内容："
    echo "   - FSMC接口配置"
    echo "   - 基本读写操作"
    echo "   - 硬件连接验证"
    echo ""
    echo "烧录命令: make flash TARGET=$TARGET"
    
else
    echo ""
    echo "❌ 编译失败！"
    echo "请检查编译错误信息"
    exit 1
fi
