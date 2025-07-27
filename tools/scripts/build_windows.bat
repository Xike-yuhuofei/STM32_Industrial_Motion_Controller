@echo off
REM =====================================================================
REM Windows批处理构建脚本 - STM32F407步进电机控制项目
REM Windows Build Script for STM32F407 Stepper Motor Control Project
REM 版本: 1.0
REM 日期: 2025-01-27
REM =====================================================================

echo ========================================
echo    STM32F407 工业运动控制卡构建系统
echo    Windows Build System
echo ========================================

REM 检查ARM工具链
where arm-none-eabi-gcc >nul 2>&1
if %errorlevel% neq 0 (
    echo [ERROR] ARM工具链未找到，请安装 arm-none-eabi-gcc
    echo [ERROR] 请确保工具链在PATH环境变量中
    pause
    exit /b 1
)

REM 显示工具链版本
echo [INFO] 检查工具链版本...
arm-none-eabi-gcc --version | findstr "arm-none-eabi-gcc"

REM 设置构建模式
set BUILD_MODE=%1
if "%BUILD_MODE%"=="" set BUILD_MODE=industrial

echo [INFO] 构建模式: %BUILD_MODE%

REM 检查是否需要清理
if "%2"=="clean" (
    echo [INFO] 清理构建目录...
    if exist build rmdir /s /q build
)

REM 开始构建
echo [INFO] 开始构建...
echo [INFO] 使用PowerShell调用GNU Make...

REM 使用PowerShell执行make命令（兼容Windows）
powershell -Command "& { $env:PATH += ';C:\Program Files (x86)\Arm GNU Toolchain arm-none-eabi\14.2 rel1\bin'; if (Get-Command 'make' -ErrorAction SilentlyContinue) { make MODE=%BUILD_MODE% } else { Write-Host '[ERROR] make工具未找到，请安装MSYS2或MinGW' -ForegroundColor Red; exit 1 } }"

if %errorlevel% neq 0 (
    echo [ERROR] 构建失败！
    pause
    exit /b 1
)

echo [SUCCESS] 构建完成！

REM 显示构建结果
if exist "build\ATK_StepMotor_%BUILD_MODE%.bin" (
    echo [INFO] 二进制文件: build\ATK_StepMotor_%BUILD_MODE%.bin
    for %%I in ("build\ATK_StepMotor_%BUILD_MODE%.bin") do echo [INFO] 文件大小: %%~zI 字节
)

REM 检查是否需要烧录
if "%3"=="flash" (
    echo [INFO] 开始烧录固件...
    STM32_Programmer_CLI -c port=SWD -w "build\ATK_StepMotor_%BUILD_MODE%.bin" 0x08000000 -v -rst
    if %errorlevel% neq 0 (
        echo [ERROR] 烧录失败！
        pause
        exit /b 1
    )
    echo [SUCCESS] 烧录完成！
)

echo [SUCCESS] 所有操作完成！
pause