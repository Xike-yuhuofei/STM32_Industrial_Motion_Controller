# 伺服电机运动演示程序构建脚本
# PowerShell脚本用于编译ATK-DMF407伺服电机运动演示程序

Write-Host "=== ATK-DMF407 伺服电机运动演示程序构建脚本 ===" -ForegroundColor Green
Write-Host "构建目标: 伺服电机±5cm运动演示" -ForegroundColor Yellow
Write-Host "硬件平台: ATK-DMF407开发板" -ForegroundColor Yellow
Write-Host "" 

# 设置构建参数
$PROJECT_NAME = "ATK_ServoMotion_Demo"
$BUILD_DIR = "build"
$TARGET_MCU = "STM32F407IGTx"
$OPTIMIZATION = "-Os"

# 检查工具链
Write-Host "检查ARM GCC工具链..." -ForegroundColor Cyan
if (!(Get-Command "arm-none-eabi-gcc" -ErrorAction SilentlyContinue)) {
    Write-Host "错误: 未找到ARM GCC工具链!" -ForegroundColor Red
    Write-Host "请确保arm-none-eabi-gcc在PATH环境变量中" -ForegroundColor Red
    exit 1
}

# 创建构建目录
if (!(Test-Path $BUILD_DIR)) {
    New-Item -ItemType Directory -Path $BUILD_DIR | Out-Null
    Write-Host "创建构建目录: $BUILD_DIR" -ForegroundColor Green
}

# 定义源文件
$SOURCES = @(
    # 核心源文件
    "Core/Src/main_servo_motion_demo.c",
    "Core/Src/servo_motion_demo.c",
    "Core/Src/motion_control.c",
    "Core/Src/stm32f4xx_it.c",
    "Core/Src/stm32f4xx_hal_msp.c",
    "Core/Src/system_stm32f4xx.c",
    "Core/Src/syscalls.c",
    "Core/Src/sysmem.c",
    
    # HAL驱动源文件
    "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c",
    "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c",
    "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c",
    "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c",
    "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c",
    "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c",
    "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c",
    "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c",
    "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c",
    "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c",
    "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c",
    "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c",
    "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_exti.c",
    "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c",
    "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c",
    "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c",
    
    # 启动文件
    "Core/Startup/startup_stm32f407igtx.s"
)

# 定义包含路径
$INCLUDES = @(
    "-ICore/Inc",
    "-IDrivers/STM32F4xx_HAL_Driver/Inc",
    "-IDrivers/STM32F4xx_HAL_Driver/Inc/Legacy",
    "-IDrivers/CMSIS/Device/ST/STM32F4xx/Include",
    "-IDrivers/CMSIS/Include"
)

# 定义编译器标志
$CFLAGS = @(
    "-mcpu=cortex-m4",
    "-mthumb",
    "-mfpu=fpv4-sp-d16",
    "-mfloat-abi=hard",
    $OPTIMIZATION,
    "-fdata-sections",
    "-ffunction-sections",
    "-Wall",
    "-fstack-usage",
    "-MMD",
    "-MP"
)

# 定义预处理器宏
$DEFINES = @(
    "-DUSE_HAL_DRIVER",
    "-DSTM32F407xx"
)

# 定义链接器标志
$LDFLAGS = @(
    "-mcpu=cortex-m4",
    "-mthumb",
    "-mfpu=fpv4-sp-d16",
    "-mfloat-abi=hard",
    "-specs=nano.specs",
    "-T$TARGET_MCU`_FLASH.ld",
    "-Wl,-Map=$BUILD_DIR/$PROJECT_NAME.map,--cref",
    "-Wl,--gc-sections",
    "-static",
    "-Wl,--start-group",
    "-lc",
    "-lm",
    "-Wl,--end-group"
)

Write-Host "开始编译源文件..." -ForegroundColor Cyan

# 编译源文件
$OBJECTS = @()
foreach ($source in $SOURCES) {
    if (Test-Path $source) {
        $basename = [System.IO.Path]::GetFileNameWithoutExtension($source)
        $object = "$BUILD_DIR/$basename.o"
        $OBJECTS += $object
        
        Write-Host "编译: $source" -ForegroundColor Gray
        
        if ($source.EndsWith(".s")) {
            # 汇编文件
            $cmd = "arm-none-eabi-gcc", $CFLAGS, $DEFINES, $INCLUDES, "-c", $source, "-o", $object
        } else {
            # C文件
            $cmd = "arm-none-eabi-gcc", $CFLAGS, $DEFINES, $INCLUDES, "-c", $source, "-o", $object
        }
        
        $result = & $cmd[0] $cmd[1..($cmd.Length-1)] 2>&1
        if ($LASTEXITCODE -ne 0) {
            Write-Host "编译失败: $source" -ForegroundColor Red
            Write-Host $result -ForegroundColor Red
            exit 1
        }
    } else {
        Write-Host "警告: 源文件不存在: $source" -ForegroundColor Yellow
    }
}

Write-Host "链接目标文件..." -ForegroundColor Cyan

# 链接
$ELF_FILE = "$BUILD_DIR/$PROJECT_NAME.elf"
$cmd = "arm-none-eabi-gcc", $OBJECTS, $LDFLAGS, "-o", $ELF_FILE
$result = & $cmd[0] $cmd[1..($cmd.Length-1)] 2>&1
if ($LASTEXITCODE -ne 0) {
    Write-Host "链接失败!" -ForegroundColor Red
    Write-Host $result -ForegroundColor Red
    exit 1
}

Write-Host "生成二进制文件..." -ForegroundColor Cyan

# 生成HEX文件
$HEX_FILE = "$BUILD_DIR/$PROJECT_NAME.hex"
& arm-none-eabi-objcopy -O ihex $ELF_FILE $HEX_FILE
if ($LASTEXITCODE -ne 0) {
    Write-Host "生成HEX文件失败!" -ForegroundColor Red
    exit 1
}

# 生成BIN文件
$BIN_FILE = "$BUILD_DIR/$PROJECT_NAME.bin"
& arm-none-eabi-objcopy -O binary -S $ELF_FILE $BIN_FILE
if ($LASTEXITCODE -ne 0) {
    Write-Host "生成BIN文件失败!" -ForegroundColor Red
    exit 1
}

# 显示大小信息
Write-Host "" 
Write-Host "=== 构建完成 ===" -ForegroundColor Green
& arm-none-eabi-size $ELF_FILE

Write-Host "" 
Write-Host "生成的文件:" -ForegroundColor Yellow
Write-Host "  ELF: $ELF_FILE" -ForegroundColor White
Write-Host "  HEX: $HEX_FILE" -ForegroundColor White
Write-Host "  BIN: $BIN_FILE" -ForegroundColor White
Write-Host "  MAP: $BUILD_DIR/$PROJECT_NAME.map" -ForegroundColor White

Write-Host "" 
Write-Host "=== 伺服电机运动演示程序构建成功! ===" -ForegroundColor Green
Write-Host "功能说明:" -ForegroundColor Yellow
Write-Host "  1. 基础运动演示 - 伺服电机±5cm单次运动" -ForegroundColor White
Write-Host "  2. 连续往返运动 - 可设置往返次数" -ForegroundColor White
Write-Host "  3. 精确定位测试 - 测试不同距离的定位精度" -ForegroundColor White
Write-Host "  4. 位置管理 - 设置零点、查看当前位置" -ForegroundColor White
Write-Host "  5. 急停功能 - 紧急停止运动" -ForegroundColor White
Write-Host "" 
Write-Host "使用方法:" -ForegroundColor Yellow
Write-Host "  1. 将生成的HEX或BIN文件烧录到ATK-DMF407开发板" -ForegroundColor White
Write-Host "  2. 连接串口(115200波特率)查看运行状态" -ForegroundColor White
Write-Host "  3. 通过串口菜单选择不同的运动模式" -ForegroundColor White
Write-Host ""