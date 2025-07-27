# PowerShell Build Script for STM32F407 Basic Mode
# Replace make tool for compilation

Write-Host "Starting STM32F407 basic mode firmware compilation..." -ForegroundColor Green

# Compiler settings
$CC = "arm-none-eabi-gcc"
$OBJCOPY = "arm-none-eabi-objcopy"
$SIZE = "arm-none-eabi-size"

# Compilation parameters
$MCU_FLAGS = @("-mcpu=cortex-m4", "-mthumb", "-mfpu=fpv4-sp-d16", "-mfloat-abi=hard")
$DEFINES = @("-DUSE_HAL_DRIVER", "-DSTM32F407xx")
$INCLUDES = @("-ICore/Inc", "-IDrivers/STM32F4xx_HAL_Driver/Inc", "-IDrivers/STM32F4xx_HAL_Driver/Inc/Legacy", "-IDrivers/CMSIS/Device/ST/STM32F4xx/Include", "-IDrivers/CMSIS/Include")
$CFLAGS = $MCU_FLAGS + $DEFINES + $INCLUDES + @("-Og", "-Wall", "-fdata-sections", "-ffunction-sections")
$LDFLAGS = $MCU_FLAGS + @("-specs=nano.specs", "-TSTM32F407IGTX_FLASH.ld", "-lc", "-lm", "-lnosys", "-Wl,-Map=build/ATK_StepMotor_basic.map,--cref", "-Wl,--gc-sections")

# Source files list
$SOURCES = @(
    "Core/Src/main.c",
    "Core/Src/motion_control.c",
    "Core/Src/position_control.c",
    "Core/Src/position_control_demo.c",
    "Core/Src/communication.c",
    "Core/Src/stm32f4xx_it.c",
    "Core/Src/stm32f4xx_hal_msp.c",
    "Core/Src/system_stm32f4xx.c",
    "Core/Src/sysmem.c",
    "Core/Src/syscalls.c",
    "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c",
    "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c",
    "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c",
    "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c",
    "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c",
    "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c",
    "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c",
    "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c",
    "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c",
    "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c",
    "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c",
    "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c",
    "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_exti.c",
    "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c",
    "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c",
    "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_sram.c",
    "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_fsmc.c"
)

# Assembly file
$ASM_SOURCE = "Core/Startup/startup_stm32f407igtx.s"

# Create build directory
if (!(Test-Path "build")) {
    New-Item -ItemType Directory -Path "build" | Out-Null
}

# Compile all C source files
$OBJECTS = @()
foreach ($source in $SOURCES) {
    $objname = [System.IO.Path]::GetFileNameWithoutExtension($source) + ".o"
    $objpath = "build/$objname"
    $OBJECTS += $objpath
    
    Write-Host "Compiling: $source" -ForegroundColor Yellow
    & $CC -c $CFLAGS $source -o $objpath
    if ($LASTEXITCODE -ne 0) {
        Write-Host "Compilation failed: $source" -ForegroundColor Red
        exit 1
    }
}

# Compile assembly file
Write-Host "Compiling assembly file: $ASM_SOURCE" -ForegroundColor Yellow
$ASM_OBJ = "build/startup_stm32f407igtx.o"
& $CC -c $MCU_FLAGS $ASM_SOURCE -o $ASM_OBJ
if ($LASTEXITCODE -ne 0) {
    Write-Host "Assembly compilation failed" -ForegroundColor Red
    exit 1
}
$OBJECTS += $ASM_OBJ

# Link to generate ELF file
Write-Host "Linking to generate ELF file..." -ForegroundColor Cyan
$ELF_FILE = "build/ATK_StepMotor_basic.elf"
& $CC $OBJECTS $LDFLAGS -o $ELF_FILE
if ($LASTEXITCODE -ne 0) {
    Write-Host "Linking failed" -ForegroundColor Red
    exit 1
}

# Generate HEX file
Write-Host "Generating HEX file..." -ForegroundColor Cyan
$HEX_FILE = "build/ATK_StepMotor_basic.hex"
& $OBJCOPY -O ihex $ELF_FILE $HEX_FILE
if ($LASTEXITCODE -ne 0) {
    Write-Host "HEX file generation failed" -ForegroundColor Red
    exit 1
}

# Generate BIN file
Write-Host "Generating BIN file..." -ForegroundColor Cyan
$BIN_FILE = "build/ATK_StepMotor_basic.bin"
& $OBJCOPY -O binary -S $ELF_FILE $BIN_FILE
if ($LASTEXITCODE -ne 0) {
    Write-Host "BIN file generation failed" -ForegroundColor Red
    exit 1
}

# Display size information
Write-Host "Firmware size information:" -ForegroundColor Green
& $SIZE $ELF_FILE

Write-Host "Compilation completed!" -ForegroundColor Green
Write-Host "Generated files:" -ForegroundColor White
Write-Host "  - $ELF_FILE" -ForegroundColor White
Write-Host "  - $HEX_FILE" -ForegroundColor White
Write-Host "  - $BIN_FILE" -ForegroundColor White

Write-Host "`nYou can use the following command to flash the firmware:" -ForegroundColor Yellow
Write-Host "STM32_Programmer_CLI -c port=SWD -w `"$BIN_FILE`" 0x08000000 -v -rst" -ForegroundColor Cyan