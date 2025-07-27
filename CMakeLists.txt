cmake_minimum_required(VERSION 3.16)

# 设置工具链
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_SOURCE_DIR}/cmake/arm-none-eabi-gcc.cmake)

# 项目定义
project(STM32_Industrial_Motion_Controller
    VERSION 2.0.0
    DESCRIPTION "STM32F407-based Industrial Motion Control System"
    LANGUAGES C ASM
)

# 全局设置
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 构建选项
option(BUILD_TESTS "Build unit tests" OFF)
option(BUILD_DOCS "Build documentation" OFF)
option(ENABLE_COVERAGE "Enable code coverage" OFF)

# 构建类型
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

# MCU定义
set(MCU_FAMILY STM32F4xx)
set(MCU_MODEL STM32F407xx)
set(CPU_TYPE cortex-m4)

# 编译器标志
set(MCU_FLAGS 
    -mcpu=${CPU_TYPE}
    -mthumb
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
)

set(COMMON_FLAGS
    ${MCU_FLAGS}
    -Wall
    -Wextra
    -fdata-sections
    -ffunction-sections
    $<$<CONFIG:Debug>:-Og -g -gdwarf-2>
    $<$<CONFIG:Release>:-O2 -DNDEBUG>
)

# 链接器标志
set(LINKER_FLAGS
    ${MCU_FLAGS}
    -specs=nano.specs
    -Wl,--gc-sections
    -Wl,--print-memory-usage
)

# 包含目录
set(INCLUDE_DIRS
    firmware/include
    firmware/src/platform
    firmware/drivers/CMSIS/Device/ST/STM32F4xx/Include
    firmware/drivers/CMSIS/Include
    firmware/drivers/STM32F4xx_HAL_Driver/Inc
    firmware/drivers/STM32F4xx_HAL_Driver/Inc/Legacy
)

# 源文件组织
set(PLATFORM_SOURCES
    firmware/src/platform/main.c
    firmware/src/platform/stm32f4xx_hal_msp.c
    firmware/src/platform/stm32f4xx_it.c
    firmware/src/platform/system_stm32f4xx.c
    firmware/src/platform/syscalls.c
    firmware/src/platform/sysmem.c
)

set(MOTION_SOURCES
    firmware/src/app/motion/motion_control.c
    firmware/src/app/motion/position_control.c
    firmware/src/app/motion/advanced_interpolation.c
    firmware/src/app/motion/advanced_motion_control.c
)

set(GCODE_SOURCES
    firmware/src/app/gcode/gcode_parser.c
    firmware/src/app/gcode/gcode_executor.c
)

set(COMMUNICATION_SOURCES
    firmware/src/app/communication/communication.c
    firmware/src/app/communication/industrial_communication.c
)

set(DIAGNOSTICS_SOURCES
    firmware/src/app/diagnostics/fault_diagnosis.c
)

set(CORE_SOURCES
    firmware/src/core/industrial_motion_controller.c
    firmware/src/core/synchronous_control.c
    firmware/src/core/error_handler.c
)

set(DEMO_SOURCES
    firmware/src/demos/servo_motion_demo.c
    firmware/src/demos/industrial_motion_demo.c
    firmware/src/demos/position_control_demo.c
)

# HAL库源文件
file(GLOB HAL_SOURCES 
    firmware/drivers/STM32F4xx_HAL_Driver/Src/*.c
)

# 启动文件
set(STARTUP_SOURCE firmware/startup/startup_stm32f407igtx.s)

# 链接脚本
set(LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/firmware/linker/STM32F407IGTX_FLASH.ld)

# 创建不同的构建目标
function(create_firmware_target TARGET_NAME SOURCES DEFINES)
    add_executable(${TARGET_NAME}
        ${PLATFORM_SOURCES}
        ${HAL_SOURCES}
        ${STARTUP_SOURCE}
        ${SOURCES}
    )
    
    target_include_directories(${TARGET_NAME} PRIVATE ${INCLUDE_DIRS})
    
    target_compile_definitions(${TARGET_NAME} PRIVATE
        USE_HAL_DRIVER
        ${MCU_MODEL}
        ${DEFINES}
    )
    
    target_compile_options(${TARGET_NAME} PRIVATE ${COMMON_FLAGS})
    
    target_link_options(${TARGET_NAME} PRIVATE 
        ${LINKER_FLAGS}
        -T${LINKER_SCRIPT}
    )
    
    # 生成二进制文件
    add_custom_command(TARGET ${TARGET_NAME} POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${TARGET_NAME}> ${TARGET_NAME}.hex
        COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${TARGET_NAME}> ${TARGET_NAME}.bin
        COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${TARGET_NAME}>
        COMMENT "Building ${TARGET_NAME}.hex and ${TARGET_NAME}.bin"
    )
endfunction()

# 工业运动控制目标
create_firmware_target(industrial_motion
    "${MOTION_SOURCES};${GCODE_SOURCES};${COMMUNICATION_SOURCES};${DIAGNOSTICS_SOURCES};${CORE_SOURCES}"
    "MODE_INDUSTRIAL"
)

# G代码解析器目标
create_firmware_target(gcode_parser
    "${GCODE_SOURCES};${MOTION_SOURCES}"
    "MODE_GCODE"
)

# 高级运动控制目标
create_firmware_target(advanced_motion
    "${MOTION_SOURCES};${CORE_SOURCES}"
    "MODE_ADVANCED"
)

# 基础控制目标
create_firmware_target(basic_control
    "${MOTION_SOURCES}"
    "MODE_BASIC"
)

# 测试构建
if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

# 文档生成
if(BUILD_DOCS)
    find_package(Doxygen)
    if(DOXYGEN_FOUND)
        add_subdirectory(docs)
    endif()
endif()

# 安装规则
install(TARGETS industrial_motion gcode_parser advanced_motion basic_control
    RUNTIME DESTINATION bin
)

# CPack配置
set(CPACK_PACKAGE_NAME "STM32_Industrial_Motion_Controller")
set(CPACK_PACKAGE_VERSION ${PROJECT_VERSION})
set(CPACK_PACKAGE_DESCRIPTION ${PROJECT_DESCRIPTION})
include(CPack)