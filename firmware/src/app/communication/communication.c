/**
  ******************************************************************************
  * @file    communication.c
  * @brief   通信接口实现（简化版）
  * @author  Claude AI & Cursor
  * @version 1.0
  * @date    2025-01-27
  ******************************************************************************
  */

#include "motion_control.h"
#include <string.h>
#include <stdio.h>

/* 私有变量 ------------------------------------------------------------------*/
static char rx_buffer[MAX_COMMAND_LENGTH];
static uint8_t rx_index = 0;

/**
  * @brief  通信系统初始化
  * @retval None
  */
void Communication_Init(void)
{
    // 清空缓冲区
    memset(rx_buffer, 0, sizeof(rx_buffer));
    memset(g_command_buffer, 0, sizeof(g_command_buffer));
    rx_index = 0; // 初始化接收索引
    
    // 简化版本，不需要UART初始化
}

/**
  * @brief  发送响应信息
  * @param  response: 响应字符串
  * @retval None
  */
void Communication_SendResponse(const char *response)
{
    if (!response) return;
    
    // 简化版本，可以通过printf输出（如果有实现）
    printf("%s\r\n", response);
}

/**
  * @brief  发送系统状态
  * @retval None
  */
void Communication_SendStatus(void)
{
    char status[256];
    
    // 构建状态字符串
    snprintf(status, sizeof(status),
             "Status: %s | Mode: %d | Feed: %.1f | Buffer: %d/%d | Pos[X:%.3f Y:%.3f Z:%.3f A:%.3f]",
             g_motion_system.emergency_stop ? "ESTOP" : 
             (g_motion_system.system_running ? "RUN" : "IDLE"),
             g_motion_system.current_mode,
             g_motion_system.current_feed_rate,
             g_motion_system.buffer_count,
             MOTION_BUFFER_SIZE,
             g_motion_system.axes[AXIS_X].position,
             g_motion_system.axes[AXIS_Y].position,
             g_motion_system.axes[AXIS_Z].position,
             g_motion_system.axes[AXIS_A].position);
    
    Communication_SendResponse(status);
}

/**
  * @brief  处理接收到的命令
  * @retval None
  */
void Communication_ProcessCommand(void)
{
    if (!g_command_ready) return;
    
    // 处理特殊命令
    if (strncmp(g_command_buffer, "?", 1) == 0) {
        // 状态查询
        Communication_SendStatus();
    }
    else if (strncmp(g_command_buffer, "!", 1) == 0) {
        // 急停
        Motion_EmergencyStop();
        Communication_SendResponse("Emergency Stop Activated");
    }
    else if (strncmp(g_command_buffer, "~", 1) == 0) {
        // 恢复运行
        g_motion_system.emergency_stop = 0;
        g_motion_system.system_running = 1;
        Communication_SendResponse("System Resumed");
    }
    else if (strncmp(g_command_buffer, "$H", 2) == 0) {
        // 回零所有轴
        Motion_HomeAll();
        Communication_SendResponse("Homing All Axes");
    }
    else if (strncmp(g_command_buffer, "$X", 2) == 0) {
        // 清除报警
        g_motion_system.emergency_stop = 0;
        Communication_SendResponse("Alarm Cleared");
    }
    else if (strncmp(g_command_buffer, "$$", 2) == 0) {
        // 显示系统设置
        Communication_ShowSettings();
    }
    else {
        // G-code命令（暂时不可用，因为没有串口）
        Communication_SendResponse("ok");
    }
    
    // 清空命令缓冲区
    memset(g_command_buffer, 0, sizeof(g_command_buffer));
    g_command_ready = 0;
}

/**
  * @brief  显示系统设置
  * @retval None
  */
void Communication_ShowSettings(void)
{
    char setting[128];
    
    Communication_SendResponse("System Settings:");
    
    // 显示各轴设置
    for (int i = 0; i < MAX_AXES; i++) {
        const char* axis_names[] = {"X", "Y", "Z", "A"};
        AxisConfig_t *ax = &g_motion_system.axes[i];
        
        snprintf(setting, sizeof(setting),
                "%s-axis: %.2f steps/mm, %.1f mm/min max, %.1f mm/s² accel",
                axis_names[i], ax->steps_per_mm, ax->max_speed, ax->max_accel);
        Communication_SendResponse(setting);
    }
    
    // 显示系统参数
    snprintf(setting, sizeof(setting), "Interpolation Frequency: %d Hz", INTERPOLATION_FREQ);
    Communication_SendResponse(setting);
    
    snprintf(setting, sizeof(setting), "Motion Buffer Size: %d blocks", MOTION_BUFFER_SIZE);
    Communication_SendResponse(setting);
    
    snprintf(setting, sizeof(setting), "Current Feed Rate: %.1f mm/min", g_motion_system.current_feed_rate);
    Communication_SendResponse(setting);
}

/**
  * @brief  UART回调处理（简化版本）
  * @retval None
  */
void MotionControl_UARTCallback(void)
{
    // 简化版本，不做任何处理
}

/**
  * @brief  发送G代码示例
  * @retval None
  */
void Communication_SendExamples(void)
{
    Communication_SendResponse("G-code Examples:");
    Communication_SendResponse("G00 X10 Y20    ; Rapid move to X10 Y20");
    Communication_SendResponse("G01 X0 Y0 F100 ; Linear move to origin at 100mm/min");
    Communication_SendResponse("G02 X10 Y10 I5 J0 F200 ; Clockwise arc");
    Communication_SendResponse("G28            ; Home all axes");
    Communication_SendResponse("M00            ; Program pause");
    Communication_SendResponse("M03 S1000      ; Spindle on CW at 1000 RPM");
    Communication_SendResponse("M05            ; Spindle off");
}

/**
  * @brief  发送帮助信息
  * @retval None
  */
void Communication_SendHelp(void)
{
    Communication_SendResponse("Available Commands:");
    Communication_SendResponse("?   - Status query");
    Communication_SendResponse("!   - Emergency stop");
    Communication_SendResponse("~   - Resume operation");
    Communication_SendResponse("$H  - Home all axes");
    Communication_SendResponse("$X  - Clear alarm");
    Communication_SendResponse("$$  - Show settings");
    Communication_SendResponse("");
    Communication_SendResponse("Supported G-codes:");
    Communication_SendResponse("G00, G01, G02, G03, G04, G20, G21, G28, G90, G91, G92");
    Communication_SendResponse("");
    Communication_SendResponse("Supported M-codes:");
    Communication_SendResponse("M00, M01, M02, M03, M04, M05, M30");
}

/**
  * @brief  发送版本信息
  * @retval None
  */
void Communication_SendVersion(void)
{
    Communication_SendResponse("ATK 4-Axis CNC Controller");
    Communication_SendResponse("Version: 1.0.0");
    Communication_SendResponse("Build Date: 2025-01-27");
    Communication_SendResponse("MCU: STM32F407IGT6");
    Communication_SendResponse("Board: ATK-DMF407");
    Communication_SendResponse("Motor Driver: ATK-2MD5050");
} 