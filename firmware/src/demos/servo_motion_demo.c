/**
  ******************************************************************************
  * @file    servo_motion_demo.c
  * @brief   伺服电机运动演示程序
  * @author  AI Assistant
  * @version 1.0
  * @date    2025-01-27
  ******************************************************************************
  */

#include "main.h"
#include "motion_control.h"
#include <stdio.h>
#include <math.h>

/* 私有宏定义 ----------------------------------------------------------------*/
#define SERVO_MOVE_DISTANCE_CM    5.0f     // 移动距离 5cm
#define SERVO_MOVE_DISTANCE_MM    50.0f    // 移动距离 50mm
#define SERVO_SPEED_MM_MIN        1000.0f  // 移动速度 1000mm/min
#define SERVO_PAUSE_TIME_MS       2000     // 暂停时间 2秒

/* 私有变量 ------------------------------------------------------------------*/
static uint8_t servo_demo_running = 0;
static uint32_t last_move_time = 0;
static uint8_t current_step = 0;

/* 私有函数声明 --------------------------------------------------------------*/
static void ServoMotion_DelayMs(uint32_t ms);
static void ServoMotion_PrintStatus(const char* message, float position);

/**
  * @brief  伺服电机运动演示主函数
  * @note   让伺服电机从当前位置正负方向各运动5cm
  * @retval None
  */
void ServoMotion_Demo(void)
{
    printf("\r\n=== 伺服电机运动演示开始 ===\r\n");
    printf("运动参数: 距离=±%.1fcm, 速度=%.0fmm/min\r\n", 
           SERVO_MOVE_DISTANCE_CM, SERVO_SPEED_MM_MIN);
    
    // 获取当前位置
    float start_position = g_motion_system.axes[AXIS_A].position;
    ServoMotion_PrintStatus("起始位置", start_position);
    
    // 确保伺服电机已使能
    Axis_Enable(AXIS_A);
    ServoMotion_DelayMs(100);
    
    // 第一步: 正方向移动5cm
    printf("\r\n步骤1: 正方向移动%.1fcm...\r\n", SERVO_MOVE_DISTANCE_CM);
    float target_pos1 = start_position + SERVO_MOVE_DISTANCE_MM;
    Axis_MoveToPosition(AXIS_A, target_pos1, SERVO_SPEED_MM_MIN);
    
    // 等待移动完成
    while (g_motion_system.axes[AXIS_A].state == AXIS_MOVING) {
        ServoMotion_DelayMs(10);
    }
    ServoMotion_PrintStatus("到达位置1", g_motion_system.axes[AXIS_A].position);
    
    // 暂停
    printf("暂停%.1f秒...\r\n", SERVO_PAUSE_TIME_MS / 1000.0f);
    ServoMotion_DelayMs(SERVO_PAUSE_TIME_MS);
    
    // 第二步: 返回起始位置
    printf("\r\n步骤2: 返回起始位置...\r\n");
    Axis_MoveToPosition(AXIS_A, start_position, SERVO_SPEED_MM_MIN);
    
    // 等待移动完成
    while (g_motion_system.axes[AXIS_A].state == AXIS_MOVING) {
        ServoMotion_DelayMs(10);
    }
    ServoMotion_PrintStatus("返回起始位置", g_motion_system.axes[AXIS_A].position);
    
    // 暂停
    printf("暂停%.1f秒...\r\n", SERVO_PAUSE_TIME_MS / 1000.0f);
    ServoMotion_DelayMs(SERVO_PAUSE_TIME_MS);
    
    // 第三步: 负方向移动5cm
    printf("\r\n步骤3: 负方向移动%.1fcm...\r\n", SERVO_MOVE_DISTANCE_CM);
    float target_pos2 = start_position - SERVO_MOVE_DISTANCE_MM;
    Axis_MoveToPosition(AXIS_A, target_pos2, SERVO_SPEED_MM_MIN);
    
    // 等待移动完成
    while (g_motion_system.axes[AXIS_A].state == AXIS_MOVING) {
        ServoMotion_DelayMs(10);
    }
    ServoMotion_PrintStatus("到达位置2", g_motion_system.axes[AXIS_A].position);
    
    // 暂停
    printf("暂停%.1f秒...\r\n", SERVO_PAUSE_TIME_MS / 1000.0f);
    ServoMotion_DelayMs(SERVO_PAUSE_TIME_MS);
    
    // 第四步: 最终返回起始位置
    printf("\r\n步骤4: 最终返回起始位置...\r\n");
    Axis_MoveToPosition(AXIS_A, start_position, SERVO_SPEED_MM_MIN);
    
    // 等待移动完成
    while (g_motion_system.axes[AXIS_A].state == AXIS_MOVING) {
        ServoMotion_DelayMs(10);
    }
    ServoMotion_PrintStatus("最终位置", g_motion_system.axes[AXIS_A].position);
    
    printf("\r\n=== 伺服电机运动演示完成 ===\r\n");
}

/**
  * @brief  伺服电机连续往返运动
  * @note   在当前位置±5cm范围内连续往返运动
  * @param  cycles 往返次数
  * @retval None
  */
void ServoMotion_ContinuousDemo(uint8_t cycles)
{
    printf("\r\n=== 伺服电机连续往返运动 ===\r\n");
    printf("往返次数: %d, 距离: ±%.1fcm\r\n", cycles, SERVO_MOVE_DISTANCE_CM);
    
    float start_position = g_motion_system.axes[AXIS_A].position;
    ServoMotion_PrintStatus("起始位置", start_position);
    
    // 确保伺服电机已使能
    Axis_Enable(AXIS_A);
    ServoMotion_DelayMs(100);
    
    for (uint8_t i = 0; i < cycles; i++) {
        printf("\r\n--- 第%d次往返 ---\r\n", i + 1);
        
        // 正方向移动
        printf("正向移动...\r\n");
        float target_pos1 = start_position + SERVO_MOVE_DISTANCE_MM;
        Axis_MoveToPosition(AXIS_A, target_pos1, SERVO_SPEED_MM_MIN);
        
        while (g_motion_system.axes[AXIS_A].state == AXIS_MOVING) {
            ServoMotion_DelayMs(10);
        }
        
        ServoMotion_DelayMs(500);  // 短暂暂停
        
        // 负方向移动
        printf("负向移动...\r\n");
        float target_pos2 = start_position - SERVO_MOVE_DISTANCE_MM;
        Axis_MoveToPosition(AXIS_A, target_pos2, SERVO_SPEED_MM_MIN);
        
        while (g_motion_system.axes[AXIS_A].state == AXIS_MOVING) {
            ServoMotion_DelayMs(10);
        }
        
        ServoMotion_DelayMs(500);  // 短暂暂停
    }
    
    // 返回起始位置
    printf("\r\n返回起始位置...\r\n");
    Axis_MoveToPosition(AXIS_A, start_position, SERVO_SPEED_MM_MIN);
    
    while (g_motion_system.axes[AXIS_A].state == AXIS_MOVING) {
        ServoMotion_DelayMs(10);
    }
    
    ServoMotion_PrintStatus("最终位置", g_motion_system.axes[AXIS_A].position);
    printf("\r\n=== 连续往返运动完成 ===\r\n");
}

/**
  * @brief  伺服电机精确定位测试
  * @note   测试伺服电机的定位精度
  * @retval None
  */
void ServoMotion_PositionTest(void)
{
    printf("\r\n=== 伺服电机精确定位测试 ===\r\n");
    
    float start_position = g_motion_system.axes[AXIS_A].position;
    ServoMotion_PrintStatus("起始位置", start_position);
    
    // 确保伺服电机已使能
    Axis_Enable(AXIS_A);
    ServoMotion_DelayMs(100);
    
    // 测试不同距离的定位精度
    float test_distances[] = {10.0f, 25.0f, 50.0f, 75.0f, 100.0f};  // mm
    uint8_t test_count = sizeof(test_distances) / sizeof(test_distances[0]);
    
    for (uint8_t i = 0; i < test_count; i++) {
        printf("\r\n测试距离: %.1fmm\r\n", test_distances[i]);
        
        // 正向移动
        float target_pos = start_position + test_distances[i];
        Axis_MoveToPosition(AXIS_A, target_pos, SERVO_SPEED_MM_MIN);
        
        while (g_motion_system.axes[AXIS_A].state == AXIS_MOVING) {
            ServoMotion_DelayMs(10);
        }
        
        float actual_pos = g_motion_system.axes[AXIS_A].position;
        float error = actual_pos - target_pos;
        printf("目标: %.3fmm, 实际: %.3fmm, 误差: %.3fmm\r\n", 
               target_pos, actual_pos, error);
        
        ServoMotion_DelayMs(1000);
        
        // 返回起始位置
        Axis_MoveToPosition(AXIS_A, start_position, SERVO_SPEED_MM_MIN);
        
        while (g_motion_system.axes[AXIS_A].state == AXIS_MOVING) {
            ServoMotion_DelayMs(10);
        }
        
        ServoMotion_DelayMs(500);
    }
    
    printf("\r\n=== 精确定位测试完成 ===\r\n");
}

/**
  * @brief  延时函数
  * @param  ms 延时毫秒数
  * @retval None
  */
static void ServoMotion_DelayMs(uint32_t ms)
{
    HAL_Delay(ms);
}

/**
  * @brief  打印状态信息
  * @param  message 消息字符串
  * @param  position 位置值(mm)
  * @retval None
  */
static void ServoMotion_PrintStatus(const char* message, float position)
{
    printf("%s: %.3fmm (%.2fcm)\r\n", message, position, position / 10.0f);
}

/**
  * @brief  获取伺服电机当前位置
  * @retval 当前位置(mm)
  */
float ServoMotion_GetCurrentPosition(void)
{
    return g_motion_system.axes[AXIS_A].position;
}

/**
  * @brief  设置伺服电机位置零点
  * @retval None
  */
void ServoMotion_SetZeroPosition(void)
{
    g_motion_system.axes[AXIS_A].position = 0.0f;
    g_motion_system.axes[AXIS_A].target_position = 0.0f;
    printf("伺服电机位置已清零\r\n");
}

/**
  * @brief  伺服电机急停
  * @retval None
  */
void ServoMotion_EmergencyStop(void)
{
    Axis_StopPWM(AXIS_A);
    printf("伺服电机急停!\r\n");
}