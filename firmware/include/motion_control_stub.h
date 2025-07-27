/**
 * @file    motion_control_stub.h
 * @brief   运动控制桩模块头文件 - 用于UI测试
 * @author  ALIENTEK
 * @date    2024
 */

#ifndef __MOTION_CONTROL_STUB_H
#define __MOTION_CONTROL_STUB_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* 电机状态枚举 */
typedef enum {
    MOTOR_STATE_IDLE = 0,
    MOTOR_STATE_RUNNING = 1,
    MOTOR_STATE_ERROR = 2
} MotorState_t;

/* 运动控制参数结构体 */
typedef struct {
    int32_t position;       // 当前位置
    int32_t target_position; // 目标位置
    uint16_t speed;         // 当前速度
    uint16_t max_speed;     // 最大速度
    bool enabled;           // 使能状态
    MotorState_t state;     // 电机状态
} MotionControl_t;

/* 运动控制函数声明 */
HAL_StatusTypeDef MotionControl_Init(void);
void MotionControl_DeInit(void);
HAL_StatusTypeDef MotionControl_Enable(void);
HAL_StatusTypeDef MotionControl_Disable(void);
HAL_StatusTypeDef MotionControl_MoveToPosition(int32_t position);
HAL_StatusTypeDef MotionControl_MoveRelative(int32_t distance);
HAL_StatusTypeDef MotionControl_SetSpeed(uint16_t speed);
HAL_StatusTypeDef MotionControl_Stop(void);
HAL_StatusTypeDef MotionControl_Home(void);

/* 状态获取函数 */
MotionControl_t* MotionControl_GetStatus(void);
int32_t MotionControl_GetPosition(void);
uint16_t MotionControl_GetSpeed(void);
bool MotionControl_IsEnabled(void);
bool MotionControl_IsMoving(void);

/* 回调函数类型 */
typedef void (*MotionCompleteCallback_t)(void);
typedef void (*MotionErrorCallback_t)(uint32_t error_code);

/* 设置回调函数 */
void MotionControl_SetCompleteCallback(MotionCompleteCallback_t callback);
void MotionControl_SetErrorCallback(MotionErrorCallback_t callback);

#endif /* __MOTION_CONTROL_STUB_H */ 