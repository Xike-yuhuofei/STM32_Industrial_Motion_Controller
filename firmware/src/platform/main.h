/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

// 步进电机方向枚举
typedef enum {
    MOTOR_DIR_CW = 0,   // 顺时针
    MOTOR_DIR_CCW = 1   // 逆时针
} MotorDirection_t;

// 步进电机速度等级枚举
typedef enum {
    SPEED_VERY_SLOW = 0,  // 50 Hz
    SPEED_SLOW = 1,       // 100 Hz  
    SPEED_MEDIUM = 2,     // 200 Hz (默认)
    SPEED_FAST = 3,       // 500 Hz
    SPEED_VERY_FAST = 4   // 1000 Hz
} MotorSpeed_t;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

// 伺服电机引脚定义 (ATK-DMF407开发板)
#define MOTOR4_PUL_PIN       GPIO_PIN_5     // PI5 - PWM脉冲输出 PUL+ (TIM8_CH1)
#define MOTOR4_PUL_PORT      GPIOI
#define MOTOR4_DIR_PIN       GPIO_PIN_14    // PF14 - 方向控制 DIR+
#define MOTOR4_DIR_PORT      GPIOF  
#define MOTOR4_ENA_PIN       GPIO_PIN_3     // PH3 - 使能控制
#define MOTOR4_ENA_PORT      GPIOH

// 步进电机参数配置
#define MOTOR_STEP_ANGLE     1.8f           // 步进角度 (度)
#define MOTOR_MICROSTEP      16             // 细分设置
#define STEPS_PER_REV        (360.0f / MOTOR_STEP_ANGLE * MOTOR_MICROSTEP)  // 每转步数 (3200)

// PWM频率配置 (Hz)
#define PWM_FREQ_VERY_SLOW   50
#define PWM_FREQ_SLOW        100
#define PWM_FREQ_MEDIUM      200    // 默认频率
#define PWM_FREQ_FAST        500
#define PWM_FREQ_VERY_FAST   1000

// 定时器配置
#define MOTOR_TIM            TIM8           // 使用TIM8
#define MOTOR_TIM_CHANNEL    TIM_CHANNEL_1  // 通道1

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

// HAL MSP函数声明
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

// 步进电机控制函数声明
void StepMotor_Init(void);
void StepMotor_Enable(void);
void StepMotor_Disable(void);
void StepMotor_SetDirection(MotorDirection_t direction);
void StepMotor_SetSpeed(MotorSpeed_t speed);
void StepMotor_Start(void);
void StepMotor_Stop(void);
void StepMotor_MoveSteps(uint32_t steps, MotorDirection_t direction);
void StepMotor_MoveAngle(float angle, MotorDirection_t direction);
void StepMotor_RotateByAngle(float angle);
uint32_t StepMotor_AngleToSteps(float angle);

// PWM控制函数声明
void Motor_SetPWMFrequency(uint32_t frequency);
void Motor_SetPWMDutyCycle(uint8_t duty_cycle);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MOTOR4_DIR_Pin GPIO_PIN_14
#define MOTOR4_DIR_GPIO_Port GPIOF
#define MOTOR4_ENA_Pin GPIO_PIN_3
#define MOTOR4_ENA_GPIO_Port GPIOH

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_TIM8_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
