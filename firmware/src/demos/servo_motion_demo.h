/**
  ******************************************************************************
  * @file    servo_motion_demo.h
  * @brief   伺服电机运动演示程序头文件
  * @author  AI Assistant
  * @version 1.0
  * @date    2025-01-27
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SERVO_MOTION_DEMO_H
#define __SERVO_MOTION_DEMO_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

/**
  * @brief  伺服电机运动演示主函数
  * @note   让伺服电机从当前位置正负方向各运动5cm
  * @retval None
  */
void ServoMotion_Demo(void);

/**
  * @brief  伺服电机连续往返运动
  * @note   在当前位置±5cm范围内连续往返运动
  * @param  cycles 往返次数
  * @retval None
  */
void ServoMotion_ContinuousDemo(uint8_t cycles);

/**
  * @brief  伺服电机精确定位测试
  * @note   测试伺服电机的定位精度
  * @retval None
  */
void ServoMotion_PositionTest(void);

/**
  * @brief  获取伺服电机当前位置
  * @retval 当前位置(mm)
  */
float ServoMotion_GetCurrentPosition(void);

/**
  * @brief  设置伺服电机位置零点
  * @retval None
  */
void ServoMotion_SetZeroPosition(void);

/**
  * @brief  伺服电机急停
  * @retval None
  */
void ServoMotion_EmergencyStop(void);

#ifdef __cplusplus
}
#endif

#endif /* __SERVO_MOTION_DEMO_H */