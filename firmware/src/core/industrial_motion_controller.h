/**
  ******************************************************************************
  * @file    industrial_motion_controller.h
  * @brief   工业运动控制卡核心架构定义
  * @author  Industrial Motion Control Team
  * @version 2.0
  * @date    2025-01-27
  ******************************************************************************
  * @attention
  * 
  * 本文件定义了高性能工业运动控制卡的核心架构，包括：
  * - FreeRTOS实时任务架构
  * - 硬件抽象层(HAL)
  * - 运动控制中间件
  * - 高级控制算法接口
  * - 工业通讯协议栈
  * - 故障诊断系统
  * 
  ******************************************************************************
  */

#ifndef __INDUSTRIAL_MOTION_CONTROLLER_H
#define __INDUSTRIAL_MOTION_CONTROLLER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#ifdef USE_FREERTOS
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"
#endif
#include "motion_control.h"
#include <stdint.h>
#include <stdbool.h>

/* 宏定义 --------------------------------------------------------------------*/
#define IMC_VERSION_MAJOR               2
#define IMC_VERSION_MINOR               0
#define IMC_VERSION_PATCH               0

// 系统配置参数
#define IMC_MAX_AXES                    8       // 最大轴数
#define IMC_MAX_SPINDLES                4       // 最大主轴数
#define IMC_MAX_SYNC_GROUPS             4       // 最大同步组数
#define IMC_MAX_INTERPOLATION_BUFFER    1024    // 插补缓冲区大小

// 任务优先级定义 (数值越大优先级越高)
#define IMC_TASK_PRIORITY_CRITICAL      7       // 关键运动控制任务
#define IMC_TASK_PRIORITY_HIGH          6       // 高速插补任务
#define IMC_TASK_PRIORITY_NORMAL        5       // 普通运动任务
#define IMC_TASK_PRIORITY_COMM          4       // 通讯任务
#define IMC_TASK_PRIORITY_MONITOR       3       // 监控任务
#define IMC_TASK_PRIORITY_LOW           2       // 低优先级任务
#define IMC_TASK_PRIORITY_IDLE          1       // 空闲任务

// 任务栈大小定义
#define IMC_STACK_SIZE_CRITICAL         2048
#define IMC_STACK_SIZE_HIGH             1536
#define IMC_STACK_SIZE_NORMAL           1024
#define IMC_STACK_SIZE_COMM             1024
#define IMC_STACK_SIZE_MONITOR          512

// 循环周期定义 (毫秒)
#define IMC_CYCLE_MOTION_CONTROL        1       // 1ms 运动控制周期
#define IMC_CYCLE_INTERPOLATION         10      // 10ms 插补周期
#define IMC_CYCLE_COMMUNICATION         20      // 20ms 通讯周期
#define IMC_CYCLE_MONITOR               100     // 100ms 监控周期

/* 枚举类型定义 --------------------------------------------------------------*/

/**
 * @brief 控制卡运行状态
 */
typedef enum {
    IMC_STATE_STOPPED = 0,          // 停止状态
    IMC_STATE_READY,                // 就绪状态
    IMC_STATE_RUNNING,              // 运行状态
    IMC_STATE_PAUSED,               // 暂停状态
    IMC_STATE_ERROR,                // 错误状态
    IMC_STATE_EMERGENCY             // 急停状态
} IMC_State_t;

/**
 * @brief 运动模式类型
 */
typedef enum {
    IMC_MODE_POINT_TO_POINT = 0,    // 点到点运动
    IMC_MODE_CONTINUOUS,            // 连续路径运动
    IMC_MODE_SYNCHRONOUS,           // 同步运动
    IMC_MODE_ELECTRONIC_GEAR,       // 电子齿轮
    IMC_MODE_ELECTRONIC_CAM,        // 电子凸轮
    IMC_MODE_FLYING_SHEAR          // 飞剪模式
} IMC_MotionMode_t;

/**
 * @brief 通讯协议类型
 */
typedef enum {
    IMC_COMM_ETHERNET = 0,          // 工业以太网
    IMC_COMM_ETHERCAT,              // EtherCAT
    IMC_COMM_MODBUS_TCP,            // Modbus TCP
    IMC_COMM_CAN,                   // CAN总线
    IMC_COMM_RS485,                 // RS485
    IMC_COMM_USB                    // USB调试
} IMC_CommProtocol_t;

/**
 * @brief 故障诊断类型
 */
typedef enum {
    IMC_FAULT_NONE = 0,             // 无故障
    IMC_FAULT_OVER_CURRENT,         // 过流故障
    IMC_FAULT_OVER_VOLTAGE,         // 过压故障
    IMC_FAULT_UNDER_VOLTAGE,        // 欠压故障
    IMC_FAULT_OVER_TEMP,            // 过温故障
    IMC_FAULT_ENCODER_ERROR,        // 编码器故障
    IMC_FAULT_POSITION_ERROR,       // 位置误差过大
    IMC_FAULT_COMM_TIMEOUT,         // 通讯超时
    IMC_FAULT_EMERGENCY_STOP        // 急停触发
} IMC_FaultType_t;

/* 结构体定义 ----------------------------------------------------------------*/

/**
 * @brief 轴参数配置结构
 */
typedef struct {
    // 基本参数
    uint8_t axis_id;                    // 轴号
    char axis_name[16];                 // 轴名称
    bool enabled;                       // 轴使能状态
    
    // 运动参数
    float max_velocity;                 // 最大速度 (mm/min)
    float max_acceleration;             // 最大加速度 (mm/s²)
    float max_jerk;                     // 最大加加速度 (mm/s³)
    float position_limit_pos;           // 正向位置限制
    float position_limit_neg;           // 负向位置限制
    
    // 编码器参数
    uint32_t encoder_resolution;        // 编码器分辨率 (脉冲/转)
    float encoder_ratio;                // 编码器传动比
    bool encoder_reverse;               // 编码器方向反向
    
    // 控制参数
    float position_kp;                  // 位置环比例增益
    float position_ki;                  // 位置环积分增益
    float position_kd;                  // 位置环微分增益
    float velocity_kp;                  // 速度环比例增益
    float velocity_ki;                  // 速度环积分增益
    float velocity_kd;                  // 速度环微分增益
    
    // 安全参数
    float following_error_limit;        // 跟随误差限制
    float current_limit;                // 电流限制
    float temperature_limit;            // 温度限制
} IMC_AxisConfig_t;

/**
 * @brief 轴状态信息结构
 */
typedef struct {
    // 位置信息
    float actual_position;              // 实际位置
    float command_position;             // 指令位置
    float position_error;               // 位置误差
    
    // 速度信息
    float actual_velocity;              // 实际速度
    float command_velocity;             // 指令速度
    
    // 控制信息
    float control_output;               // 控制输出
    float current_feedback;             // 电流反馈
    float temperature;                  // 温度
    
    // 状态标志
    bool in_position;                   // 到位标志
    bool moving;                        // 运动标志
    bool enabled;                       // 使能标志
    bool homed;                         // 回零完成标志
    
    // 故障状态
    IMC_FaultType_t fault_code;         // 故障代码
    uint32_t fault_timestamp;           // 故障时间戳
} IMC_AxisState_t;

/**
 * @brief 同步控制组配置
 */
typedef struct {
    uint8_t group_id;                   // 同步组ID
    uint8_t master_axis;                // 主轴号
    uint8_t slave_axes[IMC_MAX_AXES];   // 从轴列表
    uint8_t slave_count;                // 从轴数量
    float gear_ratio[IMC_MAX_AXES];     // 齿轮比
    bool cross_coupling_enable;         // 交叉耦合使能
    float coupling_gain;                // 耦合增益
} IMC_SyncGroup_t;

/**
 * @brief 插补数据块结构
 */
typedef struct {
    uint8_t axes_mask;                  // 参与轴掩码
    float target_position[IMC_MAX_AXES]; // 目标位置
    float feed_rate;                    // 进给速度
    float acceleration;                 // 加速度
    uint8_t interpolation_type;         // 插补类型
    uint32_t segment_time;              // 段时间
    bool block_complete;                // 数据块完成标志
} IMC_InterpolationBlock_t;

/**
 * @brief 工业控制卡系统结构
 */
typedef struct {
    // 系统状态
    IMC_State_t system_state;           // 系统状态
    uint32_t system_time;               // 系统时间
    bool emergency_stop;                // 急停状态
    
    // 轴配置和状态
    IMC_AxisConfig_t axis_config[IMC_MAX_AXES];     // 轴配置
    IMC_AxisState_t axis_state[IMC_MAX_AXES];       // 轴状态
    uint8_t axes_count;                             // 轴数量
    
    // 同步控制
    IMC_SyncGroup_t sync_groups[IMC_MAX_SYNC_GROUPS]; // 同步组
    uint8_t sync_groups_count;                         // 同步组数量
    
    // 插补缓冲区
    IMC_InterpolationBlock_t interp_buffer[IMC_MAX_INTERPOLATION_BUFFER];
    uint16_t interp_buffer_head;        // 缓冲区头指针
    uint16_t interp_buffer_tail;        // 缓冲区尾指针
    uint16_t interp_buffer_count;       // 缓冲区数据量
    
    // 性能统计
    uint32_t cycle_time_max;            // 最大循环时间
    uint32_t cycle_time_min;            // 最小循环时间
    uint32_t cycle_time_avg;            // 平均循环时间
    uint32_t interpolation_rate;        // 插补频率
    
    // 故障记录
    IMC_FaultType_t fault_history[32];  // 故障历史
    uint8_t fault_count;                // 故障计数
} IMC_System_t;

/* 全局变量声明 --------------------------------------------------------------*/
extern IMC_System_t g_imc_system;

/* FreeRTOS任务句柄声明 ------------------------------------------------------*/
#ifdef USE_FREERTOS
extern TaskHandle_t xTaskMotionControl;     // 运动控制任务
extern TaskHandle_t xTaskInterpolation;     // 插补任务
extern TaskHandle_t xTaskCommunication;     // 通讯任务
extern TaskHandle_t xTaskMonitor;           // 监控任务
extern TaskHandle_t xTaskDiagnosis;         // 诊断任务

/* FreeRTOS队列和信号量声明 --------------------------------------------------*/
extern QueueHandle_t xQueueInterpolation;   // 插补数据队列
extern QueueHandle_t xQueueCommand;         // 命令队列
extern SemaphoreHandle_t xSemMotionControl; // 运动控制信号量
extern SemaphoreHandle_t xSemConfigUpdate;  // 配置更新信号量
#else
// 非FreeRTOS环境下的替代定义
typedef void* TaskHandle_t;
typedef void* QueueHandle_t;
typedef void* SemaphoreHandle_t;
typedef void* TimerHandle_t;
#endif

/* 函数声明 ------------------------------------------------------------------*/

/* 系统初始化和管理函数 */
HAL_StatusTypeDef IMC_SystemInit(void);
HAL_StatusTypeDef IMC_SystemStart(void);
HAL_StatusTypeDef IMC_SystemStop(void);
HAL_StatusTypeDef IMC_SystemReset(void);
IMC_State_t IMC_GetSystemState(void);

/* 轴管理函数 */
HAL_StatusTypeDef IMC_AxisInit(uint8_t axis_id, const IMC_AxisConfig_t *config);
HAL_StatusTypeDef IMC_AxisEnable(uint8_t axis_id, bool enable);
HAL_StatusTypeDef IMC_AxisHome(uint8_t axis_id);
HAL_StatusTypeDef IMC_AxisMove(uint8_t axis_id, float target_position, float velocity);
HAL_StatusTypeDef IMC_AxisStop(uint8_t axis_id);
IMC_AxisState_t* IMC_GetAxisState(uint8_t axis_id);

/* 同步控制函数 */
HAL_StatusTypeDef IMC_SyncGroupCreate(uint8_t group_id, uint8_t master_axis, 
                                     const uint8_t *slave_axes, uint8_t slave_count);
HAL_StatusTypeDef IMC_SyncGroupSetRatio(uint8_t group_id, uint8_t axis_id, float ratio);
HAL_StatusTypeDef IMC_SyncGroupStart(uint8_t group_id);
HAL_StatusTypeDef IMC_SyncGroupStop(uint8_t group_id);

/* 插补控制函数 */
HAL_StatusTypeDef IMC_InterpolationAddBlock(const IMC_InterpolationBlock_t *block);
uint16_t IMC_InterpolationGetBufferCount(void);
HAL_StatusTypeDef IMC_InterpolationStart(void);
HAL_StatusTypeDef IMC_InterpolationStop(void);

/* 故障诊断函数 */
HAL_StatusTypeDef IMC_DiagnosisInit(void);
IMC_FaultType_t IMC_DiagnosisGetFault(uint8_t axis_id);
HAL_StatusTypeDef IMC_DiagnosisClearFault(uint8_t axis_id);
const char* IMC_DiagnosisGetFaultString(IMC_FaultType_t fault);

/* 通讯协议函数 */
HAL_StatusTypeDef IMC_CommInit(IMC_CommProtocol_t protocol);
HAL_StatusTypeDef IMC_CommSendData(const uint8_t *data, uint16_t length);
HAL_StatusTypeDef IMC_CommReceiveData(uint8_t *data, uint16_t *length);

/* FreeRTOS任务函数声明 */
void vTaskMotionControl(void *pvParameters);
void vTaskInterpolation(void *pvParameters);
void vTaskCommunication(void *pvParameters);
void vTaskMonitor(void *pvParameters);
void vTaskDiagnosis(void *pvParameters);

/* 中断服务函数 */
void IMC_MotionControlISR(void);
void IMC_EncoderISR(uint8_t axis_id);
void IMC_EmergencyStopISR(void);

#ifdef __cplusplus
}
#endif

#endif /* __INDUSTRIAL_MOTION_CONTROLLER_H */ 