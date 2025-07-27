/**
  ******************************************************************************
  * @file    error_codes.h
  * @brief   统一错误码定义
  * @author  Claude AI
  * @version 1.0
  * @date    2025-01-27
  ******************************************************************************
  */

#ifndef __ERROR_CODES_H
#define __ERROR_CODES_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "stm32f4xx_hal.h"

/* 基础错误码类型 ---------------------------------------------------------------*/
typedef enum {
    /* 成功状态 */
    MC_OK = 0,                      // 操作成功
    
    /* 通用错误 1-99 */
    MC_ERROR = 1,                   // 通用错误
    MC_ERROR_INVALID_PARAM,         // 无效参数
    MC_ERROR_NULL_POINTER,          // 空指针错误
    MC_ERROR_TIMEOUT,               // 超时错误
    MC_ERROR_BUSY,                  // 设备忙
    MC_ERROR_NOT_READY,             // 设备未就绪
    MC_ERROR_NOT_SUPPORTED,         // 功能不支持
    MC_ERROR_INSUFFICIENT_MEMORY,   // 内存不足
    MC_ERROR_BUFFER_OVERFLOW,       // 缓冲区溢出
    MC_ERROR_CHECKSUM,              // 校验和错误
    
    /* 硬件相关错误 100-199 */
    MC_ERROR_HAL = 100,             // HAL库错误
    MC_ERROR_GPIO,                  // GPIO错误
    MC_ERROR_TIMER,                 // 定时器错误
    MC_ERROR_UART,                  // 串口错误
    MC_ERROR_I2C,                   // I2C错误
    MC_ERROR_SPI,                   // SPI错误
    MC_ERROR_PWM,                   // PWM错误
    MC_ERROR_ADC,                   // ADC错误
    MC_ERROR_DAC,                   // DAC错误
    
    /* 运动控制错误 200-299 */
    MC_ERROR_MOTION = 200,          // 运动控制通用错误
    MC_ERROR_AXIS_INVALID,          // 无效轴号
    MC_ERROR_AXIS_NOT_HOMED,        // 轴未回零
    MC_ERROR_POSITION_LIMIT,        // 位置超限
    MC_ERROR_SPEED_LIMIT,           // 速度超限
    MC_ERROR_ACCELERATION_LIMIT,    // 加速度超限
    MC_ERROR_FOLLOWING_ERROR,       // 跟随误差过大
    MC_ERROR_MOTOR_FAULT,           // 电机故障
    MC_ERROR_ENCODER_FAULT,         // 编码器故障
    MC_ERROR_EMERGENCY_STOP,        // 急停状态
    MC_ERROR_MOTION_PLANNING,       // 运动规划错误
    
    /* 算法相关错误 300-399 */
    MC_ERROR_ALGORITHM = 300,       // 算法通用错误
    MC_ERROR_INTERPOLATION,         // 插补算法错误
    MC_ERROR_VELOCITY_PLANNING,     // 速度规划错误
    MC_ERROR_TRAJECTORY_PLANNING,   // 轨迹规划错误
    MC_ERROR_PID_PARAMETER,         // PID参数错误
    MC_ERROR_MATH_OVERFLOW,         // 数学运算溢出
    MC_ERROR_CONVERGENCE,           // 算法不收敛
    MC_ERROR_SINGULARITY,           // 奇异点错误
    
    /* 通信相关错误 400-499 */
    MC_ERROR_COMMUNICATION = 400,   // 通信通用错误
    MC_ERROR_PROTOCOL,              // 协议错误
    MC_ERROR_FRAME_FORMAT,          // 帧格式错误
    MC_ERROR_CRC,                   // CRC校验错误
    MC_ERROR_SEQUENCE,              // 序列号错误
    MC_ERROR_RESPONSE_TIMEOUT,      // 响应超时
    MC_ERROR_CONNECTION_LOST,       // 连接丢失
    MC_ERROR_BUFFER_FULL,           // 缓冲区满
    MC_ERROR_PARSE_GCODE,           // G-code解析错误
    
    /* 故障诊断错误 500-599 */
    MC_ERROR_DIAGNOSIS = 500,       // 故障诊断错误
    MC_ERROR_SENSOR_FAULT,          // 传感器故障
    MC_ERROR_TEMPERATURE_HIGH,      // 温度过高
    MC_ERROR_CURRENT_HIGH,          // 电流过大
    MC_ERROR_VIBRATION_HIGH,        // 振动过大
    MC_ERROR_BEARING_FAULT,         // 轴承故障
    MC_ERROR_LUBRICATION,           // 润滑故障
    MC_ERROR_WEAR_LIMIT,            // 磨损超限
    
    /* 配置相关错误 600-699 */
    MC_ERROR_CONFIG = 600,          // 配置错误
    MC_ERROR_CONFIG_INVALID,        // 无效配置
    MC_ERROR_CONFIG_VERSION,        // 配置版本不匹配
    MC_ERROR_EEPROM,                // EEPROM错误
    MC_ERROR_FACTORY_RESET,         // 出厂设置重置失败
    
    /* 系统相关错误 700-799 */
    MC_ERROR_SYSTEM = 700,          // 系统错误
    MC_ERROR_WATCHDOG,              // 看门狗错误
    MC_ERROR_STACK_OVERFLOW,        // 栈溢出
    MC_ERROR_HEAP_CORRUPT,          // 堆损坏
    MC_ERROR_TASK_CREATE,           // 任务创建失败
    MC_ERROR_SEMAPHORE,             // 信号量错误
    MC_ERROR_QUEUE_FULL,            // 队列满
    MC_ERROR_MUTEX_TIMEOUT,         // 互斥锁超时
    
    /* 用户自定义错误 800-999 */
    MC_ERROR_USER_BASE = 800,       // 用户自定义错误基址
    
    /* 错误码上限 */
    MC_ERROR_MAX = 999
} MotionControlResult_t;

/* 错误严重级别 -----------------------------------------------------------------*/
typedef enum {
    MC_SEVERITY_INFO = 0,           // 信息级别
    MC_SEVERITY_WARNING,            // 警告级别
    MC_SEVERITY_ERROR,              // 错误级别
    MC_SEVERITY_CRITICAL,           // 严重级别
    MC_SEVERITY_FATAL               // 致命级别
} ErrorSeverity_t;

/* 错误信息结构 -----------------------------------------------------------------*/
typedef struct {
    MotionControlResult_t code;     // 错误码
    ErrorSeverity_t severity;       // 严重级别
    uint32_t timestamp;             // 时间戳
    uint8_t module_id;              // 模块ID
    uint16_t line_number;           // 行号
    const char* file_name;          // 文件名
    const char* function_name;      // 函数名
    const char* description;        // 错误描述
} ErrorInfo_t;

/* 错误处理宏定义 ---------------------------------------------------------------*/
#define MC_RETURN_IF_ERROR(expr) \
    do { \
        MotionControlResult_t __result = (expr); \
        if (__result != MC_OK) { \
            return __result; \
        } \
    } while(0)

#define MC_CHECK_PARAM(param) \
    do { \
        if ((param) == NULL) { \
            return MC_ERROR_NULL_POINTER; \
        } \
    } while(0)

#define MC_CHECK_RANGE(value, min, max) \
    do { \
        if ((value) < (min) || (value) > (max)) { \
            return MC_ERROR_INVALID_PARAM; \
        } \
    } while(0)

#define MC_CHECK_AXIS_ID(axis_id) \
    MC_CHECK_RANGE(axis_id, 0, MAX_AXES - 1)

#define MC_LOG_ERROR(code, severity, description) \
    Error_Log(code, severity, __FILE__, __LINE__, __FUNCTION__, description)

#define MC_ASSERT(condition) \
    do { \
        if (!(condition)) { \
            MC_LOG_ERROR(MC_ERROR, MC_SEVERITY_FATAL, "Assertion failed: " #condition); \
            while(1); \
        } \
    } while(0)

/* 函数声明 -------------------------------------------------------------------*/
const char* Error_GetString(MotionControlResult_t error_code);
const char* Error_GetSeverityString(ErrorSeverity_t severity);
void Error_Log(MotionControlResult_t code, ErrorSeverity_t severity, 
               const char* file, uint16_t line, const char* function, 
               const char* description);
void Error_Handler_Custom(ErrorInfo_t* error_info);
MotionControlResult_t Error_Convert_HAL(HAL_StatusTypeDef hal_status);

/* 错误统计 -------------------------------------------------------------------*/
typedef struct {
    uint32_t total_errors;
    uint32_t error_count_by_code[MC_ERROR_MAX + 1];
    uint32_t error_count_by_severity[MC_SEVERITY_FATAL + 1];
    uint32_t last_error_code;
    uint32_t last_error_time;
} ErrorStatistics_t;

void Error_GetStatistics(ErrorStatistics_t* stats);
void Error_ClearStatistics(void);

#ifdef __cplusplus
}
#endif

#endif /* __ERROR_CODES_H */