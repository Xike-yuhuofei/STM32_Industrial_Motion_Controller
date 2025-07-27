/**
  ******************************************************************************
  * @file    error_handler.c
  * @brief   统一错误处理实现
  * @author  Claude AI
  * @version 1.0
  * @date    2025-01-27
  ******************************************************************************
  */

#include "error_codes.h"
#include "main.h"
#include <string.h>
#include <stdio.h>

/* 私有变量 -------------------------------------------------------------------*/
static ErrorStatistics_t g_error_stats = {0};
static ErrorInfo_t g_error_history[32];
static uint8_t g_error_history_index = 0;
static uint8_t g_error_history_count = 0;

/* 错误字符串映射表 -----------------------------------------------------------*/
static const struct {
    MotionControlResult_t code;
    const char* description;
} error_string_map[] = {
    {MC_OK, "操作成功"},
    {MC_ERROR, "通用错误"},
    {MC_ERROR_INVALID_PARAM, "无效参数"},
    {MC_ERROR_NULL_POINTER, "空指针错误"},
    {MC_ERROR_TIMEOUT, "超时错误"},
    {MC_ERROR_BUSY, "设备忙"},
    {MC_ERROR_NOT_READY, "设备未就绪"},
    {MC_ERROR_NOT_SUPPORTED, "功能不支持"},
    {MC_ERROR_INSUFFICIENT_MEMORY, "内存不足"},
    {MC_ERROR_BUFFER_OVERFLOW, "缓冲区溢出"},
    {MC_ERROR_CHECKSUM, "校验和错误"},
    
    {MC_ERROR_HAL, "HAL库错误"},
    {MC_ERROR_GPIO, "GPIO错误"},
    {MC_ERROR_TIMER, "定时器错误"},
    {MC_ERROR_UART, "串口错误"},
    
    {MC_ERROR_MOTION, "运动控制错误"},
    {MC_ERROR_AXIS_INVALID, "无效轴号"},
    {MC_ERROR_AXIS_NOT_HOMED, "轴未回零"},
    {MC_ERROR_POSITION_LIMIT, "位置超限"},
    {MC_ERROR_SPEED_LIMIT, "速度超限"},
    {MC_ERROR_ACCELERATION_LIMIT, "加速度超限"},
    {MC_ERROR_FOLLOWING_ERROR, "跟随误差过大"},
    {MC_ERROR_MOTOR_FAULT, "电机故障"},
    {MC_ERROR_ENCODER_FAULT, "编码器故障"},
    {MC_ERROR_EMERGENCY_STOP, "急停状态"},
    {MC_ERROR_MOTION_PLANNING, "运动规划错误"},
    
    {MC_ERROR_ALGORITHM, "算法错误"},
    {MC_ERROR_INTERPOLATION, "插补算法错误"},
    {MC_ERROR_VELOCITY_PLANNING, "速度规划错误"},
    {MC_ERROR_TRAJECTORY_PLANNING, "轨迹规划错误"},
    {MC_ERROR_PID_PARAMETER, "PID参数错误"},
    {MC_ERROR_MATH_OVERFLOW, "数学运算溢出"},
    {MC_ERROR_CONVERGENCE, "算法不收敛"},
    {MC_ERROR_SINGULARITY, "奇异点错误"},
    
    {MC_ERROR_COMMUNICATION, "通信错误"},
    {MC_ERROR_PROTOCOL, "协议错误"},
    {MC_ERROR_FRAME_FORMAT, "帧格式错误"},
    {MC_ERROR_CRC, "CRC校验错误"},
    {MC_ERROR_SEQUENCE, "序列号错误"},
    {MC_ERROR_RESPONSE_TIMEOUT, "响应超时"},
    {MC_ERROR_CONNECTION_LOST, "连接丢失"},
    {MC_ERROR_BUFFER_FULL, "缓冲区满"},
    {MC_ERROR_PARSE_GCODE, "G-code解析错误"},
    
    {MC_ERROR_DIAGNOSIS, "故障诊断错误"},
    {MC_ERROR_SENSOR_FAULT, "传感器故障"},
    {MC_ERROR_TEMPERATURE_HIGH, "温度过高"},
    {MC_ERROR_CURRENT_HIGH, "电流过大"},
    {MC_ERROR_VIBRATION_HIGH, "振动过大"},
    {MC_ERROR_BEARING_FAULT, "轴承故障"},
    {MC_ERROR_LUBRICATION, "润滑故障"},
    {MC_ERROR_WEAR_LIMIT, "磨损超限"},
    
    {MC_ERROR_CONFIG, "配置错误"},
    {MC_ERROR_CONFIG_INVALID, "无效配置"},
    {MC_ERROR_CONFIG_VERSION, "配置版本不匹配"},
    {MC_ERROR_EEPROM, "EEPROM错误"},
    {MC_ERROR_FACTORY_RESET, "出厂设置重置失败"},
    
    {MC_ERROR_SYSTEM, "系统错误"},
    {MC_ERROR_WATCHDOG, "看门狗错误"},
    {MC_ERROR_STACK_OVERFLOW, "栈溢出"},
    {MC_ERROR_HEAP_CORRUPT, "堆损坏"},
    {MC_ERROR_TASK_CREATE, "任务创建失败"},
    {MC_ERROR_SEMAPHORE, "信号量错误"},
    {MC_ERROR_QUEUE_FULL, "队列满"},
    {MC_ERROR_MUTEX_TIMEOUT, "互斥锁超时"},
};

static const char* severity_strings[] = {
    "INFO",
    "WARNING", 
    "ERROR",
    "CRITICAL",
    "FATAL"
};

/**
  * @brief  获取错误码对应的字符串描述
  * @param  error_code: 错误码
  * @retval 错误描述字符串
  */
const char* Error_GetString(MotionControlResult_t error_code)
{
    for (size_t i = 0; i < sizeof(error_string_map) / sizeof(error_string_map[0]); i++) {
        if (error_string_map[i].code == error_code) {
            return error_string_map[i].description;
        }
    }
    return "未知错误";
}

/**
  * @brief  获取错误严重级别字符串
  * @param  severity: 严重级别
  * @retval 严重级别字符串
  */
const char* Error_GetSeverityString(ErrorSeverity_t severity)
{
    if (severity <= MC_SEVERITY_FATAL) {
        return severity_strings[severity];
    }
    return "UNKNOWN";
}

/**
  * @brief  记录错误信息
  * @param  code: 错误码
  * @param  severity: 严重级别
  * @param  file: 文件名
  * @param  line: 行号
  * @param  function: 函数名
  * @param  description: 错误描述
  * @retval None
  */
void Error_Log(MotionControlResult_t code, ErrorSeverity_t severity, 
               const char* file, uint16_t line, const char* function, 
               const char* description)
{
    /* 更新统计信息 */
    g_error_stats.total_errors++;
    if (code <= MC_ERROR_MAX) {
        g_error_stats.error_count_by_code[code]++;
    }
    if (severity <= MC_SEVERITY_FATAL) {
        g_error_stats.error_count_by_severity[severity]++;
    }
    g_error_stats.last_error_code = code;
    g_error_stats.last_error_time = HAL_GetTick();
    
    /* 保存到历史记录 */
    ErrorInfo_t* error_info = &g_error_history[g_error_history_index];
    error_info->code = code;
    error_info->severity = severity;
    error_info->timestamp = HAL_GetTick();
    error_info->line_number = line;
    error_info->file_name = file;
    error_info->function_name = function;
    error_info->description = description;
    
    g_error_history_index = (g_error_history_index + 1) % 32;
    if (g_error_history_count < 32) {
        g_error_history_count++;
    }
    
    /* 格式化输出错误信息 */
    printf("[%s][%u] %s:%d %s() - %s (%s)\n",
           Error_GetSeverityString(severity),
           (unsigned int)HAL_GetTick(),
           file, line, function,
           Error_GetString(code),
           description ? description : "");
    
    /* 处理致命错误 */
    if (severity == MC_SEVERITY_FATAL) {
        Error_Handler_Custom(error_info);
    }
}

/**
  * @brief  自定义错误处理函数
  * @param  error_info: 错误信息
  * @retval None
  */
void Error_Handler_Custom(ErrorInfo_t* error_info)
{
    /* 禁用中断 */
    __disable_irq();
    
    /* 记录崩溃信息到EEPROM或Flash */
    // TODO: 保存错误信息到非易失性存储器
    
    /* 尝试安全关闭 */
    // TODO: 关闭所有电机输出
    // TODO: 保存重要数据
    
    printf("\n=== FATAL ERROR ===\n");
    printf("Code: %d (%s)\n", error_info->code, Error_GetString(error_info->code));
    printf("File: %s:%d\n", error_info->file_name, error_info->line_number);
    printf("Function: %s\n", error_info->function_name);
    printf("Description: %s\n", error_info->description);
    printf("Time: %u ms\n", (unsigned int)error_info->timestamp);
    printf("System will halt!\n");
    
    /* 进入死循环 */
    while (1) {
        /* 可以添加看门狗喂狗或LED指示 */
        HAL_Delay(1000);
    }
}

/**
  * @brief  转换HAL状态码为统一错误码
  * @param  hal_status: HAL状态码
  * @retval 统一错误码
  */
MotionControlResult_t Error_Convert_HAL(HAL_StatusTypeDef hal_status)
{
    switch (hal_status) {
        case HAL_OK:      return MC_OK;
        case HAL_ERROR:   return MC_ERROR_HAL;
        case HAL_BUSY:    return MC_ERROR_BUSY;
        case HAL_TIMEOUT: return MC_ERROR_TIMEOUT;
        default:          return MC_ERROR;
    }
}

/**
  * @brief  获取错误统计信息
  * @param  stats: 统计信息结构指针
  * @retval None
  */
void Error_GetStatistics(ErrorStatistics_t* stats)
{
    if (stats != NULL) {
        memcpy(stats, &g_error_stats, sizeof(ErrorStatistics_t));
    }
}

/**
  * @brief  清除错误统计信息
  * @retval None
  */
void Error_ClearStatistics(void)
{
    memset(&g_error_stats, 0, sizeof(ErrorStatistics_t));
    memset(g_error_history, 0, sizeof(g_error_history));
    g_error_history_index = 0;
    g_error_history_count = 0;
}

/**
  * @brief  打印错误历史记录
  * @retval None
  */
void Error_PrintHistory(void)
{
    printf("\n=== Error History ===\n");
    printf("Total errors: %u\n", (unsigned int)g_error_stats.total_errors);
    
    if (g_error_history_count == 0) {
        printf("No errors recorded.\n");
        return;
    }
    
    printf("Recent errors:\n");
    for (uint8_t i = 0; i < g_error_history_count; i++) {
        uint8_t index = (g_error_history_index - 1 - i + 32) % 32;
        ErrorInfo_t* error = &g_error_history[index];
        
        printf("%2d: [%s][%u] %s - %s\n",
               i + 1,
               Error_GetSeverityString(error->severity),
               (unsigned int)error->timestamp,
               Error_GetString(error->code),
               error->description ? error->description : "");
    }
    printf("===================\n");
}

/**
  * @brief  检查系统健康状态
  * @retval 系统健康状态
  */
MotionControlResult_t Error_CheckSystemHealth(void)
{
    /* 检查最近的致命错误 */
    if (g_error_stats.error_count_by_severity[MC_SEVERITY_FATAL] > 0) {
        return MC_ERROR_SYSTEM;
    }
    
    /* 检查错误率 */
    uint32_t total_errors = g_error_stats.total_errors;
    uint32_t uptime = HAL_GetTick();
    
    if (uptime > 60000 && total_errors > 0) { // 运行超过1分钟
        float error_rate = (float)total_errors / (uptime / 1000.0f); // 错误/秒
        if (error_rate > 0.1f) { // 错误率超过0.1个/秒
            return MC_ERROR_SYSTEM;
        }
    }
    
    return MC_OK;
}