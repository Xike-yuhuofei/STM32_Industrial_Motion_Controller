/**
 * @file    gcode_executor.c
 * @brief   G代码执行器实现
 * @date    2024-01-20
 * @author  STM32 G代码执行系统
 */

#include "gcode_parser.h"
#include "motion_control.h"
#include <stdio.h>
#include <string.h>

/* 私有变量 */
static char s_response_buffer[256];

/* 私有函数声明 */
static GCode_Error_t execute_rapid_move(const GCode_Instruction_t* instruction);
static GCode_Error_t execute_linear_move(const GCode_Instruction_t* instruction);
static GCode_Error_t execute_arc_move(const GCode_Instruction_t* instruction, bool clockwise);
static GCode_Error_t execute_dwell(const GCode_Instruction_t* instruction);
static GCode_Error_t execute_home(const GCode_Instruction_t* instruction);
static void send_ok_response(void);
static void send_error_response(GCode_Error_t error, const char* message);
static void send_position_response(void);

/**
 * @brief  G代码指令执行回调函数
 * @param  instruction: 要执行的指令
 * @retval GCode_Error_t: 执行结果
 */
GCode_Error_t GCode_ExecutorCallback(const GCode_Instruction_t* instruction)
{
    if (!instruction || !instruction->valid) {
        return GCODE_ERROR_INVALID_PARAMETER;
    }
    
    GCode_Error_t result = GCODE_OK;
    
    switch (instruction->command) {
        /* 运动指令 */
        case GCODE_G00:  // 快速定位
            result = execute_rapid_move(instruction);
            break;
            
        case GCODE_G01:  // 直线插补
            result = execute_linear_move(instruction);
            break;
            
        case GCODE_G02:  // 顺时针圆弧插补
            result = execute_arc_move(instruction, true);
            break;
            
        case GCODE_G03:  // 逆时针圆弧插补
            result = execute_arc_move(instruction, false);
            break;
            
        case GCODE_G04:  // 延时暂停
            result = execute_dwell(instruction);
            break;
            
        case GCODE_G28:  // 回零点
            result = execute_home(instruction);
            break;
            
        /* M代码 */
        case GCODE_M00:  // 程序暂停
        case GCODE_M01:  // 可选停止
            Motion_Stop();
            break;
            
        case GCODE_M02:  // 程序结束
        case GCODE_M30:  // 程序结束并返回
            Motion_Stop();
            GCode_Reset();
            break;
            
        case GCODE_M03:  // 主轴正转
        case GCODE_M04:  // 主轴反转  
        case GCODE_M05:  // 主轴停止
            // 当前项目只有步进电机，主轴控制暂时忽略
            break;
            
        default:
            result = GCODE_ERROR_INVALID_COMMAND;
            break;
    }
    
    /* 发送响应 */
    if (result == GCODE_OK) {
        send_ok_response();
    }
    
    return result;
}

/**
 * @brief  G代码错误处理回调函数
 * @param  error: 错误代码
 * @param  line_number: 行号
 * @param  message: 错误消息
 */
void GCode_ErrorCallback(GCode_Error_t error, uint32_t line_number, const char* message)
{
    send_error_response(error, message);
}

/**
 * @brief  执行快速移动
 */
static GCode_Error_t execute_rapid_move(const GCode_Instruction_t* instruction)
{
    float target_x, target_y, target_z;
    
    /* 获取目标坐标 */
    if (!GCode_ConvertCoordinates(instruction, &target_x, &target_y, &target_z)) {
        return GCODE_ERROR_MISSING_PARAMETER;
    }
    
    /* 目前只支持A轴（步进电机），将X坐标映射到A轴 */
    float target_positions[MAX_AXES] = {0};
    
    if (GCode_GetParameter(instruction, GCODE_PARAM_X, &target_positions[AXIS_A])) {
        /* 执行快速移动 */
        Motion_RapidMove(target_positions);
        
        /* 更新当前位置 */
        GCode_SetCurrentPosition(target_positions[AXIS_A], 0, 0);
    }
    
    return GCODE_OK;
}

/**
 * @brief  执行直线插补
 */
static GCode_Error_t execute_linear_move(const GCode_Instruction_t* instruction)
{
    float target_x, target_y, target_z;
    float feed_rate;
    
    /* 获取目标坐标 */
    if (!GCode_ConvertCoordinates(instruction, &target_x, &target_y, &target_z)) {
        return GCODE_ERROR_MISSING_PARAMETER;
    }
    
    /* 获取进给速度 */
    if (!GCode_GetParameter(instruction, GCODE_PARAM_F, &feed_rate)) {
        /* 使用当前进给速度 */
        feed_rate = g_gcode_parser.feed_rate;
    }
    
    /* 目前只支持A轴（步进电机） */
    float target_positions[MAX_AXES] = {0};
    
    if (GCode_GetParameter(instruction, GCODE_PARAM_X, &target_positions[AXIS_A])) {
        /* 执行直线运动 */
        Motion_LinearMove(target_positions, feed_rate);
        
        /* 更新当前位置 */
        GCode_SetCurrentPosition(target_positions[AXIS_A], 0, 0);
    }
    
    return GCODE_OK;
}

/**
 * @brief  执行圆弧插补
 */
static GCode_Error_t execute_arc_move(const GCode_Instruction_t* instruction, bool clockwise)
{
    /* 圆弧插补需要多轴支持，当前项目只有单轴步进电机 */
    /* 暂时不支持圆弧插补，返回成功但不执行 */
    return GCODE_OK;
}

/**
 * @brief  执行延时
 */
static GCode_Error_t execute_dwell(const GCode_Instruction_t* instruction)
{
    float dwell_time;
    
    if (!GCode_GetParameter(instruction, GCODE_PARAM_P, &dwell_time)) {
        return GCODE_ERROR_MISSING_PARAMETER;
    }
    
    /* 执行延时（毫秒） */
    HAL_Delay((uint32_t)(dwell_time * 1000));
    
    return GCODE_OK;
}

/**
 * @brief  执行回零
 */
static GCode_Error_t execute_home(const GCode_Instruction_t* instruction)
{
    /* 检查是否指定了特定轴 */
    if (GCode_GetParameter(instruction, GCODE_PARAM_X, NULL)) {
        Motion_Home(AXIS_A);  // X映射到A轴
    } else {
        /* 未指定轴，回零所有轴 */
        Motion_HomeAll();
    }
    
    /* 复位当前位置 */
    GCode_SetCurrentPosition(0, 0, 0);
    
    return GCODE_OK;
}

/**
 * @brief  发送OK响应
 */
static void send_ok_response(void)
{
    Communication_SendResponse("ok\r\n");
}

/**
 * @brief  发送错误响应
 */
static void send_error_response(GCode_Error_t error, const char* message)
{
    snprintf(s_response_buffer, sizeof(s_response_buffer), 
             "error:%d %s\r\n", error, message ? message : "");
    Communication_SendResponse(s_response_buffer);
}

/**
 * @brief  发送位置响应
 */
static void send_position_response(void)
{
    float x, y, z;
    GCode_GetCurrentPosition(&x, &y, &z);
    
    snprintf(s_response_buffer, sizeof(s_response_buffer),
             "X:%.3f Y:%.3f Z:%.3f A:%.3f\r\n", x, y, z, x);
    Communication_SendResponse(s_response_buffer);
}

/**
 * @brief  初始化G代码执行器
 */
void GCode_ExecutorInit(void)
{
    /* 初始化G代码解析器 */
    GCode_Init(GCode_ExecutorCallback, GCode_ErrorCallback);
    
    /* 设置默认状态 */
    GCode_SetCurrentPosition(0.0f, 0.0f, 0.0f);
}

/**
 * @brief  处理串口接收的G代码
 * @param  line: 接收到的G代码行
 */
void GCode_ProcessSerialLine(const char* line)
{
    if (!line) return;
    
    GCode_Instruction_t instruction;
    GCode_Error_t result = GCode_ParseLine(line, &instruction);
    
    if (result == GCODE_OK && instruction.valid) {
        result = GCode_ExecuteInstruction(&instruction);
    }
    
    if (result != GCODE_OK) {
        char error_msg[128];
        GCode_FormatError(result, error_msg, sizeof(error_msg));
        send_error_response(result, error_msg);
    }
}

/**
 * @brief  获取系统状态报告
 */
void GCode_SendStatusReport(void)
{
    float x, y, z;
    uint32_t line_count, error_count;
    
    GCode_GetCurrentPosition(&x, &y, &z);
    GCode_GetStats(&line_count, &error_count);
    
    snprintf(s_response_buffer, sizeof(s_response_buffer),
             "<Idle|MPos:%.3f,%.3f,%.3f|FS:%.1f|Ln:%lu|Err:%lu>\r\n",
             x, y, z, g_gcode_parser.feed_rate, line_count, error_count);
    
    Communication_SendResponse(s_response_buffer);
} 