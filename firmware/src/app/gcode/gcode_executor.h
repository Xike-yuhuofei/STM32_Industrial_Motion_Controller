/**
 * @file    gcode_executor.h
 * @brief   G代码执行器头文件
 * @date    2024-01-20
 * @author  STM32 G代码执行系统
 */

#ifndef __GCODE_EXECUTOR_H
#define __GCODE_EXECUTOR_H

#include "gcode_parser.h"

/* 函数声明 */

/**
 * @brief  G代码指令执行回调函数
 * @param  instruction: 要执行的指令
 * @retval GCode_Error_t: 执行结果
 */
GCode_Error_t GCode_ExecutorCallback(const GCode_Instruction_t* instruction);

/**
 * @brief  G代码错误处理回调函数
 * @param  error: 错误代码
 * @param  line_number: 行号
 * @param  message: 错误消息
 */
void GCode_ErrorCallback(GCode_Error_t error, uint32_t line_number, const char* message);

/**
 * @brief  初始化G代码执行器
 */
void GCode_ExecutorInit(void);

/**
 * @brief  处理串口接收的G代码
 * @param  line: 接收到的G代码行
 */
void GCode_ProcessSerialLine(const char* line);

/**
 * @brief  获取系统状态报告
 */
void GCode_SendStatusReport(void);

/* 内部函数由源文件实现，此处无需声明 */

#endif /* __GCODE_EXECUTOR_H */ 