/**
 * @file    gcode_parser.h
 * @brief   G代码解析器头文件
 * @date    2024-01-20
 * @author  STM32 G代码解析系统
 */

#ifndef __GCODE_PARSER_H
#define __GCODE_PARSER_H

#include "stm32f4xx_hal.h"
#include "build_config.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* G代码解析器配置 */
#define GCODE_MAX_LINE_LENGTH       128     // 最大G代码行长度
#define GCODE_MAX_ARGS              16      // 最大参数数量
#ifndef GCODE_BUFFER_SIZE
#define GCODE_BUFFER_SIZE           256     // G代码缓冲区大小（如果未在build_config.h定义）
#endif

/* 常用G代码指令定义 */
typedef enum {
    /* 运动指令 */
    GCODE_G00 = 0,      // 快速定位
    GCODE_G01 = 1,      // 直线插补
    GCODE_G02 = 2,      // 顺时针圆弧插补
    GCODE_G03 = 3,      // 逆时针圆弧插补
    GCODE_G04 = 4,      // 延时暂停
    
    /* 设置指令 */
    GCODE_G20 = 20,     // 设置英制单位
    GCODE_G21 = 21,     // 设置公制单位
    GCODE_G28 = 28,     // 回零点
    GCODE_G90 = 90,     // 绝对坐标
    GCODE_G91 = 91,     // 相对坐标
    GCODE_G92 = 92,     // 设置坐标系
    
    /* M代码 */
    GCODE_M00 = 1000,   // 程序暂停
    GCODE_M01 = 1001,   // 可选停止
    GCODE_M02 = 1002,   // 程序结束
    GCODE_M03 = 1003,   // 主轴正转
    GCODE_M04 = 1004,   // 主轴反转
    GCODE_M05 = 1005,   // 主轴停止
    GCODE_M30 = 1030,   // 程序结束并返回
    
    GCODE_UNKNOWN = 9999 // 未知指令
} GCode_Command_t;

/* G代码参数类型 */
typedef enum {
    GCODE_PARAM_X = 'X',    // X轴坐标
    GCODE_PARAM_Y = 'Y',    // Y轴坐标
    GCODE_PARAM_Z = 'Z',    // Z轴坐标
    GCODE_PARAM_F = 'F',    // 进给速度
    GCODE_PARAM_S = 'S',    // 主轴转速
    GCODE_PARAM_P = 'P',    // 暂停时间
    GCODE_PARAM_I = 'I',    // 圆弧中心X偏移
    GCODE_PARAM_J = 'J',    // 圆弧中心Y偏移
    GCODE_PARAM_R = 'R',    // 圆弧半径
} GCode_Param_t;

/* G代码参数结构 */
typedef struct {
    GCode_Param_t type;     // 参数类型
    float value;            // 参数值
    bool present;           // 参数是否存在
} GCode_Argument_t;

/* G代码指令结构 */
typedef struct {
    GCode_Command_t command;                        // 主指令
    GCode_Argument_t args[GCODE_MAX_ARGS];         // 参数数组
    uint8_t arg_count;                             // 参数数量
    uint32_t line_number;                          // 行号
    bool valid;                                    // 指令是否有效
} GCode_Instruction_t;

/* 坐标系统 */
typedef enum {
    COORD_ABSOLUTE = 0,     // 绝对坐标
    COORD_RELATIVE = 1      // 相对坐标
} Coordinate_Mode_t;

/* 单位系统 */
typedef enum {
    UNIT_MM = 0,           // 毫米
    UNIT_INCH = 1          // 英寸
} Unit_Mode_t;

/* G代码解析器状态 */
typedef struct {
    /* 坐标系统状态 */
    Coordinate_Mode_t coord_mode;   // 坐标模式
    Unit_Mode_t unit_mode;          // 单位模式
    float current_x;                // 当前X坐标
    float current_y;                // 当前Y坐标
    float current_z;                // 当前Z坐标
    float feed_rate;                // 当前进给速度
    float spindle_speed;            // 主轴转速
    
    /* 解析状态 */
    bool initialized;               // 是否已初始化
    uint32_t line_count;           // 处理的行数
    uint32_t error_count;          // 错误计数
    
    /* 缓冲区 */
    char line_buffer[GCODE_MAX_LINE_LENGTH];    // 行缓冲区
    uint16_t buffer_pos;                        // 缓冲区位置
} GCode_Parser_t;

/* 错误代码 */
typedef enum {
    GCODE_OK = 0,                   // 成功
    GCODE_ERROR_INVALID_COMMAND,    // 无效指令
    GCODE_ERROR_INVALID_PARAMETER,  // 无效参数
    GCODE_ERROR_MISSING_PARAMETER,  // 缺少参数
    GCODE_ERROR_OUT_OF_RANGE,       // 参数超出范围
    GCODE_ERROR_SYNTAX,             // 语法错误
    GCODE_ERROR_BUFFER_FULL,        // 缓冲区满
    GCODE_ERROR_NOT_INITIALIZED     // 未初始化
} GCode_Error_t;

/* 回调函数类型定义 */
typedef GCode_Error_t (*GCode_ExecuteCallback_t)(const GCode_Instruction_t* instruction);
typedef void (*GCode_ErrorCallback_t)(GCode_Error_t error, uint32_t line_number, const char* message);

/* 全局解析器实例 */
extern GCode_Parser_t g_gcode_parser;

/* 函数声明 */

/**
 * @brief  初始化G代码解析器
 * @param  execute_callback: 指令执行回调函数
 * @param  error_callback: 错误处理回调函数
 * @retval GCode_Error_t: 错误代码
 */
GCode_Error_t GCode_Init(GCode_ExecuteCallback_t execute_callback, 
                        GCode_ErrorCallback_t error_callback);

/**
 * @brief  解析G代码行
 * @param  line: G代码字符串
 * @param  instruction: 输出的指令结构
 * @retval GCode_Error_t: 错误代码
 */
GCode_Error_t GCode_ParseLine(const char* line, GCode_Instruction_t* instruction);

/**
 * @brief  执行G代码指令
 * @param  instruction: 要执行的指令
 * @retval GCode_Error_t: 错误代码
 */
GCode_Error_t GCode_ExecuteInstruction(const GCode_Instruction_t* instruction);

/**
 * @brief  处理字符流（逐字符添加）
 * @param  ch: 输入字符
 * @retval GCode_Error_t: 错误代码
 */
GCode_Error_t GCode_ProcessChar(char ch);

/**
 * @brief  获取参数值
 * @param  instruction: 指令结构
 * @param  param_type: 参数类型
 * @param  value: 输出参数值
 * @retval bool: 是否找到参数
 */
bool GCode_GetParameter(const GCode_Instruction_t* instruction, 
                       GCode_Param_t param_type, 
                       float* value);

/**
 * @brief  设置当前坐标
 * @param  x: X坐标
 * @param  y: Y坐标
 * @param  z: Z坐标
 */
void GCode_SetCurrentPosition(float x, float y, float z);

/**
 * @brief  获取当前坐标
 * @param  x: 输出X坐标
 * @param  y: 输出Y坐标
 * @param  z: 输出Z坐标
 */
void GCode_GetCurrentPosition(float* x, float* y, float* z);

/**
 * @brief  重置解析器状态
 */
void GCode_Reset(void);

/**
 * @brief  获取解析器统计信息
 * @param  line_count: 输出行计数
 * @param  error_count: 输出错误计数
 */
void GCode_GetStats(uint32_t* line_count, uint32_t* error_count);

/**
 * @brief  坐标转换（相对/绝对）
 * @param  instruction: 指令结构
 * @param  target_x: 输出目标X坐标
 * @param  target_y: 输出目标Y坐标
 * @param  target_z: 输出目标Z坐标
 * @retval bool: 转换是否成功
 */
bool GCode_ConvertCoordinates(const GCode_Instruction_t* instruction,
                             float* target_x, float* target_y, float* target_z);

/**
 * @brief  验证G代码指令有效性
 * @param  instruction: 指令结构
 * @retval GCode_Error_t: 验证结果
 */
GCode_Error_t GCode_ValidateInstruction(const GCode_Instruction_t* instruction);

/**
 * @brief  格式化错误信息
 * @param  error: 错误代码
 * @param  buffer: 输出缓冲区
 * @param  buffer_size: 缓冲区大小
 * @retval uint16_t: 实际写入长度
 */
uint16_t GCode_FormatError(GCode_Error_t error, char* buffer, uint16_t buffer_size);

#endif /* __GCODE_PARSER_H */ 