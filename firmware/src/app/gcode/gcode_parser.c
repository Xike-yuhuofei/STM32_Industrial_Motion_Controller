/**
  ******************************************************************************
  * @file    gcode_parser.c
  * @brief   G-code解析器实现
  * @author  Claude AI & Cursor
  * @version 1.0
  * @date    2025-01-27
  ******************************************************************************
  */

#include "gcode_parser.h"
#include "motion_control.h"
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <math.h>

/* 全局解析器实例 */
GCode_Parser_t g_gcode_parser = {0};

/* 静态回调函数指针 */
static GCode_ExecuteCallback_t s_execute_callback = NULL;
static GCode_ErrorCallback_t s_error_callback = NULL;

/* 静态函数声明 */
static GCode_Command_t parse_command(const char* str);
static bool parse_parameter(const char* str, GCode_Param_t* type, float* value);
static void trim_line(char* line);
static bool validate_parameter_range(GCode_Param_t type, float value);

/**
 * @brief  初始化G代码解析器
 */
GCode_Error_t GCode_Init(GCode_ExecuteCallback_t execute_callback, 
                        GCode_ErrorCallback_t error_callback)
{
    if (!execute_callback) {
        return GCODE_ERROR_INVALID_PARAMETER;
    }
    
    /* 清零解析器状态 */
    memset(&g_gcode_parser, 0, sizeof(GCode_Parser_t));
    
    /* 设置默认状态 */
    g_gcode_parser.coord_mode = COORD_ABSOLUTE;
    g_gcode_parser.unit_mode = UNIT_MM;
    g_gcode_parser.feed_rate = 100.0f;
    g_gcode_parser.initialized = true;
    
    /* 保存回调函数 */
    s_execute_callback = execute_callback;
    s_error_callback = error_callback;
    
    return GCODE_OK;
}

/**
 * @brief  解析G代码行
 */
GCode_Error_t GCode_ParseLine(const char* line, GCode_Instruction_t* instruction)
{
    if (!line || !instruction) {
        return GCODE_ERROR_INVALID_PARAMETER;
    }
    
    if (!g_gcode_parser.initialized) {
        return GCODE_ERROR_NOT_INITIALIZED;
    }
    
    /* 初始化指令结构 */
    memset(instruction, 0, sizeof(GCode_Instruction_t));
    instruction->line_number = ++g_gcode_parser.line_count;
    
    /* 复制并处理输入行 */
    char work_line[GCODE_MAX_LINE_LENGTH];
    strncpy(work_line, line, sizeof(work_line) - 1);
    work_line[sizeof(work_line) - 1] = '\0';
    
    /* 去除空白和注释 */
    trim_line(work_line);
    
    /* 空行直接返回 */
    if (strlen(work_line) == 0) {
        return GCODE_OK;
    }
    
    /* 解析主指令 */
    char* ptr = work_line;
    
    /* 跳过行号（如果存在） */
    if (*ptr == 'N') {
        while (*ptr && *ptr != ' ' && *ptr != '\t') ptr++;
        while (*ptr == ' ' || *ptr == '\t') ptr++;
    }
    
    /* 解析G/M指令 */
    if (*ptr == 'G' || *ptr == 'g') {
        instruction->command = parse_command(ptr);
    } else if (*ptr == 'M' || *ptr == 'm') {
        instruction->command = parse_command(ptr);
    } else {
        instruction->command = GCODE_UNKNOWN;
        return GCODE_ERROR_INVALID_COMMAND;
    }
    
    /* 跳过指令部分 */
    while (*ptr && !isspace(*ptr)) ptr++;
    
    /* 解析参数 */
    while (*ptr) {
        /* 跳过空白字符 */
        while (*ptr == ' ' || *ptr == '\t') ptr++;
        
        if (*ptr == '\0') break;
        
        /* 解析参数 */
        GCode_Param_t param_type;
        float param_value;
        
        if (parse_parameter(ptr, &param_type, &param_value)) {
            if (instruction->arg_count < GCODE_MAX_ARGS) {
                instruction->args[instruction->arg_count].type = param_type;
                instruction->args[instruction->arg_count].value = param_value;
                instruction->args[instruction->arg_count].present = true;
                instruction->arg_count++;
            } else {
                return GCODE_ERROR_SYNTAX;
            }
        }
        
        /* 跳到下一个参数 */
        while (*ptr && !isspace(*ptr)) ptr++;
    }
    
    /* 验证指令 */
    GCode_Error_t result = GCode_ValidateInstruction(instruction);
    if (result == GCODE_OK) {
        instruction->valid = true;
    }
    
    return result;
}

/**
 * @brief  获取参数值
 */
bool GCode_GetParameter(const GCode_Instruction_t* instruction, 
                       GCode_Param_t param_type, 
                       float* value)
{
    if (!instruction || !value) {
        return false;
    }
    
    for (uint8_t i = 0; i < instruction->arg_count; i++) {
        if (instruction->args[i].present && instruction->args[i].type == param_type) {
            *value = instruction->args[i].value;
            return true;
        }
    }
    
    return false;
}

/**
 * @brief  验证G代码指令有效性
 */
GCode_Error_t GCode_ValidateInstruction(const GCode_Instruction_t* instruction)
{
    if (!instruction) {
        return GCODE_ERROR_INVALID_PARAMETER;
    }
    
    /* 检查指令是否有效 */
    if (instruction->command == GCODE_UNKNOWN) {
        return GCODE_ERROR_INVALID_COMMAND;
    }
    
    return GCODE_OK;
}

/**
 * @brief  格式化错误信息
 */
uint16_t GCode_FormatError(GCode_Error_t error, char* buffer, uint16_t buffer_size)
{
    if (!buffer || buffer_size == 0) {
        return 0;
    }
    
    const char* error_msg;
    
    switch (error) {
        case GCODE_OK:
            error_msg = "成功";
            break;
        case GCODE_ERROR_INVALID_COMMAND:
            error_msg = "无效指令";
            break;
        case GCODE_ERROR_INVALID_PARAMETER:
            error_msg = "无效参数";
            break;
        default:
            error_msg = "未知错误";
            break;
    }
    
    return snprintf(buffer, buffer_size, "错误 %d: %s", error, error_msg);
}

/* === 静态函数实现 === */

/**
 * @brief  解析指令
 */
static GCode_Command_t parse_command(const char* str)
{
    if (!str) return GCODE_UNKNOWN;
    
    char cmd_char = toupper(str[0]);
    int cmd_num = atoi(&str[1]);
    
    if (cmd_char == 'G') {
        switch (cmd_num) {
            case 0: return GCODE_G00;
            case 1: return GCODE_G01;
            case 2: return GCODE_G02;
            case 3: return GCODE_G03;
            case 4: return GCODE_G04;
            case 20: return GCODE_G20;
            case 21: return GCODE_G21;
            case 28: return GCODE_G28;
            case 90: return GCODE_G90;
            case 91: return GCODE_G91;
            case 92: return GCODE_G92;
        }
    } else if (cmd_char == 'M') {
        switch (cmd_num) {
            case 0: return GCODE_M00;
            case 1: return GCODE_M01;
            case 2: return GCODE_M02;
            case 3: return GCODE_M03;
            case 4: return GCODE_M04;
            case 5: return GCODE_M05;
            case 30: return GCODE_M30;
        }
    }
    
    return GCODE_UNKNOWN;
}

/**
 * @brief  解析参数
 */
static bool parse_parameter(const char* str, GCode_Param_t* type, float* value)
{
    if (!str || !type || !value) return false;
    
    char param_char = toupper(str[0]);
    
    /* 检查是否是有效的参数字符 */
    if (!strchr("XYZFSPIJR", param_char)) {
        return false;
    }
    
    *type = (GCode_Param_t)param_char;
    
    /* 解析数值 */
    char* endptr;
    *value = strtof(&str[1], &endptr);
    
    return (endptr != &str[1]);  // 确保至少解析了一些数字
}

/**
 * @brief  清理行内容
 */
static void trim_line(char* line)
{
    if (!line) return;
    
    /* 移除注释 */
    char* comment = strchr(line, ';');
    if (comment) {
        *comment = '\0';
    }
    
    /* 移除末尾空白 */
    int len = strlen(line);
    while (len > 0 && isspace(line[len - 1])) {
        line[--len] = '\0';
    }
    
    /* 移除开头空白 */
    char* start = line;
    while (*start && isspace(*start)) start++;
    
    if (start != line) {
        memmove(line, start, strlen(start) + 1);
    }
    
    /* 转换为大写（除了注释） */
    for (int i = 0; line[i]; i++) {
        if (line[i] != ';') {
            line[i] = toupper(line[i]);
        } else {
            break;
        }
    }
}

/**
 * @brief  验证参数范围
 */
static bool validate_parameter_range(GCode_Param_t type, float value)
{
    switch (type) {
        case GCODE_PARAM_F:  // 进给速度
            return (value >= 0 && value <= 10000.0f);
            
        case GCODE_PARAM_S:  // 主轴转速
            return (value >= 0 && value <= 30000.0f);
            
        default:
            return true;
    }
}

/**
 * @brief  执行G代码指令
 */
GCode_Error_t GCode_ExecuteInstruction(const GCode_Instruction_t* instruction)
{
    if (!instruction || !instruction->valid) {
        return GCODE_ERROR_INVALID_PARAMETER;
    }
    
    if (!g_gcode_parser.initialized) {
        return GCODE_ERROR_NOT_INITIALIZED;
    }
    
    /* 处理设置类指令（直接在解析器中处理） */
    switch (instruction->command) {
        case GCODE_G20:  // 英制单位
            g_gcode_parser.unit_mode = UNIT_INCH;
            break;
            
        case GCODE_G21:  // 公制单位
            g_gcode_parser.unit_mode = UNIT_MM;
            break;
            
        case GCODE_G90:  // 绝对坐标
            g_gcode_parser.coord_mode = COORD_ABSOLUTE;
            break;
            
        case GCODE_G91:  // 相对坐标
            g_gcode_parser.coord_mode = COORD_RELATIVE;
            break;
            
        case GCODE_G92:  // 设置坐标系
        {
            float x, y, z;
            if (GCode_GetParameter(instruction, GCODE_PARAM_X, &x)) {
                g_gcode_parser.current_x = x;
            }
            if (GCode_GetParameter(instruction, GCODE_PARAM_Y, &y)) {
                g_gcode_parser.current_y = y;
            }
            if (GCode_GetParameter(instruction, GCODE_PARAM_Z, &z)) {
                g_gcode_parser.current_z = z;
            }
            break;
        }
        
        default:
            /* 其他指令通过回调函数处理 */
            if (s_execute_callback) {
                return s_execute_callback(instruction);
            }
            break;
    }
    
    /* 更新进给速度 */
    float feed_rate;
    if (GCode_GetParameter(instruction, GCODE_PARAM_F, &feed_rate)) {
        if (feed_rate > 0) {
            g_gcode_parser.feed_rate = feed_rate;
        }
    }
    
    /* 更新主轴转速 */
    float spindle_speed;
    if (GCode_GetParameter(instruction, GCODE_PARAM_S, &spindle_speed)) {
        if (spindle_speed >= 0) {
            g_gcode_parser.spindle_speed = spindle_speed;
        }
    }
    
    return GCODE_OK;
}

/**
 * @brief  处理字符流
 */
GCode_Error_t GCode_ProcessChar(char ch)
{
    if (!g_gcode_parser.initialized) {
        return GCODE_ERROR_NOT_INITIALIZED;
    }
    
    /* 处理换行符 - 执行当前行 */
    if (ch == '\n' || ch == '\r') {
        if (g_gcode_parser.buffer_pos > 0) {
            g_gcode_parser.line_buffer[g_gcode_parser.buffer_pos] = '\0';
            
            GCode_Instruction_t instruction;
            GCode_Error_t result = GCode_ParseLine(g_gcode_parser.line_buffer, &instruction);
            
            if (result == GCODE_OK && instruction.valid) {
                result = GCode_ExecuteInstruction(&instruction);
            }
            
            if (result != GCODE_OK) {
                g_gcode_parser.error_count++;
                if (s_error_callback) {
                    char error_msg[128];
                    GCode_FormatError(result, error_msg, sizeof(error_msg));
                    s_error_callback(result, instruction.line_number, error_msg);
                }
            }
            
            /* 清空缓冲区 */
            g_gcode_parser.buffer_pos = 0;
            return result;
        }
        return GCODE_OK;
    }
    
    /* 添加字符到缓冲区 */
    if (g_gcode_parser.buffer_pos < GCODE_MAX_LINE_LENGTH - 1) {
        g_gcode_parser.line_buffer[g_gcode_parser.buffer_pos++] = ch;
        return GCODE_OK;
    } else {
        return GCODE_ERROR_BUFFER_FULL;
    }
}

/**
 * @brief  设置当前坐标
 */
void GCode_SetCurrentPosition(float x, float y, float z)
{
    g_gcode_parser.current_x = x;
    g_gcode_parser.current_y = y;
    g_gcode_parser.current_z = z;
}

/**
 * @brief  获取当前坐标
 */
void GCode_GetCurrentPosition(float* x, float* y, float* z)
{
    if (x) *x = g_gcode_parser.current_x;
    if (y) *y = g_gcode_parser.current_y;
    if (z) *z = g_gcode_parser.current_z;
}

/**
 * @brief  重置解析器状态
 */
void GCode_Reset(void)
{
    g_gcode_parser.current_x = 0.0f;
    g_gcode_parser.current_y = 0.0f;
    g_gcode_parser.current_z = 0.0f;
    g_gcode_parser.coord_mode = COORD_ABSOLUTE;
    g_gcode_parser.unit_mode = UNIT_MM;
    g_gcode_parser.feed_rate = 100.0f;
    g_gcode_parser.spindle_speed = 0.0f;
    g_gcode_parser.buffer_pos = 0;
    g_gcode_parser.line_count = 0;
    g_gcode_parser.error_count = 0;
}

/**
 * @brief  获取解析器统计信息
 */
void GCode_GetStats(uint32_t* line_count, uint32_t* error_count)
{
    if (line_count) *line_count = g_gcode_parser.line_count;
    if (error_count) *error_count = g_gcode_parser.error_count;
}

/**
 * @brief  坐标转换
 */
bool GCode_ConvertCoordinates(const GCode_Instruction_t* instruction,
                             float* target_x, float* target_y, float* target_z)
{
    if (!instruction || !target_x || !target_y || !target_z) {
        return false;
    }
    
    /* 获取目标坐标 */
    float x = g_gcode_parser.current_x;
    float y = g_gcode_parser.current_y;
    float z = g_gcode_parser.current_z;
    
    /* 检查是否有坐标参数 */
    bool has_coords = false;
    float param_x, param_y, param_z;
    
    if (GCode_GetParameter(instruction, GCODE_PARAM_X, &param_x)) {
        has_coords = true;
        if (g_gcode_parser.coord_mode == COORD_ABSOLUTE) {
            x = param_x;
        } else {
            x += param_x;
        }
    }
    
    if (GCode_GetParameter(instruction, GCODE_PARAM_Y, &param_y)) {
        has_coords = true;
        if (g_gcode_parser.coord_mode == COORD_ABSOLUTE) {
            y = param_y;
        } else {
            y += param_y;
        }
    }
    
    if (GCode_GetParameter(instruction, GCODE_PARAM_Z, &param_z)) {
        has_coords = true;
        if (g_gcode_parser.coord_mode == COORD_ABSOLUTE) {
            z = param_z;
        } else {
            z += param_z;
        }
    }
    
    /* 单位转换 */
    if (g_gcode_parser.unit_mode == UNIT_INCH) {
        x *= 25.4f;  // 英寸转毫米
        y *= 25.4f;
        z *= 25.4f;
    }
    
    *target_x = x;
    *target_y = y;
    *target_z = z;
    
    return has_coords;
} 