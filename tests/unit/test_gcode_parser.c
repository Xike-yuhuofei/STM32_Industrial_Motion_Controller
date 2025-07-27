/**
 * @file    test_gcode_parser.c
 * @brief   G代码解析器测试程序
 * @date    2024-01-20
 * @author  STM32 G代码测试系统
 */

#include "main.h"
#include "gcode_parser.h"
#include "gcode_executor.h"
#include "motion_control.h"
#include <stdio.h>
#include <string.h>

/* 测试用的G代码示例 */
static const char* test_gcode_lines[] = {
    "G21",                    // 设置公制单位
    "G90",                    // 绝对坐标模式
    "G28",                    // 回零
    "G01 X10.5 F1000",        // 移动到X=10.5，速度1000mm/min
    "G01 X20 Y15 F500",       // 移动到X=20, Y=15，速度500mm/min
    "G00 X0",                 // 快速移动到X=0
    "G04 P2.5",               // 延时2.5秒
    "G91",                    // 相对坐标模式
    "G01 X5 F300",            // 相对移动X+5
    "G01 X-10",               // 相对移动X-10
    "G90",                    // 回到绝对坐标
    "M03 S1000",              // 主轴正转，转速1000
    "M05",                    // 主轴停止
    "M30",                    // 程序结束
    NULL
};

/* 测试状态 */
typedef struct {
    uint32_t tests_run;
    uint32_t tests_passed;
    uint32_t tests_failed;
    char last_error[128];
} TestResults_t;

static TestResults_t g_test_results = {0};

/* 测试函数声明 */
static void test_basic_parsing(void);
static void test_parameter_extraction(void);
static void test_coordinate_conversion(void);
static void test_gcode_execution(void);
static void test_error_handling(void);
static void run_gcode_sequence(void);
static void print_test_results(void);

/* 测试用回调函数 */
static GCode_Error_t test_execute_callback(const GCode_Instruction_t* instruction);
static void test_error_callback(GCode_Error_t error, uint32_t line_number, const char* message);

/**
 * @brief  G代码解析器主测试函数
 */
void Test_GCodeParser_Main(void)
{
    printf("\r\n=== G代码解析器测试开始 ===\r\n\r\n");
    
    /* 初始化系统 */
    MotionControl_Init();
    GCode_ExecutorInit();
    
    /* 运行各项测试 */
    test_basic_parsing();
    test_parameter_extraction();
    test_coordinate_conversion();
    test_error_handling();
    test_gcode_execution();
    run_gcode_sequence();
    
    /* 输出测试结果 */
    print_test_results();
    
    printf("\r\n=== G代码解析器测试完成 ===\r\n");
}

/**
 * @brief  基础解析功能测试
 */
static void test_basic_parsing(void)
{
    printf("测试1: 基础G代码解析...\r\n");
    
    GCode_Instruction_t instruction;
    GCode_Error_t result;
    
    /* 测试G00指令 */
    result = GCode_ParseLine("G00 X10", &instruction);
    g_test_results.tests_run++;
    if (result == GCODE_OK && instruction.command == GCODE_G00) {
        printf("  ✓ G00指令解析成功\r\n");
        g_test_results.tests_passed++;
    } else {
        printf("  ✗ G00指令解析失败\r\n");
        g_test_results.tests_failed++;
    }
    
    /* 测试G01指令 */
    result = GCode_ParseLine("G01 X20 Y30 F1000", &instruction);
    g_test_results.tests_run++;
    if (result == GCODE_OK && instruction.command == GCODE_G01) {
        printf("  ✓ G01指令解析成功\r\n");
        g_test_results.tests_passed++;
    } else {
        printf("  ✗ G01指令解析失败\r\n");
        g_test_results.tests_failed++;
    }
    
    /* 测试M指令 */
    result = GCode_ParseLine("M03 S1500", &instruction);
    g_test_results.tests_run++;
    if (result == GCODE_OK && instruction.command == GCODE_M03) {
        printf("  ✓ M03指令解析成功\r\n");
        g_test_results.tests_passed++;
    } else {
        printf("  ✗ M03指令解析失败\r\n");
        g_test_results.tests_failed++;
    }
    
    printf("\r\n");
}

/**
 * @brief  参数提取测试
 */
static void test_parameter_extraction(void)
{
    printf("测试2: 参数提取功能...\r\n");
    
    GCode_Instruction_t instruction;
    float value;
    
    GCode_ParseLine("G01 X15.5 Y-20.3 Z5 F1200", &instruction);
    
    /* 测试X参数 */
    g_test_results.tests_run++;
    if (GCode_GetParameter(&instruction, GCODE_PARAM_X, &value) && value == 15.5f) {
        printf("  ✓ X参数提取成功: %.1f\r\n", value);
        g_test_results.tests_passed++;
    } else {
        printf("  ✗ X参数提取失败\r\n");
        g_test_results.tests_failed++;
    }
    
    /* 测试Y参数 */
    g_test_results.tests_run++;
    if (GCode_GetParameter(&instruction, GCODE_PARAM_Y, &value) && value == -20.3f) {
        printf("  ✓ Y参数提取成功: %.1f\r\n", value);
        g_test_results.tests_passed++;
    } else {
        printf("  ✗ Y参数提取失败\r\n");
        g_test_results.tests_failed++;
    }
    
    /* 测试F参数 */
    g_test_results.tests_run++;
    if (GCode_GetParameter(&instruction, GCODE_PARAM_F, &value) && value == 1200.0f) {
        printf("  ✓ F参数提取成功: %.0f\r\n", value);
        g_test_results.tests_passed++;
    } else {
        printf("  ✗ F参数提取失败\r\n");
        g_test_results.tests_failed++;
    }
    
    printf("\r\n");
}

/**
 * @brief  坐标转换测试
 */
static void test_coordinate_conversion(void)
{
    printf("测试3: 坐标转换功能...\r\n");
    
    GCode_Instruction_t instruction;
    float x, y, z;
    
    /* 设置绝对坐标模式 */
    GCode_ParseLine("G90", &instruction);
    GCode_ExecuteInstruction(&instruction);
    
    /* 测试绝对坐标 */
    GCode_ParseLine("G01 X10 Y20", &instruction);
    g_test_results.tests_run++;
    if (GCode_ConvertCoordinates(&instruction, &x, &y, &z) && x == 10.0f && y == 20.0f) {
        printf("  ✓ 绝对坐标转换成功: X=%.0f, Y=%.0f\r\n", x, y);
        g_test_results.tests_passed++;
    } else {
        printf("  ✗ 绝对坐标转换失败\r\n");
        g_test_results.tests_failed++;
    }
    
    /* 设置相对坐标模式 */
    GCode_SetCurrentPosition(5.0f, 5.0f, 0.0f);
    GCode_ParseLine("G91", &instruction);
    GCode_ExecuteInstruction(&instruction);
    
    /* 测试相对坐标 */
    GCode_ParseLine("G01 X5 Y-3", &instruction);
    g_test_results.tests_run++;
    if (GCode_ConvertCoordinates(&instruction, &x, &y, &z) && x == 10.0f && y == 2.0f) {
        printf("  ✓ 相对坐标转换成功: X=%.0f, Y=%.0f\r\n", x, y);
        g_test_results.tests_passed++;
    } else {
        printf("  ✗ 相对坐标转换失败\r\n");
        g_test_results.tests_failed++;
    }
    
    printf("\r\n");
}

/**
 * @brief  错误处理测试
 */
static void test_error_handling(void)
{
    printf("测试4: 错误处理功能...\r\n");
    
    GCode_Instruction_t instruction;
    GCode_Error_t result;
    
    /* 测试无效指令 */
    result = GCode_ParseLine("H123", &instruction);
    g_test_results.tests_run++;
    if (result == GCODE_ERROR_INVALID_COMMAND) {
        printf("  ✓ 无效指令检测成功\r\n");
        g_test_results.tests_passed++;
    } else {
        printf("  ✗ 无效指令检测失败\r\n");
        g_test_results.tests_failed++;
    }
    
    /* 测试语法错误 */
    result = GCode_ParseLine("G01 XYZ", &instruction);
    g_test_results.tests_run++;
    if (result != GCODE_OK) {
        printf("  ✓ 语法错误检测成功\r\n");
        g_test_results.tests_passed++;
    } else {
        printf("  ✗ 语法错误检测失败\r\n");
        g_test_results.tests_failed++;
    }
    
    printf("\r\n");
}

/**
 * @brief  G代码执行测试
 */
static void test_gcode_execution(void)
{
    printf("测试5: G代码执行功能...\r\n");
    
    /* 初始化测试用的解析器 */
    GCode_Init(test_execute_callback, test_error_callback);
    
    /* 测试指令执行 */
    GCode_ProcessSerialLine("G28");  // 回零
    GCode_ProcessSerialLine("G01 X10 F500");  // 移动
    GCode_ProcessSerialLine("G04 P1");  // 延时
    
    printf("  ✓ G代码执行完成\r\n\r\n");
}

/**
 * @brief  运行完整G代码序列
 */
static void run_gcode_sequence(void)
{
    printf("测试6: 完整G代码序列执行...\r\n");
    
    const char** line = test_gcode_lines;
    uint32_t line_num = 1;
    
    while (*line != NULL) {
        printf("  执行第%lu行: %s\r\n", line_num, *line);
        GCode_ProcessSerialLine(*line);
        HAL_Delay(100);  // 短暂延时
        line++;
        line_num++;
    }
    
    printf("  ✓ G代码序列执行完成\r\n\r\n");
}

/**
 * @brief  打印测试结果
 */
static void print_test_results(void)
{
    printf("=== 测试结果统计 ===\r\n");
    printf("总测试数: %lu\r\n", g_test_results.tests_run);
    printf("通过测试: %lu\r\n", g_test_results.tests_passed);
    printf("失败测试: %lu\r\n", g_test_results.tests_failed);
    
    if (g_test_results.tests_failed == 0) {
        printf("🎉 所有测试通过！\r\n");
    } else {
        printf("⚠️  有 %lu 个测试失败\r\n", g_test_results.tests_failed);
    }
    
    /* 显示解析器统计信息 */
    uint32_t line_count, error_count;
    GCode_GetStats(&line_count, &error_count);
    printf("\r\n解析器统计:\r\n");
    printf("  处理行数: %lu\r\n", line_count);
    printf("  错误次数: %lu\r\n", error_count);
    
    /* 显示当前位置 */
    float x, y, z;
    GCode_GetCurrentPosition(&x, &y, &z);
    printf("  当前位置: X=%.2f, Y=%.2f, Z=%.2f\r\n", x, y, z);
}

/**
 * @brief  测试用执行回调函数
 */
static GCode_Error_t test_execute_callback(const GCode_Instruction_t* instruction)
{
    if (!instruction) return GCODE_ERROR_INVALID_PARAMETER;
    
    /* 简单打印要执行的指令 */
    printf("    执行指令: ");
    
    switch (instruction->command) {
        case GCODE_G00:
            printf("G00 快速移动");
            break;
        case GCODE_G01:
            printf("G01 直线插补");
            break;
        case GCODE_G28:
            printf("G28 回零");
            break;
        case GCODE_G04:
            printf("G04 延时");
            break;
        case GCODE_M03:
            printf("M03 主轴正转");
            break;
        case GCODE_M05:
            printf("M05 主轴停止");
            break;
        default:
            printf("指令码 %d", instruction->command);
            break;
    }
    
    /* 打印参数 */
    for (uint8_t i = 0; i < instruction->arg_count; i++) {
        printf(" %c%.1f", (char)instruction->args[i].type, instruction->args[i].value);
    }
    printf("\r\n");
    
    return GCODE_OK;
}

/**
 * @brief  测试用错误回调函数
 */
static void test_error_callback(GCode_Error_t error, uint32_t line_number, const char* message)
{
    snprintf(g_test_results.last_error, sizeof(g_test_results.last_error),
             "第%lu行: %s", line_number, message ? message : "未知错误");
    printf("    错误: %s\r\n", g_test_results.last_error);
} 