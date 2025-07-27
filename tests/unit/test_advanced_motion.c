/**
  ******************************************************************************
  * @file    test_advanced_motion.c
  * @brief   高级运动控制算法测试程序
  * @author  Claude AI & Cursor
  * @version 2.0
  * @date    2025-01-27
  ******************************************************************************
  * @attention
  * 
  * 本文件提供了高级运动控制算法的完整测试套件，包括：
  * - 梯形速度规划测试
  * - S曲线速度规划测试
  * - DDA插补算法测试
  * - 圆弧插补算法测试
  * - 样条曲线插补测试
  * - 性能基准测试
  * 
  ******************************************************************************
  */

#include "motion_control.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <assert.h>

/* 测试配置 ------------------------------------------------------------------*/
#define TEST_EPSILON        1e-3f       // 测试精度
#define PRINTF_COLOR_RED    "\033[31m"
#define PRINTF_COLOR_GREEN  "\033[32m"
#define PRINTF_COLOR_YELLOW "\033[33m"
#define PRINTF_COLOR_BLUE   "\033[34m"
#define PRINTF_COLOR_RESET  "\033[0m"

/* 测试统计 ------------------------------------------------------------------*/
typedef struct {
    uint32_t total_tests;
    uint32_t passed_tests;
    uint32_t failed_tests;
    uint32_t start_time;
    uint32_t end_time;
} TestStats_t;

static TestStats_t g_test_stats = {0};

/* 测试辅助函数 --------------------------------------------------------------*/
static void test_start(const char* test_name);
static void test_assert(uint8_t condition, const char* message);
static void test_end(void);
static void print_test_header(const char* title);
static void print_test_summary(void);
static float test_get_time_ms(void);

/**
 * @brief  运行所有高级运动控制算法测试
 */
void Test_AdvancedMotion_RunAll(void)
{
    printf(PRINTF_COLOR_BLUE);
    printf("\n========================================\n");
    printf("   STM32F407 高级运动控制算法测试\n");
    printf("   Advanced Motion Control Test Suite\n");
    printf("   编译时间: %s %s\n", __DATE__, __TIME__);
    printf("========================================\n");
    printf(PRINTF_COLOR_RESET);
    
    g_test_stats.start_time = HAL_GetTick();
    
    // 运行各项测试
    Test_VelocityPlanning_Trapezoidal();
    Test_VelocityPlanning_SCurve();
    Test_DDA_Interpolation();
    Test_Arc_Interpolation();
    Test_Arc_PointComparison();
    Test_Spline_Interpolation();
    Test_Performance_Benchmark();
    
    g_test_stats.end_time = HAL_GetTick();
    print_test_summary();
}

/**
 * @brief  梯形速度规划测试
 */
void Test_VelocityPlanning_Trapezoidal(void)
{
    print_test_header("梯形速度规划测试");
    
    VelocityPlan_t plan;
    
    // 测试1: 标准梯形速度规划
    test_start("标准梯形速度规划");
    VelocityPlanning_Trapezoidal(&plan, 100.0f, 0.0f, 1000.0f, 0.0f, 1000.0f, 500.0f);
    test_assert(plan.profile_type == VELOCITY_TRAPEZOIDAL, "速度规划类型正确");
    test_assert(fabsf(plan.total_distance - 100.0f) < TEST_EPSILON, "总距离正确");
    test_assert(plan.accel_distance > 0 && plan.decel_distance > 0, "加减速距离大于0");
    
    // 验证速度曲线
    float vel_at_half_accel = VelocityPlanning_GetCurrentVelocity(&plan, plan.accel_time / 2);
    test_assert(vel_at_half_accel > 0 && vel_at_half_accel < plan.target_velocity, "加速中速度正确");
    test_end();
    
    // 测试2: 三角形速度规划（距离不足）
    test_start("三角形速度规划");
    VelocityPlanning_Trapezoidal(&plan, 10.0f, 0.0f, 1000.0f, 0.0f, 1000.0f, 500.0f);
    test_assert(plan.const_time == 0, "无恒速段");
    test_assert(plan.target_velocity < 1000.0f, "实际目标速度小于设定值");
    test_end();
    
    // 测试3: 速度曲线连续性检查
    test_start("速度曲线连续性");
    VelocityPlanning_Trapezoidal(&plan, 50.0f, 100.0f, 800.0f, 200.0f, 1000.0f, 400.0f);
    float vel_start = VelocityPlanning_GetCurrentVelocity(&plan, 0);
    float vel_end = VelocityPlanning_GetCurrentVelocity(&plan, plan.total_time);
    test_assert(fabsf(vel_start - 100.0f) < TEST_EPSILON, "起始速度正确");
    test_assert(fabsf(vel_end - 200.0f) < TEST_EPSILON, "结束速度正确");
    test_end();
}

/**
 * @brief  S曲线速度规划测试
 */
void Test_VelocityPlanning_SCurve(void)
{
    print_test_header("S曲线速度规划测试");
    
    VelocityPlan_t plan;
    
    // 测试1: 标准S曲线规划
    test_start("标准S曲线规划");
    VelocityPlanning_SCurve(&plan, 100.0f, 0.0f, 1000.0f, 0.0f, 1000.0f, 500.0f, 1000.0f);
    test_assert(plan.profile_type == VELOCITY_S_CURVE, "S曲线规划类型正确");
    test_assert(plan.s_accel_1_time > 0, "加速阶段1时间大于0");
    test_assert(plan.total_time > 0, "总时间大于0");
    test_end();
    
    // 测试2: 速度平滑性检查
    test_start("S曲线平滑性");
    VelocityPlanning_SCurve(&plan, 80.0f, 0.0f, 800.0f, 0.0f, 1000.0f, 400.0f, 800.0f);
    
    // 检查多个时间点的速度，确保平滑
    float prev_vel = VelocityPlanning_GetCurrentVelocity(&plan, 0);
    uint8_t smooth = 1;
    for (float t = 0.1f; t < plan.total_time; t += 0.1f) {
        float curr_vel = VelocityPlanning_GetCurrentVelocity(&plan, t);
        float vel_diff = fabsf(curr_vel - prev_vel);
        if (vel_diff > 100.0f) {  // 速度变化不应过大
            smooth = 0;
            break;
        }
        prev_vel = curr_vel;
    }
    test_assert(smooth, "速度曲线平滑");
    test_end();
    
    // 测试3: Jerk限制验证
    test_start("Jerk限制验证");
    VelocityPlanning_SCurve(&plan, 60.0f, 0.0f, 600.0f, 0.0f, 800.0f, 300.0f, 600.0f);
    test_assert(plan.jerk == 600.0f, "Jerk参数设置正确");
    test_assert(plan.s_accel_1_time == plan.acceleration / plan.jerk, "加速时间计算正确");
    test_end();
}

/**
 * @brief  DDA插补算法测试
 */
void Test_DDA_Interpolation(void)
{
    print_test_header("DDA插补算法测试");
    
    DDAInterpolator_t dda;
    float start[MAX_AXES] = {0, 0, 0, 0};
    float end[MAX_AXES] = {10, 10, 5, 90};  // 10mm, 10mm, 5mm, 90度
    int32_t step_output[MAX_AXES];
    
    // 测试1: DDA初始化
    test_start("DDA初始化");
    DDA_Init(&dda, start, end);
    test_assert(dda.total_steps > 0, "总步数大于0");
    test_assert(dda.current_step == 0, "当前步数初始化为0");
    test_assert(abs(dda.dx) == 10000, "X轴增量正确 (10mm = 10000μm)");
    test_assert(abs(dda.dy) == 10000, "Y轴增量正确");
    test_end();
    
    // 测试2: DDA步进测试
    test_start("DDA步进算法");
    uint32_t total_x_steps = 0, total_y_steps = 0, total_z_steps = 0, total_a_steps = 0;
    uint32_t step_count = 0;
    
    while (DDA_Step(&dda, step_output) && step_count < 20000) {  // 防止无限循环
        total_x_steps += abs(step_output[AXIS_X]);
        total_y_steps += abs(step_output[AXIS_Y]);
        total_z_steps += abs(step_output[AXIS_Z]);
        total_a_steps += abs(step_output[AXIS_A]);
        step_count++;
    }
    
    test_assert(step_count == dda.total_steps, "步数统计正确");
    test_assert(total_x_steps == 10000, "X轴总步数正确");
    test_assert(total_y_steps == 10000, "Y轴总步数正确");
    test_end();
    
    // 测试3: DDA重置功能
    test_start("DDA重置功能");
    DDA_Reset(&dda);
    test_assert(dda.current_step == 0, "重置后当前步数为0");
    test_assert(dda.x_step == 0 && dda.y_step == 0, "重置后各轴步数为0");
    test_end();
}

/**
 * @brief  圆弧插补算法测试
 */
void Test_Arc_Interpolation(void)
{
    print_test_header("圆弧插补算法测试");
    
    ArcInterpolator_t arc;
    float start[2] = {10.0f, 0.0f};
    float end[2] = {0.0f, 10.0f};
    float center[2] = {0.0f, 0.0f};
    float position[2];
    
    // 测试1: 圆弧初始化
    test_start("圆弧插补初始化");
    Arc_Init(&arc, start, end, center, 0, 0.01f);  // 逆时针，0.01弧度分辨率
    test_assert(fabsf(arc.radius - 10.0f) < TEST_EPSILON, "半径计算正确");
    test_assert(arc.total_steps > 0, "总步数大于0");
    test_assert(!arc.clockwise, "逆时针方向正确");
    test_end();
    
    // 测试2: 圆弧路径验证
    test_start("圆弧路径验证");
    uint32_t step_count = 0;
    float max_radius_error = 0;
    
    while (Arc_Step(&arc, position) && step_count < 1000) {
        // 验证每个点到圆心的距离等于半径
        float dist_to_center = sqrtf(position[0] * position[0] + position[1] * position[1]);
        float radius_error = fabsf(dist_to_center - arc.radius);
        if (radius_error > max_radius_error) {
            max_radius_error = radius_error;
        }
        step_count++;
    }
    
    test_assert(max_radius_error < 0.1f, "圆弧精度在允许范围内");
    test_assert(step_count == arc.total_steps, "圆弧步数正确");
    test_end();
    
    // 测试3: 顺时针圆弧
    test_start("顺时针圆弧");
    Arc_Init(&arc, start, end, center, 1, 0.01f);  // 顺时针
    test_assert(arc.clockwise, "顺时针方向正确");
    Arc_Step(&arc, position);
    // 第一步应该向下移动（顺时针）
    test_assert(position[1] < start[1], "顺时针方向运动正确");
    test_end();
}

/**
 * @brief  逐点比较法圆弧插补测试
 */
void Test_Arc_PointComparison(void)
{
    print_test_header("逐点比较法圆弧插补测试");
    
    ArcInterpolator_t arc;
    float start[2] = {5.0f, 0.0f};
    float end[2] = {0.0f, 5.0f};
    float center[2] = {0.0f, 0.0f};
    int32_t step_output[2];
    
    // 测试1: 逐点比较法初始化
    test_start("逐点比较法初始化");
    Arc_PointComparison_Init(&arc, start, end, center, 0);
    test_assert(fabsf(arc.radius - 5.0f) < TEST_EPSILON, "半径计算正确");
    test_assert(arc.x_pos == 5000 && arc.y_pos == 0, "起始位置正确 (μm)");
    test_end();
    
    // 测试2: 逐点比较步进
    test_start("逐点比较步进");
    uint32_t step_count = 0;
    int32_t total_x_steps = 0, total_y_steps = 0;
    
    while (Arc_PointComparison_Step(&arc, step_output) && step_count < 10000) {
        total_x_steps += step_output[0];
        total_y_steps += step_output[1];
        step_count++;
    }
    
    test_assert(step_count > 0, "产生了插补步数");
    test_assert(abs(total_x_steps) > 0 || abs(total_y_steps) > 0, "产生了有效步进");
    test_end();
    
    // 测试3: 圆弧精度验证
    test_start("逐点比较法精度");
    Arc_PointComparison_Init(&arc, start, end, center, 0);
    float max_error = 0;
    step_count = 0;
    
    while (Arc_PointComparison_Step(&arc, step_output) && step_count < 1000) {
        // 计算当前位置到圆心的距离误差
        float current_radius = sqrtf((arc.x_pos * arc.x_pos + arc.y_pos * arc.y_pos) / 1000000.0f);
        float error = fabsf(current_radius - arc.radius);
        if (error > max_error) max_error = error;
        step_count++;
    }
    
    test_assert(max_error < 0.01f, "逐点比较法精度满足要求");
    test_end();
}

/**
 * @brief  样条曲线插补测试
 */
void Test_Spline_Interpolation(void)
{
    print_test_header("样条曲线插补测试");
    
    SplineInterpolator_t spline;
    SplinePoint_t control_points[4] = {
        {0.0f, 0.0f, 0.0f, 0.0f, 1.0f},
        {10.0f, 10.0f, 0.0f, 0.0f, 1.0f},
        {20.0f, 10.0f, 0.0f, 0.0f, 1.0f},
        {30.0f, 0.0f, 0.0f, 0.0f, 1.0f}
    };
    float position[MAX_AXES];
    
    // 测试1: 样条曲线初始化
    test_start("样条曲线初始化");
    Spline_Init(&spline, control_points, 4, 3);
    test_assert(spline.point_count == 4, "控制点数量正确");
    test_assert(spline.degree == 3, "样条阶数正确");
    test_assert(spline.total_steps == 1000, "默认步数正确");
    test_end();
    
    // 测试2: NURBS基函数测试
    test_start("NURBS基函数");
    float basis_sum = 0;
    for (uint8_t i = 0; i < spline.point_count; i++) {
        float basis = Spline_BasisFunction(0.5f, i, spline.degree, spline.knot_vector);
        basis_sum += basis;
    }
    test_assert(fabsf(basis_sum - 1.0f) < TEST_EPSILON, "基函数和为1（分割单位性）");
    test_end();
    
    // 测试3: 样条曲线求值
    test_start("样条曲线求值");
    Spline_NURBS_Evaluate(&spline, 0.0f, position);
    test_assert(fabsf(position[AXIS_X] - 0.0f) < TEST_EPSILON, "起始点正确");
    
    Spline_NURBS_Evaluate(&spline, 1.0f, position);
    test_assert(fabsf(position[AXIS_X] - 30.0f) < TEST_EPSILON, "结束点正确");
    test_end();
    
    // 测试4: 样条插补步进
    test_start("样条插补步进");
    uint32_t step_count = 0;
    float prev_x = 0, max_discontinuity = 0;
    
    while (Spline_Step(&spline, position) && step_count < 1100) {
        if (step_count > 0) {
            float discontinuity = fabsf(position[AXIS_X] - prev_x);
            if (discontinuity > max_discontinuity) {
                max_discontinuity = discontinuity;
            }
        }
        prev_x = position[AXIS_X];
        step_count++;
    }
    
    test_assert(step_count == 1000, "样条插补步数正确");
    test_assert(max_discontinuity < 1.0f, "样条曲线连续性良好");
    test_end();
}

/**
 * @brief  性能基准测试
 */
void Test_Performance_Benchmark(void)
{
    print_test_header("性能基准测试");
    
    uint32_t start_tick, end_tick;
    
    // 测试1: 速度规划性能
    test_start("速度规划性能");
    VelocityPlan_t plan;
    start_tick = HAL_GetTick();
    
    for (uint32_t i = 0; i < 1000; i++) {
        VelocityPlanning_Trapezoidal(&plan, 100.0f, 0.0f, 1000.0f, 0.0f, 1000.0f, 500.0f);
    }
    
    end_tick = HAL_GetTick();
    uint32_t planning_time = end_tick - start_tick;
    test_assert(planning_time < 100, "速度规划性能满足实时要求");
    printf("    速度规划: 1000次计算用时 %lu ms\n", planning_time);
    test_end();
    
    // 测试2: DDA插补性能
    test_start("DDA插补性能");
    DDAInterpolator_t dda;
    float start_pos[MAX_AXES] = {0, 0, 0, 0};
    float end_pos[MAX_AXES] = {10, 10, 5, 90};
    int32_t step_output[MAX_AXES];
    
    DDA_Init(&dda, start_pos, end_pos);
    start_tick = HAL_GetTick();
    
    uint32_t step_count = 0;
    while (DDA_Step(&dda, step_output) && step_count < 10000) {
        step_count++;
    }
    
    end_tick = HAL_GetTick();
    uint32_t dda_time = end_tick - start_tick;
    test_assert(dda_time < 50, "DDA插补性能满足实时要求");
    printf("    DDA插补: %lu 步用时 %lu ms\n", step_count, dda_time);
    test_end();
    
    // 测试3: 内存使用测试
    test_start("内存使用统计");
    printf("    VelocityPlan_t 大小: %lu 字节\n", sizeof(VelocityPlan_t));
    printf("    DDAInterpolator_t 大小: %lu 字节\n", sizeof(DDAInterpolator_t));
    printf("    ArcInterpolator_t 大小: %lu 字节\n", sizeof(ArcInterpolator_t));
    printf("    SplineInterpolator_t 大小: %lu 字节\n", sizeof(SplineInterpolator_t));
    printf("    MotionBlock_t 大小: %lu 字节\n", sizeof(MotionBlock_t));
    test_assert(sizeof(MotionBlock_t) < 2048, "MotionBlock内存使用合理");
    test_end();
}

/* 测试辅助函数实现 ----------------------------------------------------------*/

static void test_start(const char* test_name)
{
    printf("  [测试] %s ... ", test_name);
    g_test_stats.total_tests++;
}

static void test_assert(uint8_t condition, const char* message)
{
    if (!condition) {
        printf(PRINTF_COLOR_RED "[失败] %s" PRINTF_COLOR_RESET "\n", message);
        g_test_stats.failed_tests++;
        return;
    }
}

static void test_end(void)
{
    if (g_test_stats.failed_tests == g_test_stats.total_tests - g_test_stats.passed_tests - 1) {
        // 本次测试失败，已在test_assert中处理
        return;
    }
    
    printf(PRINTF_COLOR_GREEN "[通过]" PRINTF_COLOR_RESET "\n");
    g_test_stats.passed_tests++;
}

static void print_test_header(const char* title)
{
    printf(PRINTF_COLOR_YELLOW "\n▶ %s\n" PRINTF_COLOR_RESET, title);
}

static void print_test_summary(void)
{
    uint32_t total_time = g_test_stats.end_time - g_test_stats.start_time;
    
    printf(PRINTF_COLOR_BLUE "\n========================================\n");
    printf("           测试结果总结\n");
    printf("========================================\n" PRINTF_COLOR_RESET);
    
    printf("总测试数: %lu\n", g_test_stats.total_tests);
    printf(PRINTF_COLOR_GREEN "通过: %lu\n" PRINTF_COLOR_RESET, g_test_stats.passed_tests);
    
    if (g_test_stats.failed_tests > 0) {
        printf(PRINTF_COLOR_RED "失败: %lu\n" PRINTF_COLOR_RESET, g_test_stats.failed_tests);
    } else {
        printf("失败: 0\n");
    }
    
    float success_rate = (float)g_test_stats.passed_tests / g_test_stats.total_tests * 100.0f;
    printf("成功率: %.1f%%\n", success_rate);
    printf("总用时: %lu ms\n", total_time);
    
    if (g_test_stats.failed_tests == 0) {
        printf(PRINTF_COLOR_GREEN "\n✅ 所有测试通过！高级运动控制算法工作正常。\n" PRINTF_COLOR_RESET);
    } else {
        printf(PRINTF_COLOR_RED "\n❌ 有测试失败，请检查算法实现。\n" PRINTF_COLOR_RESET);
    }
    
    printf(PRINTF_COLOR_BLUE "========================================\n" PRINTF_COLOR_RESET);
}

static float test_get_time_ms(void)
{
    return (float)HAL_GetTick();
}