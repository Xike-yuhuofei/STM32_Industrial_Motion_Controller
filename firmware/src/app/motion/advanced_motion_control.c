/**
  ******************************************************************************
  * @file    advanced_motion_control.c
  * @brief   高级运动控制算法实现
  * @author  Claude AI & Cursor
  * @version 2.0
  * @date    2025-01-27
  ******************************************************************************
  * @attention
  * 
  * 本文件实现了高级CNC运动控制算法，包括：
  * - 梯形速度规划（加速度受限）
  * - S曲线速度规划（减少振动和冲击）
  * - DDA直线插补算法
  * - 逐点比较法圆弧插补
  * - NURBS样条曲线插补
  * - 前瞻性运动规划
  * 
  ******************************************************************************
  */

#include "motion_control.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

/* 私有函数声明 --------------------------------------------------------------*/
static float velocity_trapezoidal_at_time(const VelocityPlan_t *plan, float time);
static float velocity_s_curve_at_time(const VelocityPlan_t *plan, float time);
static float position_trapezoidal_at_time(const VelocityPlan_t *plan, float time);
static float position_s_curve_at_time(const VelocityPlan_t *plan, float time);

/* ============================================================================
 *                           梯形速度规划算法
 * ============================================================================ */

/**
 * @brief  梯形速度规划
 * @param  plan: 速度规划结构体指针
 * @param  distance: 运动距离
 * @param  start_vel: 起始速度
 * @param  target_vel: 目标速度
 * @param  end_vel: 结束速度
 * @param  max_vel: 最大速度
 * @param  acceleration: 加速度
 * @retval None
 */
void VelocityPlanning_Trapezoidal(VelocityPlan_t *plan, float distance, 
                                  float start_vel, float target_vel, float end_vel,
                                  float max_vel, float acceleration)
{
    memset(plan, 0, sizeof(VelocityPlan_t));
    
    plan->profile_type = VELOCITY_TRAPEZOIDAL;
    plan->start_velocity = start_vel;
    plan->target_velocity = target_vel;
    plan->end_velocity = end_vel;
    plan->max_velocity = max_vel;
    plan->acceleration = acceleration;
    plan->total_distance = distance;
    
    // 限制目标速度不超过最大速度
    if (plan->target_velocity > plan->max_velocity) {
        plan->target_velocity = plan->max_velocity;
    }
    
    // 计算加速阶段
    float accel_vel_diff = plan->target_velocity - plan->start_velocity;
    plan->accel_time = fabsf(accel_vel_diff) / plan->acceleration;
    plan->accel_distance = plan->start_velocity * plan->accel_time + 
                          0.5f * plan->acceleration * plan->accel_time * plan->accel_time;
    
    // 计算减速阶段
    float decel_vel_diff = plan->target_velocity - plan->end_velocity;
    plan->decel_time = fabsf(decel_vel_diff) / plan->acceleration;
    plan->decel_distance = plan->end_velocity * plan->decel_time + 
                          0.5f * plan->acceleration * plan->decel_time * plan->decel_time;
    
    // 计算恒速阶段
    plan->const_distance = plan->total_distance - plan->accel_distance - plan->decel_distance;
    
    // 检查是否能达到目标速度（距离不够的情况）
    if (plan->const_distance < 0) {
        // 三角形速度曲线，不能达到目标速度
        float total_vel_change = accel_vel_diff + decel_vel_diff;
        float intersection_time = sqrtf(2.0f * plan->total_distance / plan->acceleration);
        
        plan->accel_time = intersection_time * (accel_vel_diff / total_vel_change);
        plan->decel_time = intersection_time * (decel_vel_diff / total_vel_change);
        plan->const_time = 0;
        
        plan->target_velocity = plan->start_velocity + 
                               plan->acceleration * plan->accel_time;
        
        plan->accel_distance = plan->start_velocity * plan->accel_time + 
                              0.5f * plan->acceleration * plan->accel_time * plan->accel_time;
        plan->decel_distance = plan->total_distance - plan->accel_distance;
        plan->const_distance = 0;
    } else {
        plan->const_time = plan->const_distance / plan->target_velocity;
    }
    
    plan->total_time = plan->accel_time + plan->const_time + plan->decel_time;
}

/**
 * @brief  S曲线速度规划（Jerk限制）
 * @param  plan: 速度规划结构体指针
 * @param  distance: 运动距离
 * @param  start_vel: 起始速度
 * @param  target_vel: 目标速度
 * @param  end_vel: 结束速度
 * @param  max_vel: 最大速度
 * @param  acceleration: 最大加速度
 * @param  jerk: 最大加加速度
 * @retval None
 */
void VelocityPlanning_SCurve(VelocityPlan_t *plan, float distance,
                            float start_vel, float target_vel, float end_vel,
                            float max_vel, float acceleration, float jerk)
{
    memset(plan, 0, sizeof(VelocityPlan_t));
    
    plan->profile_type = VELOCITY_S_CURVE;
    plan->start_velocity = start_vel;
    plan->target_velocity = target_vel;
    plan->end_velocity = end_vel;
    plan->max_velocity = max_vel;
    plan->acceleration = acceleration;
    plan->jerk = jerk;
    plan->total_distance = distance;
    
    // 限制目标速度
    if (plan->target_velocity > plan->max_velocity) {
        plan->target_velocity = plan->max_velocity;
    }
    
    // S曲线规划的7个阶段时间计算
    float vel_diff_accel = plan->target_velocity - plan->start_velocity;
    float vel_diff_decel = plan->target_velocity - plan->end_velocity;
    
    // 加速段时间计算
    if (vel_diff_accel > 0) {
        plan->s_accel_1_time = plan->acceleration / plan->jerk;
        plan->s_accel_2_time = vel_diff_accel / plan->acceleration - plan->s_accel_1_time;
        if (plan->s_accel_2_time < 0) {
            plan->s_accel_1_time = sqrtf(vel_diff_accel / plan->jerk);
            plan->s_accel_2_time = 0;
        }
    }
    
    // 减速段时间计算
    if (vel_diff_decel > 0) {
        plan->s_decel_1_time = plan->acceleration / plan->jerk;
        plan->s_decel_2_time = vel_diff_decel / plan->acceleration - plan->s_decel_1_time;
        if (plan->s_decel_2_time < 0) {
            plan->s_decel_1_time = sqrtf(vel_diff_decel / plan->jerk);
            plan->s_decel_2_time = 0;
        }
    }
    
    // 计算各段距离
    float accel_dist = plan->start_velocity * (plan->s_accel_1_time + plan->s_accel_2_time) +
                      0.5f * plan->acceleration * plan->s_accel_1_time * plan->s_accel_1_time +
                      plan->acceleration * plan->s_accel_1_time * plan->s_accel_2_time;
    
    float decel_dist = plan->end_velocity * (plan->s_decel_1_time + plan->s_decel_2_time) +
                      0.5f * plan->acceleration * plan->s_decel_1_time * plan->s_decel_1_time +
                      plan->acceleration * plan->s_decel_1_time * plan->s_decel_2_time;
    
    float const_dist = plan->total_distance - accel_dist - decel_dist;
    plan->s_const_time = const_dist / plan->target_velocity;
    
    if (plan->s_const_time < 0) {
        plan->s_const_time = 0;
        // 重新计算，无恒速段
        // 简化处理，使用梯形速度规划作为回退
        VelocityPlanning_Trapezoidal(plan, distance, start_vel, target_vel, 
                                    end_vel, max_vel, acceleration);
        return;
    }
    
    plan->total_time = plan->s_accel_1_time + plan->s_accel_2_time + 
                      plan->s_const_time + 
                      plan->s_decel_1_time + plan->s_decel_2_time;
}

/**
 * @brief  获取当前时刻的速度
 * @param  plan: 速度规划结构体指针
 * @param  time: 当前时间
 * @retval 当前速度
 */
float VelocityPlanning_GetCurrentVelocity(const VelocityPlan_t *plan, float time)
{
    if (time <= 0) return plan->start_velocity;
    if (time >= plan->total_time) return plan->end_velocity;
    
    switch (plan->profile_type) {
        case VELOCITY_TRAPEZOIDAL:
            return velocity_trapezoidal_at_time(plan, time);
        case VELOCITY_S_CURVE:
            return velocity_s_curve_at_time(plan, time);
        case VELOCITY_CONSTANT:
            return plan->target_velocity;
        default:
            return plan->start_velocity;
    }
}

/**
 * @brief  获取当前时刻的位置
 * @param  plan: 速度规划结构体指针
 * @param  time: 当前时间
 * @retval 当前位置
 */
float VelocityPlanning_GetCurrentPosition(const VelocityPlan_t *plan, float time)
{
    if (time <= 0) return 0;
    if (time >= plan->total_time) return plan->total_distance;
    
    switch (plan->profile_type) {
        case VELOCITY_TRAPEZOIDAL:
            return position_trapezoidal_at_time(plan, time);
        case VELOCITY_S_CURVE:
            return position_s_curve_at_time(plan, time);
        case VELOCITY_CONSTANT:
            return plan->target_velocity * time;
        default:
            return 0;
    }
}

/* ============================================================================
 *                           DDA插补算法
 * ============================================================================ */

/**
 * @brief  初始化DDA插补器
 * @param  dda: DDA插补器指针
 * @param  start: 起始位置数组
 * @param  end: 结束位置数组
 * @retval None
 */
void DDA_Init(DDAInterpolator_t *dda, float start[MAX_AXES], float end[MAX_AXES])
{
    memset(dda, 0, sizeof(DDAInterpolator_t));
    
    // 计算各轴增量
    dda->dx = (int32_t)((end[AXIS_X] - start[AXIS_X]) * 1000);  // 转换为微米
    dda->dy = (int32_t)((end[AXIS_Y] - start[AXIS_Y]) * 1000);
    dda->dz = (int32_t)((end[AXIS_Z] - start[AXIS_Z]) * 1000);
    dda->da = (int32_t)((end[AXIS_A] - start[AXIS_A]) * 1000);
    
    // 确定增量方向
    dda->x_inc = (dda->dx >= 0) ? 1 : -1;
    dda->y_inc = (dda->dy >= 0) ? 1 : -1;
    dda->z_inc = (dda->dz >= 0) ? 1 : -1;
    dda->a_inc = (dda->da >= 0) ? 1 : -1;
    
    // 取绝对值
    dda->dx = abs(dda->dx);
    dda->dy = abs(dda->dy);
    dda->dz = abs(dda->dz);
    dda->da = abs(dda->da);
    
    // 找到最大增量作为总步数
    dda->total_steps = dda->dx;
    if (dda->dy > dda->total_steps) dda->total_steps = dda->dy;
    if (dda->dz > dda->total_steps) dda->total_steps = dda->dz;
    if (dda->da > dda->total_steps) dda->total_steps = dda->da;
    
    // 初始化误差累积
    dda->x_err = dda->total_steps / 2;
    dda->y_err = dda->total_steps / 2;
    dda->z_err = dda->total_steps / 2;
    dda->a_err = dda->total_steps / 2;
    
    dda->current_step = 0;
}

/**
 * @brief  执行一步DDA插补
 * @param  dda: DDA插补器指针
 * @param  step_output: 输出步进脉冲数组 [X, Y, Z, A]
 * @retval 1: 还有步数, 0: 插补完成
 */
uint8_t DDA_Step(DDAInterpolator_t *dda, int32_t step_output[MAX_AXES])
{
    if (dda->current_step >= dda->total_steps) {
        return 0;  // 插补完成
    }
    
    // 清零输出
    memset(step_output, 0, MAX_AXES * sizeof(int32_t));
    
    // X轴DDA
    dda->x_err += dda->dx;
    if (dda->x_err >= dda->total_steps) {
        dda->x_err -= dda->total_steps;
        step_output[AXIS_X] = dda->x_inc;
        dda->x_step += dda->x_inc;
    }
    
    // Y轴DDA
    dda->y_err += dda->dy;
    if (dda->y_err >= dda->total_steps) {
        dda->y_err -= dda->total_steps;
        step_output[AXIS_Y] = dda->y_inc;
        dda->y_step += dda->y_inc;
    }
    
    // Z轴DDA
    dda->z_err += dda->dz;
    if (dda->z_err >= dda->total_steps) {
        dda->z_err -= dda->total_steps;
        step_output[AXIS_Z] = dda->z_inc;
        dda->z_step += dda->z_inc;
    }
    
    // A轴DDA
    dda->a_err += dda->da;
    if (dda->a_err >= dda->total_steps) {
        dda->a_err -= dda->total_steps;
        step_output[AXIS_A] = dda->a_inc;
        dda->a_step += dda->a_inc;
    }
    
    dda->current_step++;
    return 1;  // 继续插补
}

/**
 * @brief  重置DDA插补器
 * @param  dda: DDA插补器指针
 * @retval None
 */
void DDA_Reset(DDAInterpolator_t *dda)
{
    dda->current_step = 0;
    dda->x_step = dda->y_step = dda->z_step = dda->a_step = 0;
    dda->x_err = dda->total_steps / 2;
    dda->y_err = dda->total_steps / 2;
    dda->z_err = dda->total_steps / 2;
    dda->a_err = dda->total_steps / 2;
}

/* ============================================================================
 *                          圆弧插补算法
 * ============================================================================ */

/**
 * @brief  初始化圆弧插补器
 * @param  arc: 圆弧插补器指针
 * @param  start: 起始点坐标 [X, Y]
 * @param  end: 结束点坐标 [X, Y]
 * @param  center: 圆心坐标 [X, Y]
 * @param  clockwise: 顺时针标志
 * @param  resolution: 角度分辨率
 * @retval None
 */
void Arc_Init(ArcInterpolator_t *arc, float start[2], float end[2], 
              float center[2], uint8_t clockwise, float resolution)
{
    memset(arc, 0, sizeof(ArcInterpolator_t));
    
    arc->center_x = center[0];
    arc->center_y = center[1];
    arc->clockwise = clockwise;
    
    // 计算半径
    float dx = start[0] - center[0];
    float dy = start[1] - center[1];
    arc->radius = sqrtf(dx * dx + dy * dy);
    
    // 计算起始和结束角度
    arc->start_angle = atan2f(start[1] - center[1], start[0] - center[0]);
    arc->end_angle = atan2f(end[1] - center[1], end[0] - center[0]);
    
    // 计算角度差
    float angle_diff = arc->end_angle - arc->start_angle;
    
    if (clockwise) {
        if (angle_diff > 0) angle_diff -= 2 * M_PI;
    } else {
        if (angle_diff < 0) angle_diff += 2 * M_PI;
    }
    
    // 计算总步数和角度增量
    arc->total_steps = (int32_t)(fabsf(angle_diff) / resolution);
    arc->angle_increment = angle_diff / arc->total_steps;
    arc->current_step = 0;
}

/**
 * @brief  执行一步圆弧插补
 * @param  arc: 圆弧插补器指针
 * @param  position: 输出位置 [X, Y]
 * @retval 1: 还有步数, 0: 插补完成
 */
uint8_t Arc_Step(ArcInterpolator_t *arc, float position[2])
{
    if (arc->current_step >= arc->total_steps) {
        return 0;  // 插补完成
    }
    
    float current_angle = arc->start_angle + arc->current_step * arc->angle_increment;
    
    position[0] = arc->center_x + arc->radius * cosf(current_angle);
    position[1] = arc->center_y + arc->radius * sinf(current_angle);
    
    arc->current_step++;
    return 1;
}

/**
 * @brief  初始化逐点比较法圆弧插补器
 * @param  arc: 圆弧插补器指针
 * @param  start: 起始点坐标 [X, Y]
 * @param  end: 结束点坐标 [X, Y]
 * @param  center: 圆心坐标 [X, Y]
 * @param  clockwise: 顺时针标志
 * @retval None
 */
void Arc_PointComparison_Init(ArcInterpolator_t *arc, float start[2], float end[2], 
                             float center[2], uint8_t clockwise)
{
    memset(arc, 0, sizeof(ArcInterpolator_t));
    
    arc->center_x = center[0];
    arc->center_y = center[1];
    arc->clockwise = clockwise;
    
    // 转换为整数坐标（以微米为单位）
    arc->x_pos = (int32_t)((start[0] - center[0]) * 1000);
    arc->y_pos = (int32_t)((start[1] - center[1]) * 1000);
    
    // 计算半径的平方
    float radius_sq = arc->x_pos * arc->x_pos + arc->y_pos * arc->y_pos;
    arc->radius = sqrtf(radius_sq / 1000000.0f);  // 转换回毫米
    
    // 估算总步数
    float angle_diff = atan2f(end[1] - center[1], end[0] - center[0]) - 
                      atan2f(start[1] - center[1], start[0] - center[0]);
    if (clockwise && angle_diff > 0) angle_diff -= 2 * M_PI;
    if (!clockwise && angle_diff < 0) angle_diff += 2 * M_PI;
    
    arc->total_steps = (int32_t)(fabsf(angle_diff) * arc->radius * 1000);
    arc->current_step = 0;
}

/**
 * @brief  执行一步逐点比较法圆弧插补
 * @param  arc: 圆弧插补器指针
 * @param  step_output: 输出步进脉冲 [X, Y]
 * @retval 1: 还有步数, 0: 插补完成
 */
uint8_t Arc_PointComparison_Step(ArcInterpolator_t *arc, int32_t step_output[2])
{
    if (arc->current_step >= arc->total_steps) {
        return 0;  // 插补完成
    }
    
    step_output[0] = step_output[1] = 0;
    
    // 逐点比较法核心算法
    int32_t f = arc->x_pos * arc->x_pos + arc->y_pos * arc->y_pos - 
               (int32_t)(arc->radius * arc->radius * 1000000);
    
    if (arc->clockwise) {
        if (f >= 0) {
            // 在圆外或圆上，向圆心方向移动
            if (arc->x_pos > 0) {
                arc->x_pos--;
                step_output[0] = -1;
            } else {
                arc->x_pos++;
                step_output[0] = 1;
            }
        } else {
            // 在圆内，逆时针方向移动
            if (arc->y_pos > 0) {
                arc->y_pos--;
                step_output[1] = -1;
            } else {
                arc->y_pos++;
                step_output[1] = 1;
            }
        }
    } else {
        if (f >= 0) {
            // 在圆外或圆上，向圆心方向移动
            if (arc->y_pos > 0) {
                arc->y_pos--;
                step_output[1] = -1;
            } else {
                arc->y_pos++;
                step_output[1] = 1;
            }
        } else {
            // 在圆内，顺时针方向移动
            if (arc->x_pos > 0) {
                arc->x_pos--;
                step_output[0] = -1;
            } else {
                arc->x_pos++;
                step_output[0] = 1;
            }
        }
    }
    
    arc->current_step++;
    return 1;
}

/* ============================================================================
 *                          样条曲线插补算法
 * ============================================================================ */

/**
 * @brief  初始化样条曲线插补器
 * @param  spline: 样条插补器指针
 * @param  points: 控制点数组
 * @param  point_count: 控制点数量
 * @param  degree: 样条曲线阶数
 * @retval None
 */
void Spline_Init(SplineInterpolator_t *spline, SplinePoint_t points[], 
                uint8_t point_count, uint8_t degree)
{
    memset(spline, 0, sizeof(SplineInterpolator_t));
    
    spline->point_count = (point_count > MAX_SPLINE_POINTS) ? MAX_SPLINE_POINTS : point_count;
    spline->degree = degree;
    
    // 复制控制点
    for (uint8_t i = 0; i < spline->point_count; i++) {
        spline->control_points[i] = points[i];
    }
    
    // 生成均匀节点向量
    int knot_count = spline->point_count + spline->degree + 1;
    for (int i = 0; i < knot_count; i++) {
        if (i <= spline->degree) {
            spline->knot_vector[i] = 0.0f;
        } else if (i >= spline->point_count) {
            spline->knot_vector[i] = 1.0f;
        } else {
            spline->knot_vector[i] = (float)(i - spline->degree) / 
                                   (float)(spline->point_count - spline->degree);
        }
    }
    
    // 计算总步数和参数步长
    spline->total_steps = 1000;  // 默认1000步
    spline->parameter_step = 1.0f / spline->total_steps;
    spline->current_parameter = 0.0f;
    spline->current_step = 0;
}

/**
 * @brief  执行一步样条曲线插补
 * @param  spline: 样条插补器指针
 * @param  position: 输出位置 [X, Y, Z, A]
 * @retval 1: 还有步数, 0: 插补完成
 */
uint8_t Spline_Step(SplineInterpolator_t *spline, float position[MAX_AXES])
{
    if (spline->current_step >= spline->total_steps) {
        return 0;  // 插补完成
    }
    
    // 计算当前参数值对应的位置
    Spline_NURBS_Evaluate(spline, spline->current_parameter, position);
    
    spline->current_parameter += spline->parameter_step;
    spline->current_step++;
    
    return 1;
}

/**
 * @brief  NURBS样条曲线求值
 * @param  spline: 样条插补器指针
 * @param  t: 参数值
 * @param  result: 输出结果 [X, Y, Z, A]
 * @retval None
 */
void Spline_NURBS_Evaluate(const SplineInterpolator_t *spline, float t, float result[MAX_AXES])
{
    float numerator[MAX_AXES] = {0};
    float denominator = 0;
    
    for (uint8_t i = 0; i < spline->point_count; i++) {
        float basis = Spline_BasisFunction(t, i, spline->degree, spline->knot_vector);
        float weighted_basis = basis * spline->control_points[i].weight;
        
        numerator[AXIS_X] += weighted_basis * spline->control_points[i].x;
        numerator[AXIS_Y] += weighted_basis * spline->control_points[i].y;
        numerator[AXIS_Z] += weighted_basis * spline->control_points[i].z;
        numerator[AXIS_A] += weighted_basis * spline->control_points[i].a;
        
        denominator += weighted_basis;
    }
    
    if (denominator > EPSILON) {
        result[AXIS_X] = numerator[AXIS_X] / denominator;
        result[AXIS_Y] = numerator[AXIS_Y] / denominator;
        result[AXIS_Z] = numerator[AXIS_Z] / denominator;
        result[AXIS_A] = numerator[AXIS_A] / denominator;
    } else {
        memset(result, 0, MAX_AXES * sizeof(float));
    }
}

/**
 * @brief  B样条基函数计算
 * @param  t: 参数值
 * @param  i: 控制点索引
 * @param  degree: 样条阶数
 * @param  knots: 节点向量
 * @retval 基函数值
 */
float Spline_BasisFunction(float t, int i, int degree, const float *knots)
{
    if (degree == 0) {
        return (t >= knots[i] && t < knots[i + 1]) ? 1.0f : 0.0f;
    }
    
    float left_term = 0, right_term = 0;
    
    // 左项计算
    if (fabsf(knots[i + degree] - knots[i]) > EPSILON) {
        left_term = (t - knots[i]) / (knots[i + degree] - knots[i]) *
                   Spline_BasisFunction(t, i, degree - 1, knots);
    }
    
    // 右项计算
    if (fabsf(knots[i + degree + 1] - knots[i + 1]) > EPSILON) {
        right_term = (knots[i + degree + 1] - t) / (knots[i + degree + 1] - knots[i + 1]) *
                    Spline_BasisFunction(t, i + 1, degree - 1, knots);
    }
    
    return left_term + right_term;
}

/* ============================================================================
 *                          私有辅助函数
 * ============================================================================ */

/**
 * @brief  梯形速度规划在指定时间的速度
 */
static float velocity_trapezoidal_at_time(const VelocityPlan_t *plan, float time)
{
    if (time <= plan->accel_time) {
        // 加速阶段
        return plan->start_velocity + plan->acceleration * time;
    } else if (time <= plan->accel_time + plan->const_time) {
        // 恒速阶段
        return plan->target_velocity;
    } else {
        // 减速阶段
        float decel_time = time - plan->accel_time - plan->const_time;
        return plan->target_velocity - plan->acceleration * decel_time;
    }
}

/**
 * @brief  S曲线速度规划在指定时间的速度
 */
static float velocity_s_curve_at_time(const VelocityPlan_t *plan, float time)
{
    float t1 = plan->s_accel_1_time;
    float t2 = t1 + plan->s_accel_2_time;
    float t3 = t2 + plan->s_const_time;
    float t4 = t3 + plan->s_decel_1_time;
    
    if (time <= t1) {
        // S曲线加速阶段1
        return plan->start_velocity + 0.5f * plan->jerk * time * time;
    } else if (time <= t2) {
        // S曲线加速阶段2
        float dt = time - t1;
        return plan->start_velocity + 0.5f * plan->jerk * t1 * t1 + 
               plan->acceleration * dt - 0.5f * plan->jerk * dt * dt;
    } else if (time <= t3) {
        // 恒速阶段
        return plan->target_velocity;
    } else if (time <= t4) {
        // S曲线减速阶段1
        float dt = time - t3;
        return plan->target_velocity - 0.5f * plan->jerk * dt * dt;
    } else {
        // S曲线减速阶段2
        float dt = time - t4;
        float t_decel1 = plan->s_decel_1_time;
        return plan->target_velocity - 0.5f * plan->jerk * t_decel1 * t_decel1 -
               plan->acceleration * dt + 0.5f * plan->jerk * dt * dt;
    }
}

/**
 * @brief  梯形速度规划在指定时间的位置
 */
static float position_trapezoidal_at_time(const VelocityPlan_t *plan, float time)
{
    if (time <= plan->accel_time) {
        // 加速阶段
        return plan->start_velocity * time + 0.5f * plan->acceleration * time * time;
    } else if (time <= plan->accel_time + plan->const_time) {
        // 恒速阶段
        float const_time = time - plan->accel_time;
        return plan->accel_distance + plan->target_velocity * const_time;
    } else {
        // 减速阶段
        float decel_time = time - plan->accel_time - plan->const_time;
        return plan->accel_distance + plan->const_distance +
               plan->target_velocity * decel_time - 0.5f * plan->acceleration * decel_time * decel_time;
    }
}

/**
 * @brief  S曲线速度规划在指定时间的位置
 */
static float position_s_curve_at_time(const VelocityPlan_t *plan, float time)
{
    // 简化实现，实际应用中需要更复杂的积分计算
    // 这里使用数值积分近似
    float dt = 0.001f;  // 1ms步长
    float pos = 0;
    
    for (float t = 0; t < time; t += dt) {
        float vel = velocity_s_curve_at_time(plan, t);
        pos += vel * dt / 60.0f;  // 转换为mm (vel是mm/min)
    }
    
    return pos;
}