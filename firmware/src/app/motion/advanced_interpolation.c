/**
  ******************************************************************************
  * @file    advanced_interpolation.c
  * @brief   高级插补算法模块实现
  * @author  Industrial Motion Control Team
  * @version 2.0
  * @date    2025-01-27
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "advanced_interpolation.h"
#include <string.h>
#include <stdlib.h>

/* Private macros ------------------------------------------------------------*/
#define INTERP_MIN_SEGMENT_TIME     0.001f  // 最小段时间 (秒)
#define INTERP_CORNER_THRESHOLD     0.1f    // 拐角阈值
#define NURBS_PARAMETER_STEP        0.001f  // NURBS参数步长

/* Private variables ---------------------------------------------------------*/
AdvancedInterpController_t g_adv_interp_controller;

/* Private function prototypes -----------------------------------------------*/
static HAL_StatusTypeDef AdvInterp_ProcessBlock(InterpBlock_t *block);
static HAL_StatusTypeDef AdvInterp_UpdateVelocity(InterpBlock_t *block, float dt);
static float AdvInterp_BSplineBasis(float t, int i, int p, const float *knots);
static void AdvInterp_EvaluateNURBS(const NURBSInterp_t *nurbs, float t, Point3D_t *point);
static float AdvInterp_CalculateCurvature(const Point3D_t *p1, const Point3D_t *p2, const Point3D_t *p3);

/* 公共函数实现 --------------------------------------------------------------*/

/**
 * @brief 初始化高级插补控制器
 * @param interpolation_period 插补周期 (ms)
 * @retval HAL状态
 */
HAL_StatusTypeDef AdvInterp_Init(float interpolation_period)
{
    // 清零控制器结构
    memset(&g_adv_interp_controller, 0, sizeof(AdvancedInterpController_t));
    
    // 设置插补周期
    g_adv_interp_controller.interpolation_period = interpolation_period;
    
    // 初始化前瞻控制参数
    g_adv_interp_controller.lookahead.enabled = true;
    g_adv_interp_controller.lookahead.min_corner_velocity = INTERP_MIN_VELOCITY;
    g_adv_interp_controller.lookahead.max_corner_acceleration = 1000.0f; // mm/s²
    
    // 初始化轮廓误差控制
    g_adv_interp_controller.contour_control.enable = true;
    g_adv_interp_controller.contour_control.cross_coupling_gain = 0.5f;
    g_adv_interp_controller.contour_control.contour_tolerance = 0.01f; // mm
    
    return HAL_OK;
}

/**
 * @brief 复位插补控制器
 * @retval HAL状态
 */
HAL_StatusTypeDef AdvInterp_Reset(void)
{
    // 停止当前插补
    g_adv_interp_controller.current_block = NULL;
    
    // 清空缓冲区
    g_adv_interp_controller.buffer_head = 0;
    g_adv_interp_controller.buffer_tail = 0;
    g_adv_interp_controller.buffer_count = 0;
    
    // 清空前瞻缓冲区
    g_adv_interp_controller.lookahead.head = 0;
    g_adv_interp_controller.lookahead.tail = 0;
    g_adv_interp_controller.lookahead.count = 0;
    
    // 重置性能统计
    g_adv_interp_controller.blocks_processed = 0;
    g_adv_interp_controller.average_velocity = 0.0f;
    g_adv_interp_controller.peak_velocity = 0.0f;
    g_adv_interp_controller.total_distance = 0.0f;
    g_adv_interp_controller.total_time = 0;
    
    return HAL_OK;
}

/**
 * @brief 添加插补数据块
 * @param block 插补数据块
 * @retval HAL状态
 */
HAL_StatusTypeDef AdvInterp_AddBlock(const InterpBlock_t *block)
{
    if (block == NULL) {
        return HAL_ERROR;
    }
    
    // 检查缓冲区是否已满
    if (g_adv_interp_controller.buffer_count >= INTERP_BUFFER_SIZE) {
        return HAL_ERROR;
    }
    
    // 复制数据块到缓冲区
    memcpy(&g_adv_interp_controller.buffer[g_adv_interp_controller.buffer_tail],
           block, sizeof(InterpBlock_t));
    
    // 如果启用前瞻，添加到前瞻缓冲区
    if (g_adv_interp_controller.lookahead.enabled) {
        // 执行速度前瞻优化
        AdvInterp_ProcessLookahead();
    }
    
    // 更新缓冲区指针
    g_adv_interp_controller.buffer_tail = 
        (g_adv_interp_controller.buffer_tail + 1) % INTERP_BUFFER_SIZE;
    g_adv_interp_controller.buffer_count++;
    
    return HAL_OK;
}

/**
 * @brief 准备直线插补
 * @param block 插补块
 * @param start 起点
 * @param end 终点
 * @param params 插补参数
 * @retval HAL状态
 */
HAL_StatusTypeDef AdvInterp_PrepareLinear(InterpBlock_t *block,
                                         const Point3D_t *start,
                                         const Point3D_t *end,
                                         const InterpParams_t *params)
{
    if (block == NULL || start == NULL || end == NULL || params == NULL) {
        return HAL_ERROR;
    }
    
    // 设置插补类型
    block->type = INTERP_TYPE_LINEAR;
    block->params = *params;
    block->state = INTERP_STATE_IDLE;
    
    // 设置直线参数
    block->data.linear.start_point = *start;
    block->data.linear.end_point = *end;
    
    // 计算方向向量
    block->data.linear.direction.x = end->x - start->x;
    block->data.linear.direction.y = end->y - start->y;
    block->data.linear.direction.z = end->z - start->z;
    
    // 计算长度
    block->data.linear.length = AdvInterp_CalculateDistance(start, end);
    
    // 归一化方向向量
    if (block->data.linear.length > INTERP_EPSILON) {
        block->data.linear.direction.x /= block->data.linear.length;
        block->data.linear.direction.y /= block->data.linear.length;
        block->data.linear.direction.z /= block->data.linear.length;
    }
    
    // 初始化运行时参数
    block->current_parameter = 0.0f;
    block->current_velocity = 0.0f;
    block->current_acceleration = 0.0f;
    block->current_position = *start;
    block->current_direction = block->data.linear.direction;
    block->remaining_distance = block->data.linear.length;
    
    // 进行速度规划
    AdvInterp_PlanVelocity(block);
    
    return HAL_OK;
}

/**
 * @brief 准备圆弧插补
 * @param block 插补块
 * @param start 起点
 * @param end 终点
 * @param center 圆心
 * @param clockwise 顺时针标志
 * @param params 插补参数
 * @retval HAL状态
 */
HAL_StatusTypeDef AdvInterp_PrepareCircular(InterpBlock_t *block,
                                           const Point3D_t *start,
                                           const Point3D_t *end,
                                           const Point3D_t *center,
                                           bool clockwise,
                                           const InterpParams_t *params)
{
    if (block == NULL || start == NULL || end == NULL || 
        center == NULL || params == NULL) {
        return HAL_ERROR;
    }
    
    // 设置插补类型
    block->type = INTERP_TYPE_CIRCULAR;
    block->params = *params;
    block->state = INTERP_STATE_IDLE;
    
    // 设置圆弧参数
    CircularInterp_t *arc = &block->data.circular;
    arc->start_point = *start;
    arc->end_point = *end;
    arc->center = *center;
    arc->clockwise = clockwise;
    
    // 计算半径
    arc->radius = AdvInterp_CalculateDistance(start, center);
    
    // 计算起始角度
    arc->start_angle = atan2f(start->y - center->y, start->x - center->x);
    float end_angle = atan2f(end->y - center->y, end->x - center->x);
    
    // 计算扫描角度
    arc->sweep_angle = end_angle - arc->start_angle;
    
    // 调整角度范围
    if (clockwise) {
        if (arc->sweep_angle > 0) {
            arc->sweep_angle -= 2 * M_PI;
        }
    } else {
        if (arc->sweep_angle < 0) {
            arc->sweep_angle += 2 * M_PI;
        }
    }
    
    // 计算法向量（默认XY平面）
    arc->normal.x = 0.0f;
    arc->normal.y = 0.0f;
    arc->normal.z = 1.0f;
    
    // 初始化运行时参数
    block->current_parameter = 0.0f;
    block->current_position = *start;
    block->remaining_distance = fabsf(arc->sweep_angle) * arc->radius;
    
    // 进行速度规划
    AdvInterp_PlanVelocity(block);
    
    return HAL_OK;
}

/**
 * @brief 准备螺旋插补
 * @param block 插补块
 * @param start 起点
 * @param end 终点
 * @param center 螺旋中心
 * @param pitch 螺距
 * @param clockwise 顺时针标志
 * @param params 插补参数
 * @retval HAL状态
 */
HAL_StatusTypeDef AdvInterp_PrepareSpiral(InterpBlock_t *block,
                                         const Point3D_t *start,
                                         const Point3D_t *end,
                                         const Point3D_t *center,
                                         float pitch,
                                         bool clockwise,
                                         const InterpParams_t *params)
{
    if (block == NULL || start == NULL || end == NULL || 
        center == NULL || params == NULL) {
        return HAL_ERROR;
    }
    
    // 设置插补类型
    block->type = INTERP_TYPE_SPIRAL;
    block->params = *params;
    block->state = INTERP_STATE_IDLE;
    
    // 设置螺旋参数
    SpiralInterp_t *spiral = &block->data.spiral;
    spiral->start_point = *start;
    spiral->end_point = *end;
    spiral->center = *center;
    spiral->pitch = pitch;
    spiral->clockwise = clockwise;
    
    // 计算起始和结束半径
    spiral->radius_start = sqrtf((start->x - center->x) * (start->x - center->x) +
                                (start->y - center->y) * (start->y - center->y));
    spiral->radius_end = sqrtf((end->x - center->x) * (end->x - center->x) +
                              (end->y - center->y) * (end->y - center->y));
    
    // 计算圈数
    float z_travel = end->z - start->z;
    spiral->turns = z_travel / pitch;
    
    // 计算总长度（近似）
    float avg_radius = (spiral->radius_start + spiral->radius_end) / 2.0f;
    float spiral_length = sqrtf((2 * M_PI * avg_radius * spiral->turns) * 
                               (2 * M_PI * avg_radius * spiral->turns) + 
                               z_travel * z_travel);
    
    // 初始化运行时参数
    block->current_parameter = 0.0f;
    block->current_position = *start;
    block->remaining_distance = spiral_length;
    
    // 进行速度规划
    AdvInterp_PlanVelocity(block);
    
    return HAL_OK;
}

/**
 * @brief 准备NURBS插补
 * @param block 插补块
 * @param control_points 控制点数组
 * @param weights 权重数组
 * @param num_points 控制点数量
 * @param degree 曲线阶数
 * @param params 插补参数
 * @retval HAL状态
 */
HAL_StatusTypeDef AdvInterp_PrepareNURBS(InterpBlock_t *block,
                                        const Point3D_t *control_points,
                                        const float *weights,
                                        uint8_t num_points,
                                        uint8_t degree,
                                        const InterpParams_t *params)
{
    if (block == NULL || control_points == NULL || 
        weights == NULL || params == NULL) {
        return HAL_ERROR;
    }
    
    if (num_points > NURBS_MAX_CONTROL_POINTS || degree >= num_points) {
        return HAL_ERROR;
    }
    
    // 设置插补类型
    block->type = INTERP_TYPE_NURBS;
    block->params = *params;
    block->state = INTERP_STATE_IDLE;
    
    // 设置NURBS参数
    NURBSInterp_t *nurbs = &block->data.nurbs;
    nurbs->num_control_points = num_points;
    nurbs->degree = degree;
    
    // 复制控制点和权重
    for (uint8_t i = 0; i < num_points; i++) {
        nurbs->control_points[i] = control_points[i];
        nurbs->weights[i] = weights[i];
    }
    
    // 生成均匀节点向量
    int n = num_points - 1;
    int p = degree;
    int m = n + p + 1;
    
    // 前p+1个节点为0
    for (int i = 0; i <= p; i++) {
        nurbs->knots[i] = 0.0f;
    }
    
    // 中间节点均匀分布
    for (int i = p + 1; i < m - p; i++) {
        nurbs->knots[i] = (float)(i - p) / (float)(m - 2 * p - 1);
    }
    
    // 后p+1个节点为1
    for (int i = m - p; i <= m; i++) {
        nurbs->knots[i] = 1.0f;
    }
    
    // 设置参数范围
    nurbs->start_param = 0.0f;
    nurbs->end_param = 1.0f;
    
    // 计算曲线长度（数值积分）
    float length = 0.0f;
    Point3D_t prev_point, curr_point;
    AdvInterp_EvaluateNURBS(nurbs, 0.0f, &prev_point);
    
    for (float t = NURBS_PARAMETER_STEP; t <= 1.0f; t += NURBS_PARAMETER_STEP) {
        AdvInterp_EvaluateNURBS(nurbs, t, &curr_point);
        length += AdvInterp_CalculateDistance(&prev_point, &curr_point);
        prev_point = curr_point;
    }
    
    // 初始化运行时参数
    block->current_parameter = 0.0f;
    block->current_position = control_points[0];
    block->remaining_distance = length;
    
    // 进行速度规划
    AdvInterp_PlanVelocity(block);
    
    return HAL_OK;
}

/**
 * @brief 执行插补（主循环调用）
 * @retval HAL状态
 */
HAL_StatusTypeDef AdvInterp_Execute(void)
{
    // 检查是否有当前执行块
    if (g_adv_interp_controller.current_block == NULL) {
        // 从缓冲区获取新块
        if (g_adv_interp_controller.buffer_count > 0) {
            g_adv_interp_controller.current_block = 
                &g_adv_interp_controller.buffer[g_adv_interp_controller.buffer_head];
            g_adv_interp_controller.current_block->state = INTERP_STATE_ACCELERATING;
            g_adv_interp_controller.current_block->start_time = HAL_GetTick();
        } else {
            return HAL_OK; // 无数据块
        }
    }
    
    // 处理当前块
    InterpBlock_t *block = g_adv_interp_controller.current_block;
    HAL_StatusTypeDef status = AdvInterp_ProcessBlock(block);
    
    // 检查是否完成
    if (block->state == INTERP_STATE_FINISHED) {
        // 更新统计
        g_adv_interp_controller.blocks_processed++;
        g_adv_interp_controller.total_distance += 
            (block->remaining_distance == 0) ? 
            block->data.linear.length : 0; // 简化处理
        
        // 移到下一个块
        g_adv_interp_controller.buffer_head = 
            (g_adv_interp_controller.buffer_head + 1) % INTERP_BUFFER_SIZE;
        g_adv_interp_controller.buffer_count--;
        g_adv_interp_controller.current_block = NULL;
    }
    
    // 更新周期计数器
    g_adv_interp_controller.cycle_counter++;
    
    return status;
}

/**
 * @brief 处理插补块
 * @param block 插补块
 * @retval HAL状态
 */
static HAL_StatusTypeDef AdvInterp_ProcessBlock(InterpBlock_t *block)
{
    float dt = g_adv_interp_controller.interpolation_period / 1000.0f; // 转换为秒
    
    // 更新速度
    AdvInterp_UpdateVelocity(block, dt);
    
    // 根据插补类型执行
    switch (block->type) {
        case INTERP_TYPE_LINEAR:
            return AdvInterp_ExecuteLinear(block, block->current_parameter,
                                         &block->current_position,
                                         &block->current_direction);
            
        case INTERP_TYPE_CIRCULAR:
            return AdvInterp_ExecuteCircular(block, block->current_parameter,
                                           &block->current_position,
                                           &block->current_direction);
            
        case INTERP_TYPE_SPIRAL:
            return AdvInterp_ExecuteSpiral(block, block->current_parameter,
                                         &block->current_position,
                                         &block->current_direction);
            
        case INTERP_TYPE_NURBS:
            return AdvInterp_ExecuteNURBS(block, block->current_parameter,
                                        &block->current_position,
                                        &block->current_direction);
            
        default:
            return HAL_ERROR;
    }
}

/**
 * @brief 执行直线插补
 * @param block 插补块
 * @param parameter 参数值(0-1)
 * @param position 输出位置
 * @param velocity 输出速度向量
 * @retval HAL状态
 */
HAL_StatusTypeDef AdvInterp_ExecuteLinear(InterpBlock_t *block,
                                         float parameter,
                                         Point3D_t *position,
                                         Point3D_t *velocity)
{
    if (block == NULL || position == NULL || velocity == NULL) {
        return HAL_ERROR;
    }
    
    LinearInterp_t *linear = &block->data.linear;
    
    // 计算位置
    position->x = linear->start_point.x + parameter * 
                 (linear->end_point.x - linear->start_point.x);
    position->y = linear->start_point.y + parameter * 
                 (linear->end_point.y - linear->start_point.y);
    position->z = linear->start_point.z + parameter * 
                 (linear->end_point.z - linear->start_point.z);
    
    // 速度方向即为直线方向
    *velocity = linear->direction;
    
    // 更新参数
    float distance_traveled = block->current_velocity * 
                            (g_adv_interp_controller.interpolation_period / 1000.0f);
    block->current_parameter += distance_traveled / linear->length;
    block->remaining_distance = linear->length * (1.0f - block->current_parameter);
    
    // 检查是否完成
    if (block->current_parameter >= 1.0f) {
        block->state = INTERP_STATE_FINISHED;
        *position = linear->end_point;
    }
    
    return HAL_OK;
}

/**
 * @brief 执行螺旋插补
 * @param block 插补块
 * @param parameter 参数值(0-1)
 * @param position 输出位置
 * @param velocity 输出速度向量
 * @retval HAL状态
 */
HAL_StatusTypeDef AdvInterp_ExecuteSpiral(InterpBlock_t *block,
                                         float parameter,
                                         Point3D_t *position,
                                         Point3D_t *velocity)
{
    if (block == NULL || position == NULL || velocity == NULL) {
        return HAL_ERROR;
    }
    
    SpiralInterp_t *spiral = &block->data.spiral;
    
    // 计算当前角度
    float angle = 2 * M_PI * spiral->turns * parameter;
    if (spiral->clockwise) {
        angle = -angle;
    }
    
    // 计算当前半径（线性插值）
    float radius = spiral->radius_start + 
                  (spiral->radius_end - spiral->radius_start) * parameter;
    
    // 计算位置
    position->x = spiral->center.x + radius * cosf(angle);
    position->y = spiral->center.y + radius * sinf(angle);
    position->z = spiral->start_point.z + 
                 (spiral->end_point.z - spiral->start_point.z) * parameter;
    
    // 计算速度方向（切线方向）
    float tangent_angle = angle + (spiral->clockwise ? -M_PI_2 : M_PI_2);
    velocity->x = -sinf(tangent_angle);
    velocity->y = cosf(tangent_angle);
    velocity->z = spiral->pitch / (2 * M_PI * radius);
    
    // 归一化速度向量
    AdvInterp_NormalizeVector(velocity);
    
    // 更新参数
    float dt = g_adv_interp_controller.interpolation_period / 1000.0f;
    float distance_traveled = block->current_velocity * dt;
    float total_length = block->remaining_distance + distance_traveled;
    block->current_parameter += distance_traveled / total_length;
    block->remaining_distance -= distance_traveled;
    
    // 检查是否完成
    if (block->current_parameter >= 1.0f) {
        block->state = INTERP_STATE_FINISHED;
        *position = spiral->end_point;
    }
    
    return HAL_OK;
}

/**
 * @brief 执行NURBS插补
 * @param block 插补块
 * @param parameter 参数值(0-1)
 * @param position 输出位置
 * @param velocity 输出速度向量
 * @retval HAL状态
 */
HAL_StatusTypeDef AdvInterp_ExecuteNURBS(InterpBlock_t *block,
                                        float parameter,
                                        Point3D_t *position,
                                        Point3D_t *velocity)
{
    if (block == NULL || position == NULL || velocity == NULL) {
        return HAL_ERROR;
    }
    
    NURBSInterp_t *nurbs = &block->data.nurbs;
    
    // 计算当前位置
    AdvInterp_EvaluateNURBS(nurbs, parameter, position);
    
    // 计算速度方向（数值微分）
    Point3D_t pos_next;
    float dt = 0.001f; // 小步长
    AdvInterp_EvaluateNURBS(nurbs, parameter + dt, &pos_next);
    
    velocity->x = (pos_next.x - position->x) / dt;
    velocity->y = (pos_next.y - position->y) / dt;
    velocity->z = (pos_next.z - position->z) / dt;
    
    // 归一化速度向量
    AdvInterp_NormalizeVector(velocity);
    
    // 更新参数
    float distance_traveled = block->current_velocity * 
                            (g_adv_interp_controller.interpolation_period / 1000.0f);
    
    // 自适应参数步长（基于局部曲率）
    Point3D_t pos_prev;
    AdvInterp_EvaluateNURBS(nurbs, parameter - dt, &pos_prev);
    float curvature = AdvInterp_CalculateCurvature(&pos_prev, position, &pos_next);
    
    float param_step = distance_traveled / (block->remaining_distance + distance_traveled);
    if (curvature > INTERP_EPSILON) {
        param_step *= (1.0f / (1.0f + curvature * 10.0f)); // 曲率越大，步长越小
    }
    
    block->current_parameter += param_step;
    block->remaining_distance -= distance_traveled;
    
    // 检查是否完成
    if (block->current_parameter >= 1.0f) {
        block->state = INTERP_STATE_FINISHED;
        AdvInterp_EvaluateNURBS(nurbs, 1.0f, position);
    }
    
    return HAL_OK;
}

/**
 * @brief 速度规划
 * @param block 插补块
 * @retval HAL状态
 */
HAL_StatusTypeDef AdvInterp_PlanVelocity(InterpBlock_t *block)
{
    if (block == NULL) {
        return HAL_ERROR;
    }
    
    float total_distance = block->remaining_distance;
    float max_vel = block->params.feed_rate / 60.0f; // mm/s
    float accel = block->params.acceleration;
    float jerk = block->params.jerk;
    
    // 根据速度模式进行规划
    switch (block->params.velocity_mode) {
        case VELOCITY_MODE_TRAPEZOIDAL:
            // 梯形速度规划
            {
                float accel_distance = (max_vel * max_vel) / (2.0f * accel);
                if (2 * accel_distance > total_distance) {
                    // 三角形速度曲线
                    max_vel = sqrtf(total_distance * accel);
                    accel_distance = total_distance / 2.0f;
                }
                block->estimated_time = (uint32_t)((total_distance / max_vel + 
                                                   max_vel / accel) * 1000.0f);
            }
            break;
            
        case VELOCITY_MODE_S_CURVE:
            // S曲线速度规划
            {
                float tj = accel / jerk; // 加加速时间
                float ta = max_vel / accel - tj; // 恒加速时间
                
                if (ta < 0) {
                    // 没有恒加速段
                    tj = sqrtf(max_vel / jerk);
                    ta = 0;
                }
                
                float accel_distance = max_vel * (2 * tj + ta) / 2.0f;
                if (2 * accel_distance > total_distance) {
                    // 需要降低最大速度
                    // 简化计算
                    max_vel = sqrtf(total_distance * accel / 2.0f);
                }
                
                block->estimated_time = (uint32_t)((total_distance / max_vel + 
                                                   2 * tj + ta) * 1000.0f);
            }
            break;
            
        default:
            block->estimated_time = (uint32_t)((total_distance / max_vel) * 1000.0f);
            break;
    }
    
    return HAL_OK;
}

/**
 * @brief 处理前瞻控制
 * @retval HAL状态
 */
HAL_StatusTypeDef AdvInterp_ProcessLookahead(void)
{
    LookaheadControl_t *lookahead = &g_adv_interp_controller.lookahead;
    
    if (!lookahead->enabled || lookahead->count < 2) {
        return HAL_OK;
    }
    
    // 遍历前瞻缓冲区，计算拐角速度
    for (uint8_t i = 0; i < lookahead->count - 1; i++) {
        InterpBlock_t *block1 = &lookahead->blocks[i];
        InterpBlock_t *block2 = &lookahead->blocks[(i + 1) % INTERP_LOOKAHEAD_DEPTH];
        
        // 计算拐角速度
        float corner_velocity = AdvInterp_CalculateCornerVelocity(block1, block2);
        
        // 限制块末速度
        if (block1->params.feed_rate > corner_velocity * 60.0f) {
            block1->params.feed_rate = corner_velocity * 60.0f;
            
            // 重新进行速度规划
            AdvInterp_PlanVelocity(block1);
        }
    }
    
    return HAL_OK;
}

/**
 * @brief 计算拐角速度
 * @param block1 第一个插补块
 * @param block2 第二个插补块
 * @retval 拐角速度 (mm/s)
 */
float AdvInterp_CalculateCornerVelocity(const InterpBlock_t *block1,
                                       const InterpBlock_t *block2)
{
    // 获取两个块的方向向量
    Point3D_t dir1 = block1->current_direction;
    Point3D_t dir2 = block2->current_direction;
    
    // 计算夹角
    float cos_angle = AdvInterp_DotProduct(&dir1, &dir2);
    
    // 限制余弦值范围
    if (cos_angle > 1.0f) cos_angle = 1.0f;
    if (cos_angle < -1.0f) cos_angle = -1.0f;
    
    float angle = acosf(cos_angle);
    
    // 根据夹角计算拐角速度
    if (angle < INTERP_CORNER_THRESHOLD) {
        // 小角度，不限制速度
        return INTERP_MAX_VELOCITY / 60.0f;
    }
    
    // 使用向心加速度限制计算拐角速度
    float max_accel = g_adv_interp_controller.lookahead.max_corner_acceleration;
    float corner_radius = block1->params.tolerance / (1.0f - cos_angle);
    float corner_velocity = sqrtf(max_accel * corner_radius);
    
    // 限制最小速度
    if (corner_velocity < g_adv_interp_controller.lookahead.min_corner_velocity / 60.0f) {
        corner_velocity = g_adv_interp_controller.lookahead.min_corner_velocity / 60.0f;
    }
    
    return corner_velocity;
}

/**
 * @brief 更新轮廓误差
 * @param command_pos 指令位置
 * @param actual_pos 实际位置
 * @retval HAL状态
 */
HAL_StatusTypeDef AdvInterp_UpdateContourError(const Point3D_t *command_pos,
                                              const Point3D_t *actual_pos)
{
    if (!g_adv_interp_controller.contour_control.enable) {
        return HAL_OK;
    }
    
    // 计算误差向量
    g_adv_interp_controller.contour_control.error_vector.x = 
        command_pos->x - actual_pos->x;
    g_adv_interp_controller.contour_control.error_vector.y = 
        command_pos->y - actual_pos->y;
    g_adv_interp_controller.contour_control.error_vector.z = 
        command_pos->z - actual_pos->z;
    
    // 计算轮廓误差（垂直于运动方向的误差分量）
    if (g_adv_interp_controller.current_block != NULL) {
        Point3D_t *dir = &g_adv_interp_controller.current_block->current_direction;
        Point3D_t *err = &g_adv_interp_controller.contour_control.error_vector;
        
        // 计算误差在运动方向上的投影
        float proj = AdvInterp_DotProduct(err, dir);
        
        // 计算垂直分量
        Point3D_t perp_error;
        perp_error.x = err->x - proj * dir->x;
        perp_error.y = err->y - proj * dir->y;
        perp_error.z = err->z - proj * dir->z;
        
        // 计算轮廓误差大小
        float contour_error = sqrtf(perp_error.x * perp_error.x +
                                   perp_error.y * perp_error.y +
                                   perp_error.z * perp_error.z);
        
        g_adv_interp_controller.contour_control.max_contour_error = 
            fmaxf(g_adv_interp_controller.contour_control.max_contour_error,
                  contour_error);
    }
    
    return HAL_OK;
}

/* 私有函数实现 --------------------------------------------------------------*/

/**
 * @brief 更新速度
 * @param block 插补块
 * @param dt 时间间隔
 * @retval HAL状态
 */
static HAL_StatusTypeDef AdvInterp_UpdateVelocity(InterpBlock_t *block, float dt)
{
    float target_vel = block->params.feed_rate / 60.0f; // mm/s
    float accel = block->params.acceleration;
    float jerk = block->params.jerk;
    
    switch (block->params.velocity_mode) {
        case VELOCITY_MODE_TRAPEZOIDAL:
            // 梯形速度更新
            if (block->state == INTERP_STATE_ACCELERATING) {
                block->current_velocity += accel * dt;
                if (block->current_velocity >= target_vel) {
                    block->current_velocity = target_vel;
                    block->state = INTERP_STATE_CONSTANT;
                }
            } else if (block->state == INTERP_STATE_DECELERATING) {
                float decel_distance = (block->current_velocity * block->current_velocity) / 
                                     (2.0f * accel);
                if (block->remaining_distance <= decel_distance) {
                    block->current_velocity -= accel * dt;
                    if (block->current_velocity <= 0) {
                        block->current_velocity = 0;
                        block->state = INTERP_STATE_FINISHED;
                    }
                }
            } else if (block->state == INTERP_STATE_CONSTANT) {
                float decel_distance = (block->current_velocity * block->current_velocity) / 
                                     (2.0f * accel);
                if (block->remaining_distance <= decel_distance) {
                    block->state = INTERP_STATE_DECELERATING;
                }
            }
            break;
            
        case VELOCITY_MODE_S_CURVE:
            // S曲线速度更新
            if (block->state == INTERP_STATE_ACCELERATING) {
                block->current_acceleration += jerk * dt;
                if (block->current_acceleration > accel) {
                    block->current_acceleration = accel;
                }
                
                block->current_velocity += block->current_acceleration * dt;
                if (block->current_velocity >= target_vel) {
                    block->current_velocity = target_vel;
                    block->current_acceleration = 0;
                    block->state = INTERP_STATE_CONSTANT;
                }
            }
            // 简化处理，实际S曲线更复杂
            break;
            
        default:
            block->current_velocity = target_vel;
            break;
    }
    
    // 更新性能统计
    if (block->current_velocity > g_adv_interp_controller.peak_velocity) {
        g_adv_interp_controller.peak_velocity = block->current_velocity;
    }
    
    return HAL_OK;
}

/**
 * @brief 计算NURBS基函数
 * @param t 参数值
 * @param i 基函数索引
 * @param p 阶数
 * @param knots 节点向量
 * @retval 基函数值
 */
static float AdvInterp_BSplineBasis(float t, int i, int p, const float *knots)
{
    if (p == 0) {
        return (t >= knots[i] && t < knots[i + 1]) ? 1.0f : 0.0f;
    }
    
    float left = 0.0f, right = 0.0f;
    
    if (knots[i + p] != knots[i]) {
        left = (t - knots[i]) / (knots[i + p] - knots[i]) * 
               AdvInterp_BSplineBasis(t, i, p - 1, knots);
    }
    
    if (knots[i + p + 1] != knots[i + 1]) {
        right = (knots[i + p + 1] - t) / (knots[i + p + 1] - knots[i + 1]) * 
                AdvInterp_BSplineBasis(t, i + 1, p - 1, knots);
    }
    
    return left + right;
}

/**
 * @brief 计算NURBS曲线点
 * @param nurbs NURBS参数
 * @param t 参数值
 * @param point 输出点
 */
static void AdvInterp_EvaluateNURBS(const NURBSInterp_t *nurbs, float t, Point3D_t *point)
{
    float x = 0.0f, y = 0.0f, z = 0.0f, w = 0.0f;
    
    for (int i = 0; i < nurbs->num_control_points; i++) {
        float basis = AdvInterp_BSplineBasis(t, i, nurbs->degree, nurbs->knots);
        float weighted_basis = basis * nurbs->weights[i];
        
        x += nurbs->control_points[i].x * weighted_basis;
        y += nurbs->control_points[i].y * weighted_basis;
        z += nurbs->control_points[i].z * weighted_basis;
        w += weighted_basis;
    }
    
    if (w > INTERP_EPSILON) {
        point->x = x / w;
        point->y = y / w;
        point->z = z / w;
    }
}

/**
 * @brief 计算曲率
 * @param p1 第一个点
 * @param p2 第二个点
 * @param p3 第三个点
 * @retval 曲率值
 */
static float AdvInterp_CalculateCurvature(const Point3D_t *p1, const Point3D_t *p2, 
                                         const Point3D_t *p3)
{
    // 使用三点计算曲率
    float a = AdvInterp_CalculateDistance(p1, p2);
    float b = AdvInterp_CalculateDistance(p2, p3);
    float c = AdvInterp_CalculateDistance(p1, p3);
    
    // 海伦公式计算面积
    float s = (a + b + c) / 2.0f;
    float area = sqrtf(s * (s - a) * (s - b) * (s - c));
    
    // 曲率 = 4 * 面积 / (a * b * c)
    if (a * b * c > INTERP_EPSILON) {
        return 4.0f * area / (a * b * c);
    }
    
    return 0.0f;
}

/* 工具函数实现 --------------------------------------------------------------*/

/**
 * @brief 计算两点间距离
 * @param point1 第一个点
 * @param point2 第二个点
 * @retval 距离值
 */
float AdvInterp_CalculateDistance(const Point3D_t *point1, const Point3D_t *point2)
{
    float dx = point2->x - point1->x;
    float dy = point2->y - point1->y;
    float dz = point2->z - point1->z;
    
    return sqrtf(dx * dx + dy * dy + dz * dz);
}

/**
 * @brief 归一化向量
 * @param vector 向量
 * @retval HAL状态
 */
HAL_StatusTypeDef AdvInterp_NormalizeVector(Point3D_t *vector)
{
    float length = sqrtf(vector->x * vector->x + 
                        vector->y * vector->y + 
                        vector->z * vector->z);
    
    if (length > INTERP_EPSILON) {
        vector->x /= length;
        vector->y /= length;
        vector->z /= length;
        return HAL_OK;
    }
    
    return HAL_ERROR;
}

/**
 * @brief 计算向量点积
 * @param vec1 第一个向量
 * @param vec2 第二个向量
 * @retval 点积值
 */
float AdvInterp_DotProduct(const Point3D_t *vec1, const Point3D_t *vec2)
{
    return vec1->x * vec2->x + vec1->y * vec2->y + vec1->z * vec2->z;
}

/**
 * @brief 计算向量叉积
 * @param vec1 第一个向量
 * @param vec2 第二个向量
 * @param result 结果向量
 * @retval HAL状态
 */
HAL_StatusTypeDef AdvInterp_CrossProduct(const Point3D_t *vec1,
                                        const Point3D_t *vec2,
                                        Point3D_t *result)
{
    if (vec1 == NULL || vec2 == NULL || result == NULL) {
        return HAL_ERROR;
    }
    
    result->x = vec1->y * vec2->z - vec1->z * vec2->y;
    result->y = vec1->z * vec2->x - vec1->x * vec2->z;
    result->z = vec1->x * vec2->y - vec1->y * vec2->x;
    
    return HAL_OK;
} 