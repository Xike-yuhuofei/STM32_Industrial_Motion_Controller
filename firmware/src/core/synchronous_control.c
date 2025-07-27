/**
  ******************************************************************************
  * @file    synchronous_control.c
  * @brief   同步控制算法模块实现
  * @author  Industrial Motion Control Team
  * @version 2.0
  * @date    2025-01-27
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "synchronous_control.h"
#include "industrial_motion_controller.h"
#include <string.h>
#include <math.h>

/* Private macros ------------------------------------------------------------*/
#define SYNC_ANGLE_EPSILON      0.001f   // 角度精度
#define SYNC_POSITION_EPSILON   0.01f    // 位置精度
#define SYNC_MAX_ERROR          10.0f    // 最大同步误差

/* Private variables ---------------------------------------------------------*/
SyncController_t g_sync_controller;

/* Private function prototypes -----------------------------------------------*/
static HAL_StatusTypeDef Sync_ValidateGroupId(uint8_t group_id);
static HAL_StatusTypeDef Sync_ValidateAxisId(uint8_t axis_id);
static float Sync_InterpolateLinear(float x0, float y0, float x1, float y1, float x);
static float Sync_InterpolateCubic(const CamPoint_t *points, uint16_t index, float parameter);
static HAL_StatusTypeDef Sync_UpdateGroupState(uint8_t group_id);

/* 公共函数实现 --------------------------------------------------------------*/

/**
 * @brief 初始化同步控制器
 * @param sync_period 同步周期 (ms)
 * @retval HAL状态
 */
HAL_StatusTypeDef Sync_Init(float sync_period)
{
    // 清零控制器结构
    memset(&g_sync_controller, 0, sizeof(SyncController_t));
    
    // 设置同步周期
    g_sync_controller.sync_period = sync_period;
    g_sync_controller.global_enable = false;
    g_sync_controller.num_groups = 0;
    
    // 初始化性能统计
    g_sync_controller.total_sync_cycles = 0;
    g_sync_controller.max_cycle_time = 0;
    g_sync_controller.avg_cycle_time = 0;
    
    return HAL_OK;
}

/**
 * @brief 复位同步控制器
 * @retval HAL状态
 */
HAL_StatusTypeDef Sync_Reset(void)
{
    // 停止所有同步组
    for (uint8_t i = 0; i < g_sync_controller.num_groups; i++) {
        g_sync_controller.groups[i].enabled = false;
        g_sync_controller.groups[i].state = SYNC_STATE_IDLE;
    }
    
    // 清零性能统计
    g_sync_controller.total_sync_cycles = 0;
    g_sync_controller.max_cycle_time = 0;
    g_sync_controller.avg_cycle_time = 0;
    
    return HAL_OK;
}

/**
 * @brief 使能/禁用同步控制
 * @param enable 使能标志
 * @retval HAL状态
 */
HAL_StatusTypeDef Sync_Enable(bool enable)
{
    g_sync_controller.global_enable = enable;
    
    if (!enable) {
        // 禁用时停止所有同步组
        for (uint8_t i = 0; i < g_sync_controller.num_groups; i++) {
            g_sync_controller.groups[i].state = SYNC_STATE_IDLE;
        }
    }
    
    return HAL_OK;
}

/**
 * @brief 更新同步控制（主循环调用）
 * @retval HAL状态
 */
HAL_StatusTypeDef Sync_Update(void)
{
    if (!g_sync_controller.global_enable) {
        return HAL_OK;
    }
    
    uint32_t cycle_start = HAL_GetTick();
    
    // 更新所有同步组
    for (uint8_t i = 0; i < g_sync_controller.num_groups; i++) {
        if (g_sync_controller.groups[i].enabled) {
            Sync_UpdateGroupState(i);
            
            // 根据同步类型执行相应的控制
            switch (g_sync_controller.groups[i].sync_type) {
                case SYNC_TYPE_ELECTRONIC_GEAR:
                    Sync_UpdateGearControl(i);
                    break;
                    
                case SYNC_TYPE_ELECTRONIC_CAM:
                    Sync_UpdateCamControl(i);
                    break;
                    
                case SYNC_TYPE_MASTER_SLAVE:
                    Sync_UpdateMasterSlaveControl(i);
                    break;
                    
                case SYNC_TYPE_CROSS_COUPLING:
                    // 交叉耦合需要特殊处理
                    break;
                    
                case SYNC_TYPE_FLYING_SHEAR:
                    Sync_UpdateFlyingShearControl(i);
                    break;
                    
                case SYNC_TYPE_COORDINATED_MOTION:
                    Sync_UpdateCoordinatedMotion(i);
                    break;
                    
                default:
                    break;
            }
        }
    }
    
    // 更新性能统计
    uint32_t cycle_time = HAL_GetTick() - cycle_start;
    if (cycle_time > g_sync_controller.max_cycle_time) {
        g_sync_controller.max_cycle_time = cycle_time;
    }
    
    g_sync_controller.avg_cycle_time = 
        (g_sync_controller.avg_cycle_time * g_sync_controller.total_sync_cycles + cycle_time) /
        (g_sync_controller.total_sync_cycles + 1);
    
    g_sync_controller.total_sync_cycles++;
    
    return HAL_OK;
}

/**
 * @brief 创建同步组
 * @param group_id 组ID
 * @param sync_type 同步类型
 * @param master_axis 主轴号
 * @param slave_axes 从轴数组
 * @param num_slaves 从轴数量
 * @retval HAL状态
 */
HAL_StatusTypeDef Sync_CreateGroup(uint8_t group_id, 
                                  SyncType_t sync_type,
                                  uint8_t master_axis,
                                  const uint8_t *slave_axes,
                                  uint8_t num_slaves)
{
    if (group_id >= SYNC_MAX_GROUPS || slave_axes == NULL) {
        return HAL_ERROR;
    }
    
    if (num_slaves > SYNC_MAX_SLAVES_PER_GROUP) {
        return HAL_ERROR;
    }
    
    SyncGroup_t *group = &g_sync_controller.groups[group_id];
    
    // 初始化同步组
    memset(group, 0, sizeof(SyncGroup_t));
    group->group_id = group_id;
    group->sync_type = sync_type;
    group->master_axis = master_axis;
    group->num_slaves = num_slaves;
    group->state = SYNC_STATE_IDLE;
    group->enabled = false;
    
    // 复制从轴列表
    memcpy(group->slave_axes, slave_axes, num_slaves * sizeof(uint8_t));
    
    // 初始化同步参数（根据类型）
    switch (sync_type) {
        case SYNC_TYPE_ELECTRONIC_GEAR:
            // 初始化电子齿轮参数
            for (uint8_t i = 0; i < num_slaves; i++) {
                group->params.gear[i].gear_ratio = 1.0f;
                group->params.gear[i].mode = GEAR_MODE_SIMPLE;
                group->params.gear[i].enable_backlash_comp = false;
                group->params.gear[i].enable_lead_comp = false;
            }
            break;
            
        case SYNC_TYPE_ELECTRONIC_CAM:
            // 初始化电子凸轮参数
            for (uint8_t i = 0; i < num_slaves; i++) {
                group->params.cam[i].num_points = 0;
                group->params.cam[i].master_cycle = 360.0f;
                group->params.cam[i].slave_cycle = 360.0f;
                group->params.cam[i].interpolation_mode = CAM_MODE_LINEAR;
                group->params.cam[i].cyclic = true;
            }
            break;
            
        case SYNC_TYPE_CROSS_COUPLING:
            // 初始化交叉耦合参数
            group->params.coupling.coupling_gain_x = 1.0f;
            group->params.coupling.coupling_gain_y = 1.0f;
            group->params.coupling.coupling_gain_z = 1.0f;
            group->params.coupling.contour_tolerance = 0.1f;
            group->params.coupling.max_coupling_output = 10.0f;
            group->params.coupling.enable = true;
            break;
            
        case SYNC_TYPE_FLYING_SHEAR:
            // 初始化飞剪参数
            group->params.flying_shear.material_velocity = 100.0f; // mm/s
            group->params.flying_shear.cutting_length = 1000.0f;   // mm
            group->params.flying_shear.max_cutting_velocity = 200.0f; // mm/s
            group->params.flying_shear.mode = FLYING_SHEAR_ACCELERATION;
            group->params.flying_shear.auto_return = true;
            break;
            
        default:
            break;
    }
    
    // 更新组数量
    if (group_id >= g_sync_controller.num_groups) {
        g_sync_controller.num_groups = group_id + 1;
    }
    
    return HAL_OK;
}

/**
 * @brief 删除同步组
 * @param group_id 组ID
 * @retval HAL状态
 */
HAL_StatusTypeDef Sync_DeleteGroup(uint8_t group_id)
{
    if (Sync_ValidateGroupId(group_id) != HAL_OK) {
        return HAL_ERROR;
    }
    
    // 停止同步组
    g_sync_controller.groups[group_id].enabled = false;
    g_sync_controller.groups[group_id].state = SYNC_STATE_IDLE;
    
    // 清零组数据
    memset(&g_sync_controller.groups[group_id], 0, sizeof(SyncGroup_t));
    
    return HAL_OK;
}

/**
 * @brief 使能/禁用同步组
 * @param group_id 组ID
 * @param enable 使能标志
 * @retval HAL状态
 */
HAL_StatusTypeDef Sync_EnableGroup(uint8_t group_id, bool enable)
{
    if (Sync_ValidateGroupId(group_id) != HAL_OK) {
        return HAL_ERROR;
    }
    
    SyncGroup_t *group = &g_sync_controller.groups[group_id];
    
    if (enable && group->state == SYNC_STATE_IDLE) {
        group->state = SYNC_STATE_PREPARING;
        group->enabled = true;
    } else if (!enable) {
        group->state = SYNC_STATE_IDLE;
        group->enabled = false;
    }
    
    return HAL_OK;
}

/**
 * @brief 设置齿轮比
 * @param group_id 组ID
 * @param slave_index 从轴索引
 * @param gear_ratio 齿轮比
 * @retval HAL状态
 */
HAL_StatusTypeDef Sync_SetGearRatio(uint8_t group_id, 
                                   uint8_t slave_index,
                                   float gear_ratio)
{
    if (Sync_ValidateGroupId(group_id) != HAL_OK) {
        return HAL_ERROR;
    }
    
    SyncGroup_t *group = &g_sync_controller.groups[group_id];
    
    if (group->sync_type != SYNC_TYPE_ELECTRONIC_GEAR) {
        return HAL_ERROR;
    }
    
    if (slave_index >= group->num_slaves) {
        return HAL_ERROR;
    }
    
    // 验证齿轮比范围
    if (gear_ratio < SYNC_GEAR_RATIO_MIN || gear_ratio > SYNC_GEAR_RATIO_MAX) {
        return HAL_ERROR;
    }
    
    group->params.gear[slave_index].gear_ratio = gear_ratio;
    
    return HAL_OK;
}

/**
 * @brief 更新电子齿轮控制
 * @param group_id 组ID
 * @retval HAL状态
 */
HAL_StatusTypeDef Sync_UpdateGearControl(uint8_t group_id)
{
    if (Sync_ValidateGroupId(group_id) != HAL_OK) {
        return HAL_ERROR;
    }
    
    SyncGroup_t *group = &g_sync_controller.groups[group_id];
    
    if (group->sync_type != SYNC_TYPE_ELECTRONIC_GEAR || !group->enabled) {
        return HAL_ERROR;
    }
    
    // 获取主轴位置和速度
    if (group->master_axis < SYNC_MAX_AXES) {
        group->master_position = g_sync_controller.axis_positions[group->master_axis];
        group->master_velocity = g_sync_controller.axis_velocities[group->master_axis];
    }
    
    // 计算从轴位置
    for (uint8_t i = 0; i < group->num_slaves; i++) {
        ElectronicGear_t *gear = &group->params.gear[i];
        
        // 计算从轴目标位置
        float slave_position = Sync_CalculateGearOutput(gear, 
                                                       group->master_position,
                                                       group->master_velocity);
        
        // 更新从轴位置
        group->slave_positions[i] = slave_position;
        group->slave_velocities[i] = group->master_velocity * gear->gear_ratio;
        
        // 计算同步误差
        if (group->slave_axes[i] < SYNC_MAX_AXES) {
            float actual_position = g_sync_controller.axis_positions[group->slave_axes[i]];
            group->sync_errors[i] = slave_position - actual_position;
            
            // 更新最大同步误差
            if (fabsf(group->sync_errors[i]) > group->max_sync_error) {
                group->max_sync_error = fabsf(group->sync_errors[i]);
            }
        }
    }
    
    // 更新状态
    if (group->state == SYNC_STATE_PREPARING) {
        // 检查是否所有从轴都已就位
        bool all_ready = true;
        for (uint8_t i = 0; i < group->num_slaves; i++) {
            if (fabsf(group->sync_errors[i]) > SYNC_POSITION_EPSILON) {
                all_ready = false;
                break;
            }
        }
        
        if (all_ready) {
            group->state = SYNC_STATE_SYNCHRONIZED;
        }
    }
    
    group->sync_cycles++;
    
    return HAL_OK;
}

/**
 * @brief 计算电子齿轮输出
 * @param gear 齿轮参数
 * @param master_position 主轴位置
 * @param master_velocity 主轴速度
 * @retval 从轴位置
 */
float Sync_CalculateGearOutput(const ElectronicGear_t *gear,
                              float master_position,
                              float master_velocity)
{
    float slave_position = master_position * gear->gear_ratio;
    
    // 反向间隙补偿
    if (gear->enable_backlash_comp && gear->backlash_compensation != 0) {
        static float last_direction = 0;
        float current_direction = (master_velocity > 0) ? 1.0f : -1.0f;
        
        if (current_direction != last_direction && last_direction != 0) {
            // 方向改变，应用反向间隙补偿
            slave_position += current_direction * gear->backlash_compensation;
        }
        
        last_direction = current_direction;
    }
    
    // 超前补偿
    if (gear->enable_lead_comp && gear->lead_compensation != 0) {
        slave_position += master_velocity * gear->lead_compensation;
    }
    
    // 变比齿轮处理
    if (gear->mode == GEAR_MODE_VARIABLE_RATIO) {
        // 限制齿轮比在允许范围内
        float current_ratio = gear->gear_ratio;
        if (current_ratio < gear->ratio_min) {
            current_ratio = gear->ratio_min;
        } else if (current_ratio > gear->ratio_max) {
            current_ratio = gear->ratio_max;
        }
        
        slave_position = master_position * current_ratio;
    }
    
    return slave_position;
}

/**
 * @brief 加载凸轮表
 * @param group_id 组ID
 * @param slave_index 从轴索引
 * @param points 凸轮点数组
 * @param num_points 点数量
 * @retval HAL状态
 */
HAL_StatusTypeDef Sync_LoadCamTable(uint8_t group_id,
                                   uint8_t slave_index,
                                   const CamPoint_t *points,
                                   uint16_t num_points)
{
    if (Sync_ValidateGroupId(group_id) != HAL_OK || points == NULL) {
        return HAL_ERROR;
    }
    
    SyncGroup_t *group = &g_sync_controller.groups[group_id];
    
    if (group->sync_type != SYNC_TYPE_ELECTRONIC_CAM) {
        return HAL_ERROR;
    }
    
    if (slave_index >= group->num_slaves || num_points > SYNC_MAX_CAM_POINTS) {
        return HAL_ERROR;
    }
    
    ElectronicCam_t *cam = &group->params.cam[slave_index];
    
    // 复制凸轮点
    memcpy(cam->points, points, num_points * sizeof(CamPoint_t));
    cam->num_points = num_points;
    
    // 验证凸轮表
    if (Sync_ValidateCamTable(points, num_points) != HAL_OK) {
        return HAL_ERROR;
    }
    
    // 初始化凸轮参数
    cam->current_segment = 0;
    cam->current_parameter = 0;
    cam->in_transition = false;
    
    return HAL_OK;
}

/**
 * @brief 更新电子凸轮控制
 * @param group_id 组ID
 * @retval HAL状态
 */
HAL_StatusTypeDef Sync_UpdateCamControl(uint8_t group_id)
{
    if (Sync_ValidateGroupId(group_id) != HAL_OK) {
        return HAL_ERROR;
    }
    
    SyncGroup_t *group = &g_sync_controller.groups[group_id];
    
    if (group->sync_type != SYNC_TYPE_ELECTRONIC_CAM || !group->enabled) {
        return HAL_ERROR;
    }
    
    // 获取主轴位置
    if (group->master_axis < SYNC_MAX_AXES) {
        group->master_position = g_sync_controller.axis_positions[group->master_axis];
        group->master_velocity = g_sync_controller.axis_velocities[group->master_axis];
    }
    
    // 计算每个从轴的凸轮输出
    for (uint8_t i = 0; i < group->num_slaves; i++) {
        ElectronicCam_t *cam = &group->params.cam[i];
        
        if (cam->num_points < 2) {
            continue;
        }
        
        // 归一化主轴位置到凸轮周期
        float master_angle = fmodf(group->master_position, cam->master_cycle);
        if (master_angle < 0) {
            master_angle += cam->master_cycle;
        }
        
        // 插补凸轮输出
        float slave_velocity, slave_acceleration;
        float slave_position = Sync_InterpolateCam(cam, master_angle, 
                                                  &slave_velocity, 
                                                  &slave_acceleration);
        
        // 更新从轴位置
        group->slave_positions[i] = slave_position;
        group->slave_velocities[i] = slave_velocity * group->master_velocity;
        
        // 计算同步误差
        if (group->slave_axes[i] < SYNC_MAX_AXES) {
            float actual_position = g_sync_controller.axis_positions[group->slave_axes[i]];
            group->sync_errors[i] = slave_position - actual_position;
        }
    }
    
    group->sync_cycles++;
    
    return HAL_OK;
}

/**
 * @brief 插补电子凸轮
 * @param cam 凸轮参数
 * @param master_position 主轴位置
 * @param slave_velocity 从轴速度输出
 * @param slave_acceleration 从轴加速度输出
 * @retval 从轴位置
 */
float Sync_InterpolateCam(const ElectronicCam_t *cam,
                         float master_position,
                         float *slave_velocity,
                         float *slave_acceleration)
{
    if (cam == NULL || cam->num_points < 2) {
        return 0.0f;
    }
    
    // 查找当前段
    uint16_t segment = 0;
    for (uint16_t i = 0; i < cam->num_points - 1; i++) {
        if (master_position >= cam->points[i].master_position &&
            master_position < cam->points[i + 1].master_position) {
            segment = i;
            break;
        }
    }
    
    // 处理循环凸轮
    if (cam->cyclic && master_position >= cam->points[cam->num_points - 1].master_position) {
        segment = cam->num_points - 1;
    }
    
    // 根据插值模式计算输出
    float slave_position = 0;
    
    switch (cam->interpolation_mode) {
        case CAM_MODE_LINEAR:
            // 线性插值
            if (segment < cam->num_points - 1) {
                float t = (master_position - cam->points[segment].master_position) /
                         (cam->points[segment + 1].master_position - 
                          cam->points[segment].master_position);
                
                slave_position = Sync_InterpolateLinear(
                    cam->points[segment].master_position,
                    cam->points[segment].slave_position,
                    cam->points[segment + 1].master_position,
                    cam->points[segment + 1].slave_position,
                    master_position
                );
                
                if (slave_velocity != NULL) {
                    *slave_velocity = (cam->points[segment + 1].slave_position - 
                                      cam->points[segment].slave_position) /
                                     (cam->points[segment + 1].master_position - 
                                      cam->points[segment].master_position);
                }
                
                if (slave_acceleration != NULL) {
                    *slave_acceleration = 0; // 线性插值加速度为0
                }
            }
            break;
            
        case CAM_MODE_CUBIC_SPLINE:
            // 三次样条插值
            {
                float t = (master_position - cam->points[segment].master_position) /
                         (cam->points[segment + 1].master_position - 
                          cam->points[segment].master_position);
                
                slave_position = Sync_InterpolateCubic(cam->points, segment, t);
                
                // 计算速度和加速度（数值微分）
                if (slave_velocity != NULL || slave_acceleration != NULL) {
                    float dt = 0.001f;
                    float pos_plus = Sync_InterpolateCubic(cam->points, segment, t + dt);
                    float pos_minus = Sync_InterpolateCubic(cam->points, segment, t - dt);
                    
                    if (slave_velocity != NULL) {
                        *slave_velocity = (pos_plus - pos_minus) / (2 * dt);
                    }
                    
                    if (slave_acceleration != NULL) {
                        *slave_acceleration = (pos_plus - 2 * slave_position + pos_minus) / 
                                            (dt * dt);
                    }
                }
            }
            break;
            
        default:
            break;
    }
    
    return slave_position;
}

/**
 * @brief 设置交叉耦合参数
 * @param group_id 组ID
 * @param coupling_params 耦合参数
 * @retval HAL状态
 */
HAL_StatusTypeDef Sync_SetCouplingParameters(uint8_t group_id,
                                            const CrossCouplingControl_t *coupling_params)
{
    if (Sync_ValidateGroupId(group_id) != HAL_OK || coupling_params == NULL) {
        return HAL_ERROR;
    }
    
    SyncGroup_t *group = &g_sync_controller.groups[group_id];
    
    if (group->sync_type != SYNC_TYPE_CROSS_COUPLING) {
        return HAL_ERROR;
    }
    
    // 复制耦合参数
    memcpy(&group->params.coupling, coupling_params, sizeof(CrossCouplingControl_t));
    
    return HAL_OK;
}

/**
 * @brief 更新交叉耦合控制
 * @param group_id 组ID
 * @param command_positions 指令位置数组
 * @param actual_positions 实际位置数组
 * @retval HAL状态
 */
HAL_StatusTypeDef Sync_UpdateCouplingControl(uint8_t group_id,
                                            const float *command_positions,
                                            const float *actual_positions)
{
    if (Sync_ValidateGroupId(group_id) != HAL_OK || 
        command_positions == NULL || actual_positions == NULL) {
        return HAL_ERROR;
    }
    
    SyncGroup_t *group = &g_sync_controller.groups[group_id];
    
    if (group->sync_type != SYNC_TYPE_CROSS_COUPLING || !group->enabled) {
        return HAL_ERROR;
    }
    
    CrossCouplingControl_t *coupling = &group->params.coupling;
    
    if (!coupling->enable) {
        return HAL_OK;
    }
    
    // 计算位置误差
    float position_errors[SYNC_MAX_AXES];
    for (uint8_t i = 0; i <= group->num_slaves; i++) {
        uint8_t axis_id = (i == 0) ? group->master_axis : group->slave_axes[i - 1];
        if (axis_id < SYNC_MAX_AXES) {
            position_errors[i] = command_positions[i] - actual_positions[i];
        }
    }
    
    // 计算耦合补偿
    float coupling_outputs[SYNC_MAX_AXES];
    Sync_CalculateCouplingCorrection(coupling, position_errors, coupling_outputs);
    
    // 应用耦合补偿到各轴
    for (uint8_t i = 0; i <= group->num_slaves; i++) {
        group->slave_positions[i] = command_positions[i] + coupling_outputs[i];
    }
    
    // 计算轮廓误差
    float contour_error = 0;
    if (group->num_slaves >= 1) {
        // 简化：使用2D轮廓误差
        float ex = position_errors[0];
        float ey = position_errors[1];
        
        // 计算运动方向
        float vx = group->master_velocity;
        float vy = group->slave_velocities[0];
        float v_mag = sqrtf(vx * vx + vy * vy);
        
        if (v_mag > 0.01f) {
            // 归一化速度向量
            vx /= v_mag;
            vy /= v_mag;
            
            // 计算垂直于运动方向的误差分量
            float tangent_error = ex * vx + ey * vy;
            float normal_error_x = ex - tangent_error * vx;
            float normal_error_y = ey - tangent_error * vy;
            
            contour_error = sqrtf(normal_error_x * normal_error_x + 
                                normal_error_y * normal_error_y);
        }
    }
    
    coupling->contour_error = contour_error;
    if (contour_error > coupling->max_contour_error) {
        coupling->max_contour_error = contour_error;
    }
    
    return HAL_OK;
}

/**
 * @brief 计算交叉耦合补偿
 * @param coupling 耦合参数
 * @param position_errors 位置误差数组
 * @param coupling_outputs 耦合输出数组
 * @retval HAL状态
 */
HAL_StatusTypeDef Sync_CalculateCouplingCorrection(const CrossCouplingControl_t *coupling,
                                                  const float *position_errors,
                                                  float *coupling_outputs)
{
    if (coupling == NULL || position_errors == NULL || coupling_outputs == NULL) {
        return HAL_ERROR;
    }
    
    // 简化的2D交叉耦合控制
    float ex = position_errors[0];
    float ey = position_errors[1];
    
    // 计算耦合补偿
    float cx = coupling->coupling_gain_x * (ex - ey);
    float cy = coupling->coupling_gain_y * (ey - ex);
    
    // 限制输出
    if (cx > coupling->max_coupling_output) {
        cx = coupling->max_coupling_output;
    } else if (cx < -coupling->max_coupling_output) {
        cx = -coupling->max_coupling_output;
    }
    
    if (cy > coupling->max_coupling_output) {
        cy = coupling->max_coupling_output;
    } else if (cy < -coupling->max_coupling_output) {
        cy = -coupling->max_coupling_output;
    }
    
    coupling_outputs[0] = cx;
    coupling_outputs[1] = cy;
    
    // 3轴及以上的扩展
    if (position_errors[2] != 0) {
        float ez = position_errors[2];
        float cz = coupling->coupling_gain_z * (ez - (ex + ey) / 2.0f);
        
        if (cz > coupling->max_coupling_output) {
            cz = coupling->max_coupling_output;
        } else if (cz < -coupling->max_coupling_output) {
            cz = -coupling->max_coupling_output;
        }
        
        coupling_outputs[2] = cz;
    }
    
    return HAL_OK;
}

/**
 * @brief 开始飞剪控制
 * @param group_id 组ID
 * @retval HAL状态
 */
HAL_StatusTypeDef Sync_StartFlyingShear(uint8_t group_id)
{
    if (Sync_ValidateGroupId(group_id) != HAL_OK) {
        return HAL_ERROR;
    }
    
    SyncGroup_t *group = &g_sync_controller.groups[group_id];
    
    if (group->sync_type != SYNC_TYPE_FLYING_SHEAR) {
        return HAL_ERROR;
    }
    
    FlyingShearControl_t *shear = &group->params.flying_shear;
    
    // 初始化飞剪状态
    shear->in_cutting_cycle = true;
    shear->sync_position = 0;
    shear->cutting_position = 0;
    
    group->state = SYNC_STATE_SYNCHRONIZING;
    
    return HAL_OK;
}

/**
 * @brief 更新飞剪控制
 * @param group_id 组ID
 * @retval HAL状态
 */
HAL_StatusTypeDef Sync_UpdateFlyingShearControl(uint8_t group_id)
{
    if (Sync_ValidateGroupId(group_id) != HAL_OK) {
        return HAL_ERROR;
    }
    
    SyncGroup_t *group = &g_sync_controller.groups[group_id];
    
    if (group->sync_type != SYNC_TYPE_FLYING_SHEAR || !group->enabled) {
        return HAL_ERROR;
    }
    
    FlyingShearControl_t *shear = &group->params.flying_shear;
    
    if (!shear->in_cutting_cycle) {
        return HAL_OK;
    }
    
    // 更新同步位置（材料位置）
    shear->sync_position += shear->material_velocity * 
                           (g_sync_controller.sync_period / 1000.0f);
    
    // 计算剪切位置
    float target_position = shear->sync_position + shear->cutting_length;
    
    // 根据飞剪模式计算刀具速度
    float cutter_velocity = 0;
    
    switch (shear->mode) {
        case FLYING_SHEAR_ACCELERATION:
            // 加速飞剪：刀具从静止加速到材料速度
            if (shear->cutting_position < shear->acceleration_distance) {
                // 加速阶段
                cutter_velocity = shear->material_velocity * 
                                 (shear->cutting_position / shear->acceleration_distance);
            } else {
                // 同步阶段
                cutter_velocity = shear->material_velocity;
            }
            break;
            
        case FLYING_SHEAR_CONSTANT:
            // 匀速飞剪：刀具始终与材料同速
            cutter_velocity = shear->material_velocity;
            break;
            
        case FLYING_SHEAR_DECELERATION:
            // 减速飞剪：刀具从高速减速到材料速度
            if (shear->cutting_position < shear->deceleration_distance) {
                // 减速阶段
                cutter_velocity = shear->max_cutting_velocity - 
                                 (shear->max_cutting_velocity - shear->material_velocity) *
                                 (shear->cutting_position / shear->deceleration_distance);
            } else {
                // 同步阶段
                cutter_velocity = shear->material_velocity;
            }
            break;
    }
    
    // 更新刀具位置
    shear->cutting_position += cutter_velocity * 
                              (g_sync_controller.sync_period / 1000.0f);
    
    // 设置从轴（刀具轴）位置
    if (group->num_slaves > 0) {
        group->slave_positions[0] = shear->cutting_position;
        group->slave_velocities[0] = cutter_velocity;
    }
    
    // 检查是否完成切割
    if (shear->cutting_position >= target_position) {
        shear->cut_count++;
        
        if (shear->auto_return) {
            // 自动返回起始位置
            shear->cutting_position = 0;
            shear->sync_position = fmodf(shear->sync_position, shear->cutting_length);
        } else {
            shear->in_cutting_cycle = false;
            group->state = SYNC_STATE_IDLE;
        }
    }
    
    return HAL_OK;
}

/**
 * @brief 更新轴位置
 * @param axis_id 轴号
 * @param position 位置
 * @param velocity 速度
 * @retval HAL状态
 */
HAL_StatusTypeDef Sync_UpdateAxisPosition(uint8_t axis_id,
                                         float position,
                                         float velocity)
{
    if (axis_id >= SYNC_MAX_AXES) {
        return HAL_ERROR;
    }
    
    g_sync_controller.axis_positions[axis_id] = position;
    g_sync_controller.axis_velocities[axis_id] = velocity;
    
    return HAL_OK;
}

/**
 * @brief 获取从轴指令
 * @param group_id 组ID
 * @param slave_index 从轴索引
 * @param position 位置输出
 * @param velocity 速度输出
 * @retval HAL状态
 */
HAL_StatusTypeDef Sync_GetSlaveCommand(uint8_t group_id,
                                      uint8_t slave_index,
                                      float *position,
                                      float *velocity)
{
    if (Sync_ValidateGroupId(group_id) != HAL_OK || 
        position == NULL || velocity == NULL) {
        return HAL_ERROR;
    }
    
    SyncGroup_t *group = &g_sync_controller.groups[group_id];
    
    if (slave_index >= group->num_slaves) {
        return HAL_ERROR;
    }
    
    *position = group->slave_positions[slave_index];
    *velocity = group->slave_velocities[slave_index];
    
    return HAL_OK;
}

/**
 * @brief 获取同步误差
 * @param group_id 组ID
 * @param slave_index 从轴索引
 * @retval 同步误差
 */
float Sync_GetSyncError(uint8_t group_id, uint8_t slave_index)
{
    if (Sync_ValidateGroupId(group_id) != HAL_OK) {
        return 0.0f;
    }
    
    SyncGroup_t *group = &g_sync_controller.groups[group_id];
    
    if (slave_index >= group->num_slaves) {
        return 0.0f;
    }
    
    return group->sync_errors[slave_index];
}

/* 私有函数实现 --------------------------------------------------------------*/

/**
 * @brief 验证组ID
 * @param group_id 组ID
 * @retval HAL状态
 */
static HAL_StatusTypeDef Sync_ValidateGroupId(uint8_t group_id)
{
    if (group_id >= SYNC_MAX_GROUPS || group_id >= g_sync_controller.num_groups) {
        return HAL_ERROR;
    }
    
    return HAL_OK;
}

/**
 * @brief 更新同步组状态
 * @param group_id 组ID
 * @retval HAL状态
 */
static HAL_StatusTypeDef Sync_UpdateGroupState(uint8_t group_id)
{
    SyncGroup_t *group = &g_sync_controller.groups[group_id];
    
    // 检查同步误差
    bool sync_ok = true;
    float total_error = 0;
    
    for (uint8_t i = 0; i < group->num_slaves; i++) {
        if (fabsf(group->sync_errors[i]) > SYNC_MAX_ERROR) {
            sync_ok = false;
            group->error_count++;
        }
        total_error += fabsf(group->sync_errors[i]);
    }
    
    // 更新平均同步误差
    if (group->num_slaves > 0) {
        group->avg_sync_error = total_error / group->num_slaves;
    }
    
    // 状态转换
    switch (group->state) {
        case SYNC_STATE_PREPARING:
            if (sync_ok) {
                group->state = SYNC_STATE_SYNCHRONIZED;
            }
            break;
            
        case SYNC_STATE_SYNCHRONIZED:
            if (!sync_ok) {
                group->state = SYNC_STATE_ERROR;
            }
            break;
            
        case SYNC_STATE_ERROR:
            if (sync_ok) {
                group->state = SYNC_STATE_SYNCHRONIZED;
            }
            break;
            
        default:
            break;
    }
    
    return HAL_OK;
}

/**
 * @brief 线性插值
 * @param x0 起点X
 * @param y0 起点Y
 * @param x1 终点X
 * @param y1 终点Y
 * @param x 插值点X
 * @retval 插值结果Y
 */
static float Sync_InterpolateLinear(float x0, float y0, float x1, float y1, float x)
{
    if (fabsf(x1 - x0) < SYNC_ANGLE_EPSILON) {
        return y0;
    }
    
    return y0 + (y1 - y0) * (x - x0) / (x1 - x0);
}

/**
 * @brief 三次插值
 * @param points 凸轮点数组
 * @param index 当前索引
 * @param parameter 参数(0-1)
 * @retval 插值结果
 */
static float Sync_InterpolateCubic(const CamPoint_t *points, uint16_t index, float parameter)
{
    // 简化的三次插值实现
    float p0 = points[index].slave_position;
    float p1 = points[index + 1].slave_position;
    float v0 = points[index].slave_velocity;
    float v1 = points[index + 1].slave_velocity;
    
    float t = parameter;
    float t2 = t * t;
    float t3 = t2 * t;
    
    // Hermite插值多项式
    float h1 = 2 * t3 - 3 * t2 + 1;
    float h2 = -2 * t3 + 3 * t2;
    float h3 = t3 - 2 * t2 + t;
    float h4 = t3 - t2;
    
    return h1 * p0 + h2 * p1 + h3 * v0 + h4 * v1;
}

/**
 * @brief 验证凸轮表
 * @param points 凸轮点数组
 * @param num_points 点数量
 * @retval HAL状态
 */
HAL_StatusTypeDef Sync_ValidateCamTable(const CamPoint_t *points,
                                       uint16_t num_points)
{
    if (points == NULL || num_points < 2) {
        return HAL_ERROR;
    }
    
    // 检查主轴位置单调递增
    for (uint16_t i = 1; i < num_points; i++) {
        if (points[i].master_position <= points[i - 1].master_position) {
            return HAL_ERROR;
        }
    }
    
    return HAL_OK;
}

/**
 * @brief 归一化角度
 * @param angle 角度
 * @retval 归一化后的角度(0-360)
 */
float Sync_NormalizeAngle(float angle)
{
    angle = fmodf(angle, 360.0f);
    if (angle < 0) {
        angle += 360.0f;
    }
    return angle;
} 