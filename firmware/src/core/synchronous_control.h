/**
  ******************************************************************************
  * @file    synchronous_control.h
  * @brief   同步控制算法模块
  * @author  Industrial Motion Control Team
  * @version 2.0
  * @date    2025-01-27
  ******************************************************************************
  * @attention
  * 
  * 本模块实现了工业级CNC控制系统的同步控制算法：
  * - 电子齿轮箱（Electronic Gear Box）
  * - 电子凸轮（Electronic Cam）
  * - 主从同步控制
  * - 交叉耦合控制
  * - 飞剪控制
  * - 多轴协调运动
  * 
  ******************************************************************************
  */

#ifndef __SYNCHRONOUS_CONTROL_H
#define __SYNCHRONOUS_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "motion_control.h"
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

/* 宏定义 --------------------------------------------------------------------*/
#define SYNC_MAX_AXES                   8       // 最大轴数
#define SYNC_MAX_GROUPS                 4       // 最大同步组数
#define SYNC_MAX_CAM_POINTS             1024    // 最大凸轮点数
#define SYNC_MAX_SLAVES_PER_GROUP       6       // 每组最大从轴数
#define SYNC_GEAR_RATIO_MAX             1000.0f // 最大齿轮比
#define SYNC_GEAR_RATIO_MIN             0.001f  // 最小齿轮比
#define SYNC_CAM_RESOLUTION             0.01f   // 凸轮分辨率 (度)
#define SYNC_COUPLING_GAIN_MAX          10.0f   // 最大耦合增益

/* 枚举类型定义 --------------------------------------------------------------*/

/**
 * @brief 同步控制类型
 */
typedef enum {
    SYNC_TYPE_NONE = 0,             // 无同步
    SYNC_TYPE_ELECTRONIC_GEAR,      // 电子齿轮
    SYNC_TYPE_ELECTRONIC_CAM,       // 电子凸轮
    SYNC_TYPE_MASTER_SLAVE,         // 主从同步
    SYNC_TYPE_CROSS_COUPLING,       // 交叉耦合
    SYNC_TYPE_FLYING_SHEAR,         // 飞剪控制
    SYNC_TYPE_COORDINATED_MOTION    // 协调运动
} SyncType_t;

/**
 * @brief 同步状态
 */
typedef enum {
    SYNC_STATE_IDLE = 0,            // 空闲状态
    SYNC_STATE_PREPARING,           // 准备状态
    SYNC_STATE_SYNCHRONIZING,       // 同步中
    SYNC_STATE_SYNCHRONIZED,        // 已同步
    SYNC_STATE_DESYNCHRONIZING,     // 去同步中
    SYNC_STATE_ERROR                // 错误状态
} SyncState_t;

/**
 * @brief 齿轮模式
 */
typedef enum {
    GEAR_MODE_SIMPLE = 0,           // 简单齿轮
    GEAR_MODE_DIFFERENTIAL,         // 差动齿轮
    GEAR_MODE_PLANETARY,            // 行星齿轮
    GEAR_MODE_VARIABLE_RATIO        // 变比齿轮
} GearMode_t;

/**
 * @brief 凸轮模式
 */
typedef enum {
    CAM_MODE_LINEAR = 0,            // 线性插值
    CAM_MODE_CUBIC_SPLINE,          // 三次样条
    CAM_MODE_BEZIER,                // 贝塞尔曲线
    CAM_MODE_POLYNOMIAL             // 多项式拟合
} CamMode_t;

/**
 * @brief 飞剪模式
 */
typedef enum {
    FLYING_SHEAR_ACCELERATION = 0,  // 加速飞剪
    FLYING_SHEAR_CONSTANT,          // 匀速飞剪
    FLYING_SHEAR_DECELERATION       // 减速飞剪
} FlyingShearMode_t;

/* 结构体定义 ----------------------------------------------------------------*/

/**
 * @brief 电子齿轮参数
 */
typedef struct {
    float gear_ratio;               // 齿轮比 (从轴/主轴)
    float backlash_compensation;    // 反向间隙补偿
    float lead_compensation;        // 超前补偿
    GearMode_t mode;                // 齿轮模式
    bool enable_backlash_comp;      // 使能反向间隙补偿
    bool enable_lead_comp;          // 使能超前补偿
    
    // 变比齿轮参数
    float ratio_min;                // 最小齿轮比
    float ratio_max;                // 最大齿轮比
    float ratio_change_rate;        // 齿轮比变化率
} ElectronicGear_t;

/**
 * @brief 凸轮点结构
 */
typedef struct {
    float master_position;          // 主轴位置 (度)
    float slave_position;           // 从轴位置 (度)
    float slave_velocity;           // 从轴速度 (度/s)
    float slave_acceleration;       // 从轴加速度 (度/s²)
} CamPoint_t;

/**
 * @brief 电子凸轮参数
 */
typedef struct {
    CamPoint_t points[SYNC_MAX_CAM_POINTS]; // 凸轮点表
    uint16_t num_points;            // 凸轮点数量
    float master_cycle;             // 主轴周期 (度)
    float slave_cycle;              // 从轴周期 (度)
    CamMode_t interpolation_mode;   // 插值模式
    bool cyclic;                    // 循环模式
    bool smooth_transition;         // 平滑过渡
    
    // 运行时状态
    uint16_t current_segment;       // 当前段
    float current_parameter;        // 当前参数
    bool in_transition;             // 过渡状态
} ElectronicCam_t;

/**
 * @brief 交叉耦合控制参数
 */
typedef struct {
    float coupling_gain_x;          // X轴耦合增益
    float coupling_gain_y;          // Y轴耦合增益
    float coupling_gain_z;          // Z轴耦合增益
    float contour_tolerance;        // 轮廓容差
    float max_coupling_output;      // 最大耦合输出
    bool enable;                    // 使能标志
    
    // 轮廓误差
    float contour_error;            // 当前轮廓误差
    float max_contour_error;        // 最大轮廓误差
    float contour_error_integral;   // 轮廓误差积分
} CrossCouplingControl_t;

/**
 * @brief 飞剪控制参数
 */
typedef struct {
    float material_velocity;        // 材料速度
    float cutting_length;           // 切割长度
    float acceleration_distance;    // 加速距离
    float deceleration_distance;    // 减速距离
    float max_cutting_velocity;     // 最大切割速度
    FlyingShearMode_t mode;         // 飞剪模式
    bool auto_return;               // 自动返回
    
    // 运行时状态
    float sync_position;            // 同步位置
    float cutting_position;         // 切割位置
    bool in_cutting_cycle;          // 切割周期中
    uint32_t cut_count;            // 切割计数
} FlyingShearControl_t;

/**
 * @brief 同步组配置
 */
typedef struct {
    uint8_t group_id;               // 组ID
    SyncType_t sync_type;           // 同步类型
    SyncState_t state;              // 同步状态
    bool enabled;                   // 使能状态
    
    // 轴配置
    uint8_t master_axis;            // 主轴号
    uint8_t slave_axes[SYNC_MAX_SLAVES_PER_GROUP]; // 从轴列表
    uint8_t num_slaves;             // 从轴数量
    
    // 同步参数
    union {
        ElectronicGear_t gear[SYNC_MAX_SLAVES_PER_GROUP];     // 电子齿轮参数
        ElectronicCam_t cam[SYNC_MAX_SLAVES_PER_GROUP];       // 电子凸轮参数
        CrossCouplingControl_t coupling;                      // 交叉耦合参数
        FlyingShearControl_t flying_shear;                    // 飞剪参数
    } params;
    
    // 运行时状态
    float master_position;          // 主轴位置
    float master_velocity;          // 主轴速度
    float slave_positions[SYNC_MAX_SLAVES_PER_GROUP];     // 从轴位置
    float slave_velocities[SYNC_MAX_SLAVES_PER_GROUP];    // 从轴速度
    float sync_errors[SYNC_MAX_SLAVES_PER_GROUP];         // 同步误差
    
    // 性能统计
    float max_sync_error;           // 最大同步误差
    float avg_sync_error;           // 平均同步误差
    uint32_t sync_cycles;           // 同步周期数
    uint32_t error_count;           // 错误计数
} SyncGroup_t;

/**
 * @brief 同步控制器
 */
typedef struct {
    // 同步组
    SyncGroup_t groups[SYNC_MAX_GROUPS];    // 同步组数组
    uint8_t num_groups;                     // 同步组数量
    
    // 全局参数
    float sync_period;              // 同步周期 (ms)
    bool global_enable;             // 全局使能
    
    // 轴位置反馈
    float axis_positions[SYNC_MAX_AXES];    // 轴位置
    float axis_velocities[SYNC_MAX_AXES];   // 轴速度
    
    // 性能统计
    uint32_t total_sync_cycles;     // 总同步周期
    float max_cycle_time;           // 最大周期时间
    float avg_cycle_time;           // 平均周期时间
} SyncController_t;

/* 全局变量声明 --------------------------------------------------------------*/
extern SyncController_t g_sync_controller;

/* 函数声明 ------------------------------------------------------------------*/

/* 系统初始化和管理 */
HAL_StatusTypeDef Sync_Init(float sync_period);
HAL_StatusTypeDef Sync_Reset(void);
HAL_StatusTypeDef Sync_Enable(bool enable);
HAL_StatusTypeDef Sync_Update(void);

/* 同步组管理 */
HAL_StatusTypeDef Sync_CreateGroup(uint8_t group_id, 
                                  SyncType_t sync_type,
                                  uint8_t master_axis,
                                  const uint8_t *slave_axes,
                                  uint8_t num_slaves);
HAL_StatusTypeDef Sync_DeleteGroup(uint8_t group_id);
HAL_StatusTypeDef Sync_EnableGroup(uint8_t group_id, bool enable);
SyncGroup_t* Sync_GetGroup(uint8_t group_id);
SyncState_t Sync_GetGroupState(uint8_t group_id);

/* 电子齿轮控制 */
HAL_StatusTypeDef Sync_SetGearRatio(uint8_t group_id, 
                                   uint8_t slave_index,
                                   float gear_ratio);
HAL_StatusTypeDef Sync_SetGearParameters(uint8_t group_id,
                                        uint8_t slave_index,
                                        const ElectronicGear_t *gear_params);
HAL_StatusTypeDef Sync_UpdateGearControl(uint8_t group_id);
float Sync_CalculateGearOutput(const ElectronicGear_t *gear,
                              float master_position,
                              float master_velocity);

/* 电子凸轮控制 */
HAL_StatusTypeDef Sync_LoadCamTable(uint8_t group_id,
                                   uint8_t slave_index,
                                   const CamPoint_t *points,
                                   uint16_t num_points);
HAL_StatusTypeDef Sync_SetCamParameters(uint8_t group_id,
                                       uint8_t slave_index,
                                       const ElectronicCam_t *cam_params);
HAL_StatusTypeDef Sync_UpdateCamControl(uint8_t group_id);
float Sync_InterpolateCam(const ElectronicCam_t *cam,
                         float master_position,
                         float *slave_velocity,
                         float *slave_acceleration);

/* 交叉耦合控制 */
HAL_StatusTypeDef Sync_SetCouplingParameters(uint8_t group_id,
                                            const CrossCouplingControl_t *coupling_params);
HAL_StatusTypeDef Sync_UpdateCouplingControl(uint8_t group_id,
                                            const float *command_positions,
                                            const float *actual_positions);
HAL_StatusTypeDef Sync_CalculateCouplingCorrection(const CrossCouplingControl_t *coupling,
                                                  const float *position_errors,
                                                  float *coupling_outputs);

/* 飞剪控制 */
HAL_StatusTypeDef Sync_SetFlyingShearParameters(uint8_t group_id,
                                               const FlyingShearControl_t *shear_params);
HAL_StatusTypeDef Sync_StartFlyingShear(uint8_t group_id);
HAL_StatusTypeDef Sync_StopFlyingShear(uint8_t group_id);
HAL_StatusTypeDef Sync_UpdateFlyingShearControl(uint8_t group_id);

/* 主从同步控制 */
HAL_StatusTypeDef Sync_StartMasterSlave(uint8_t group_id);
HAL_StatusTypeDef Sync_StopMasterSlave(uint8_t group_id);
HAL_StatusTypeDef Sync_UpdateMasterSlaveControl(uint8_t group_id);
HAL_StatusTypeDef Sync_SetSlaveOffset(uint8_t group_id,
                                     uint8_t slave_index,
                                     float position_offset);

/* 协调运动控制 */
HAL_StatusTypeDef Sync_StartCoordinatedMotion(uint8_t group_id,
                                             const float *target_positions,
                                             float max_velocity);
HAL_StatusTypeDef Sync_UpdateCoordinatedMotion(uint8_t group_id);
HAL_StatusTypeDef Sync_SetCoordinationParameters(uint8_t group_id,
                                                float velocity_ratio,
                                                float acceleration_ratio);

/* 同步过渡控制 */
HAL_StatusTypeDef Sync_StartSynchronization(uint8_t group_id,
                                           float sync_time);
HAL_StatusTypeDef Sync_StopSynchronization(uint8_t group_id,
                                          float desync_time);
HAL_StatusTypeDef Sync_UpdateSyncTransition(uint8_t group_id);

/* 位置和速度更新 */
HAL_StatusTypeDef Sync_UpdateAxisPosition(uint8_t axis_id,
                                         float position,
                                         float velocity);
HAL_StatusTypeDef Sync_GetSlaveCommand(uint8_t group_id,
                                      uint8_t slave_index,
                                      float *position,
                                      float *velocity);

/* 误差监控和补偿 */
HAL_StatusTypeDef Sync_MonitorSyncErrors(uint8_t group_id);
float Sync_GetSyncError(uint8_t group_id, uint8_t slave_index);
HAL_StatusTypeDef Sync_CompensateSyncError(uint8_t group_id,
                                          uint8_t slave_index,
                                          float compensation);

/* 性能统计 */
HAL_StatusTypeDef Sync_GetPerformanceStats(uint8_t group_id,
                                          float *max_sync_error,
                                          float *avg_sync_error,
                                          uint32_t *sync_cycles);
HAL_StatusTypeDef Sync_ResetPerformanceStats(uint8_t group_id);

/* 诊断和调试 */
HAL_StatusTypeDef Sync_DiagnoseGroup(uint8_t group_id,
                                    char *diagnosis_buffer,
                                    uint16_t buffer_size);
HAL_StatusTypeDef Sync_EnableDebugOutput(uint8_t group_id, bool enable);

/* 工具函数 */
float Sync_NormalizeAngle(float angle);
float Sync_CalculateAngleDifference(float angle1, float angle2);
HAL_StatusTypeDef Sync_ValidateGearRatio(float gear_ratio);
HAL_StatusTypeDef Sync_ValidateCamTable(const CamPoint_t *points,
                                       uint16_t num_points);

#ifdef __cplusplus
}
#endif

#endif /* __SYNCHRONOUS_CONTROL_H */ 