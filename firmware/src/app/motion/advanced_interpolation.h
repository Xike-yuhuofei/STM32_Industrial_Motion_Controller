/**
  ******************************************************************************
  * @file    advanced_interpolation.h
  * @brief   高级插补算法模块
  * @author  Industrial Motion Control Team
  * @version 2.0
  * @date    2025-01-27
  ******************************************************************************
  * @attention
  * 
  * 本模块实现了工业级CNC控制系统的高级插补算法：
  * - 螺旋插补算法
  * - NURBS样条插补
  * - 高精度圆弧插补
  * - 自适应前瞻控制
  * - 轮廓误差控制
  * - 速度前瞻优化
  * 
  ******************************************************************************
  */

#ifndef __ADVANCED_INTERPOLATION_H
#define __ADVANCED_INTERPOLATION_H

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
#define INTERP_MAX_AXES                 8       // 最大轴数
#define INTERP_BUFFER_SIZE              1024    // 插补缓冲区大小
#define INTERP_LOOKAHEAD_DEPTH          64      // 前瞻深度
#define INTERP_MIN_VELOCITY             0.1f    // 最小速度 (mm/min)
#define INTERP_MAX_VELOCITY             60000.0f // 最大速度 (mm/min)
#define INTERP_EPSILON                  1e-6f   // 浮点精度
#define NURBS_MAX_CONTROL_POINTS        32      // NURBS最大控制点数
#define SPIRAL_MAX_TURNS                100     // 螺旋最大圈数

/* 枚举类型定义 --------------------------------------------------------------*/

/**
 * @brief 插补类型
 */
typedef enum {
    INTERP_TYPE_LINEAR = 0,         // 直线插补
    INTERP_TYPE_CIRCULAR,           // 圆弧插补
    INTERP_TYPE_SPIRAL,             // 螺旋插补
    INTERP_TYPE_NURBS,              // NURBS样条插补
    INTERP_TYPE_BSPLINE,            // B样条插补
    INTERP_TYPE_BEZIER,             // 贝塞尔曲线插补
    INTERP_TYPE_PARABOLIC,          // 抛物线插补
    INTERP_TYPE_INVOLUTE            // 渐开线插补
} InterpType_t;

/**
 * @brief 插补状态
 */
typedef enum {
    INTERP_STATE_IDLE = 0,          // 空闲状态
    INTERP_STATE_ACCELERATING,      // 加速阶段
    INTERP_STATE_CONSTANT,          // 匀速阶段
    INTERP_STATE_DECELERATING,      // 减速阶段
    INTERP_STATE_FINISHED,          // 完成状态
    INTERP_STATE_ERROR              // 错误状态
} InterpState_t;

/**
 * @brief 速度模式
 */
typedef enum {
    VELOCITY_MODE_TRAPEZOIDAL = 0,  // 梯形速度曲线
    VELOCITY_MODE_S_CURVE,          // S曲线
    VELOCITY_MODE_T_CURVE,          // T曲线
    VELOCITY_MODE_SMOOTH,           // 平滑曲线
    VELOCITY_MODE_CUSTOM            // 自定义曲线
} VelocityMode_t;

/* 结构体定义 ----------------------------------------------------------------*/

/**
 * @brief 3D点结构
 */
typedef struct {
    float x;                        // X坐标
    float y;                        // Y坐标
    float z;                        // Z坐标
} Point3D_t;

/**
 * @brief 插补参数结构
 */
typedef struct {
    float feed_rate;                // 进给速度 (mm/min)
    float acceleration;             // 加速度 (mm/s²)
    float jerk;                     // 加加速度 (mm/s³)
    float tolerance;                // 容差 (mm)
    VelocityMode_t velocity_mode;   // 速度模式
    bool lookahead_enable;          // 前瞻使能
    float corner_velocity_ratio;    // 拐角速度比例
} InterpParams_t;

/**
 * @brief 直线插补结构
 */
typedef struct {
    Point3D_t start_point;          // 起点
    Point3D_t end_point;            // 终点
    Point3D_t direction;            // 方向向量
    float length;                   // 长度
} LinearInterp_t;

/**
 * @brief 圆弧插补结构
 */
typedef struct {
    Point3D_t center;               // 圆心
    Point3D_t start_point;          // 起点
    Point3D_t end_point;            // 终点
    Point3D_t normal;               // 法向量
    float radius;                   // 半径
    float start_angle;              // 起始角度
    float sweep_angle;              // 扫描角度
    bool clockwise;                 // 顺时针方向
} CircularInterp_t;

/**
 * @brief 螺旋插补结构
 */
typedef struct {
    Point3D_t center;               // 螺旋中心
    Point3D_t start_point;          // 起点
    Point3D_t end_point;            // 终点
    float radius_start;             // 起始半径
    float radius_end;               // 结束半径
    float pitch;                    // 螺距
    float turns;                    // 圈数
    bool clockwise;                 // 顺时针方向
} SpiralInterp_t;

/**
 * @brief NURBS插补结构
 */
typedef struct {
    Point3D_t control_points[NURBS_MAX_CONTROL_POINTS]; // 控制点
    float weights[NURBS_MAX_CONTROL_POINTS];            // 权重
    float knots[NURBS_MAX_CONTROL_POINTS + 8];          // 节点向量
    uint8_t num_control_points;                         // 控制点数量
    uint8_t degree;                                     // 曲线阶数
    float start_param;                                  // 起始参数
    float end_param;                                    // 结束参数
} NURBSInterp_t;

/**
 * @brief 插补数据块
 */
typedef struct {
    InterpType_t type;              // 插补类型
    InterpParams_t params;          // 插补参数
    InterpState_t state;            // 插补状态
    
    union {
        LinearInterp_t linear;      // 直线插补数据
        CircularInterp_t circular;  // 圆弧插补数据
        SpiralInterp_t spiral;      // 螺旋插补数据
        NURBSInterp_t nurbs;        // NURBS插补数据
    } data;
    
    // 运行时状态
    float current_parameter;        // 当前参数
    float current_velocity;         // 当前速度
    float current_acceleration;     // 当前加速度
    Point3D_t current_position;     // 当前位置
    Point3D_t current_direction;    // 当前方向
    float remaining_distance;       // 剩余距离
    uint32_t start_time;           // 开始时间
    uint32_t estimated_time;       // 预估时间
} InterpBlock_t;

/**
 * @brief 前瞻控制结构
 */
typedef struct {
    InterpBlock_t blocks[INTERP_LOOKAHEAD_DEPTH]; // 前瞻缓冲区
    uint8_t head;                               // 头指针
    uint8_t tail;                               // 尾指针
    uint8_t count;                              // 数据量
    bool enabled;                               // 前瞻使能
    float min_corner_velocity;                  // 最小拐角速度
    float max_corner_acceleration;              // 最大拐角加速度
} LookaheadControl_t;

/**
 * @brief 轮廓误差控制结构
 */
typedef struct {
    float cross_coupling_gain;      // 交叉耦合增益
    float contour_tolerance;        // 轮廓容差
    float max_contour_error;        // 最大轮廓误差
    Point3D_t error_vector;         // 误差向量
    bool enable;                    // 使能标志
} ContourErrorControl_t;

/**
 * @brief 高级插补控制器
 */
typedef struct {
    // 插补缓冲区
    InterpBlock_t buffer[INTERP_BUFFER_SIZE];
    uint16_t buffer_head;           // 缓冲区头指针
    uint16_t buffer_tail;           // 缓冲区尾指针
    uint16_t buffer_count;          // 缓冲区数据量
    
    // 当前执行状态
    InterpBlock_t *current_block;   // 当前执行块
    float interpolation_period;     // 插补周期 (ms)
    uint32_t cycle_counter;         // 周期计数器
    
    // 前瞻控制
    LookaheadControl_t lookahead;   // 前瞻控制
    
    // 轮廓误差控制
    ContourErrorControl_t contour_control; // 轮廓误差控制
    
    // 性能统计
    uint32_t blocks_processed;      // 已处理块数
    float average_velocity;         // 平均速度
    float peak_velocity;            // 峰值速度
    float total_distance;           // 总距离
    uint32_t total_time;           // 总时间
} AdvancedInterpController_t;

/* 全局变量声明 --------------------------------------------------------------*/
extern AdvancedInterpController_t g_adv_interp_controller;

/* 函数声明 ------------------------------------------------------------------*/

/* 系统初始化和管理 */
HAL_StatusTypeDef AdvInterp_Init(float interpolation_period);
HAL_StatusTypeDef AdvInterp_Reset(void);
HAL_StatusTypeDef AdvInterp_Start(void);
HAL_StatusTypeDef AdvInterp_Stop(void);
HAL_StatusTypeDef AdvInterp_Pause(void);
HAL_StatusTypeDef AdvInterp_Resume(void);

/* 插补数据块管理 */
HAL_StatusTypeDef AdvInterp_AddBlock(const InterpBlock_t *block);
HAL_StatusTypeDef AdvInterp_GetCurrentBlock(InterpBlock_t **block);
uint16_t AdvInterp_GetBufferCount(void);
uint16_t AdvInterp_GetBufferFreeSpace(void);
HAL_StatusTypeDef AdvInterp_ClearBuffer(void);

/* 直线插补 */
HAL_StatusTypeDef AdvInterp_PrepareLinear(InterpBlock_t *block,
                                         const Point3D_t *start,
                                         const Point3D_t *end,
                                         const InterpParams_t *params);
HAL_StatusTypeDef AdvInterp_ExecuteLinear(InterpBlock_t *block,
                                         float parameter,
                                         Point3D_t *position,
                                         Point3D_t *velocity);

/* 圆弧插补 */
HAL_StatusTypeDef AdvInterp_PrepareCircular(InterpBlock_t *block,
                                           const Point3D_t *start,
                                           const Point3D_t *end,
                                           const Point3D_t *center,
                                           bool clockwise,
                                           const InterpParams_t *params);
HAL_StatusTypeDef AdvInterp_ExecuteCircular(InterpBlock_t *block,
                                           float parameter,
                                           Point3D_t *position,
                                           Point3D_t *velocity);

/* 螺旋插补 */
HAL_StatusTypeDef AdvInterp_PrepareSpiral(InterpBlock_t *block,
                                         const Point3D_t *start,
                                         const Point3D_t *end,
                                         const Point3D_t *center,
                                         float pitch,
                                         bool clockwise,
                                         const InterpParams_t *params);
HAL_StatusTypeDef AdvInterp_ExecuteSpiral(InterpBlock_t *block,
                                         float parameter,
                                         Point3D_t *position,
                                         Point3D_t *velocity);

/* NURBS插补 */
HAL_StatusTypeDef AdvInterp_PrepareNURBS(InterpBlock_t *block,
                                        const Point3D_t *control_points,
                                        const float *weights,
                                        uint8_t num_points,
                                        uint8_t degree,
                                        const InterpParams_t *params);
HAL_StatusTypeDef AdvInterp_ExecuteNURBS(InterpBlock_t *block,
                                        float parameter,
                                        Point3D_t *position,
                                        Point3D_t *velocity);

/* 前瞻控制 */
HAL_StatusTypeDef AdvInterp_EnableLookahead(bool enable);
HAL_StatusTypeDef AdvInterp_SetLookaheadParams(float min_corner_velocity,
                                              float max_corner_acceleration);
HAL_StatusTypeDef AdvInterp_ProcessLookahead(void);
float AdvInterp_CalculateCornerVelocity(const InterpBlock_t *block1,
                                       const InterpBlock_t *block2);

/* 轮廓误差控制 */
HAL_StatusTypeDef AdvInterp_EnableContourControl(bool enable);
HAL_StatusTypeDef AdvInterp_SetContourParams(float coupling_gain,
                                            float tolerance);
HAL_StatusTypeDef AdvInterp_UpdateContourError(const Point3D_t *command_pos,
                                              const Point3D_t *actual_pos);
float AdvInterp_GetContourError(void);

/* 速度规划 */
HAL_StatusTypeDef AdvInterp_PlanVelocity(InterpBlock_t *block);
HAL_StatusTypeDef AdvInterp_OptimizeVelocityProfile(InterpBlock_t *blocks,
                                                   uint8_t count);
float AdvInterp_CalculateMaxVelocity(const InterpBlock_t *block,
                                    float curvature);

/* 工具函数 */
float AdvInterp_CalculateDistance(const Point3D_t *point1,
                                const Point3D_t *point2);
float AdvInterp_CalculateAngle(const Point3D_t *vec1,
                             const Point3D_t *vec2);
HAL_StatusTypeDef AdvInterp_NormalizeVector(Point3D_t *vector);
HAL_StatusTypeDef AdvInterp_CrossProduct(const Point3D_t *vec1,
                                        const Point3D_t *vec2,
                                        Point3D_t *result);
float AdvInterp_DotProduct(const Point3D_t *vec1,
                         const Point3D_t *vec2);

/* 性能监控 */
HAL_StatusTypeDef AdvInterp_GetPerformanceStats(uint32_t *blocks_processed,
                                               float *average_velocity,
                                               float *peak_velocity,
                                               uint32_t *total_time);
HAL_StatusTypeDef AdvInterp_ResetPerformanceStats(void);

/* 主要执行函数 */
HAL_StatusTypeDef AdvInterp_Execute(void);
HAL_StatusTypeDef AdvInterp_Cycle(void);

#ifdef __cplusplus
}
#endif

#endif /* __ADVANCED_INTERPOLATION_H */ 