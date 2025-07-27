/**
  ******************************************************************************
  * @file    motion_control.h
  * @brief   多轴运动控制系统头文件 - 高级算法版本
  * @author  Claude AI & Cursor
  * @version 2.0
  * @date    2025-01-27
  ******************************************************************************
  * @attention
  * 
  * 本文件定义了完整的4轴CNC运动控制系统，包括：
  * - 多轴协调控制
  * - G-code协议支持  
  * - 高级运动轨迹规划（梯形速度、S曲线）
  * - 高精度插补算法（DDA、逐点比较法、样条曲线）
  * - 上位机通信接口
  * 
  ******************************************************************************
  */

#ifndef __MOTION_CONTROL_H
#define __MOTION_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

/* Forward declarations -----------------------------------------------------*/
struct __UART_HandleTypeDef;

/* 系统配置 -------------------------------------------------------------------*/
#define MAX_AXES                4           // 最大轴数 (X, Y, Z, A)
#define MAX_COMMAND_LENGTH      128         // 最大命令长度
#define MOTION_BUFFER_SIZE      64          // 运动缓冲区大小
#define INTERPOLATION_FREQ      1000        // 插补频率 (Hz)
#define MAX_SPLINE_POINTS       32          // 样条曲线最大控制点数

/* 数学常量 ------------------------------------------------------------------*/
#ifndef M_PI
#define M_PI                    3.14159265358979323846f
#endif
#ifndef M_PI_2
#define M_PI_2                  1.57079632679489661923f
#endif
#define EPSILON                 1e-6f       // 浮点数比较精度

/* 轴编号定义 ----------------------------------------------------------------*/
typedef enum {
    AXIS_X = 0,     // X轴
    AXIS_Y = 1,     // Y轴  
    AXIS_Z = 2,     // Z轴
    AXIS_A = 3      // A轴 (当前步进电机轴)
} AxisNumber_t;

/* 运动模式定义 --------------------------------------------------------------*/
typedef enum {
    MOTION_IDLE = 0,        // 空闲状态
    MOTION_RAPID,           // 快速移动 (G00)
    MOTION_LINEAR,          // 直线插补 (G01)
    MOTION_CW_ARC,          // 顺时针圆弧 (G02)
    MOTION_CCW_ARC,         // 逆时针圆弧 (G03)
    MOTION_SPLINE,          // 样条曲线插补 (高级)
    MOTION_HOMING           // 回零运动
} MotionMode_t;

/* 速度规划类型 --------------------------------------------------------------*/
typedef enum {
    VELOCITY_TRAPEZOIDAL = 0,   // 梯形速度规划
    VELOCITY_S_CURVE,           // S曲线速度规划
    VELOCITY_CONSTANT           // 恒速运动
} VelocityProfile_t;

/* 插补算法类型 --------------------------------------------------------------*/
typedef enum {
    INTERPOLATION_DDA = 0,      // DDA直线插补
    INTERPOLATION_BRESENHAM,    // Bresenham算法
    INTERPOLATION_COMPARISON,   // 逐点比较法（圆弧）
    INTERPOLATION_NURBS         // NURBS样条曲线
} InterpolationMethod_t;

/* 控制器类型 ----------------------------------------------------------------*/
typedef enum {
    CONTROLLER_OPEN_LOOP = 0,   // 开环控制
    CONTROLLER_POSITION_PID,    // 位置PID控制
    CONTROLLER_VELOCITY_PID,    // 速度PID控制  
    CONTROLLER_CASCADE_PID,     // 三环级联PID控制
    CONTROLLER_FEEDFORWARD,     // 前馈控制
    CONTROLLER_ADAPTIVE,        // 自适应控制
    POS_VELOCITY_CURRENT        // 位置+速度+电流三环控制
} ControllerType_t;

/* 控制器状态 ----------------------------------------------------------------*/
typedef enum {
    CTRL_IDLE = 0,              // 控制器空闲
    CTRL_ACTIVE,                // 控制器激活
    CTRL_TUNING,                // 参数整定中
    CTRL_ERROR                  // 控制器错误
} ControllerState_t;

/* 轴状态定义 ----------------------------------------------------------------*/
typedef enum {
    AXIS_IDLE = 0,          // 轴空闲
    AXIS_MOVING,            // 轴运动中
    AXIS_HOMING,            // 轴回零中
    AXIS_ERROR              // 轴错误
} AxisState_t;

/* 速度规划阶段 --------------------------------------------------------------*/
typedef enum {
    PHASE_ACCEL = 0,        // 加速阶段
    PHASE_CONST,            // 恒速阶段
    PHASE_DECEL,            // 减速阶段
    PHASE_S_ACCEL_1,        // S曲线加速阶段1
    PHASE_S_ACCEL_2,        // S曲线加速阶段2
    PHASE_S_CONST,          // S曲线恒速阶段
    PHASE_S_DECEL_1,        // S曲线减速阶段1
    PHASE_S_DECEL_2         // S曲线减速阶段2
} VelocityPhase_t;

/* 速度规划参数结构 ----------------------------------------------------------*/
typedef struct {
    VelocityProfile_t profile_type;     // 速度规划类型
    float start_velocity;               // 起始速度
    float target_velocity;              // 目标速度
    float end_velocity;                 // 结束速度
    float max_velocity;                 // 最大速度
    float acceleration;                 // 加速度
    float jerk;                         // 加加速度（S曲线用）
    
    // 梯形速度规划参数
    float accel_distance;               // 加速段距离
    float const_distance;               // 恒速段距离
    float decel_distance;               // 减速段距离
    float accel_time;                   // 加速时间
    float const_time;                   // 恒速时间
    float decel_time;                   // 减速时间
    
    // S曲线速度规划参数
    float s_accel_1_time;               // S曲线加速阶段1时间
    float s_accel_2_time;               // S曲线加速阶段2时间
    float s_const_time;                 // S曲线恒速时间
    float s_decel_1_time;               // S曲线减速阶段1时间
    float s_decel_2_time;               // S曲线减速阶段2时间
    
    float total_time;                   // 总运动时间
    float total_distance;               // 总运动距离
} VelocityPlan_t;

/* DDA插补器结构 -------------------------------------------------------------*/
typedef struct {
    int32_t dx, dy, dz, da;             // 各轴增量
    int32_t x_inc, y_inc, z_inc, a_inc; // 各轴增量符号
    int32_t x_err, y_err, z_err, a_err; // 各轴误差累积
    int32_t x_step, y_step, z_step, a_step; // 各轴当前步数
    int32_t total_steps;                // 总步数
    int32_t current_step;               // 当前步数
} DDAInterpolator_t;

/* 圆弧插补器结构 ------------------------------------------------------------*/
typedef struct {
    float center_x, center_y;           // 圆心坐标
    float radius;                       // 半径
    float start_angle, end_angle;       // 起始和结束角度
    float angle_increment;              // 角度增量
    uint8_t clockwise;                  // 顺时针标志
    int32_t total_steps;                // 总步数
    int32_t current_step;               // 当前步数
    
    // 逐点比较法参数
    int32_t x_err, y_err;               // 误差累积
    int32_t x_pos, y_pos;               // 当前位置
} ArcInterpolator_t;

/* 样条曲线控制点 ------------------------------------------------------------*/
typedef struct {
    float x, y, z, a;                   // 控制点坐标
    float weight;                       // 权重（NURBS）
} SplinePoint_t;

/* 样条曲线插补器 ------------------------------------------------------------*/
typedef struct {
    SplinePoint_t control_points[MAX_SPLINE_POINTS]; // 控制点数组
    float knot_vector[MAX_SPLINE_POINTS + 4];        // 节点向量
    uint8_t point_count;                             // 控制点数量
    uint8_t degree;                                  // 样条曲线阶数
    float parameter_step;                            // 参数步长
    float current_parameter;                         // 当前参数值
    uint32_t total_steps;                            // 总步数
    uint32_t current_step;                           // 当前步数
} SplineInterpolator_t;

/* PID参数结构 ---------------------------------------------------------------*/
typedef struct {
    float kp;                       // 比例增益
    float ki;                       // 积分增益
    float kd;                       // 微分增益
} PIDParams_t;

/* 性能统计结构 --------------------------------------------------------------*/
typedef struct {
    float rms_error;                // RMS误差
    float max_error;                // 最大误差
    float average_error;            // 平均误差
    float steady_state_error;       // 稳态误差
    float overshoot;                // 超调量
    float settling_time;            // 调节时间
} PerformanceStats_t;

/* PID控制器结构 -------------------------------------------------------------*/
typedef struct {
    // PID参数
    float kp;                       // 比例增益
    float ki;                       // 积分增益
    float kd;                       // 微分增益
    
    // 控制变量
    float setpoint;                 // 设定值
    float feedback;                 // 反馈值
    float output;                   // 输出值
    float error;                    // 当前误差
    float error_prev;               // 上次误差
    float error_sum;                // 误差积分
    float error_diff;               // 误差微分
    
    // 限制参数
    float output_max;               // 输出上限
    float output_min;               // 输出下限
    float integral_max;             // 积分限幅
    float integral_min;             // 积分下限
    
    // 控制参数
    float sample_time;              // 采样时间 (s)
    uint8_t integral_enable;        // 积分使能
    uint8_t differential_enable;    // 微分使能
    uint8_t anti_windup_enable;     // 抗饱和使能
    
    // 状态
    ControllerState_t state;        // 控制器状态
    uint32_t update_count;          // 更新计数
    float last_update_time;         // 上次更新时间
} PIDController_t;

/* 前馈控制器结构 ------------------------------------------------------------*/
typedef struct {
    // 前馈参数
    float position_gain;            // 位置前馈增益
    float velocity_gain;            // 速度前馈增益
    float acceleration_gain;        // 加速度前馈增益
    
    // 输入信号
    float target_position;          // 目标位置
    float target_velocity;          // 目标速度
    float target_acceleration;      // 目标加速度
    
    // 输出信号
    float position_feedforward;     // 位置前馈输出
    float velocity_feedforward;     // 速度前馈输出
    float acceleration_feedforward; // 加速度前馈输出
    float total_feedforward;        // 总前馈输出
    
    // 控制参数
    uint8_t position_ff_enable;     // 位置前馈使能
    uint8_t velocity_ff_enable;     // 速度前馈使能
    uint8_t acceleration_ff_enable; // 加速度前馈使能
    
    // 滤波器参数
    float velocity_filter_tc;       // 速度滤波时间常数
    float acceleration_filter_tc;   // 加速度滤波时间常数
    float velocity_filtered;        // 滤波后速度
    float acceleration_filtered;    // 滤波后加速度
} FeedforwardController_t;

/* 自适应控制器结构 ----------------------------------------------------------*/
typedef struct {
    // 自适应参数
    float adaptation_rate;          // 自适应率
    float forgetting_factor;        // 遗忘因子
    float reference_model_gain;     // 参考模型增益
    
    // 参数估计
    float estimated_kp;             // 估计的比例增益
    float estimated_ki;             // 估计的积分增益
    float estimated_kd;             // 估计的微分增益
    float parameter_covariance[3][3]; // 参数协方差矩阵
    
    // 负载观测
    float load_estimate;            // 负载估计值
    float load_disturbance;         // 负载扰动
    float friction_estimate;        // 摩擦力估计
    
    // 在线识别
    float input_history[10];        // 输入历史数据
    float output_history[10];       // 输出历史数据
    uint8_t history_index;          // 历史数据索引
    
    // 控制参数
    uint8_t adaptation_enable;      // 自适应使能
    uint8_t load_observer_enable;   // 负载观测使能
    float adaptation_threshold;     // 自适应阈值
    float stability_margin;         // 稳定裕度
    
    // 状态变量
    float tracking_error;           // 跟踪误差
    float adaptation_signal;        // 自适应信号
    uint32_t adaptation_count;      // 自适应计数
} AdaptiveController_t;

/* 死区补偿器结构 ------------------------------------------------------------*/
typedef struct {
    // 死区参数
    float deadzone_positive;        // 正向死区大小
    float deadzone_negative;        // 反向死区大小
    float compensation_gain;        // 补偿增益
    
    // 输入输出信号
    float input;                    // 输入信号
    float output;                   // 输出信号
    float compensation;             // 补偿量
    
    // 自学习参数
    float learning_rate;            // 学习率
    uint8_t adaptive_enable;        // 自适应使能
    float learning_threshold;       // 学习阈值
    
    // 状态变量
    int8_t direction;               // 运动方向
    uint8_t deadzone_detected;      // 死区检测标志
    uint8_t compensation_active;    // 补偿激活标志
    
    // 滤波器参数
    float filter_coefficient;       // 滤波系数
    float filtered_compensation;    // 滤波后补偿量
    
    // 补偿输出
    float compensation_output;      // 补偿输出值
    float total_compensation;       // 总补偿量
    
    // 兼容性保留
    float backlash_compensation;    // 反向间隙补偿
    uint8_t compensation_type;      // 补偿类型 (0=固定, 1=自适应)
    float compensation_delay;       // 补偿延时
    float last_direction;           // 上次运动方向
    float current_direction;        // 当前运动方向
    float direction_change_time;    // 方向改变时间
    uint8_t direction_changed;      // 方向改变标志
    float error_threshold;          // 误差阈值
    uint8_t learning_enable;        // 学习使能
    uint32_t learning_count;        // 学习计数
} DeadzoneCompensator_t;

/* 三环控制器结构 ------------------------------------------------------------*/
typedef struct {
    // 三个PID控制器
    PIDController_t position_loop;       // 位置环PID
    PIDController_t velocity_loop;       // 速度环PID
    PIDController_t current_loop;        // 电流环PID
    
    // 前馈控制器
    FeedforwardController_t feedforward; // 前馈控制器
    
    // 自适应控制器
    AdaptiveController_t adaptive;  // 自适应控制器
    
    // 死区补偿器
    DeadzoneCompensator_t deadzone; // 死区补偿器
    
    // 控制器类型和状态
    ControllerType_t controller_type; // 控制器类型
    ControllerState_t state;        // 总体状态
    
    // 设定值
    float position_setpoint;        // 位置设定值
    float velocity_setpoint;        // 速度设定值
    float current_setpoint;         // 电流设定值
    
    // 输入输出
    float position_command;         // 位置指令
    float velocity_command;         // 速度指令
    float current_command;          // 电流指令
    float position_feedback;        // 位置反馈
    float velocity_feedback;        // 速度反馈
    float current_feedback;         // 电流反馈
    float control_output;           // 最终控制输出
    
    // 各环输出
    float position_output;          // 位置环输出
    float velocity_output;          // 速度环输出
    float current_output;           // 电流环输出
    
    // 性能指标
    float position_error;           // 位置误差
    float velocity_error;           // 速度误差
    float current_error;            // 电流误差
    float steady_state_error;       // 稳态误差
    float overshoot;                // 超调量
    float settling_time;            // 调节时间
    
    // 控制参数
    uint8_t cascade_enable;         // 级联控制使能
    uint8_t feedforward_enable;     // 前馈控制使能
    uint8_t adaptive_enable;        // 自适应控制使能
    uint8_t deadzone_enable;        // 死区补偿使能
    uint8_t position_loop_enable;   // 位置环使能
    uint8_t velocity_loop_enable;   // 速度环使能
    uint8_t current_loop_enable;    // 电流环使能
    float sample_frequency;         // 采样频率 (Hz)
    
    // 诊断信息
    uint32_t control_cycles;        // 控制周期计数
    float max_position_error;       // 最大位置误差
    float rms_position_error;       // 位置误差RMS值
    uint8_t saturation_flag;        // 饱和标志
} CascadeController_t;

/* 单轴配置结构 --------------------------------------------------------------*/
typedef struct {
    // 硬件配置
    TIM_HandleTypeDef *htim;        // 定时器句柄
    uint32_t channel;               // 定时器通道
    GPIO_TypeDef *dir_port;         // 方向GPIO端口
    uint16_t dir_pin;               // 方向GPIO引脚
    GPIO_TypeDef *ena_port;         // 使能GPIO端口
    uint16_t ena_pin;               // 使能GPIO引脚
    
    // 运动参数
    float steps_per_mm;             // 每毫米步数
    float max_speed;                // 最大速度 (mm/min)
    float max_accel;                // 最大加速度 (mm/s²)
    float max_jerk;                 // 最大加加速度 (mm/s³)
    float home_speed;               // 回零速度 (mm/min)
    
    // 当前状态
    AxisState_t state;              // 轴状态
    float position;                 // 当前位置 (mm)
    float target_position;          // 目标位置 (mm)
    float current_speed;            // 当前速度 (mm/min)
    float current_accel;            // 当前加速度 (mm/s²)
    uint8_t direction;              // 当前方向 (0=负向, 1=正向)
    uint8_t enabled;                // 轴使能状态
    
    // 三环级联控制器
    CascadeController_t controller; // 三环级联控制器
    
    // 编码器反馈
    float encoder_position;         // 编码器位置反馈
    float encoder_velocity;         // 编码器速度反馈
    float encoder_resolution;       // 编码器分辨率 (脉冲/mm)
    uint8_t encoder_enabled;        // 编码器使能标志
    
    // 电流反馈
    float current_feedback;         // 电流反馈值
    float current_limit;            // 电流限制值
    uint8_t current_sensor_enabled; // 电流传感器使能
} AxisConfig_t;

/* 插补数据结构 --------------------------------------------------------------*/
typedef struct {
    MotionMode_t mode;                          // 运动模式
    InterpolationMethod_t interp_method;        // 插补算法
    VelocityProfile_t velocity_profile;         // 速度规划类型
    
    float start_pos[MAX_AXES];                  // 起始位置
    float end_pos[MAX_AXES];                    // 结束位置
    float center_pos[2];                        // 圆弧中心 (仅X,Y)
    float feed_rate;                            // 进给速度 (mm/min)
    float length;                               // 运动长度
    
    // 速度规划
    VelocityPlan_t velocity_plan;               // 速度规划参数
    VelocityPhase_t current_phase;              // 当前速度阶段
    
    // 插补器
    union {
        DDAInterpolator_t dda;                  // DDA插补器
        ArcInterpolator_t arc;                  // 圆弧插补器
        SplineInterpolator_t spline;            // 样条插补器
    } interpolator;
    
    uint32_t steps_total;                       // 总插补步数
    uint32_t steps_current;                     // 当前插补步数
    float time_elapsed;                         // 已用时间
    uint8_t active;                             // 运动块是否激活
} MotionBlock_t;

/* G-code解析结果结构 --------------------------------------------------------*/
typedef struct {
    uint8_t has_G;                  // 是否有G指令
    uint8_t G_code;                 // G指令代码
    uint8_t has_M;                  // 是否有M指令
    uint8_t M_code;                 // M指令代码
    uint8_t has_F;                  // 是否有F参数
    float F_value;                  // F参数值 (进给速度)
    uint8_t has_pos[MAX_AXES];      // 各轴是否有位置参数
    float pos_value[MAX_AXES];      // 各轴位置参数值
    uint8_t has_I, has_J;           // 是否有圆弧参数
    float I_value, J_value;         // 圆弧参数值
} GCodeCommand_t;

/* 系统状态结构 --------------------------------------------------------------*/
typedef struct {
    AxisConfig_t axes[MAX_AXES];    // 各轴配置
    MotionMode_t current_mode;      // 当前运动模式
    float current_feed_rate;        // 当前进给速度
    MotionBlock_t motion_buffer[MOTION_BUFFER_SIZE];  // 运动缓冲区
    uint8_t buffer_head;            // 缓冲区头指针
    uint8_t buffer_tail;            // 缓冲区尾指针
    uint8_t buffer_count;           // 缓冲区中的运动块数量
    uint8_t system_running;         // 系统运行标志
    uint8_t emergency_stop;         // 急停标志
} MotionControlSystem_t;

/* 全局变量声明 --------------------------------------------------------------*/
extern MotionControlSystem_t g_motion_system;
extern char g_command_buffer[MAX_COMMAND_LENGTH];
extern uint8_t g_command_ready;

/* 函数声明 ------------------------------------------------------------------*/

/* 系统初始化函数 */
void MotionControl_Init(void);
void MotionControl_ConfigAxis(AxisNumber_t axis, TIM_HandleTypeDef *htim, uint32_t channel,
                             GPIO_TypeDef *dir_port, uint16_t dir_pin,
                             GPIO_TypeDef *ena_port, uint16_t ena_pin,
                             float steps_per_mm, float max_speed, float max_accel);

/* 轴控制函数 */
void Axis_Enable(AxisNumber_t axis);
void Axis_Disable(AxisNumber_t axis);
void Axis_SetDirection(AxisNumber_t axis, uint8_t direction);
void Axis_SetSpeed(AxisNumber_t axis, float speed_mm_min);
void Axis_StartPWM(AxisNumber_t axis);
void Axis_StopPWM(AxisNumber_t axis);
void Axis_MoveToPosition(AxisNumber_t axis, float position, float speed);

/* 速度规划函数 */
void VelocityPlanning_Trapezoidal(VelocityPlan_t *plan, float distance, 
                                  float start_vel, float target_vel, float end_vel,
                                  float max_vel, float acceleration);
void VelocityPlanning_SCurve(VelocityPlan_t *plan, float distance,
                            float start_vel, float target_vel, float end_vel,
                            float max_vel, float acceleration, float jerk);
float VelocityPlanning_GetCurrentVelocity(const VelocityPlan_t *plan, float time);
float VelocityPlanning_GetCurrentPosition(const VelocityPlan_t *plan, float time);

/* DDA插补算法 */
void DDA_Init(DDAInterpolator_t *dda, float start[MAX_AXES], float end[MAX_AXES]);
uint8_t DDA_Step(DDAInterpolator_t *dda, int32_t step_output[MAX_AXES]);
void DDA_Reset(DDAInterpolator_t *dda);

/* 圆弧插补算法 */
void Arc_Init(ArcInterpolator_t *arc, float start[2], float end[2], 
              float center[2], uint8_t clockwise, float resolution);
uint8_t Arc_Step(ArcInterpolator_t *arc, float position[2]);
void Arc_PointComparison_Init(ArcInterpolator_t *arc, float start[2], float end[2], 
                             float center[2], uint8_t clockwise);
uint8_t Arc_PointComparison_Step(ArcInterpolator_t *arc, int32_t step_output[2]);

/* 样条曲线插补算法 */
void Spline_Init(SplineInterpolator_t *spline, SplinePoint_t points[], 
                uint8_t point_count, uint8_t degree);
uint8_t Spline_Step(SplineInterpolator_t *spline, float position[MAX_AXES]);
void Spline_NURBS_Evaluate(const SplineInterpolator_t *spline, float t, float result[MAX_AXES]);
float Spline_BasisFunction(float t, int i, int degree, const float *knots);

/* 运动控制函数 */
void Motion_RapidMove(float pos[MAX_AXES]);
void Motion_LinearMove(float pos[MAX_AXES], float feed_rate);
void Motion_ArcMove(float pos[MAX_AXES], float center[2], float feed_rate, uint8_t clockwise);
void Motion_SplineMove(SplinePoint_t points[], uint8_t point_count, float feed_rate);
void Motion_Home(AxisNumber_t axis);
void Motion_HomeAll(void);
void Motion_Stop(void);
void Motion_EmergencyStop(void);

/* 插补算法函数 */
void Interpolation_Linear(MotionBlock_t *block, float progress, float result_pos[MAX_AXES]);
void Interpolation_Arc(MotionBlock_t *block, float progress, float result_pos[MAX_AXES]);
void Interpolation_Spline(MotionBlock_t *block, float progress, float result_pos[MAX_AXES]);
void Interpolation_Execute(void);

/* 高级运动规划 */
void MotionPlanning_OptimizeVelocity(MotionBlock_t *block);
void MotionPlanning_LookAhead(MotionBlock_t *blocks, uint8_t count);
void MotionPlanning_CornerSmoothing(MotionBlock_t *block1, MotionBlock_t *block2);

/* G-code解析函数 */
uint8_t GCode_Parse(const char *command, GCodeCommand_t *result);
void GCode_Execute(const GCodeCommand_t *command);
void GCode_ProcessLine(const char *line);

/* G-code执行函数 */
void GCode_ExecuteG00(const GCodeCommand_t *command);
void GCode_ExecuteG01(const GCodeCommand_t *command);
void GCode_ExecuteG02(const GCodeCommand_t *command);
void GCode_ExecuteG03(const GCodeCommand_t *command);
void GCode_ExecuteG28(const GCodeCommand_t *command);
void GCode_ExecuteM114(void);

/* 通信接口函数 */
void Communication_Init(void);
void Communication_SendResponse(const char *response);
void Communication_SendStatus(void);
void Communication_ProcessCommand(void);
void Communication_ShowSettings(void);
void Communication_SendExamples(void);
void Communication_SendHelp(void);
void Communication_SendVersion(void);

/* UART中断处理函数 */
void HAL_UART_RxCpltCallback(struct __UART_HandleTypeDef *huart);
void HAL_UART_TxCpltCallback(struct __UART_HandleTypeDef *huart);
void HAL_UART_ErrorCallback(struct __UART_HandleTypeDef *huart);
void USART2_IRQHandler(void);

/* 轨迹规划函数 */
float TrajectoryPlanning_CalcLength(float start[MAX_AXES], float end[MAX_AXES]);
float TrajectoryPlanning_CalcArcLength(float start[2], float end[2], float center[2], uint8_t clockwise);
uint32_t TrajectoryPlanning_CalcSteps(float length, float feed_rate);

/* 数学工具函数 */
float Math_Distance2D(float x1, float y1, float x2, float y2);
float Math_Distance3D(float x1, float y1, float z1, float x2, float y2, float z2);
float Math_AngleBetween(float x1, float y1, float x2, float y2);
void Math_RotatePoint(float *x, float *y, float cx, float cy, float angle);

/* 系统监控函数 */
void Monitor_UpdateAxisPositions(void);
void Monitor_CheckLimits(void);
void Monitor_UpdateStatus(void);

/* PID控制器函数 -------------------------------------------------------------*/
void PID_Init(PIDController_t *pid, float kp, float ki, float kd, float sample_time);
void PID_SetParameters(PIDController_t *pid, float kp, float ki, float kd);
void PID_SetLimits(PIDController_t *pid, float output_min, float output_max, 
                   float integral_min, float integral_max);
void PID_SetSetpoint(PIDController_t *pid, float setpoint);
float PID_Update(PIDController_t *pid, float feedback);
void PID_Reset(PIDController_t *pid);
void PID_EnableAntiWindup(PIDController_t *pid, uint8_t enable);

/* 前馈控制器函数 ------------------------------------------------------------*/
void Feedforward_Init(FeedforwardController_t *ff);
void Feedforward_SetGains(FeedforwardController_t *ff, float pos_gain, 
                         float vel_gain, float accel_gain);
void Feedforward_SetInputs(FeedforwardController_t *ff, float target_pos, 
                          float target_vel, float target_accel);
float Feedforward_Update(FeedforwardController_t *ff);
void Feedforward_EnableFilters(FeedforwardController_t *ff, float vel_tc, float accel_tc);

/* 自适应控制器函数 ----------------------------------------------------------*/
void Adaptive_Init(AdaptiveController_t *adaptive);
void Adaptive_SetParameters(AdaptiveController_t *adaptive, float adaptation_rate, 
                           float forgetting_factor);
void Adaptive_UpdateParameters(AdaptiveController_t *adaptive, float input, float output, 
                              float reference);
void Adaptive_EstimateLoad(AdaptiveController_t *adaptive, float torque_input, 
                          float velocity_output);
float Adaptive_GetEstimatedParameter(AdaptiveController_t *adaptive, uint8_t param_index);

/* 死区补偿器函数 ------------------------------------------------------------*/
void Deadzone_Init(DeadzoneCompensator_t *deadzone);
void Deadzone_SetParameters(DeadzoneCompensator_t *deadzone, float pos_deadzone, 
                           float neg_deadzone, float backlash_comp);
float Deadzone_Compensate(DeadzoneCompensator_t *deadzone, float input, float velocity);
void Deadzone_UpdateDirection(DeadzoneCompensator_t *deadzone, float velocity);
void Deadzone_EnableLearning(DeadzoneCompensator_t *deadzone, uint8_t enable, 
                            float learning_rate);

/* 三环控制器函数 ------------------------------------------------------------*/
void CascadeController_Init(CascadeController_t *controller, ControllerType_t type);
void CascadeController_ConfigPositionLoop(CascadeController_t *controller, float kp, 
                                         float ki, float kd);
void CascadeController_ConfigVelocityLoop(CascadeController_t *controller, float kp, 
                                         float ki, float kd);
void CascadeController_ConfigCurrentLoop(CascadeController_t *controller, float kp, 
                                        float ki, float kd);
void CascadeController_SetCommand(CascadeController_t *controller, float position_cmd, 
                                 float velocity_cmd, float current_cmd);
void CascadeController_SetFeedback(CascadeController_t *controller, float position_fb, 
                                  float velocity_fb, float current_fb);
float CascadeController_Update(CascadeController_t *controller);
void CascadeController_Reset(CascadeController_t *controller);
void CascadeController_EnableFeatures(CascadeController_t *controller, uint8_t cascade, 
                                     uint8_t feedforward, uint8_t adaptive, uint8_t deadzone);

/* 三环控制器别名函数声明（兼容性） ------------------------------------------*/
void Cascade_Init(CascadeController_t *cascade);
void Cascade_SetParameters(CascadeController_t *cascade,
                          float pos_kp, float pos_ki, float pos_kd,
                          float vel_kp, float vel_ki, float vel_kd,
                          float cur_kp, float cur_ki, float cur_kd);
void Cascade_SetSetpoints(CascadeController_t *cascade, float position_ref, 
                         float velocity_ref, float current_ref);
void Cascade_SetFeedback(CascadeController_t *cascade, float position_fb, 
                        float velocity_fb, float current_fb);
float Cascade_Update(CascadeController_t *cascade, float position_ref, 
                    float velocity_ref, float current_ref);
void Cascade_Reset(CascadeController_t *cascade);

/* 位置控制算法函数 ----------------------------------------------------------*/
HAL_StatusTypeDef PositionControl_Init(AxisNumber_t axis);
float PositionControl_Execute(AxisNumber_t axis, float position_ref, float velocity_ref, float acceleration_ref);
HAL_StatusTypeDef PositionControl_SetParameters(AxisNumber_t axis, ControllerType_t control_mode,
                                               PIDParams_t pos_pid, PIDParams_t vel_pid, PIDParams_t cur_pid);
void PositionControl_SetTarget(AxisNumber_t axis, float target_position);
void PositionControl_UpdateFeedback(AxisNumber_t axis, float encoder_position, 
                                   float encoder_velocity, float current_feedback);
void PositionControl_TuneParameters(AxisNumber_t axis, float pos_kp, float pos_ki, float pos_kd,
                                   float vel_kp, float vel_ki, float vel_kd,
                                   float cur_kp, float cur_ki, float cur_kd);
HAL_StatusTypeDef PositionControl_EnableAdaptive(AxisNumber_t axis, uint8_t enable);
HAL_StatusTypeDef PositionControl_EnableFeedforward(AxisNumber_t axis, uint8_t enable);
HAL_StatusTypeDef PositionControl_EnableDeadzone(AxisNumber_t axis, uint8_t enable);
float PositionControl_GetError(AxisNumber_t axis);
float PositionControl_GetOutput(AxisNumber_t axis);

/* 自动整定函数 --------------------------------------------------------------*/
void AutoTuning_Start(AxisNumber_t axis, uint8_t loop_type);
void AutoTuning_Stop(AxisNumber_t axis);
uint8_t AutoTuning_IsComplete(AxisNumber_t axis);
void AutoTuning_GetResults(AxisNumber_t axis, float *kp, float *ki, float *kd);
void AutoTuning_ZieglerNichols(AxisNumber_t axis, float *kp, float *ki, float *kd);
void AutoTuning_CohenCoon(AxisNumber_t axis, float *kp, float *ki, float *kd);
HAL_StatusTypeDef AutoTune_CalculateParameters(AxisNumber_t axis);
HAL_StatusTypeDef AutoTune_Update(AxisNumber_t axis);
HAL_StatusTypeDef AutoTune_Start(AxisNumber_t axis, uint8_t loop_type, uint8_t method);
HAL_StatusTypeDef AutoTune_Stop(AxisNumber_t axis);
uint8_t AutoTune_IsRunning(AxisNumber_t axis);
HAL_StatusTypeDef AutoTune_GetResults(AxisNumber_t axis, PIDParams_t *pos_params, 
                                    PIDParams_t *vel_params, PIDParams_t *cur_params);

/* 性能分析函数 --------------------------------------------------------------*/
void Performance_AnalyzeResponse(AxisNumber_t axis, float *overshoot, float *settling_time, 
                               float *steady_error);
void Performance_CalculateRMS(AxisNumber_t axis, float *rms_position_error, 
                             float *rms_velocity_error);
void Performance_GetStatistics(AxisNumber_t axis, float *max_error, float *avg_error, 
                              float *std_deviation);
HAL_StatusTypeDef Performance_UpdateStatistics(AxisNumber_t axis, float position_error, 
                                              float velocity_error);
HAL_StatusTypeDef Performance_GetPerformanceData(AxisNumber_t axis, PerformanceStats_t *stats);
HAL_StatusTypeDef Performance_ResetStatistics(AxisNumber_t axis);

/* 诊断和调试函数 ------------------------------------------------------------*/
void Diagnostics_LogControllerData(AxisNumber_t axis);
void Diagnostics_CheckStability(AxisNumber_t axis, uint8_t *stable_flag);
void Diagnostics_DetectOscillation(AxisNumber_t axis, uint8_t *oscillation_flag);
void Diagnostics_MonitorSaturation(AxisNumber_t axis, uint8_t *saturation_flag);
HAL_StatusTypeDef Diagnostics_UpdateData(AxisNumber_t axis, float control_output, 
                                        float position_error);
HAL_StatusTypeDef Diagnostics_CheckControllerHealth(AxisNumber_t axis, uint8_t *health_status);
HAL_StatusTypeDef Diagnostics_GetDiagnosticInfo(AxisNumber_t axis, char *info_buffer, 
                                               uint16_t buffer_size);

/* 演示程序函数 ------------------------------------------------------------*/
void PositionControl_Demo(void);
uint8_t PositionControl_IsDemoRunning(void);
uint32_t PositionControl_GetDemoTime(void);

/* 测试函数声明 ------------------------------------------------------------*/
void Test_AdvancedMotion_RunAll(void);
void Test_VelocityPlanning_Trapezoidal(void);
void Test_VelocityPlanning_SCurve(void);
void Test_DDA_Interpolation(void);
void Test_Arc_Interpolation(void);
void Test_Arc_PointComparison(void);
void Test_Spline_Interpolation(void);
void Test_Performance_Benchmark(void);

/* 回调函数 */
void MotionControl_TimerCallback(void);  // 插补定时器回调
void MotionControl_UARTCallback(void);   // 串口接收回调
void PositionControl_Callback(void);     // 位置控制回调 (高频)

#ifdef __cplusplus
}
#endif

#endif /* __MOTION_CONTROL_H */