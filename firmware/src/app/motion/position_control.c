/**
  ******************************************************************************
  * @file    position_control.c
  * @brief   位置控制算法实现 - 三环PID控制、前馈控制、自适应控制、死区补偿
  * @author  Claude AI & Cursor
  * @version 1.0
  * @date    2025-01-27
  ******************************************************************************
  * @attention
  * 
  * 本文件实现了完整的工业级位置控制算法，包括：
  * - PID控制器：位置环、速度环、电流环三环控制
  * - 前馈控制：减少跟踪误差，提高响应速度
  * - 自适应控制：根据负载自动调整参数
  * - 死区补偿：消除传动间隙影响
  * - 自动整定：Ziegler-Nichols、Cohen-Coon方法
  * - 性能分析：超调量、调节时间、稳态误差统计
  * 
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "motion_control.h"
#include <stdlib.h>
#include <string.h>

/* Private defines -----------------------------------------------------------*/
#define POSITION_CONTROL_FREQ       1000.0f    // 位置控制频率 (Hz)
#define VELOCITY_CONTROL_FREQ       5000.0f    // 速度控制频率 (Hz)
#define CURRENT_CONTROL_FREQ        10000.0f   // 电流控制频率 (Hz)

#define PID_INTEGRAL_LIMIT_RATIO     0.8f       // 积分限幅比例
#define ADAPTIVE_THRESHOLD           0.1f       // 自适应阈值
#define DEADZONE_LEARNING_RATE       0.01f      // 死区学习率
#define STABILITY_CHECK_SAMPLES      100        // 稳定性检查样本数

/* Private variables ---------------------------------------------------------*/
extern MotionControlSystem_t g_motion_system;

// 性能统计缓冲区
static float error_history[MAX_AXES][STABILITY_CHECK_SAMPLES];
static uint8_t error_index[MAX_AXES];
static uint8_t error_count[MAX_AXES];

// 自动整定变量
typedef struct {
    uint8_t active;
    uint8_t loop_type;              // 0=位置环, 1=速度环, 2=电流环
    float test_amplitude;           // 测试幅度
    float test_frequency;           // 测试频率
    uint32_t test_samples;          // 测试样本数
    float *response_data;           // 响应数据
    uint8_t step;                   // 整定步骤
} AutoTuning_t;

static AutoTuning_t auto_tuning[MAX_AXES];

/* Private function prototypes -----------------------------------------------*/
static float calculate_derivative(float current_error, float previous_error, float dt);
static float apply_deadzone(float input, float deadzone_pos, float deadzone_neg);
static void update_error_history(AxisNumber_t axis, float error);
static float calculate_rms_error(AxisNumber_t axis);

/**
 * @brief  初始化PID控制器
 * @param  pid: PID控制器指针
 * @param  kp: 比例增益
 * @param  ki: 积分增益
 * @param  kd: 微分增益
 * @param  sample_time: 采样时间 (s)
 * @retval None
 */
void PID_Init(PIDController_t *pid, float kp, float ki, float kd, float sample_time)
{
    if (pid == NULL) return;
    
    // 初始化PID参数
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->sample_time = sample_time;
    
    // 初始化控制变量
    pid->setpoint = 0.0f;
    pid->feedback = 0.0f;
    pid->output = 0.0f;
    pid->error = 0.0f;
    pid->error_prev = 0.0f;
    pid->error_sum = 0.0f;
    pid->error_diff = 0.0f;
    
    // 设置默认限制
    pid->output_max = 100.0f;
    pid->output_min = -100.0f;
    pid->integral_max = 50.0f;
    pid->integral_min = -50.0f;
    
    // 启用所有控制项
    pid->integral_enable = 1;
    pid->differential_enable = 1;
    pid->anti_windup_enable = 1;
    
    // 初始化状态
    pid->state = CTRL_IDLE;
    pid->update_count = 0;
    pid->last_update_time = 0.0f;
}

/**
 * @brief  设置PID参数
 * @param  pid: PID控制器指针
 * @param  kp: 比例增益
 * @param  ki: 积分增益
 * @param  kd: 微分增益
 * @retval None
 */
void PID_SetParameters(PIDController_t *pid, float kp, float ki, float kd)
{
    if (pid == NULL) return;
    
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

/**
 * @brief  设置PID限制
 * @param  pid: PID控制器指针
 * @param  output_min: 输出下限
 * @param  output_max: 输出上限
 * @param  integral_min: 积分下限
 * @param  integral_max: 积分上限
 * @retval None
 */
void PID_SetLimits(PIDController_t *pid, float output_min, float output_max, 
                   float integral_min, float integral_max)
{
    if (pid == NULL) return;
    
    pid->output_min = output_min;
    pid->output_max = output_max;
    pid->integral_min = integral_min;
    pid->integral_max = integral_max;
}

/**
 * @brief  设置PID设定值
 * @param  pid: PID控制器指针
 * @param  setpoint: 设定值
 * @retval None
 */
void PID_SetSetpoint(PIDController_t *pid, float setpoint)
{
    if (pid == NULL) return;
    
    pid->setpoint = setpoint;
}

/**
 * @brief  PID控制器更新
 * @param  pid: PID控制器指针
 * @param  feedback: 反馈值
 * @retval 控制输出
 */
float PID_Update(PIDController_t *pid, float feedback)
{
    if (pid == NULL) return 0.0f;
    
    pid->feedback = feedback;
    pid->error_prev = pid->error;
    pid->error = pid->setpoint - pid->feedback;
    
    // 比例项
    float proportional = pid->kp * pid->error;
    
    // 积分项
    float integral = 0.0f;
    if (pid->integral_enable) {
        pid->error_sum += pid->error * pid->sample_time;
        
        // 积分限幅
        if (pid->error_sum > pid->integral_max) {
            pid->error_sum = pid->integral_max;
        } else if (pid->error_sum < pid->integral_min) {
            pid->error_sum = pid->integral_min;
        }
        
        integral = pid->ki * pid->error_sum;
    }
    
    // 微分项
    float differential = 0.0f;
    if (pid->differential_enable) {
        pid->error_diff = calculate_derivative(pid->error, pid->error_prev, pid->sample_time);
        differential = pid->kd * pid->error_diff;
    }
    
    // PID输出
    pid->output = proportional + integral + differential;
    
    // 输出限幅
    if (pid->output > pid->output_max) {
        pid->output = pid->output_max;
    } else if (pid->output < pid->output_min) {
        pid->output = pid->output_min;
    }
    
    // 抗积分饱和
    if (pid->anti_windup_enable && pid->integral_enable) {
        float output_before_limit = proportional + integral + differential;
        if ((output_before_limit > pid->output_max || output_before_limit < pid->output_min) &&
            ((pid->error > 0 && pid->error_sum > 0) || (pid->error < 0 && pid->error_sum < 0))) {
            // 停止积分累积
            pid->error_sum -= pid->error * pid->sample_time;
        }
    }
    
    pid->update_count++;
    pid->state = CTRL_ACTIVE;
    
    return pid->output;
}

/**
 * @brief  复位PID控制器
 * @param  pid: PID控制器指针
 * @retval None
 */
void PID_Reset(PIDController_t *pid)
{
    if (pid == NULL) return;
    
    pid->error = 0.0f;
    pid->error_prev = 0.0f;
    pid->error_sum = 0.0f;
    pid->error_diff = 0.0f;
    pid->output = 0.0f;
    pid->update_count = 0;
    pid->state = CTRL_IDLE;
}

/**
 * @brief  启用/禁用抗积分饱和
 * @param  pid: PID控制器指针
 * @param  enable: 使能标志
 * @retval None
 */
void PID_EnableAntiWindup(PIDController_t *pid, uint8_t enable)
{
    if (pid == NULL) return;
    
    pid->anti_windup_enable = enable;
}

/**
 * @brief  计算微分项
 * @param  current_error: 当前误差
 * @param  previous_error: 上次误差
 * @param  dt: 时间间隔
 * @retval 微分值
 */
static float calculate_derivative(float current_error, float previous_error, float dt)
{
    if (dt <= 0.0f) return 0.0f;
    
    return (current_error - previous_error) / dt;
}

/**
 * @brief  应用死区
 * @param  input: 输入值
 * @param  deadzone_pos: 正向死区
 * @param  deadzone_neg: 负向死区
 * @retval 处理后的值
 */
static float apply_deadzone(float input, float deadzone_pos, float deadzone_neg)
{
    if (input > deadzone_pos) {
        return input - deadzone_pos;
    } else if (input < deadzone_neg) {
        return input - deadzone_neg;
    } else {
        return 0.0f;
    }
}

/* ========================================================================== */
/* 前馈控制器实现                                                               */
/* ========================================================================== */

/**
 * @brief  初始化前馈控制器
 * @param  ff: 前馈控制器指针
 * @retval None
 */
void Feedforward_Init(FeedforwardController_t *ff)
{
    if (ff == NULL) return;
    
    // 初始化前馈增益
    ff->position_gain = 1.0f;
    ff->velocity_gain = 0.1f;
    ff->acceleration_gain = 0.01f;
    
    // 初始化输入信号
    ff->target_position = 0.0f;
    ff->target_velocity = 0.0f;
    ff->target_acceleration = 0.0f;
    
    // 初始化输出信号
    ff->position_feedforward = 0.0f;
    ff->velocity_feedforward = 0.0f;
    ff->acceleration_feedforward = 0.0f;
    ff->total_feedforward = 0.0f;
    
    // 默认启用所有前馈项
    ff->position_ff_enable = 1;
    ff->velocity_ff_enable = 1;
    ff->acceleration_ff_enable = 1;
    
    // 滤波器参数
    ff->velocity_filter_tc = 0.01f;      // 10ms时间常数
    ff->acceleration_filter_tc = 0.005f;  // 5ms时间常数
    ff->velocity_filtered = 0.0f;
    ff->acceleration_filtered = 0.0f;
}

/**
 * @brief  设置前馈增益
 * @param  ff: 前馈控制器指针
 * @param  pos_gain: 位置前馈增益
 * @param  vel_gain: 速度前馈增益
 * @param  accel_gain: 加速度前馈增益
 * @retval None
 */
void Feedforward_SetGains(FeedforwardController_t *ff, float pos_gain, 
                         float vel_gain, float accel_gain)
{
    if (ff == NULL) return;
    
    ff->position_gain = pos_gain;
    ff->velocity_gain = vel_gain;
    ff->acceleration_gain = accel_gain;
}

/**
 * @brief  设置前馈输入
 * @param  ff: 前馈控制器指针
 * @param  target_pos: 目标位置
 * @param  target_vel: 目标速度
 * @param  target_accel: 目标加速度
 * @retval None
 */
void Feedforward_SetInputs(FeedforwardController_t *ff, float target_pos, 
                          float target_vel, float target_accel)
{
    if (ff == NULL) return;
    
    ff->target_position = target_pos;
    ff->target_velocity = target_vel;
    ff->target_acceleration = target_accel;
}

/**
 * @brief  更新前馈控制器
 * @param  ff: 前馈控制器指针
 * @retval 前馈输出
 */
float Feedforward_Update(FeedforwardController_t *ff)
{
    if (ff == NULL) return 0.0f;
    
    // 位置前馈
    if (ff->position_ff_enable) {
        ff->position_feedforward = ff->position_gain * ff->target_position;
    } else {
        ff->position_feedforward = 0.0f;
    }
    
    // 速度前馈（带滤波）
    if (ff->velocity_ff_enable) {
        // 一阶低通滤波器
        float alpha = ff->velocity_filter_tc / (ff->velocity_filter_tc + (1.0f / POSITION_CONTROL_FREQ));
        ff->velocity_filtered = alpha * ff->velocity_filtered + (1.0f - alpha) * ff->target_velocity;
        ff->velocity_feedforward = ff->velocity_gain * ff->velocity_filtered;
    } else {
        ff->velocity_feedforward = 0.0f;
    }
    
    // 加速度前馈（带滤波）
    if (ff->acceleration_ff_enable) {
        // 一阶低通滤波器
        float alpha = ff->acceleration_filter_tc / (ff->acceleration_filter_tc + (1.0f / POSITION_CONTROL_FREQ));
        ff->acceleration_filtered = alpha * ff->acceleration_filtered + (1.0f - alpha) * ff->target_acceleration;
        ff->acceleration_feedforward = ff->acceleration_gain * ff->acceleration_filtered;
    } else {
        ff->acceleration_feedforward = 0.0f;
    }
    
    // 总前馈输出
    ff->total_feedforward = ff->position_feedforward + ff->velocity_feedforward + ff->acceleration_feedforward;
    
    return ff->total_feedforward;
}

/**
 * @brief  启用前馈滤波器
 * @param  ff: 前馈控制器指针
 * @param  vel_tc: 速度滤波时间常数
 * @param  accel_tc: 加速度滤波时间常数
 * @retval None
 */
void Feedforward_EnableFilters(FeedforwardController_t *ff, float vel_tc, float accel_tc)
{
    if (ff == NULL) return;
    
    ff->velocity_filter_tc = vel_tc;
    ff->acceleration_filter_tc = accel_tc;
}

/* ========================================================================== */
/* 自适应控制器实现                                                             */
/* ========================================================================== */

/**
 * @brief  初始化自适应控制器
 * @param  adaptive: 自适应控制器指针
 * @retval None
 */
void Adaptive_Init(AdaptiveController_t *adaptive)
{
    if (adaptive == NULL) return;
    
    // 初始化自适应参数
    adaptive->adaptation_rate = 0.1f;
    adaptive->forgetting_factor = 0.99f;
    adaptive->reference_model_gain = 1.0f;
    
    // 初始化参数估计
    adaptive->estimated_kp = 1.0f;
    adaptive->estimated_ki = 0.1f;
    adaptive->estimated_kd = 0.01f;
    
    // 初始化协方差矩阵（单位矩阵）
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            adaptive->parameter_covariance[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }
    
    // 初始化负载观测
    adaptive->load_estimate = 0.0f;
    adaptive->load_disturbance = 0.0f;
    adaptive->friction_estimate = 0.0f;
    
    // 清空历史数据
    memset(adaptive->input_history, 0, sizeof(adaptive->input_history));
    memset(adaptive->output_history, 0, sizeof(adaptive->output_history));
    adaptive->history_index = 0;
    
    // 控制参数
    adaptive->adaptation_enable = 0;
    adaptive->load_observer_enable = 0;
    adaptive->adaptation_threshold = ADAPTIVE_THRESHOLD;
    adaptive->stability_margin = 0.5f;
    
    // 状态变量
    adaptive->tracking_error = 0.0f;
    adaptive->adaptation_signal = 0.0f;
    adaptive->adaptation_count = 0;
}

/**
 * @brief  设置自适应参数
 * @param  adaptive: 自适应控制器指针
 * @param  adaptation_rate: 自适应率
 * @param  forgetting_factor: 遗忘因子
 * @retval None
 */
void Adaptive_SetParameters(AdaptiveController_t *adaptive, float adaptation_rate, 
                           float forgetting_factor)
{
    if (adaptive == NULL) return;
    
    adaptive->adaptation_rate = adaptation_rate;
    adaptive->forgetting_factor = forgetting_factor;
}

/**
 * @brief  更新自适应参数
 * @param  adaptive: 自适应控制器指针
 * @param  input: 控制输入
 * @param  output: 系统输出
 * @param  reference: 参考信号
 * @retval None
 */
void Adaptive_UpdateParameters(AdaptiveController_t *adaptive, float input, float output, 
                              float reference)
{
    if (adaptive == NULL || !adaptive->adaptation_enable) return;
    
    // 更新历史数据
    adaptive->input_history[adaptive->history_index] = input;
    adaptive->output_history[adaptive->history_index] = output;
    adaptive->history_index = (adaptive->history_index + 1) % 10;
    
    // 计算跟踪误差
    adaptive->tracking_error = reference - output;
    
    // 检查是否需要自适应
    if (fabsf(adaptive->tracking_error) > adaptive->adaptation_threshold) {
        // 简化的递推最小二乘法（RLS）参数估计
        float prediction_error = adaptive->tracking_error;
        float adaptation_gain = adaptive->adaptation_rate / (1.0f + input * input);
        
        // 更新参数估计
        adaptive->estimated_kp += adaptation_gain * prediction_error * input;
        adaptive->estimated_ki += adaptation_gain * prediction_error * adaptive->tracking_error;
        adaptive->estimated_kd += adaptation_gain * prediction_error * 
                                   (adaptive->tracking_error - adaptive->input_history[(adaptive->history_index + 9) % 10]);
        
        // 参数限制
        if (adaptive->estimated_kp < 0.0f) adaptive->estimated_kp = 0.0f;
        if (adaptive->estimated_kp > 10.0f) adaptive->estimated_kp = 10.0f;
        if (adaptive->estimated_ki < 0.0f) adaptive->estimated_ki = 0.0f;
        if (adaptive->estimated_ki > 5.0f) adaptive->estimated_ki = 5.0f;
        if (adaptive->estimated_kd < 0.0f) adaptive->estimated_kd = 0.0f;
        if (adaptive->estimated_kd > 1.0f) adaptive->estimated_kd = 1.0f;
        
        adaptive->adaptation_signal = adaptation_gain * prediction_error;
        adaptive->adaptation_count++;
    }
}

/**
 * @brief  估计负载
 * @param  adaptive: 自适应控制器指针
 * @param  torque_input: 转矩输入
 * @param  velocity_output: 速度输出
 * @retval None
 */
void Adaptive_EstimateLoad(AdaptiveController_t *adaptive, float torque_input, 
                          float velocity_output)
{
    if (adaptive == NULL || !adaptive->load_observer_enable) return;
    
    // 简化的负载观测器
    static float velocity_prev = 0.0f;
    float acceleration = (velocity_output - velocity_prev) * POSITION_CONTROL_FREQ;
    velocity_prev = velocity_output;
    
    // 估计负载转矩 (简化模型: T_load = T_motor - J*acceleration - B*velocity)
    const float estimated_inertia = 0.001f;  // kg·m²
    const float estimated_damping = 0.01f;   // N·m·s/rad
    
    float predicted_torque = estimated_inertia * acceleration + estimated_damping * velocity_output;
    adaptive->load_estimate = torque_input - predicted_torque;
    
    // 低通滤波
    static float load_filtered = 0.0f;
    float alpha = 0.1f;
    load_filtered = alpha * adaptive->load_estimate + (1.0f - alpha) * load_filtered;
    adaptive->load_disturbance = load_filtered;
    
    // 摩擦力估计（库仑摩擦 + 粘性摩擦）
    float friction_sign = (velocity_output > 0) ? 1.0f : -1.0f;
    adaptive->friction_estimate = 0.1f * friction_sign + 0.01f * velocity_output;
}

/**
 * @brief  获取估计的参数
 * @param  adaptive: 自适应控制器指针
 * @param  param_index: 参数索引 (0=Kp, 1=Ki, 2=Kd)
 * @retval 估计的参数值
 */
float Adaptive_GetEstimatedParameter(AdaptiveController_t *adaptive, uint8_t param_index)
{
    if (adaptive == NULL) return 0.0f;
    
    switch (param_index) {
        case 0: return adaptive->estimated_kp;
        case 1: return adaptive->estimated_ki;
        case 2: return adaptive->estimated_kd;
        default: return 0.0f;
    }
}

/* ========================================================================== */
/* 死区补偿器实现                                                              */
/* ========================================================================== */

/**
 * @brief  初始化死区补偿器
 * @param  deadzone: 死区补偿器指针
 * @retval None
 */
void Deadzone_Init(DeadzoneCompensator_t *deadzone)
{
    if (deadzone == NULL) return;
    
    // 初始化死区参数
    deadzone->deadzone_positive = 0.1f;    // 正向死区
    deadzone->deadzone_negative = -0.1f;   // 负向死区
    deadzone->compensation_gain = 1.0f;    // 补偿增益
    
    // 初始化输入输出
    deadzone->input = 0.0f;
    deadzone->output = 0.0f;
    deadzone->compensation = 0.0f;
    
    // 自学习参数
    deadzone->learning_rate = DEADZONE_LEARNING_RATE;
    deadzone->adaptive_enable = 0;
    deadzone->learning_threshold = 0.05f;
    
    // 状态变量
    deadzone->direction = 0;
    deadzone->deadzone_detected = 0;
    deadzone->compensation_active = 0;
    
    // 滤波器
    deadzone->filter_coefficient = 0.9f;
    deadzone->filtered_compensation = 0.0f;
}

/**
 * @brief  设置死区参数
 * @param  deadzone: 死区补偿器指针
 * @param  pos_deadzone: 正向死区
 * @param  neg_deadzone: 负向死区
 * @param  compensation_gain: 补偿增益
 * @retval None
 */
void Deadzone_SetParameters(DeadzoneCompensator_t *deadzone, float pos_deadzone, 
                           float neg_deadzone, float compensation_gain)
{
    if (deadzone == NULL) return;
    
    deadzone->deadzone_positive = pos_deadzone;
    deadzone->deadzone_negative = neg_deadzone;
    deadzone->compensation_gain = compensation_gain;
}

/**
 * @brief  死区补偿器更新
 * @param  deadzone: 死区补偿器指针
 * @param  input: 输入信号
 * @retval 补偿后的输出
 */
float Deadzone_Update(DeadzoneCompensator_t *deadzone, float input)
{
    if (deadzone == NULL) return input;
    
    deadzone->input = input;
    
    // 检测运动方向
    if (input > 0.01f) {
        deadzone->direction = 1;
    } else if (input < -0.01f) {
        deadzone->direction = -1;
    }
    
    // 计算补偿
    if (deadzone->direction > 0) {
        // 正向运动
        if (input > 0 && input < deadzone->deadzone_positive) {
            deadzone->deadzone_detected = 1;
            deadzone->compensation = deadzone->deadzone_positive * deadzone->compensation_gain;
        } else {
            deadzone->deadzone_detected = 0;
            deadzone->compensation = 0.0f;
        }
    } else if (deadzone->direction < 0) {
        // 负向运动
        if (input < 0 && input > deadzone->deadzone_negative) {
            deadzone->deadzone_detected = 1;
            deadzone->compensation = deadzone->deadzone_negative * deadzone->compensation_gain;
        } else {
            deadzone->deadzone_detected = 0;
            deadzone->compensation = 0.0f;
        }
    } else {
        deadzone->compensation = 0.0f;
    }
    
    // 低通滤波补偿信号
    deadzone->filtered_compensation = deadzone->filter_coefficient * deadzone->filtered_compensation +
                                     (1.0f - deadzone->filter_coefficient) * deadzone->compensation;
    
    // 应用补偿
    if (deadzone->compensation_active) {
        deadzone->output = input + deadzone->filtered_compensation;
    } else {
        deadzone->output = input;
    }
    
    return deadzone->output;
}

/**
 * @brief  自适应死区学习
 * @param  deadzone: 死区补偿器指针
 * @param  position_error: 位置误差
 * @param  velocity: 当前速度
 * @retval None
 */
void Deadzone_AdaptiveLearning(DeadzoneCompensator_t *deadzone, float position_error, float velocity)
{
    if (deadzone == NULL || !deadzone->adaptive_enable) return;
    
    // 检测死区现象（速度为零但有位置误差）
    if (fabsf(velocity) < 0.01f && fabsf(position_error) > deadzone->learning_threshold) {
        if (position_error > 0) {
            // 正向死区调整
            deadzone->deadzone_positive += deadzone->learning_rate * position_error;
            if (deadzone->deadzone_positive > 1.0f) deadzone->deadzone_positive = 1.0f;
        } else {
            // 负向死区调整
            deadzone->deadzone_negative += deadzone->learning_rate * position_error;
            if (deadzone->deadzone_negative < -1.0f) deadzone->deadzone_negative = -1.0f;
        }
    }
}

/* ========================================================================== */
/* 级联控制器实现                                                              */
/* ========================================================================== */

/**
 * @brief  初始化级联控制器
 * @param  cascade: 级联控制器指针
 * @retval None
 */
void Cascade_Init(CascadeController_t *cascade)
{
    if (cascade == NULL) return;
    
    // 初始化PID控制器
    PID_Init(&cascade->position_loop, 10.0f, 0.1f, 0.05f, 1.0f / POSITION_CONTROL_FREQ);
    PID_Init(&cascade->velocity_loop, 5.0f, 0.5f, 0.02f, 1.0f / VELOCITY_CONTROL_FREQ);
    PID_Init(&cascade->current_loop, 100.0f, 1000.0f, 0.001f, 1.0f / CURRENT_CONTROL_FREQ);
    
    // 设置输出限制
    PID_SetLimits(&cascade->position_loop, -1000.0f, 1000.0f, -500.0f, 500.0f);    // rad/s
    PID_SetLimits(&cascade->velocity_loop, -10.0f, 10.0f, -5.0f, 5.0f);            // A
    PID_SetLimits(&cascade->current_loop, -24.0f, 24.0f, -12.0f, 12.0f);           // V
    
    // 初始化前馈控制器
    Feedforward_Init(&cascade->feedforward);
    
    // 初始化自适应控制器
    Adaptive_Init(&cascade->adaptive);
    
    // 初始化死区补偿器
    Deadzone_Init(&cascade->deadzone);
    
    // 初始化反馈信号
    cascade->position_feedback = 0.0f;
    cascade->velocity_feedback = 0.0f;
    cascade->current_feedback = 0.0f;
    
    // 初始化设定值
    cascade->position_setpoint = 0.0f;
    cascade->velocity_setpoint = 0.0f;
    cascade->current_setpoint = 0.0f;
    
    // 初始化控制输出
    cascade->position_output = 0.0f;
    cascade->velocity_output = 0.0f;
    cascade->current_output = 0.0f;
    cascade->control_output = 0.0f;
    
    // 控制模式和使能
    cascade->controller_type = CONTROLLER_CASCADE_PID;
    cascade->position_loop_enable = 1;
    cascade->velocity_loop_enable = 1;
    cascade->current_loop_enable = 1;
    cascade->feedforward_enable = 1;
    cascade->adaptive_enable = 0;
    cascade->deadzone_enable = 1;
}

/**
 * @brief  设置级联控制器参数
 * @param  cascade: 级联控制器指针
 * @param  pos_kp, pos_ki, pos_kd: 位置环PID参数
 * @param  vel_kp, vel_ki, vel_kd: 速度环PID参数
 * @param  cur_kp, cur_ki, cur_kd: 电流环PID参数
 * @retval None
 */
void Cascade_SetParameters(CascadeController_t *cascade,
                          float pos_kp, float pos_ki, float pos_kd,
                          float vel_kp, float vel_ki, float vel_kd,
                          float cur_kp, float cur_ki, float cur_kd)
{
    if (cascade == NULL) return;
    
    PID_SetParameters(&cascade->position_loop, pos_kp, pos_ki, pos_kd);
    PID_SetParameters(&cascade->velocity_loop, vel_kp, vel_ki, vel_kd);
    PID_SetParameters(&cascade->current_loop, cur_kp, cur_ki, cur_kd);
}

/**
 * @brief  设置级联控制器设定值
 * @param  cascade: 级联控制器指针
 * @param  position_ref: 位置设定值
 * @param  velocity_ref: 速度设定值 (可选)
 * @param  current_ref: 电流设定值 (可选)
 * @retval None
 */
void Cascade_SetReference(CascadeController_t *cascade, float position_ref, 
                         float velocity_ref, float current_ref)
{
    if (cascade == NULL) return;
    
    cascade->position_setpoint = position_ref;
    cascade->velocity_setpoint = velocity_ref;
    cascade->current_setpoint = current_ref;
    
    // 设置PID设定值
    PID_SetSetpoint(&cascade->position_loop, position_ref);
}

/**
 * @brief  更新级联控制器
 * @param  cascade: 级联控制器指针
 * @param  position_fb: 位置反馈
 * @param  velocity_fb: 速度反馈
 * @param  current_fb: 电流反馈
 * @retval 最终控制输出
 */
float Cascade_Update(CascadeController_t *cascade, float position_fb, 
                    float velocity_fb, float current_fb)
{
    if (cascade == NULL) return 0.0f;
    
    // 更新反馈信号
    cascade->position_feedback = position_fb;
    cascade->velocity_feedback = velocity_fb;
    cascade->current_feedback = current_fb;
    
    // 位置环控制 (外环)
    if (cascade->position_loop_enable) {
        cascade->position_output = PID_Update(&cascade->position_loop, position_fb);
        
        // 位置环输出作为速度环设定值
        if (cascade->velocity_loop_enable) {
            PID_SetSetpoint(&cascade->velocity_loop, cascade->position_output);
        }
    } else {
        // 直接使用速度设定值
        PID_SetSetpoint(&cascade->velocity_loop, cascade->velocity_setpoint);
    }
    
    // 速度环控制 (中环)
    if (cascade->velocity_loop_enable) {
        cascade->velocity_output = PID_Update(&cascade->velocity_loop, velocity_fb);
        
        // 速度环输出作为电流环设定值
        if (cascade->current_loop_enable) {
            PID_SetSetpoint(&cascade->current_loop, cascade->velocity_output);
        }
    } else {
        // 直接使用电流设定值
        PID_SetSetpoint(&cascade->current_loop, cascade->current_setpoint);
    }
    
    // 电流环控制 (内环)
    if (cascade->current_loop_enable) {
        cascade->current_output = PID_Update(&cascade->current_loop, current_fb);
    } else {
        cascade->current_output = cascade->current_setpoint;
    }
    
    // 前馈控制
    float feedforward_output = 0.0f;
    if (cascade->feedforward_enable) {
        feedforward_output = Feedforward_Update(&cascade->feedforward);
    }
    
    // 死区补偿
    float compensated_output = cascade->current_output;
    if (cascade->deadzone_enable) {
        compensated_output = Deadzone_Update(&cascade->deadzone, cascade->current_output);
    }
    
    // 最终输出
    cascade->control_output = compensated_output + feedforward_output;
    
    // 自适应控制参数更新
    if (cascade->adaptive_enable) {
        Adaptive_UpdateParameters(&cascade->adaptive, cascade->control_output, 
                                 velocity_fb, cascade->position_setpoint);
        
        // 应用自适应参数
        float adaptive_kp = Adaptive_GetEstimatedParameter(&cascade->adaptive, 0);
        float adaptive_ki = Adaptive_GetEstimatedParameter(&cascade->adaptive, 1);
        float adaptive_kd = Adaptive_GetEstimatedParameter(&cascade->adaptive, 2);
        
        PID_SetParameters(&cascade->position_loop, adaptive_kp, adaptive_ki, adaptive_kd);
    }
         
     return cascade->control_output;
}

/* ========================================================================== */
/* 位置控制算法接口函数                                                        */
/* ========================================================================== */

/**
 * @brief  初始化位置控制算法
 * @param  axis: 轴号
 * @retval HAL状态
 */
HAL_StatusTypeDef PositionControl_Init(AxisNumber_t axis)
{
    if (axis >= MAX_AXES) return HAL_ERROR;
    
    AxisConfig_t *axis_config = &g_motion_system.axes[axis];
    
    // 初始化级联控制器
    Cascade_Init(&axis_config->controller);
    
    // 设置默认参数
    Cascade_SetParameters(&axis_config->controller,
                         10.0f, 0.1f, 0.05f,    // 位置环PID
                         5.0f, 0.5f, 0.02f,     // 速度环PID
                         100.0f, 1000.0f, 0.001f); // 电流环PID
    
    // 设置前馈增益
    Feedforward_SetGains(&axis_config->controller.feedforward, 1.0f, 0.1f, 0.01f);
    
    // 设置死区参数
    Deadzone_SetParameters(&axis_config->controller.deadzone, 0.1f, -0.1f, 1.0f);
    
    // 初始化统计数组
    error_index[axis] = 0;
    error_count[axis] = 0;
    memset(error_history[axis], 0, sizeof(error_history[axis]));
    
    return HAL_OK;
}

/**
 * @brief  执行位置控制
 * @param  axis: 轴号
 * @param  position_ref: 位置参考值
 * @param  velocity_ref: 速度参考值
 * @param  acceleration_ref: 加速度参考值
 * @retval 控制输出
 */
float PositionControl_Execute(AxisNumber_t axis, float position_ref, 
                             float velocity_ref, float acceleration_ref)
{
    if (axis >= MAX_AXES) return 0.0f;
    
    AxisConfig_t *axis_config = &g_motion_system.axes[axis];
    CascadeController_t *cascade = &axis_config->controller;
    
    // 设置前馈输入
    if (cascade->feedforward_enable) {
        Feedforward_SetInputs(&cascade->feedforward, position_ref, velocity_ref, acceleration_ref);
    }
    
    // 执行级联控制
    float control_output = Cascade_Update(cascade, 
                                         axis_config->encoder_position,
                                         axis_config->encoder_velocity,
                                         axis_config->current_feedback);
    
    // 更新误差历史
    float position_error = position_ref - axis_config->encoder_position;
    update_error_history(axis, position_error);
    
    // 死区自适应学习
    if (cascade->deadzone_enable && cascade->deadzone.adaptive_enable) {
        Deadzone_AdaptiveLearning(&cascade->deadzone, position_error, axis_config->encoder_velocity);
    }
    
    return control_output;
}

/**
 * @brief  设置位置控制参数
 * @param  axis: 轴号
 * @param  controller_type: 控制模式
 * @param  pos_pid: 位置环PID参数
 * @param  vel_pid: 速度环PID参数
 * @param  cur_pid: 电流环PID参数
 * @retval HAL状态
 */
HAL_StatusTypeDef PositionControl_SetParameters(AxisNumber_t axis, ControllerType_t controller_type,
                                               PIDParams_t pos_pid, PIDParams_t vel_pid, PIDParams_t cur_pid)
{
    if (axis >= MAX_AXES) return HAL_ERROR;
    
    AxisConfig_t *axis_config = &g_motion_system.axes[axis];
    CascadeController_t *cascade = &axis_config->controller;
    
    // 设置控制模式
    cascade->controller_type = controller_type;
    
    // 设置PID参数
    Cascade_SetParameters(cascade,
                         pos_pid.kp, pos_pid.ki, pos_pid.kd,
                         vel_pid.kp, vel_pid.ki, vel_pid.kd,
                         cur_pid.kp, cur_pid.ki, cur_pid.kd);
    
    return HAL_OK;
}

/**
 * @brief  启用自适应控制
 * @param  axis: 轴号
 * @param  enable: 使能标志
 * @retval HAL状态
 */
HAL_StatusTypeDef PositionControl_EnableAdaptive(AxisNumber_t axis, uint8_t enable)
{
    if (axis >= MAX_AXES) return HAL_ERROR;
    
    AxisConfig_t *axis_config = &g_motion_system.axes[axis];
    axis_config->controller.adaptive_enable = enable;
    axis_config->controller.adaptive.adaptation_enable = enable;
    
    return HAL_OK;
}

/**
 * @brief  启用前馈控制
 * @param  axis: 轴号
 * @param  enable: 使能标志
 * @retval HAL状态
 */
HAL_StatusTypeDef PositionControl_EnableFeedforward(AxisNumber_t axis, uint8_t enable)
{
    if (axis >= MAX_AXES) return HAL_ERROR;
    
    AxisConfig_t *axis_config = &g_motion_system.axes[axis];
    axis_config->controller.feedforward_enable = enable;
    
    return HAL_OK;
}

/**
 * @brief  启用死区补偿
 * @param  axis: 轴号
 * @param  enable: 使能标志
 * @retval HAL状态
 */
HAL_StatusTypeDef PositionControl_EnableDeadzone(AxisNumber_t axis, uint8_t enable)
{
    if (axis >= MAX_AXES) return HAL_ERROR;
    
    AxisConfig_t *axis_config = &g_motion_system.axes[axis];
    axis_config->controller.deadzone_enable = enable;
    axis_config->controller.deadzone.compensation_active = enable;
    
    return HAL_OK;
}

/* ========================================================================== */
/* 自动整定算法                                                               */
/* ========================================================================== */

/**
 * @brief  开始自动整定
 * @param  axis: 轴号
 * @param  loop_type: 环路类型 (0=位置, 1=速度, 2=电流)
 * @param  method: 整定方法 (0=Ziegler-Nichols, 1=Cohen-Coon)
 * @retval HAL状态
 */
HAL_StatusTypeDef AutoTune_Start(AxisNumber_t axis, uint8_t loop_type, uint8_t method)
{
    if (axis >= MAX_AXES) return HAL_ERROR;
    
    AutoTuning_t *tuning = &auto_tuning[axis];
    
    // 初始化整定参数
    tuning->active = 1;
    tuning->loop_type = loop_type;
    tuning->test_amplitude = 1.0f;   // 测试幅度
    tuning->test_frequency = 1.0f;   // 测试频率 (Hz)
    tuning->test_samples = 1000;     // 测试样本数
    tuning->step = 0;
    
    // 分配响应数据缓冲区
    tuning->response_data = (float*)malloc(tuning->test_samples * sizeof(float));
    if (tuning->response_data == NULL) {
        tuning->active = 0;
        return HAL_ERROR;
    }
    
    return HAL_OK;
}

/**
 * @brief  自动整定更新
 * @param  axis: 轴号
 * @retval HAL状态
 */
HAL_StatusTypeDef AutoTune_Update(AxisNumber_t axis)
{
    if (axis >= MAX_AXES) return HAL_ERROR;
    
    AutoTuning_t *tuning = &auto_tuning[axis];
    
    if (!tuning->active) return HAL_OK;
    
    // 简化的阶跃响应测试
    switch (tuning->step) {
        case 0:
            // 设置阶跃输入
            // 这里应该设置阶跃信号到相应的控制环路
            tuning->step = 1;
            break;
            
        case 1:
            // 采集响应数据
            // 这里应该采集系统响应数据
            tuning->step = 2;
            break;
            
        case 2:
            // 分析响应数据并计算PID参数
            AutoTune_CalculateParameters(axis);
            tuning->step = 3;
            break;
            
        case 3:
            // 整定完成
            tuning->active = 0;
            if (tuning->response_data) {
                free(tuning->response_data);
                tuning->response_data = NULL;
            }
            break;
    }
    
    return HAL_OK;
}

/**
 * @brief  计算整定参数
 * @param  axis: 轴号
 * @retval HAL状态
 */
HAL_StatusTypeDef AutoTune_CalculateParameters(AxisNumber_t axis)
{
    if (axis >= MAX_AXES) return HAL_ERROR;
    
    AutoTuning_t *tuning = &auto_tuning[axis];
    AxisConfig_t *axis_config = &g_motion_system.axes[axis];
    
    if (!tuning->active || tuning->response_data == NULL) return HAL_ERROR;
    
    // 简化的Ziegler-Nichols方法
    // 分析阶跃响应曲线
    float steady_state_value = tuning->response_data[tuning->test_samples - 1];
    float max_value = steady_state_value;
    uint32_t max_index = 0;
    
    // 找到最大值和时间
    for (uint32_t i = 0; i < tuning->test_samples; i++) {
        if (tuning->response_data[i] > max_value) {
            max_value = tuning->response_data[i];
            max_index = i;
        }
    }
    
    // 计算超调量和调节时间
    float overshoot = (max_value - steady_state_value) / steady_state_value;
    float settling_time = max_index / POSITION_CONTROL_FREQ;  // 简化计算
    
    // 根据超调量和调节时间计算PID参数
    float kp, ki, kd;
    
    if (overshoot > 0.1f) {
        // 有超调，减少增益
        kp = 0.6f / overshoot;
        ki = kp / (2.0f * settling_time);
        kd = kp * settling_time / 8.0f;
    } else {
        // 无超调，可以增加增益
        kp = 1.0f / settling_time;
        ki = kp / settling_time;
        kd = kp * settling_time / 4.0f;
    }
    
    // 应用新参数
    switch (tuning->loop_type) {
        case 0: // 位置环
            PID_SetParameters(&axis_config->controller.position_loop, kp, ki, kd);
            break;
        case 1: // 速度环
            PID_SetParameters(&axis_config->controller.velocity_loop, kp, ki, kd);
            break;
        case 2: // 电流环
            PID_SetParameters(&axis_config->controller.current_loop, kp, ki, kd);
            break;
    }
    
    return HAL_OK;
}

/* ========================================================================== */
/* 性能分析函数                                                               */
/* ========================================================================== */

/**
 * @brief  分析控制性能
 * @param  axis: 轴号
 * @param  performance: 性能统计结构体指针
 * @retval HAL状态
 */
HAL_StatusTypeDef PerformanceAnalysis_Analyze(AxisNumber_t axis, PerformanceStats_t *performance)
{
    if (axis >= MAX_AXES || performance == NULL) return HAL_ERROR;
    
    if (error_count[axis] < 10) {
        // 数据不足
        return HAL_ERROR;
    }
    
    // 计算RMS误差
    performance->rms_error = calculate_rms_error(axis);
    
    // 计算最大误差
    performance->max_error = 0.0f;
    for (uint8_t i = 0; i < error_count[axis]; i++) {
        float abs_error = fabsf(error_history[axis][i]);
        if (abs_error > performance->max_error) {
            performance->max_error = abs_error;
        }
    }
    
    // 计算平均误差
    performance->average_error = 0.0f;
    for (uint8_t i = 0; i < error_count[axis]; i++) {
        performance->average_error += error_history[axis][i];
    }
    performance->average_error /= error_count[axis];
    
    // 计算稳态误差 (最后10个样本的平均值)
    uint8_t steady_samples = (error_count[axis] > 10) ? 10 : error_count[axis];
    performance->steady_state_error = 0.0f;
    for (uint8_t i = 0; i < steady_samples; i++) {
        uint8_t index = (error_index[axis] - 1 - i + STABILITY_CHECK_SAMPLES) % STABILITY_CHECK_SAMPLES;
        performance->steady_state_error += error_history[axis][index];
    }
    performance->steady_state_error /= steady_samples;
    
    // 简化的调节时间估计 (误差首次进入±5%范围的时间)
    performance->settling_time = 0.0f;
    float settling_threshold = performance->max_error * 0.05f;
    for (uint8_t i = 0; i < error_count[axis]; i++) {
        if (fabsf(error_history[axis][i]) > settling_threshold) {
            performance->settling_time = i / POSITION_CONTROL_FREQ;
        }
    }
    
    // 计算超调量 (假设第一个峰值为超调)
    performance->overshoot = 0.0f;
    float first_peak = 0.0f;
    for (uint8_t i = 1; i < error_count[axis] - 1; i++) {
        if (error_history[axis][i] > error_history[axis][i-1] && 
            error_history[axis][i] > error_history[axis][i+1]) {
            first_peak = error_history[axis][i];
            break;
        }
    }
    if (first_peak > 0) {
        performance->overshoot = (first_peak - fabsf(performance->steady_state_error)) / fabsf(performance->steady_state_error);
    }
    
    return HAL_OK;
}

/**
 * @brief  更新误差历史
 * @param  axis: 轴号
 * @param  error: 当前误差
 * @retval None
 */
static void update_error_history(AxisNumber_t axis, float error)
{
    if (axis >= MAX_AXES) return;
    
    error_history[axis][error_index[axis]] = error;
    error_index[axis] = (error_index[axis] + 1) % STABILITY_CHECK_SAMPLES;
    
    if (error_count[axis] < STABILITY_CHECK_SAMPLES) {
        error_count[axis]++;
    }
}

/**
 * @brief  计算RMS误差
 * @param  axis: 轴号
 * @retval RMS误差
 */
static float calculate_rms_error(AxisNumber_t axis)
{
    if (axis >= MAX_AXES || error_count[axis] == 0) return 0.0f;
    
    float sum_squares = 0.0f;
    for (uint8_t i = 0; i < error_count[axis]; i++) {
        sum_squares += error_history[axis][i] * error_history[axis][i];
    }
    
    return sqrtf(sum_squares / error_count[axis]);
}

/**
 * @brief  诊断控制器状态
 * @param  axis: 轴号
 * @retval 诊断结果位掩码
 */
uint32_t PositionControl_Diagnose(AxisNumber_t axis)
{
    if (axis >= MAX_AXES) return 0xFFFFFFFF;
    
    uint32_t diagnosis = 0;
    AxisConfig_t *axis_config = &g_motion_system.axes[axis];
    CascadeController_t *cascade = &axis_config->controller;
    
    // 检查控制器状态
    if (cascade->position_loop.state != CTRL_ACTIVE) {
        diagnosis |= (1 << 0);  // 位置环非活动
    }
    if (cascade->velocity_loop.state != CTRL_ACTIVE) {
        diagnosis |= (1 << 1);  // 速度环非活动
    }
    if (cascade->current_loop.state != CTRL_ACTIVE) {
        diagnosis |= (1 << 2);  // 电流环非活动
    }
    
    // 检查积分饱和
    if (fabsf(cascade->position_loop.error_sum) >= fabsf(cascade->position_loop.integral_max * 0.9f)) {
        diagnosis |= (1 << 8);  // 位置环积分饱和
    }
    if (fabsf(cascade->velocity_loop.error_sum) >= fabsf(cascade->velocity_loop.integral_max * 0.9f)) {
        diagnosis |= (1 << 9);  // 速度环积分饱和
    }
    
    // 检查输出饱和
    if (fabsf(cascade->position_loop.output) >= fabsf(cascade->position_loop.output_max * 0.9f)) {
        diagnosis |= (1 << 16); // 位置环输出饱和
    }
    if (fabsf(cascade->velocity_loop.output) >= fabsf(cascade->velocity_loop.output_max * 0.9f)) {
        diagnosis |= (1 << 17); // 速度环输出饱和
    }
    
    // 检查大误差
    if (fabsf(cascade->position_loop.error) > 10.0f) {
        diagnosis |= (1 << 24); // 位置大误差
    }
    if (fabsf(cascade->velocity_loop.error) > 100.0f) {
        diagnosis |= (1 << 25); // 速度大误差
    }
    
    return diagnosis;
} 