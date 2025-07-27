/**
  ******************************************************************************
  * @file    motion_control.c
  * @brief   多轴运动控制系统实现
  * @author  Claude AI & Cursor
  * @version 1.0
  * @date    2025-01-27
  ******************************************************************************
  */

#include "motion_control.h"

/* 数学常量和精度定义 ---------------------------------------------------------*/
#define EPSILON 1e-6f  // 浮点数比较精度

/* 全局变量定义 --------------------------------------------------------------*/
MotionControlSystem_t g_motion_system;
char g_command_buffer[MAX_COMMAND_LENGTH];
uint8_t g_command_ready = 0;

/* 私有变量 ------------------------------------------------------------------*/
static TIM_HandleTypeDef htim1;  // X,Y,Z轴定时器
static TIM_HandleTypeDef htim6;  // 插补定时器

/* 私有函数声明 --------------------------------------------------------------*/
static void MotionControl_InitTimers(void);
static void MotionControl_InitGPIO(void);

/**
  * @brief  运动控制系统初始化
  * @retval None
  */
void MotionControl_Init(void)
{
    // 清零系统结构
    memset(&g_motion_system, 0, sizeof(MotionControlSystem_t));
    
    // 初始化定时器和GPIO
    MotionControl_InitTimers();
    MotionControl_InitGPIO();
    
    // 配置各轴默认参数
    // X轴配置 (TIM1_CH1, PA8)
    MotionControl_ConfigAxis(AXIS_X, &htim1, TIM_CHANNEL_1,
                            GPIOD, GPIO_PIN_14,  // DIR
                            GPIOD, GPIO_PIN_15,  // ENA
                            160.0f,              // 160步/mm (1.8°, 16细分, 5mm螺距)
                            3000.0f,             // 最大速度 3000mm/min
                            1000.0f);            // 最大加速度 1000mm/s²
    
    // Y轴配置 (TIM1_CH2, PA9)
    MotionControl_ConfigAxis(AXIS_Y, &htim1, TIM_CHANNEL_2,
                            GPIOD, GPIO_PIN_12,  // DIR
                            GPIOD, GPIO_PIN_13,  // ENA
                            160.0f, 3000.0f, 1000.0f);
    
    // Z轴配置 (TIM1_CH3, PA10)
    MotionControl_ConfigAxis(AXIS_Z, &htim1, TIM_CHANNEL_3,
                            GPIOD, GPIO_PIN_10,  // DIR
                            GPIOD, GPIO_PIN_11,  // ENA
                            160.0f, 1500.0f, 500.0f);  // Z轴较慢
    
    // A轴配置 (TIM8_CH1, PI5) - 使用主程序中定义的引脚
    extern TIM_HandleTypeDef htim8;
    MotionControl_ConfigAxis(AXIS_A, &htim8, TIM_CHANNEL_1,
                            GPIOF, GPIO_PIN_14,  // DIR (PF14)
                            GPIOH, GPIO_PIN_3,   // ENA (PH3)
                            17.777f,             // 3200步/转 ÷ 180°/转 = 17.777步/度
                            1800.0f,             // 最大速度 1800°/min (5转/秒)
                            600.0f);             // 最大加速度 600°/s²
    
    // 设置默认参数
    g_motion_system.current_mode = MOTION_IDLE;
    g_motion_system.current_feed_rate = 1000.0f;  // 默认进给速度 1000mm/min
    g_motion_system.system_running = 1;
    g_motion_system.emergency_stop = 0;
    
    // 启动插补定时器 (1kHz)
    HAL_TIM_Base_Start_IT(&htim6);
    
    // 使能所有轴
    for (int i = 0; i < MAX_AXES; i++) {
        Axis_Enable((AxisNumber_t)i);
    }
}

/**
  * @brief  配置单轴参数
  */
void MotionControl_ConfigAxis(AxisNumber_t axis, TIM_HandleTypeDef *htim, uint32_t channel,
                             GPIO_TypeDef *dir_port, uint16_t dir_pin,
                             GPIO_TypeDef *ena_port, uint16_t ena_pin,
                             float steps_per_mm, float max_speed, float max_accel)
{
    if (axis >= MAX_AXES) return;
    
    AxisConfig_t *ax = &g_motion_system.axes[axis];
    
    // 硬件配置
    ax->htim = htim;
    ax->channel = channel;
    ax->dir_port = dir_port;
    ax->dir_pin = dir_pin;
    ax->ena_port = ena_port;
    ax->ena_pin = ena_pin;
    
    // 运动参数
    ax->steps_per_mm = steps_per_mm;
    ax->max_speed = max_speed;
    ax->max_accel = max_accel;
    ax->home_speed = max_speed * 0.1f;  // 回零速度为最大速度的10%
    
    // 状态初始化
    ax->state = AXIS_IDLE;
    ax->position = 0.0f;
    ax->target_position = 0.0f;
    ax->current_speed = 0.0f;
    ax->direction = 1;  // 默认正向
    ax->enabled = 0;
}

/**
  * @brief  使能轴
  */
void Axis_Enable(AxisNumber_t axis)
{
    if (axis >= MAX_AXES) return;
    
    AxisConfig_t *ax = &g_motion_system.axes[axis];
    HAL_GPIO_WritePin(ax->ena_port, ax->ena_pin, GPIO_PIN_RESET);  // 低电平使能
    ax->enabled = 1;
}

/**
  * @brief  禁用轴
  */
void Axis_Disable(AxisNumber_t axis)
{
    if (axis >= MAX_AXES) return;
    
    AxisConfig_t *ax = &g_motion_system.axes[axis];
    HAL_GPIO_WritePin(ax->ena_port, ax->ena_pin, GPIO_PIN_SET);    // 高电平禁用
    ax->enabled = 0;
    Axis_StopPWM(axis);
}

/**
  * @brief  设置轴方向
  */
void Axis_SetDirection(AxisNumber_t axis, uint8_t direction)
{
    if (axis >= MAX_AXES) return;
    
    AxisConfig_t *ax = &g_motion_system.axes[axis];
    HAL_GPIO_WritePin(ax->dir_port, ax->dir_pin, direction ? GPIO_PIN_SET : GPIO_PIN_RESET);
    ax->direction = direction;
}

/**
  * @brief  设置轴速度
  */
void Axis_SetSpeed(AxisNumber_t axis, float speed_mm_min)
{
    if (axis >= MAX_AXES) return;
    
    AxisConfig_t *ax = &g_motion_system.axes[axis];
    
    // 限制最大速度
    if (speed_mm_min > ax->max_speed) {
        speed_mm_min = ax->max_speed;
    }
    
    // 计算频率 (步/秒)
    float steps_per_sec = (speed_mm_min * ax->steps_per_mm) / 60.0f;
    
    // 检查最小速度，避免除零错误
    if (steps_per_sec < 1.0f) {
        Axis_StopPWM(axis);
        return;
    }
    
    // 计算定时器参数 (假设84MHz APB1时钟)
    uint32_t prescaler = 999;        // 预分频器 (84MHz / 1000 = 84kHz)
    
    // 防止除零错误
    if (steps_per_sec == 0.0f) {
        Axis_StopPWM(axis);
        return;
    }
    
    uint32_t period = (uint32_t)((84000.0f / steps_per_sec) - 1);
    
    // 停止PWM
    HAL_TIM_PWM_Stop(ax->htim, ax->channel);
    
    // 更新定时器配置
    __HAL_TIM_SET_PRESCALER(ax->htim, prescaler);
    __HAL_TIM_SET_AUTORELOAD(ax->htim, period);
    __HAL_TIM_SET_COMPARE(ax->htim, ax->channel, period / 2);  // 50%占空比
    
    ax->current_speed = speed_mm_min;
}

/**
  * @brief  启动轴PWM
  */
void Axis_StartPWM(AxisNumber_t axis)
{
    if (axis >= MAX_AXES) return;
    
    AxisConfig_t *ax = &g_motion_system.axes[axis];
    if (ax->enabled && ax->current_speed > 0) {
        HAL_TIM_PWM_Start(ax->htim, ax->channel);
        ax->state = AXIS_MOVING;
    }
}

/**
  * @brief  停止轴PWM
  */
void Axis_StopPWM(AxisNumber_t axis)
{
    if (axis >= MAX_AXES) return;
    
    AxisConfig_t *ax = &g_motion_system.axes[axis];
    HAL_TIM_PWM_Stop(ax->htim, ax->channel);
    ax->state = AXIS_IDLE;
    ax->current_speed = 0.0f;
}

/**
  * @brief  轴移动到指定位置
  */
void Axis_MoveToPosition(AxisNumber_t axis, float position, float speed)
{
    if (axis >= MAX_AXES) return;
    
    AxisConfig_t *ax = &g_motion_system.axes[axis];
    
    // 计算移动方向
    uint8_t direction = (position > ax->position) ? 1 : 0;
    Axis_SetDirection(axis, direction);
    
    // 设置目标位置和速度
    ax->target_position = position;
    Axis_SetSpeed(axis, speed);
    
    // 启动PWM
    Axis_StartPWM(axis);
}

/**
  * @brief  快速移动 (G00)
  */
void Motion_RapidMove(float pos[MAX_AXES])
{
    // 创建运动块
    if (g_motion_system.buffer_count >= MOTION_BUFFER_SIZE) return;
    
    MotionBlock_t *block = &g_motion_system.motion_buffer[g_motion_system.buffer_head];
    
    block->mode = MOTION_RAPID;
    
    // 复制当前位置到起始位置
    for (int i = 0; i < MAX_AXES; i++) {
        block->start_pos[i] = g_motion_system.axes[i].position;
        block->end_pos[i] = pos[i];
    }
    
    // 计算移动长度
    block->length = TrajectoryPlanning_CalcLength(block->start_pos, block->end_pos);
    
    // 使用最大速度的平均值
    float max_speed = 0;
    for (int i = 0; i < MAX_AXES; i++) {
        max_speed += g_motion_system.axes[i].max_speed;
    }
    block->feed_rate = max_speed / MAX_AXES;
    
    // 计算插补步数
    block->steps_total = TrajectoryPlanning_CalcSteps(block->length, block->feed_rate);
    block->steps_current = 0;
    block->active = 1;
    
    // 更新缓冲区指针
    g_motion_system.buffer_head = (g_motion_system.buffer_head + 1) % MOTION_BUFFER_SIZE;
    g_motion_system.buffer_count++;
}

/**
  * @brief  直线插补移动 (G01)
  */
void Motion_LinearMove(float pos[MAX_AXES], float feed_rate)
{
    // 创建运动块
    if (g_motion_system.buffer_count >= MOTION_BUFFER_SIZE) return;
    
    MotionBlock_t *block = &g_motion_system.motion_buffer[g_motion_system.buffer_head];
    
    block->mode = MOTION_LINEAR;
    block->feed_rate = feed_rate;
    
    // 复制位置
    for (int i = 0; i < MAX_AXES; i++) {
        block->start_pos[i] = g_motion_system.axes[i].position;
        block->end_pos[i] = pos[i];
    }
    
    // 计算移动长度和插补步数
    block->length = TrajectoryPlanning_CalcLength(block->start_pos, block->end_pos);
    block->steps_total = TrajectoryPlanning_CalcSteps(block->length, feed_rate);
    block->steps_current = 0;
    block->active = 1;
    
    // 更新缓冲区
    g_motion_system.buffer_head = (g_motion_system.buffer_head + 1) % MOTION_BUFFER_SIZE;
    g_motion_system.buffer_count++;
}

/**
  * @brief  圆弧插补移动 (G02/G03)
  */
void Motion_ArcMove(float pos[MAX_AXES], float center[2], float feed_rate, uint8_t clockwise)
{
    // 创建运动块
    if (g_motion_system.buffer_count >= MOTION_BUFFER_SIZE) return;
    
    MotionBlock_t *block = &g_motion_system.motion_buffer[g_motion_system.buffer_head];
    
    block->mode = clockwise ? MOTION_CW_ARC : MOTION_CCW_ARC;
    block->feed_rate = feed_rate;
    
    // 复制位置
    for (int i = 0; i < MAX_AXES; i++) {
        block->start_pos[i] = g_motion_system.axes[i].position;
        block->end_pos[i] = pos[i];
    }
    
    // 复制圆弧中心
    block->center_pos[0] = center[0];
    block->center_pos[1] = center[1];
    
    // 计算圆弧长度
    block->length = TrajectoryPlanning_CalcArcLength(block->start_pos, block->end_pos, center, clockwise);
    block->steps_total = TrajectoryPlanning_CalcSteps(block->length, feed_rate);
    block->steps_current = 0;
    block->active = 1;
    
    // 更新缓冲区
    g_motion_system.buffer_head = (g_motion_system.buffer_head + 1) % MOTION_BUFFER_SIZE;
    g_motion_system.buffer_count++;
}

/**
  * @brief  直线插补算法
  */
void Interpolation_Linear(MotionBlock_t *block, float progress, float result_pos[MAX_AXES])
{
    for (int i = 0; i < MAX_AXES; i++) {
        result_pos[i] = block->start_pos[i] + 
                       (block->end_pos[i] - block->start_pos[i]) * progress;
    }
}

/**
  * @brief  圆弧插补算法
  */
void Interpolation_Arc(MotionBlock_t *block, float progress, float result_pos[MAX_AXES])
{
    // 计算起始和结束角度
    float start_x = block->start_pos[0] - block->center_pos[0];
    float start_y = block->start_pos[1] - block->center_pos[1];
    float end_x = block->end_pos[0] - block->center_pos[0];
    float end_y = block->end_pos[1] - block->center_pos[1];
    
    float start_angle = atan2f(start_y, start_x);
    float end_angle = atan2f(end_y, end_x);
    
    // 处理角度跨越
    if (block->mode == MOTION_CW_ARC) {
        if (end_angle > start_angle) {
            end_angle -= 2 * M_PI;
        }
    } else {
        if (end_angle < start_angle) {
            end_angle += 2 * M_PI;
        }
    }
    
    // 当前角度
    float current_angle = start_angle + (end_angle - start_angle) * progress;
    
    // 计算半径
    float radius = sqrtf(start_x * start_x + start_y * start_y);
    
    // 计算当前位置
    result_pos[0] = block->center_pos[0] + radius * cosf(current_angle);
    result_pos[1] = block->center_pos[1] + radius * sinf(current_angle);
    
    // Z轴和A轴直线插补
    for (int i = 2; i < MAX_AXES; i++) {
        result_pos[i] = block->start_pos[i] + 
                       (block->end_pos[i] - block->start_pos[i]) * progress;
    }
}

/**
  * @brief  执行插补 (定时器中断调用)
  */
void Interpolation_Execute(void)
{
    if (!g_motion_system.system_running || g_motion_system.emergency_stop) {
        return;
    }
    
    // 检查是否有运动块需要执行
    if (g_motion_system.buffer_count == 0) {
        return;
    }
    
    MotionBlock_t *block = &g_motion_system.motion_buffer[g_motion_system.buffer_tail];
    
    if (!block->active) {
        return;
    }
    
    // 计算插补进度
    float progress = (float)block->steps_current / (float)block->steps_total;
    
    if (progress >= 1.0f) {
        // 运动完成，移除运动块
        block->active = 0;
        g_motion_system.buffer_tail = (g_motion_system.buffer_tail + 1) % MOTION_BUFFER_SIZE;
        g_motion_system.buffer_count--;
        
        // 更新当前位置到目标位置
        for (int i = 0; i < MAX_AXES; i++) {
            g_motion_system.axes[i].position = block->end_pos[i];
            Axis_StopPWM((AxisNumber_t)i);
        }
        return;
    }
    
    // 执行插补
    float result_pos[MAX_AXES];
    
    switch (block->mode) {
        case MOTION_RAPID:
        case MOTION_LINEAR:
            Interpolation_Linear(block, progress, result_pos);
            break;
            
        case MOTION_CW_ARC:
        case MOTION_CCW_ARC:
            Interpolation_Arc(block, progress, result_pos);
            break;
            
        default:
            return;
    }
    
    // 更新各轴位置和PWM
    for (int i = 0; i < MAX_AXES; i++) {
        AxisConfig_t *ax = &g_motion_system.axes[i];
        
        // 计算移动方向
        uint8_t direction = (result_pos[i] > ax->position) ? 1 : 0;
        if (direction != ax->direction) {
            Axis_SetDirection((AxisNumber_t)i, direction);
        }
        
        // 计算瞬时速度
        float distance = fabsf(result_pos[i] - ax->position);
        float speed = distance * INTERPOLATION_FREQ * 60.0f;
        
        if (speed > 0) {
            Axis_SetSpeed((AxisNumber_t)i, speed);
            Axis_StartPWM((AxisNumber_t)i);
        } else {
            Axis_StopPWM((AxisNumber_t)i);
        }
        
        // 更新位置
        ax->position = result_pos[i];
    }
    
    // 递增插补步数
    block->steps_current++;
}

/**
  * @brief  轨迹规划 - 计算直线长度
  */
float TrajectoryPlanning_CalcLength(float start[MAX_AXES], float end[MAX_AXES])
{
    float sum = 0;
    for (int i = 0; i < MAX_AXES; i++) {
        float diff = end[i] - start[i];
        sum += diff * diff;
    }
    return sqrtf(sum);
}

/**
  * @brief  轨迹规划 - 计算圆弧长度
  */
float TrajectoryPlanning_CalcArcLength(float start[2], float end[2], float center[2], uint8_t clockwise)
{
    float start_x = start[0] - center[0];
    float start_y = start[1] - center[1];
    float end_x = end[0] - center[0];
    float end_y = end[1] - center[1];
    
    float start_angle = atan2f(start_y, start_x);
    float end_angle = atan2f(end_y, end_x);
    
    float angle_diff;
    if (clockwise) {
        angle_diff = start_angle - end_angle;
        if (angle_diff < 0) {
            angle_diff += 2 * M_PI;
        }
    } else {
        angle_diff = end_angle - start_angle;
        if (angle_diff < 0) {
            angle_diff += 2 * M_PI;
        }
    }
    
    float radius = sqrtf(start_x * start_x + start_y * start_y);
    return radius * angle_diff;
}

/**
  * @brief  轨迹规划 - 计算插补步数
  */
uint32_t TrajectoryPlanning_CalcSteps(float length, float feed_rate)
{
    // 根据长度和进给速度计算时间，再根据插补频率计算步数
    float time = (length * 60.0f) / feed_rate;  // 时间(秒)
    return (uint32_t)(time * INTERPOLATION_FREQ);
}

/**
  * @brief  数学工具 - 2D距离
  */
float Math_Distance2D(float x1, float y1, float x2, float y2)
{
    float dx = x2 - x1;
    float dy = y2 - y1;
    return sqrtf(dx * dx + dy * dy);
}

/**
  * @brief  数学工具 - 3D距离
  */
float Math_Distance3D(float x1, float y1, float z1, float x2, float y2, float z2)
{
    float dx = x2 - x1;
    float dy = y2 - y1;
    float dz = z2 - z1;
    return sqrtf(dx * dx + dy * dy + dz * dz);
}

/**
  * @brief  单轴回零
  */
void Motion_Home(AxisNumber_t axis)
{
    if (axis >= MAX_AXES) return;
    
    AxisConfig_t *ax = &g_motion_system.axes[axis];
    
    // 设置回零状态
    ax->state = AXIS_HOMING;
    
    // 使用回零速度移动到原点
    // TODO: 实现限位开关检测
    // 这里简单地将位置设为0
    ax->position = 0.0f;
    ax->target_position = 0.0f;
    ax->state = AXIS_IDLE;
}

/**
  * @brief  所有轴回零
  */
void Motion_HomeAll(void)
{
    // 清空运动缓冲区
    g_motion_system.buffer_count = 0;
    g_motion_system.buffer_head = 0;
    g_motion_system.buffer_tail = 0;
    
    // 停止所有运动
    for (int i = 0; i < MAX_AXES; i++) {
        Axis_StopPWM((AxisNumber_t)i);
        Motion_Home((AxisNumber_t)i);
    }
}

/**
  * @brief  停止运动
  */
void Motion_Stop(void)
{
    // 停止所有轴
    for (int i = 0; i < MAX_AXES; i++) {
        Axis_StopPWM((AxisNumber_t)i);
    }
    
    // 清空运动缓冲区
    g_motion_system.buffer_count = 0;
    g_motion_system.buffer_head = 0;
    g_motion_system.buffer_tail = 0;
    
    // 更新状态
    g_motion_system.current_mode = MOTION_IDLE;
}

/**
  * @brief  急停
  */
void Motion_EmergencyStop(void)
{
    g_motion_system.emergency_stop = 1;
    
    // 停止所有轴
    for (int i = 0; i < MAX_AXES; i++) {
        Axis_StopPWM((AxisNumber_t)i);
    }
    
    // 清空运动缓冲区
    g_motion_system.buffer_count = 0;
    g_motion_system.buffer_head = 0;
    g_motion_system.buffer_tail = 0;
    
    // 更新状态
    g_motion_system.current_mode = MOTION_IDLE;
    g_motion_system.system_running = 0;
}

/**
  * @brief  插补定时器回调 (1kHz)
  */
void MotionControl_TimerCallback(void)
{
    Interpolation_Execute();
}

/**
  * @brief  初始化定时器
  */
static void MotionControl_InitTimers(void)
{
    // TIM1配置 (X,Y,Z轴PWM)
    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 999;        // 84MHz / 1000 = 84kHz
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 839;           // 100Hz默认频率
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    
    if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
        Error_Handler();
    }
    
    // 配置TIM1的3个PWM通道
    TIM_OC_InitTypeDef sConfigOC = {0};
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 420;  // 50%占空比
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2);
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3);
    
    // TIM6配置 (插补定时器, 1kHz)
    htim6.Instance = TIM6;
    htim6.Init.Prescaler = 83999;      // 84MHz / 84000 = 1kHz
    htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim6.Init.Period = 0;             // 1ms
    htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    
    if (HAL_TIM_Base_Init(&htim6) != HAL_OK) {
        Error_Handler();
    }
}

/**
  * @brief  初始化GPIO
  */
static void MotionControl_InitGPIO(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    // 使能时钟
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    
    // 配置X,Y,Z轴方向和使能引脚
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    
    // X轴 (DIR: PD14, ENA: PD15)
    GPIO_InitStruct.Pin = GPIO_PIN_14 | GPIO_PIN_15;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
    
    // Y轴 (DIR: PD12, ENA: PD13)
    GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
    
    // Z轴 (DIR: PD10, ENA: PD11)
    GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
    
    // 配置PWM引脚
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
    
    // X轴PWM (PA8, TIM1_CH1)
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    // Y轴PWM (PA9, TIM1_CH2)
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    // Z轴PWM (PA10, TIM1_CH3)
    GPIO_InitStruct.Pin = GPIO_PIN_10;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}