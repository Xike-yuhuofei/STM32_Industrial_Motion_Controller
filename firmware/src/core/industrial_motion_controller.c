/**
  ******************************************************************************
  * @file    industrial_motion_controller.c
  * @brief   工业运动控制卡核心实现（完整版）
  * @author  Industrial Motion Control Team
  * @version 2.0
  * @date    2025-01-27
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "industrial_motion_controller.h"
#include "motion_control.h"
#include "advanced_interpolation.h"
#include "synchronous_control.h"
#include "fault_diagnosis.h"
#include "industrial_communication.h"
#include <string.h>
#include <math.h>

/* Private macros ------------------------------------------------------------*/
#define IMC_FLOAT_EPSILON           1e-6f
#define IMC_CYCLE_TIME_US           1000    // 1ms cycle time in microseconds
#define EPSILON                     1e-6f   // 浮点数比较精度

/* Private variables ---------------------------------------------------------*/
IMC_System_t g_imc_system;

/* FreeRTOS任务句柄 ----------------------------------------------------------*/
#ifdef USE_FREERTOS
TaskHandle_t xTaskMotionControl = NULL;     // 运动控制任务
TaskHandle_t xTaskInterpolation = NULL;     // 插补任务
TaskHandle_t xTaskCommunication = NULL;     // 通讯任务
TaskHandle_t xTaskMonitor = NULL;           // 监控任务
TaskHandle_t xTaskDiagnosis = NULL;         // 诊断任务

/* FreeRTOS队列和信号量 ------------------------------------------------------*/
QueueHandle_t xQueueInterpolation = NULL;   // 插补数据队列
QueueHandle_t xQueueCommand = NULL;         // 命令队列
SemaphoreHandle_t xSemMotionControl = NULL; // 运动控制信号量
SemaphoreHandle_t xSemConfigUpdate = NULL;  // 配置更新信号量
#endif

/* Private function prototypes -----------------------------------------------*/
static HAL_StatusTypeDef IMC_CreateTasks(void);
static HAL_StatusTypeDef IMC_CreateQueuesAndSemaphores(void);
static HAL_StatusTypeDef IMC_InitializeControllers(void);
static HAL_StatusTypeDef IMC_UpdateAxisControl(uint8_t axis_id);
static float IMC_CalculateSCurveVelocity(float current_pos, float target_pos, 
                                         float current_vel, float max_vel,
                                         float max_accel, float max_jerk);

/* 公共函数实现 --------------------------------------------------------------*/

/**
 * @brief 系统初始化
 * @retval HAL状态
 */
HAL_StatusTypeDef IMC_SystemInit(void)
{
    // 清零系统结构
    memset(&g_imc_system, 0, sizeof(IMC_System_t));
    
    // 设置系统初始状态
    g_imc_system.system_state = IMC_STATE_STOPPED;
    g_imc_system.axes_count = 0;
    g_imc_system.emergency_stop = false;
    
    // 初始化性能统计
    g_imc_system.cycle_time_max = 0;
    g_imc_system.cycle_time_min = UINT32_MAX;
    g_imc_system.cycle_time_avg = 0;
    
    // 初始化插补缓冲区
    g_imc_system.interp_buffer_head = 0;
    g_imc_system.interp_buffer_tail = 0;
    g_imc_system.interp_buffer_count = 0;
    
    // 初始化各个子系统
    if (IMC_InitializeControllers() != HAL_OK) {
        return HAL_ERROR;
    }
    
#ifdef USE_FREERTOS
    // 创建FreeRTOS资源
    if (IMC_CreateQueuesAndSemaphores() != HAL_OK) {
        return HAL_ERROR;
    }
    
    if (IMC_CreateTasks() != HAL_OK) {
        return HAL_ERROR;
    }
#endif
    
    g_imc_system.system_state = IMC_STATE_READY;
    
    return HAL_OK;
}

/**
 * @brief 初始化控制器子系统
 * @retval HAL状态
 */
static HAL_StatusTypeDef IMC_InitializeControllers(void)
{
    // 初始化高级插补控制器
    if (AdvInterp_Init(IMC_CYCLE_INTERPOLATION) != HAL_OK) {
        return HAL_ERROR;
    }
    
    // 初始化同步控制器
    if (Sync_Init(IMC_CYCLE_MOTION_CONTROL) != HAL_OK) {
        return HAL_ERROR;
    }
    
    // 初始化故障诊断系统
    if (FaultDiag_Init() != HAL_OK) {
        return HAL_ERROR;
    }
    
    // 初始化工业通讯系统
    if (CommInit() != HAL_OK) {
        return HAL_ERROR;
    }
    
    return HAL_OK;
}

#ifdef USE_FREERTOS
/**
 * @brief 创建FreeRTOS任务
 * @retval HAL状态
 */
static HAL_StatusTypeDef IMC_CreateTasks(void)
{
    // 创建运动控制任务 (最高优先级)
    if (xTaskCreate(vTaskMotionControl, "MotionCtrl", IMC_STACK_SIZE_CRITICAL,
                    NULL, IMC_TASK_PRIORITY_CRITICAL, &xTaskMotionControl) != pdPASS) {
        return HAL_ERROR;
    }
    
    // 创建插补任务
    if (xTaskCreate(vTaskInterpolation, "Interpolation", IMC_STACK_SIZE_HIGH,
                    NULL, IMC_TASK_PRIORITY_HIGH, &xTaskInterpolation) != pdPASS) {
        return HAL_ERROR;
    }
    
    // 创建通讯任务
    if (xTaskCreate(vTaskCommunication, "Communication", IMC_STACK_SIZE_COMM,
                    NULL, IMC_TASK_PRIORITY_COMM, &xTaskCommunication) != pdPASS) {
        return HAL_ERROR;
    }
    
    // 创建监控任务
    if (xTaskCreate(vTaskMonitor, "Monitor", IMC_STACK_SIZE_MONITOR,
                    NULL, IMC_TASK_PRIORITY_MONITOR, &xTaskMonitor) != pdPASS) {
        return HAL_ERROR;
    }
    
    // 创建诊断任务
    if (xTaskCreate(vTaskDiagnosis, "Diagnosis", IMC_STACK_SIZE_MONITOR,
                    NULL, IMC_TASK_PRIORITY_MONITOR, &xTaskDiagnosis) != pdPASS) {
        return HAL_ERROR;
    }
    
    return HAL_OK;
}

/**
 * @brief 创建队列和信号量
 * @retval HAL状态
 */
static HAL_StatusTypeDef IMC_CreateQueuesAndSemaphores(void)
{
    // 创建插补数据队列
    xQueueInterpolation = xQueueCreate(IMC_MAX_INTERPOLATION_BUFFER, 
                                      sizeof(IMC_InterpolationBlock_t));
    if (xQueueInterpolation == NULL) {
        return HAL_ERROR;
    }
    
    // 创建命令队列
    xQueueCommand = xQueueCreate(32, sizeof(uint32_t));
    if (xQueueCommand == NULL) {
        return HAL_ERROR;
    }
    
    // 创建运动控制信号量
    xSemMotionControl = xSemaphoreCreateBinary();
    if (xSemMotionControl == NULL) {
        return HAL_ERROR;
    }
    
    // 创建配置更新信号量
    xSemConfigUpdate = xSemaphoreCreateMutex();
    if (xSemConfigUpdate == NULL) {
        return HAL_ERROR;
    }
    
    return HAL_OK;
}
#endif

/**
 * @brief 启动系统
 * @retval HAL状态
 */
HAL_StatusTypeDef IMC_SystemStart(void)
{
    if (g_imc_system.system_state != IMC_STATE_READY) {
        return HAL_ERROR;
    }
    
    if (g_imc_system.emergency_stop) {
        return HAL_ERROR;
    }
    
    // 启动各个子系统
    AdvInterp_Start();
    Sync_Enable(true);
    FaultDiag_Enable(true);
    CommEnable(true);
    
    g_imc_system.system_state = IMC_STATE_RUNNING;
    
#ifdef USE_FREERTOS
    // 发送启动信号
    xSemaphoreGive(xSemMotionControl);
#endif
    
    return HAL_OK;
}

/**
 * @brief 停止系统
 * @retval HAL状态
 */
HAL_StatusTypeDef IMC_SystemStop(void)
{
    // 停止所有轴
    for (uint8_t i = 0; i < g_imc_system.axes_count; i++) {
        IMC_AxisStop(i);
    }
    
    g_imc_system.system_state = IMC_STATE_STOPPED;
    
    return HAL_OK;
}

/**
 * @brief 系统复位
 * @retval HAL状态
 */
HAL_StatusTypeDef IMC_SystemReset(void)
{
    IMC_SystemStop();
    
    // 复位所有轴状态
    for (uint8_t i = 0; i < IMC_MAX_AXES; i++) {
        memset(&g_imc_system.axis_state[i], 0, sizeof(IMC_AxisState_t));
        g_imc_system.axis_config[i].enabled = false;
    }
    
    g_imc_system.emergency_stop = false;
    g_imc_system.system_state = IMC_STATE_READY;
    
    return HAL_OK;
}

/**
 * @brief 获取系统状态
 * @retval 系统状态
 */
IMC_State_t IMC_GetSystemState(void)
{
    return g_imc_system.system_state;
}

/**
 * @brief 轴初始化
 * @param axis_id 轴号
 * @param config 轴配置
 * @retval HAL状态
 */
HAL_StatusTypeDef IMC_AxisInit(uint8_t axis_id, const IMC_AxisConfig_t *config)
{
    if (axis_id >= IMC_MAX_AXES || config == NULL) {
        return HAL_ERROR;
    }
    
    // 复制配置
    memcpy(&g_imc_system.axis_config[axis_id], config, sizeof(IMC_AxisConfig_t));
    
    // 初始化轴状态
    memset(&g_imc_system.axis_state[axis_id], 0, sizeof(IMC_AxisState_t));
    g_imc_system.axis_state[axis_id].enabled = config->enabled;
    
    if (axis_id >= g_imc_system.axes_count) {
        g_imc_system.axes_count = axis_id + 1;
    }
    
    return HAL_OK;
}

/**
 * @brief 轴使能/禁用
 * @param axis_id 轴号
 * @param enable 使能标志
 * @retval HAL状态
 */
HAL_StatusTypeDef IMC_AxisEnable(uint8_t axis_id, bool enable)
{
    if (axis_id >= IMC_MAX_AXES) {
        return HAL_ERROR;
    }
    
    g_imc_system.axis_state[axis_id].enabled = enable;
    
    return HAL_OK;
}

/**
 * @brief 轴回零
 * @param axis_id 轴号
 * @retval HAL状态
 */
HAL_StatusTypeDef IMC_AxisHome(uint8_t axis_id)
{
    if (axis_id >= IMC_MAX_AXES) {
        return HAL_ERROR;
    }
    
    if (!g_imc_system.axis_state[axis_id].enabled) {
        return HAL_ERROR;
    }
    
    // 简化的回零操作
    g_imc_system.axis_state[axis_id].command_position = 0.0f;
    g_imc_system.axis_state[axis_id].actual_position = 0.0f;
    g_imc_system.axis_state[axis_id].homed = true;
    
    return HAL_OK;
}

/**
 * @brief 轴运动(高级版本，支持S曲线)
 * @param axis_id 轴号
 * @param target_position 目标位置
 * @param velocity 运动速度
 * @retval HAL状态
 */
HAL_StatusTypeDef IMC_AxisMove(uint8_t axis_id, float target_position, float velocity)
{
    if (axis_id >= IMC_MAX_AXES) {
        return HAL_ERROR;
    }
    
    if (!g_imc_system.axis_state[axis_id].enabled) {
        return HAL_ERROR;
    }
    
    // 检查位置限制
    if (target_position > g_imc_system.axis_config[axis_id].position_limit_pos ||
        target_position < g_imc_system.axis_config[axis_id].position_limit_neg) {
        return HAL_ERROR;
    }
    
    // 检查速度限制
    if (velocity > g_imc_system.axis_config[axis_id].max_velocity) {
        velocity = g_imc_system.axis_config[axis_id].max_velocity;
    }
    
    // 创建插补块
    IMC_InterpolationBlock_t interp_block;
    interp_block.axes_mask = (1 << axis_id);
    interp_block.target_position[axis_id] = target_position;
    interp_block.feed_rate = velocity;
    interp_block.acceleration = g_imc_system.axis_config[axis_id].max_acceleration;
    interp_block.interpolation_type = INTERP_TYPE_LINEAR;
    interp_block.block_complete = false;
    
    // 计算段时间（使用S曲线规划）
    float distance = fabsf(target_position - g_imc_system.axis_state[axis_id].actual_position);
    float max_jerk = g_imc_system.axis_config[axis_id].max_jerk;
    
    // S曲线时间计算
    float accel_time = g_imc_system.axis_config[axis_id].max_acceleration / max_jerk;
    float const_accel_time = (velocity - max_jerk * accel_time * accel_time) / 
                            g_imc_system.axis_config[axis_id].max_acceleration;
    float total_accel_time = 2 * accel_time + const_accel_time;
    float accel_distance = velocity * total_accel_time / 2.0f;
    
    if (2 * accel_distance > distance) {
        // 需要降低最大速度
        velocity = sqrtf(distance * g_imc_system.axis_config[axis_id].max_acceleration);
    }
    
    float total_time = distance / velocity * 1000.0f; // 转换为ms
    interp_block.segment_time = (uint32_t)total_time;
    
    // 添加到插补缓冲区
    return IMC_InterpolationAddBlock(&interp_block);
}

/**
 * @brief 轴停止
 * @param axis_id 轴号
 * @retval HAL状态
 */
HAL_StatusTypeDef IMC_AxisStop(uint8_t axis_id)
{
    if (axis_id >= IMC_MAX_AXES) {
        return HAL_ERROR;
    }
    
    g_imc_system.axis_state[axis_id].command_velocity = 0.0f;
    g_imc_system.axis_state[axis_id].moving = false;
    
    return HAL_OK;
}

/**
 * @brief 获取轴状态
 * @param axis_id 轴号
 * @retval 轴状态指针
 */
IMC_AxisState_t* IMC_GetAxisState(uint8_t axis_id)
{
    if (axis_id >= IMC_MAX_AXES) {
        return NULL;
    }
    
    return &g_imc_system.axis_state[axis_id];
}

/**
 * @brief 系统更新（主循环调用）
 * @retval HAL状态
 */
HAL_StatusTypeDef IMC_Update(void)
{
    // 简化的系统更新
    for (uint8_t i = 0; i < g_imc_system.axes_count; i++) {
        if (g_imc_system.axis_state[i].moving) {
            // 简单的位置模拟
            float position_error = g_imc_system.axis_state[i].command_position - 
                                 g_imc_system.axis_state[i].actual_position;
            
            if (fabsf(position_error) > EPSILON) {
                float step = (position_error > 0) ? 0.1f : -0.1f;
                g_imc_system.axis_state[i].actual_position += step;
            } else {
                g_imc_system.axis_state[i].actual_position = g_imc_system.axis_state[i].command_position;
                g_imc_system.axis_state[i].moving = false;
                g_imc_system.axis_state[i].in_position = true;
            }
        }
    }
    
    return HAL_OK;
}

/**
 * @brief 添加插补数据块
 * @param block 插补数据块
 * @retval HAL状态
 */
HAL_StatusTypeDef IMC_InterpolationAddBlock(const IMC_InterpolationBlock_t *block)
{
    if (block == NULL) {
        return HAL_ERROR;
    }
    
#ifdef USE_FREERTOS
    // 使用配置更新信号量保护
    if (xSemaphoreTake(xSemConfigUpdate, portMAX_DELAY) != pdTRUE) {
        return HAL_ERROR;
    }
#endif
    
    // 检查缓冲区是否已满
    if (g_imc_system.interp_buffer_count >= IMC_MAX_INTERPOLATION_BUFFER) {
#ifdef USE_FREERTOS
        xSemaphoreGive(xSemConfigUpdate);
#endif
        return HAL_ERROR;
    }
    
    // 添加到缓冲区
    memcpy(&g_imc_system.interp_buffer[g_imc_system.interp_buffer_tail], 
           block, sizeof(IMC_InterpolationBlock_t));
    
    g_imc_system.interp_buffer_tail = 
        (g_imc_system.interp_buffer_tail + 1) % IMC_MAX_INTERPOLATION_BUFFER;
    g_imc_system.interp_buffer_count++;
    
#ifdef USE_FREERTOS
    xSemaphoreGive(xSemConfigUpdate);
    
    // 通知插补任务
    if (xQueueInterpolation != NULL) {
        xQueueSend(xQueueInterpolation, block, 0);
    }
#endif
    
    return HAL_OK;
}

/**
 * @brief 运动控制中断服务例程
 */
void IMC_MotionControlISR(void)
{
    static uint32_t cycle_start_time = 0;
    uint32_t current_time = HAL_GetTick();
    
    // 计算周期时间
    if (cycle_start_time != 0) {
        uint32_t cycle_time = current_time - cycle_start_time;
        
        // 更新性能统计
        if (cycle_time > g_imc_system.cycle_time_max) {
            g_imc_system.cycle_time_max = cycle_time;
        }
        if (cycle_time < g_imc_system.cycle_time_min) {
            g_imc_system.cycle_time_min = cycle_time;
        }
        
        // 计算平均值（指数移动平均）
        g_imc_system.cycle_time_avg = 
            (g_imc_system.cycle_time_avg * 0.95f) + (cycle_time * 0.05f);
    }
    cycle_start_time = current_time;
    
    // 更新所有轴的控制
    for (uint8_t i = 0; i < g_imc_system.axes_count; i++) {
        if (g_imc_system.axis_state[i].enabled) {
            IMC_UpdateAxisControl(i);
        }
    }
    
    // 更新系统时间
    g_imc_system.system_time++;
}

/**
 * @brief 更新单轴控制
 * @param axis_id 轴号
 * @retval HAL状态
 */
static HAL_StatusTypeDef IMC_UpdateAxisControl(uint8_t axis_id)
{
    IMC_AxisConfig_t *config = &g_imc_system.axis_config[axis_id];
    IMC_AxisState_t *state = &g_imc_system.axis_state[axis_id];
    
    // 计算位置误差
    state->position_error = state->command_position - state->actual_position;
    
    // 检查跟随误差
    if (fabsf(state->position_error) > config->following_error_limit) {
        state->fault_code = IMC_FAULT_POSITION_ERROR;
        state->fault_timestamp = g_imc_system.system_time;
        return HAL_ERROR;
    }
    
    // S曲线速度规划
    if (state->moving) {
        state->command_velocity = IMC_CalculateSCurveVelocity(
            state->actual_position,
            state->command_position,
            state->actual_velocity,
            config->max_velocity,
            config->max_acceleration,
            config->max_jerk
        );
        
        // 位置环PID控制
        float position_output = config->position_kp * state->position_error;
        
        // 速度环PID控制
        float velocity_error = state->command_velocity - state->actual_velocity;
        float velocity_output = config->velocity_kp * velocity_error;
        
        // 控制输出限制
        state->control_output = position_output + velocity_output;
        if (state->control_output > config->current_limit) {
            state->control_output = config->current_limit;
        } else if (state->control_output < -config->current_limit) {
            state->control_output = -config->current_limit;
        }
        
        // 温度监控
        if (state->temperature > config->temperature_limit) {
            state->fault_code = IMC_FAULT_OVER_TEMP;
            state->fault_timestamp = g_imc_system.system_time;
            return HAL_ERROR;
        }
    }
    
    return HAL_OK;
}

/**
 * @brief 计算S曲线速度
 * @param current_pos 当前位置
 * @param target_pos 目标位置
 * @param current_vel 当前速度
 * @param max_vel 最大速度
 * @param max_accel 最大加速度
 * @param max_jerk 最大加加速度
 * @retval 计算得到的速度
 */
static float IMC_CalculateSCurveVelocity(float current_pos, float target_pos, 
                                         float current_vel, float max_vel,
                                         float max_accel, float max_jerk)
{
    float distance = target_pos - current_pos;
    float direction = (distance > 0) ? 1.0f : -1.0f;
    distance = fabsf(distance);
    
    // 计算减速距离
    float decel_distance = (current_vel * current_vel) / (2.0f * max_accel);
    
    if (distance <= decel_distance) {
        // 需要减速
        float target_vel = sqrtf(2.0f * max_accel * distance);
        
        // 应用加加速度限制
        float vel_change = target_vel - fabsf(current_vel);
        float max_vel_change = max_jerk * (IMC_CYCLE_TIME_US / 1000000.0f);
        
        if (fabsf(vel_change) > max_vel_change) {
            vel_change = (vel_change > 0) ? max_vel_change : -max_vel_change;
        }
        
        return (fabsf(current_vel) + vel_change) * direction;
    } else {
        // 可以加速或保持速度
        float target_vel = max_vel;
        float vel_change = target_vel - fabsf(current_vel);
        float max_vel_change = max_jerk * (IMC_CYCLE_TIME_US / 1000000.0f);
        
        if (fabsf(vel_change) > max_vel_change) {
            vel_change = (vel_change > 0) ? max_vel_change : -max_vel_change;
        }
        
        return (fabsf(current_vel) + vel_change) * direction;
    }
}

#ifdef USE_FREERTOS
/**
 * @brief 运动控制任务
 * @param pvParameters 任务参数
 */
void vTaskMotionControl(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(IMC_CYCLE_MOTION_CONTROL);
    
    for (;;) {
        // 等待精确的周期时间
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        
        // 执行运动控制
        if (g_imc_system.system_state == IMC_STATE_RUNNING) {
            IMC_MotionControlISR();
        }
    }
}

/**
 * @brief 插补任务
 * @param pvParameters 任务参数
 */
void vTaskInterpolation(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(IMC_CYCLE_INTERPOLATION);
    
    for (;;) {
        // 等待精确的周期时间
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        
        // 执行插补
        if (g_imc_system.system_state == IMC_STATE_RUNNING) {
            AdvInterp_Execute();
        }
    }
}

/**
 * @brief 通讯任务
 * @param pvParameters 任务参数
 */
void vTaskCommunication(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(IMC_CYCLE_COMMUNICATION);
    
    for (;;) {
        // 等待精确的周期时间
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        
        // 执行通讯更新
        CommUpdate();
    }
}

/**
 * @brief 监控任务
 * @param pvParameters 任务参数
 */
void vTaskMonitor(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(IMC_CYCLE_MONITOR);
    
    for (;;) {
        // 等待精确的周期时间
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        
        // 更新性能统计
        g_imc_system.interpolation_rate = 1000 / IMC_CYCLE_INTERPOLATION;
        
        // 检查系统状态
        for (uint8_t i = 0; i < g_imc_system.axes_count; i++) {
            if (g_imc_system.axis_state[i].fault_code != IMC_FAULT_NONE) {
                // 记录故障
                if (g_imc_system.fault_count < 32) {
                    g_imc_system.fault_history[g_imc_system.fault_count++] = 
                        g_imc_system.axis_state[i].fault_code;
                }
            }
        }
    }
}

/**
 * @brief 诊断任务
 * @param pvParameters 任务参数
 */
void vTaskDiagnosis(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(IMC_CYCLE_MONITOR);
    
    for (;;) {
        // 等待精确的周期时间
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        
        // 执行故障诊断
        FaultDiag_Update();
        
        // 检查各轴状态
        for (uint8_t i = 0; i < g_imc_system.axes_count; i++) {
            FaultType_t fault = FaultDiag_DetectFault(i);
            if (fault != FAULT_TYPE_NONE) {
                // 更新轴故障状态
                g_imc_system.axis_state[i].fault_code = 
                    (IMC_FaultType_t)(IMC_FAULT_OVER_CURRENT + fault - 1);
            }
        }
    }
}
#endif 