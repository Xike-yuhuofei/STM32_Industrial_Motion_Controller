/**
  ******************************************************************************
  * @file    fault_diagnosis.c
  * @brief   故障诊断系统实现
  * @author  Industrial Motion Control Team
  * @version 2.0
  * @date    2025-01-27
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "fault_diagnosis.h"
#include "industrial_motion_controller.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

/* Private macros ------------------------------------------------------------*/
#define FFT_LOG2_SIZE           9       // log2(512) = 9
#define VIBRATION_RMS_THRESHOLD 10.0f   // 振动有效值阈值
#define TEMPERATURE_WARNING     60.0f   // 温度警告值
#define TEMPERATURE_CRITICAL    80.0f   // 温度临界值
#define CURRENT_WARNING_RATIO   0.8f    // 电流警告比例
#define BEARING_FREQ_TOLERANCE  0.1f    // 轴承频率容差

/* Private variables ---------------------------------------------------------*/
FaultDiagnosisSystem_t g_fault_diag_system;
static bool g_diag_initialized = false;  // 初始化标志

/* Private function prototypes -----------------------------------------------*/
static void FaultDiag_InitThresholds(void);
static HAL_StatusTypeDef FaultDiag_CheckThresholds(uint8_t sensor_id);
static HAL_StatusTypeDef FaultDiag_UpdateAxisDiagnostics(uint8_t axis_id);
static HAL_StatusTypeDef ValidateAxisId(uint8_t axis_id);
static void FaultDiag_FFTBitReverse(float *data, uint16_t size);
static void FaultDiag_FFTCompute(float *real, float *imag, uint16_t size);
static float FaultDiag_CalculateBearingFrequency(float shaft_speed, uint8_t bearing_type);
static HAL_StatusTypeDef FaultDiag_AnalyzeSpectrum(const float *spectrum, uint16_t size, 
                                                   float sample_rate, SpectrumAnalysis_t *result);
static HAL_StatusTypeDef ValidateSensorId(uint8_t sensor_id);
static uint32_t GetCurrentTimeMs(void);

/* 公共函数实现 --------------------------------------------------------------*/

/**
 * @brief 初始化故障诊断系统
 * @retval HAL状态
 */
HAL_StatusTypeDef FaultDiag_Init(void)
{
    // 清零系统结构
    memset(&g_fault_diag_system, 0, sizeof(FaultDiagnosisSystem_t));
    
    // 初始化系统状态
    g_fault_diag_system.system_state = DIAG_STATE_NORMAL;
    g_fault_diag_system.diag_enabled = true;
    g_fault_diag_system.diag_cycle_time = 100; // 100ms
    
    // 初始化阈值
    FaultDiag_InitThresholds();
    
    // 初始化故障历史
    g_fault_diag_system.fault_head = 0;
    g_fault_diag_system.fault_count = 0;
    g_fault_diag_system.next_fault_id = 1;
    
    // 初始化性能统计
    g_fault_diag_system.total_diag_cycles = 0;
    
    // 设置初始化完成标志
    g_diag_initialized = true;
    g_fault_diag_system.fault_detection_count = 0;
    g_fault_diag_system.false_alarm_count = 0;
    g_fault_diag_system.diag_accuracy = 100.0f;
    
    return HAL_OK;
}

/**
 * @brief 初始化阈值
 */
static void FaultDiag_InitThresholds(void)
{
    // 温度阈值（℃）
    g_fault_diag_system.temperature_thresholds[0] = 40.0f;  // 信息级别
    g_fault_diag_system.temperature_thresholds[1] = 50.0f;  // 警告级别
    g_fault_diag_system.temperature_thresholds[2] = 60.0f;  // 轻微故障
    g_fault_diag_system.temperature_thresholds[3] = 70.0f;  // 严重故障
    g_fault_diag_system.temperature_thresholds[4] = 80.0f;  // 致命故障
    
    // 电流阈值（相对额定电流比例）
    g_fault_diag_system.current_thresholds[0] = 0.6f;   // 信息级别
    g_fault_diag_system.current_thresholds[1] = 0.8f;   // 警告级别
    g_fault_diag_system.current_thresholds[2] = 1.0f;   // 轻微故障
    g_fault_diag_system.current_thresholds[3] = 1.2f;   // 严重故障
    g_fault_diag_system.current_thresholds[4] = 1.5f;   // 致命故障
    
    // 振动阈值（mm/s RMS）
    g_fault_diag_system.vibration_thresholds[0] = 1.0f;  // 信息级别
    g_fault_diag_system.vibration_thresholds[1] = 2.8f;  // 警告级别
    g_fault_diag_system.vibration_thresholds[2] = 4.5f;  // 轻微故障
    g_fault_diag_system.vibration_thresholds[3] = 7.1f;  // 严重故障
    g_fault_diag_system.vibration_thresholds[4] = 11.2f; // 致命故障
}

/**
 * @brief 复位故障诊断系统
 */
HAL_StatusTypeDef FaultDiag_Reset(void) {
    if (!g_diag_initialized) {
        return HAL_ERROR;
    }
    
    // 清除所有故障记录
    memset(g_fault_diag_system.fault_history, 0, sizeof(g_fault_diag_system.fault_history));
    g_fault_diag_system.fault_head = 0;
    g_fault_diag_system.fault_count = 0;
    g_fault_diag_system.next_fault_id = 1;
    
    // 重置系统状态
    g_fault_diag_system.system_state = DIAG_STATE_NORMAL;
    
    // 重置性能统计
    g_fault_diag_system.total_diag_cycles = 0;
    g_fault_diag_system.fault_detection_count = 0;
    g_fault_diag_system.false_alarm_count = 0;
    g_fault_diag_system.diag_accuracy = 100.0f;
    
    return HAL_OK;
}

/**
 * @brief 使能/禁用故障诊断
 */
HAL_StatusTypeDef FaultDiag_Enable(bool enable) {
    if (!g_diag_initialized) {
        return HAL_ERROR;
    }
    
    g_fault_diag_system.diag_enabled = enable;
    
    if (enable) {
        g_fault_diag_system.system_state = DIAG_STATE_NORMAL;
    } else {
        g_fault_diag_system.system_state = DIAG_STATE_DISABLED;
    }
    
    return HAL_OK;
}

/**
 * @brief 更新故障诊断系统
 */
HAL_StatusTypeDef FaultDiag_Update(void) {
    if (!g_diag_initialized || !g_fault_diag_system.diag_enabled) {
        return HAL_ERROR;
    }
    
    g_fault_diag_system.total_diag_cycles++;
    
    // 检查所有轴的状态
    for (uint8_t i = 0; i < g_fault_diag_system.axis_count; i++) {
        FaultDiag_CheckTemperature(i);
        FaultDiag_CheckCurrent(i);
        FaultDiag_CheckVibration(i);
        FaultDiag_CheckPosition(i);
    }
    
    // 检查通讯状态
    FaultDiag_CheckCommunication();
    
    return HAL_OK;
}

/**
 * @brief 添加传感器
 */
HAL_StatusTypeDef FaultDiag_AddSensor(const SensorConfig_t *sensor_config) {
    if (!g_diag_initialized || !sensor_config) {
        return HAL_ERROR;
    }
    
    if (g_fault_diag_system.sensor_count >= DIAG_MAX_SENSORS) {
        return HAL_ERROR;
    }
    
    uint8_t sensor_id = g_fault_diag_system.sensor_count;
    
    // 复制传感器配置
    memcpy(&g_fault_diag_system.sensor_configs[sensor_id], sensor_config, sizeof(SensorConfig_t));
    
    // 初始化传感器数据
    memset(&g_fault_diag_system.sensor_data[sensor_id], 0, sizeof(SensorData_t));
    g_fault_diag_system.sensor_data[sensor_id].valid = true;
    
    g_fault_diag_system.sensor_count++;
    
    return HAL_OK;
}

/**
 * @brief 移除传感器
 */
HAL_StatusTypeDef FaultDiag_RemoveSensor(uint8_t sensor_id) {
    if (ValidateSensorId(sensor_id) != HAL_OK) {
        return HAL_ERROR;
    }
    
    // 标记传感器为无效
    g_fault_diag_system.sensor_configs[sensor_id].enabled = false;
    g_fault_diag_system.sensor_data[sensor_id].valid = false;
    
    return HAL_OK;
}

/**
 * @brief 更新传感器数据
 */
HAL_StatusTypeDef FaultDiag_UpdateSensorData(uint8_t sensor_id, float value) {
    if (ValidateSensorId(sensor_id) != HAL_OK) {
        return HAL_ERROR;
    }
    
    SensorData_t* sensor_data = &g_fault_diag_system.sensor_data[sensor_id];
    SensorConfig_t* sensor_config = &g_fault_diag_system.sensor_configs[sensor_id];
    
    if (!sensor_config->enabled || !sensor_data->valid) {
        return HAL_ERROR;
    }
    
    // 更新传感器数据
    sensor_data->current_value = value;
    sensor_data->last_update = GetCurrentTimeMs();
    sensor_data->sample_count++;
    
    // 更新统计值
    if (sensor_data->sample_count == 1) {
        sensor_data->min_value = value;
        sensor_data->max_value = value;
        sensor_data->average_value = value;
    } else {
        if (value < sensor_data->min_value) sensor_data->min_value = value;
        if (value > sensor_data->max_value) sensor_data->max_value = value;
        
        // 简单移动平均
        sensor_data->average_value = (sensor_data->average_value * (sensor_data->sample_count - 1) + value) / sensor_data->sample_count;
    }
    
    // 计算RMS值（简化版本）
    sensor_data->rms_value = value; // 简化处理
    
    // 检查阈值
    if (value > sensor_config->fault_threshold) {
        FaultDiag_LogFault(FAULT_TYPE_OVERTEMPERATURE, FAULT_SEVERITY_MAJOR, 
                          sensor_config->axis_id, sensor_id, value, "传感器值超过故障阈值");
    } else if (value > sensor_config->warning_threshold) {
        FaultDiag_LogFault(FAULT_TYPE_OVERTEMPERATURE, FAULT_SEVERITY_WARNING, 
                          sensor_config->axis_id, sensor_id, value, "传感器值超过警告阈值");
    }
    
    return HAL_OK;
}

/**
 * @brief 获取传感器数据
 */
SensorData_t* FaultDiag_GetSensorData(uint8_t sensor_id) {
    if (ValidateSensorId(sensor_id) != HAL_OK) {
        return NULL;
    }
    
    return &g_fault_diag_system.sensor_data[sensor_id];
}

/**
 * @brief 校准传感器
 */
HAL_StatusTypeDef FaultDiag_CalibrateSensor(uint8_t sensor_id, float reference_value) {
    if (ValidateSensorId(sensor_id) != HAL_OK) {
        return HAL_ERROR;
    }
    
    SensorConfig_t* config = &g_fault_diag_system.sensor_configs[sensor_id];
    SensorData_t* data = &g_fault_diag_system.sensor_data[sensor_id];
    
    if (data->current_value != 0.0f) {
        config->offset = reference_value - data->current_value;
        config->calibrated = true;
    }
    
    return HAL_OK;
}

/**
 * @brief 检查温度
 */
HAL_StatusTypeDef FaultDiag_CheckTemperature(uint8_t axis_id) {
    if (ValidateAxisId(axis_id) != HAL_OK) {
        return HAL_ERROR;
    }
    
    AxisDiagnostics_t* axis_diag = &g_fault_diag_system.axis_diag[axis_id];
    
    // 简化的温度检查
    if (axis_diag->motor_temperature > g_fault_diag_system.temperature_thresholds[3]) {
        FaultDiag_LogFault(FAULT_TYPE_OVERTEMPERATURE, FAULT_SEVERITY_MAJOR, 
                          axis_id, 0, axis_diag->motor_temperature, "电机温度过高");
        axis_diag->state = DIAG_STATE_FAULT;
    } else if (axis_diag->motor_temperature > g_fault_diag_system.temperature_thresholds[2]) {
        axis_diag->state = DIAG_STATE_WARNING;
    }
    
    return HAL_OK;
}

/**
 * @brief 检查电流
 */
HAL_StatusTypeDef FaultDiag_CheckCurrent(uint8_t axis_id) {
    if (ValidateAxisId(axis_id) != HAL_OK) {
        return HAL_ERROR;
    }
    
    AxisDiagnostics_t* axis_diag = &g_fault_diag_system.axis_diag[axis_id];
    
    // 简化的电流检查
    if (axis_diag->motor_current > g_fault_diag_system.current_thresholds[3]) {
        FaultDiag_LogFault(FAULT_TYPE_OVERCURRENT, FAULT_SEVERITY_MAJOR, 
                          axis_id, 0, axis_diag->motor_current, "电机电流过大");
        axis_diag->state = DIAG_STATE_FAULT;
    } else if (axis_diag->motor_current > g_fault_diag_system.current_thresholds[2]) {
        axis_diag->state = DIAG_STATE_WARNING;
    }
    
    return HAL_OK;
}

/**
 * @brief 检查振动
 */
HAL_StatusTypeDef FaultDiag_CheckVibration(uint8_t axis_id) {
    if (ValidateAxisId(axis_id) != HAL_OK) {
        return HAL_ERROR;
    }
    
    AxisDiagnostics_t* axis_diag = &g_fault_diag_system.axis_diag[axis_id];
    
    // 简化的振动检查
    float vibration_level = axis_diag->vibration_x.rms_value;
    
    if (vibration_level > g_fault_diag_system.vibration_thresholds[3]) {
        FaultDiag_LogFault(FAULT_TYPE_VIBRATION_ABNORMAL, FAULT_SEVERITY_MAJOR, 
                          axis_id, 0, vibration_level, "振动水平过高");
        axis_diag->state = DIAG_STATE_FAULT;
    } else if (vibration_level > g_fault_diag_system.vibration_thresholds[2]) {
        axis_diag->state = DIAG_STATE_WARNING;
    }
    
    return HAL_OK;
}

/**
 * @brief 检查位置
 */
HAL_StatusTypeDef FaultDiag_CheckPosition(uint8_t axis_id) {
    if (ValidateAxisId(axis_id) != HAL_OK) {
        return HAL_ERROR;
    }
    
    AxisDiagnostics_t* axis_diag = &g_fault_diag_system.axis_diag[axis_id];
    
    // 简化的位置误差检查
    if (axis_diag->position_error > 1000.0f) { // 1mm误差阈值
        FaultDiag_LogFault(FAULT_TYPE_POSITION_ERROR, FAULT_SEVERITY_MAJOR, 
                          axis_id, 0, axis_diag->position_error, "位置误差过大");
        axis_diag->state = DIAG_STATE_FAULT;
    }
    
    return HAL_OK;
}

/**
 * @brief 检查通讯
 */
HAL_StatusTypeDef FaultDiag_CheckCommunication(void) {
    // 简化的通讯检查
    // 在实际应用中这里会检查各种通讯链路状态
    return HAL_OK;
}

/**
 * @brief 记录故障
 */
HAL_StatusTypeDef FaultDiag_LogFault(FaultType_t fault_type,
                                    FaultSeverity_t severity,
                                    uint8_t axis_id,
                                    uint8_t sensor_id,
                                    float fault_value,
                                    const char *description) {
    if (!g_diag_initialized) {
        return HAL_ERROR;
    }
    
    // 如果故障记录已满，覆盖最旧的记录
    if (g_fault_diag_system.fault_count >= DIAG_HISTORY_SIZE) {
        g_fault_diag_system.fault_head = (g_fault_diag_system.fault_head + 1) % DIAG_HISTORY_SIZE;
    } else {
        g_fault_diag_system.fault_count++;
    }
    
    uint16_t index = (g_fault_diag_system.fault_head + g_fault_diag_system.fault_count - 1) % DIAG_HISTORY_SIZE;
    FaultRecord_t* record = &g_fault_diag_system.fault_history[index];
    
    // 填充故障记录
    record->fault_id = g_fault_diag_system.next_fault_id++;
    record->fault_type = fault_type;
    record->severity = severity;
    record->axis_id = axis_id;
    record->sensor_id = sensor_id;
    record->occurrence_time = GetCurrentTimeMs();
    record->clear_time = 0;
    record->duration = 0;
    record->fault_value = fault_value;
    record->threshold_value = 0.0f; // 需要根据具体传感器类型设置
    record->acknowledged = false;
    record->cleared = false;
    
    // 复制描述信息
    if (description) {
        strncpy(record->description, description, sizeof(record->description) - 1);
        record->description[sizeof(record->description) - 1] = '\0';
    }
    
    g_fault_diag_system.fault_detection_count++;
    
    // 根据严重级别更新系统状态
    if (severity >= FAULT_SEVERITY_MAJOR) {
        g_fault_diag_system.system_state = DIAG_STATE_FAULT;
    } else if (severity == FAULT_SEVERITY_WARNING && g_fault_diag_system.system_state == DIAG_STATE_NORMAL) {
        g_fault_diag_system.system_state = DIAG_STATE_WARNING;
    }
    
    return HAL_OK;
}

/**
 * @brief 获取系统状态
 */
HAL_StatusTypeDef FaultDiag_GetSystemStatus(DiagState_t *system_state,
                                           uint16_t *active_fault_count,
                                           float *overall_health) {
    if (!g_diag_initialized || !system_state || !active_fault_count || !overall_health) {
        return HAL_ERROR;
    }
    
    *system_state = g_fault_diag_system.system_state;
    *active_fault_count = g_fault_diag_system.fault_count;
    *overall_health = g_fault_diag_system.diag_accuracy;
    
    return HAL_OK;
}

/* 私有函数实现 --------------------------------------------------------------*/

/**
 * @brief 验证轴ID
 */
static HAL_StatusTypeDef ValidateAxisId(uint8_t axis_id) {
    if (!g_diag_initialized || axis_id >= DIAG_MAX_AXES) {
        return HAL_ERROR;
    }
    return HAL_OK;
}

/**
 * @brief 验证传感器ID
 */
static HAL_StatusTypeDef ValidateSensorId(uint8_t sensor_id) {
    if (!g_diag_initialized || sensor_id >= g_fault_diag_system.sensor_count) {
        return HAL_ERROR;
    }
    return HAL_OK;
}

/**
 * @brief 获取当前时间（毫秒）
 */
static uint32_t GetCurrentTimeMs(void) {
    return HAL_GetTick();
}

/**
 * @brief 检测轴故障
 * @param axis_id 轴号
 * @retval 故障类型
 */
FaultType_t FaultDiag_DetectFault(uint8_t axis_id)
{
    if (axis_id >= g_fault_diag_system.axis_count) {
        return FAULT_TYPE_NONE;
    }
    
    AxisDiagnostics_t *axis_diag = &g_fault_diag_system.axis_diag[axis_id];
    
    // 检查温度
    if (axis_diag->motor_temperature > g_fault_diag_system.temperature_thresholds[4]) {
        return FAULT_TYPE_OVERTEMPERATURE;
    }
    
    // 检查电流
    if (axis_diag->peak_current > axis_diag->average_current * 2.0f) {
        return FAULT_TYPE_OVERCURRENT;
    }
    
    // 检查振动
    if (axis_diag->vibration_x.rms_value > g_fault_diag_system.vibration_thresholds[4] ||
        axis_diag->vibration_y.rms_value > g_fault_diag_system.vibration_thresholds[4] ||
        axis_diag->vibration_z.rms_value > g_fault_diag_system.vibration_thresholds[4]) {
        return FAULT_TYPE_VIBRATION_ABNORMAL;
    }
    
    // 检查位置误差
    IMC_AxisState_t *axis_state = IMC_GetAxisState(axis_id);
    if (axis_state != NULL) {
        if (fabsf(axis_state->position_error) > 
            g_imc_system.axis_config[axis_id].following_error_limit) {
            return FAULT_TYPE_POSITION_ERROR;
        }
    }
    
    return FAULT_TYPE_NONE;
}

/**
 * @brief 执行FFT变换
 * @param time_data 时域数据
 * @param frequency_data 频域数据输出
 * @param size FFT大小
 * @retval HAL状态
 */
HAL_StatusTypeDef FaultDiag_PerformFFT(const float *time_data,
                                      float *frequency_data,
                                      uint16_t size)
{
    if (time_data == NULL || frequency_data == NULL) {
        return HAL_ERROR;
    }
    
    // 检查size是否为2的幂
    if ((size & (size - 1)) != 0) {
        return HAL_ERROR;
    }
    
    // 复制实部数据
    float *real = (float*)malloc(size * sizeof(float));
    float *imag = (float*)malloc(size * sizeof(float));
    
    if (real == NULL || imag == NULL) {
        free(real);
        free(imag);
        return HAL_ERROR;
    }
    
    memcpy(real, time_data, size * sizeof(float));
    memset(imag, 0, size * sizeof(float));
    
    // 位反转
    FaultDiag_FFTBitReverse(real, size);
    FaultDiag_FFTBitReverse(imag, size);
    
    // FFT计算
    FaultDiag_FFTCompute(real, imag, size);
    
    // 计算幅值谱
    for (uint16_t i = 0; i < size / 2; i++) {
        frequency_data[i] = sqrtf(real[i] * real[i] + imag[i] * imag[i]) / (size / 2);
    }
    
    free(real);
    free(imag);
    
    return HAL_OK;
}

/**
 * @brief FFT位反转
 * @param data 数据数组
 * @param size 数组大小
 */
static void FaultDiag_FFTBitReverse(float *data, uint16_t size)
{
    uint16_t j = 0;
    for (uint16_t i = 0; i < size - 1; i++) {
        if (i < j) {
            float temp = data[i];
            data[i] = data[j];
            data[j] = temp;
        }
        
        uint16_t k = size >> 1;
        while (k <= j) {
            j -= k;
            k >>= 1;
        }
        j += k;
    }
}

/**
 * @brief FFT蝶形运算
 * @param real 实部数组
 * @param imag 虚部数组
 * @param size 数组大小
 */
static void FaultDiag_FFTCompute(float *real, float *imag, uint16_t size)
{
    uint16_t step = 1;
    
    for (uint16_t level = 1; level < size; level <<= 1) {
        uint16_t step2 = step << 1;
        float theta = -M_PI / step;
        float sin_theta = sinf(theta);
        float cos_theta = cosf(theta);
        
        float ur = 1.0f;
        float ui = 0.0f;
        
        for (uint16_t j = 0; j < step; j++) {
            for (uint16_t i = j; i < size; i += step2) {
                uint16_t k = i + step;
                
                float tr = real[k] * ur - imag[k] * ui;
                float ti = real[k] * ui + imag[k] * ur;
                
                real[k] = real[i] - tr;
                imag[k] = imag[i] - ti;
                real[i] += tr;
                imag[i] += ti;
            }
            
            float ur_temp = ur;
            ur = ur * cos_theta - ui * sin_theta;
            ui = ur_temp * sin_theta + ui * cos_theta;
        }
        
        step = step2;
    }
}

/**
 * @brief 分析振动数据
 * @param axis_id 轴号
 * @param vibration_data 振动数据
 * @param data_size 数据大小
 * @retval HAL状态
 */
HAL_StatusTypeDef FaultDiag_AnalyzeVibration(uint8_t axis_id,
                                            const float *vibration_data,
                                            uint16_t data_size)
{
    if (axis_id >= g_fault_diag_system.axis_count || vibration_data == NULL) {
        return HAL_ERROR;
    }
    
    VibrationAnalysis_t *vibration = &g_fault_diag_system.axis_diag[axis_id].vibration_x;
    
    // 复制时域数据
    uint16_t copy_size = (data_size < DIAG_VIBRATION_SAMPLES) ? 
                        data_size : DIAG_VIBRATION_SAMPLES;
    memcpy(vibration->time_domain, vibration_data, copy_size * sizeof(float));
    
    // 计算RMS值
    vibration->rms_value = FaultDiag_CalculateRMS(vibration_data, data_size);
    
    // 计算峰值
    vibration->peak_value = FaultDiag_CalculatePeak(vibration_data, data_size);
    
    // 计算波峰因子
    vibration->crest_factor = FaultDiag_CalculateCrestFactor(vibration_data, data_size);
    
    // 计算峭度
    vibration->kurtosis = FaultDiag_CalculateKurtosis(vibration_data, data_size);
    
    // 执行FFT分析
    FaultDiag_PerformFFT(vibration->time_domain, vibration->frequency_domain, DIAG_FFT_SIZE);
    
    // 频谱分析
    SpectrumAnalysis_t spectrum_result;
    FaultDiag_AnalyzeSpectrum(vibration->frequency_domain, DIAG_FFT_SIZE / 2,
                             vibration->sample_rate, &spectrum_result);
    
    // 更新振动分析结果
    vibration->dominant_frequency = spectrum_result.peak_frequency;
    vibration->amplitude_at_1x = spectrum_result.fundamental_frequency;
    vibration->amplitude_at_2x = spectrum_result.harmonics[1];
    vibration->analysis_valid = true;
    
    // 检测轴承故障
    FaultDiag_DetectBearingFault(vibration);
    
    // 检测不平衡
    FaultDiag_DetectUnbalance(vibration);
    
    // 检测不对中
    FaultDiag_DetectMisalignment(vibration);
    
    return HAL_OK;
}

/**
 * @brief 检测轴承故障
 * @param vibration 振动分析数据
 * @retval HAL状态
 */
HAL_StatusTypeDef FaultDiag_DetectBearingFault(const VibrationAnalysis_t *vibration)
{
    if (vibration == NULL || !vibration->analysis_valid) {
        return HAL_ERROR;
    }
    
    // 轴承故障特征频率（示例值）
    float shaft_speed = 1800.0f / 60.0f; // 30 Hz
    float bpfo = 3.585f * shaft_speed;  // 外圈故障频率
    float bpfi = 5.415f * shaft_speed;  // 内圈故障频率
    float bsf = 2.322f * shaft_speed;   // 滚动体故障频率
    float ftf = 0.398f * shaft_speed;   // 保持架故障频率
    
    // 在频谱中查找轴承故障特征频率
    bool bearing_fault = false;
    
    for (uint16_t i = 0; i < DIAG_FFT_SIZE / 2; i++) {
        float freq = i * vibration->sample_rate / DIAG_FFT_SIZE;
        float amplitude = vibration->frequency_domain[i];
        
        // 检查外圈故障
        if (fabsf(freq - bpfo) < BEARING_FREQ_TOLERANCE * bpfo &&
            amplitude > vibration->rms_value * 2.0f) {
            bearing_fault = true;
            break;
        }
        
        // 检查内圈故障
        if (fabsf(freq - bpfi) < BEARING_FREQ_TOLERANCE * bpfi &&
            amplitude > vibration->rms_value * 2.0f) {
            bearing_fault = true;
            break;
        }
        
        // 检查滚动体故障
        if (fabsf(freq - bsf) < BEARING_FREQ_TOLERANCE * bsf &&
            amplitude > vibration->rms_value * 1.5f) {
            bearing_fault = true;
            break;
        }
    }
    
    if (bearing_fault) {
        FaultDiag_LogFault(FAULT_TYPE_BEARING_WEAR, FAULT_SEVERITY_WARNING,
                          0, 0, vibration->peak_value, "轴承磨损检测");
    }
    
    return HAL_OK;
}

/**
 * @brief 检测不平衡
 * @param vibration 振动分析数据
 * @retval HAL状态
 */
HAL_StatusTypeDef FaultDiag_DetectUnbalance(const VibrationAnalysis_t *vibration)
{
    if (vibration == NULL || !vibration->analysis_valid) {
        return HAL_ERROR;
    }
    
    // 不平衡的特征：1倍频振动占主导
    float ratio_1x = vibration->amplitude_at_1x / vibration->rms_value;
    
    if (ratio_1x > 0.8f && vibration->amplitude_at_1x > 
        g_fault_diag_system.vibration_thresholds[1]) {
        FaultDiag_LogFault(FAULT_TYPE_MOTOR_UNBALANCE, FAULT_SEVERITY_WARNING,
                          0, 0, vibration->amplitude_at_1x, "电机不平衡检测");
    }
    
    return HAL_OK;
}

/**
 * @brief 检测不对中
 * @param vibration 振动分析数据
 * @retval HAL状态
 */
HAL_StatusTypeDef FaultDiag_DetectMisalignment(const VibrationAnalysis_t *vibration)
{
    if (vibration == NULL || !vibration->analysis_valid) {
        return HAL_ERROR;
    }
    
    // 不对中的特征：2倍频振动较大
    float ratio_2x = vibration->amplitude_at_2x / vibration->amplitude_at_1x;
    
    if (ratio_2x > 0.5f && vibration->amplitude_at_2x > 
        g_fault_diag_system.vibration_thresholds[1]) {
        FaultDiag_LogFault(FAULT_TYPE_COUPLING_LOOSE, FAULT_SEVERITY_WARNING,
                          0, 0, vibration->amplitude_at_2x, "联轴器不对中检测");
    }
    
    return HAL_OK;
}

/**
 * @brief 计算健康指数
 * @param axis_id 轴号
 * @param health_index 健康指数输出
 * @retval HAL状态
 */
HAL_StatusTypeDef FaultDiag_CalculateHealthIndex(uint8_t axis_id,
                                                float *health_index)
{
    if (axis_id >= g_fault_diag_system.axis_count || health_index == NULL) {
        return HAL_ERROR;
    }
    
    AxisDiagnostics_t *axis_diag = &g_fault_diag_system.axis_diag[axis_id];
    float score = 100.0f;
    
    // 温度评分（30%权重）
    float temp_score = 100.0f;
    if (axis_diag->motor_temperature > TEMPERATURE_WARNING) {
        temp_score = 100.0f * (TEMPERATURE_CRITICAL - axis_diag->motor_temperature) /
                    (TEMPERATURE_CRITICAL - TEMPERATURE_WARNING);
        if (temp_score < 0) temp_score = 0;
    }
    score -= (100.0f - temp_score) * 0.3f;
    
    // 振动评分（30%权重）
    float vib_score = 100.0f;
    float max_vib = fmaxf(axis_diag->vibration_x.rms_value,
                         fmaxf(axis_diag->vibration_y.rms_value,
                               axis_diag->vibration_z.rms_value));
    if (max_vib > g_fault_diag_system.vibration_thresholds[1]) {
        vib_score = 100.0f * (g_fault_diag_system.vibration_thresholds[4] - max_vib) /
                   (g_fault_diag_system.vibration_thresholds[4] - 
                    g_fault_diag_system.vibration_thresholds[1]);
        if (vib_score < 0) vib_score = 0;
    }
    score -= (100.0f - vib_score) * 0.3f;
    
    // 运行时间评分（20%权重）
    float runtime_score = 100.0f;
    uint32_t expected_life = 10000; // 预期寿命（小时）
    if (axis_diag->total_runtime > expected_life * 3600) {
        runtime_score = 0;
    } else if (axis_diag->total_runtime > expected_life * 3600 * 0.8f) {
        runtime_score = 100.0f * (expected_life * 3600 - axis_diag->total_runtime) /
                       (expected_life * 3600 * 0.2f);
    }
    score -= (100.0f - runtime_score) * 0.2f;
    
    // 故障历史评分（20%权重）
    float fault_score = 100.0f;
    if (axis_diag->fault_count > 0) {
        fault_score = 100.0f / (1.0f + axis_diag->fault_count * 0.1f);
    }
    score -= (100.0f - fault_score) * 0.2f;
    
    // 限制范围
    if (score < 0) score = 0;
    if (score > 100) score = 100;
    
    *health_index = score;
    axis_diag->health_index = score;
    
    return HAL_OK;
}

/**
 * @brief 预测维护时间
 * @param axis_id 轴号
 * @param maintenance_time 维护时间输出
 * @retval HAL状态
 */
HAL_StatusTypeDef FaultDiag_PredictMaintenanceTime(uint8_t axis_id,
                                                  uint32_t *maintenance_time)
{
    if (axis_id >= g_fault_diag_system.axis_count || maintenance_time == NULL) {
        return HAL_ERROR;
    }
    
    AxisDiagnostics_t *axis_diag = &g_fault_diag_system.axis_diag[axis_id];
    
    // 基于健康指数预测维护时间
    float health_index = axis_diag->health_index;
    
    if (health_index < 20.0f) {
        // 立即维护
        *maintenance_time = 0;
    } else if (health_index < 50.0f) {
        // 24小时内维护
        *maintenance_time = 24 * 3600;
    } else if (health_index < 70.0f) {
        // 一周内维护
        *maintenance_time = 7 * 24 * 3600;
    } else {
        // 基于退化趋势预测
        // 简化模型：假设线性退化
        float degradation_rate = 0.01f; // 每天退化1%
        float days_to_maintenance = (health_index - 50.0f) / degradation_rate;
        *maintenance_time = (uint32_t)(days_to_maintenance * 24 * 3600);
    }
    
    axis_diag->next_maintenance = HAL_GetTick() / 1000 + *maintenance_time;
    
    return HAL_OK;
}

/**
 * @brief 分析频谱
 * @param spectrum 频谱数据
 * @param size 频谱大小
 * @param sample_rate 采样率
 * @param result 分析结果
 * @retval HAL状态
 */
static HAL_StatusTypeDef FaultDiag_AnalyzeSpectrum(const float *spectrum, uint16_t size,
                                                   float sample_rate, SpectrumAnalysis_t *result)
{
    if (spectrum == NULL || result == NULL) {
        return HAL_ERROR;
    }
    
    memset(result, 0, sizeof(SpectrumAnalysis_t));
    
    // 查找峰值
    float max_amplitude = 0;
    uint16_t max_index = 0;
    
    for (uint16_t i = 1; i < size; i++) {
        if (spectrum[i] > max_amplitude) {
            max_amplitude = spectrum[i];
            max_index = i;
        }
    }
    
    result->peak_frequency = max_index * sample_rate / (size * 2);
    result->peak_amplitude = max_amplitude;
    
    // 计算基频（假设为峰值频率）
    result->fundamental_frequency = result->peak_frequency;
    
    // 计算谐波
    for (int h = 0; h < 10; h++) {
        uint16_t harmonic_index = (h + 1) * max_index;
        if (harmonic_index < size) {
            result->harmonics[h] = spectrum[harmonic_index];
        }
    }
    
    // 计算噪声底
    float noise_sum = 0;
    uint16_t noise_count = 0;
    for (uint16_t i = size / 2; i < size; i++) {
        noise_sum += spectrum[i];
        noise_count++;
    }
    result->noise_floor = noise_sum / noise_count;
    
    return HAL_OK;
}

/* 工具函数实现 --------------------------------------------------------------*/

/**
 * @brief 计算RMS值
 * @param data 数据数组
 * @param size 数组大小
 * @retval RMS值
 */
float FaultDiag_CalculateRMS(const float *data, uint16_t size)
{
    if (data == NULL || size == 0) {
        return 0.0f;
    }
    
    float sum = 0.0f;
    for (uint16_t i = 0; i < size; i++) {
        sum += data[i] * data[i];
    }
    
    return sqrtf(sum / size);
}

/**
 * @brief 计算峰值
 * @param data 数据数组
 * @param size 数组大小
 * @retval 峰值
 */
float FaultDiag_CalculatePeak(const float *data, uint16_t size)
{
    if (data == NULL || size == 0) {
        return 0.0f;
    }
    
    float peak = 0.0f;
    for (uint16_t i = 0; i < size; i++) {
        float abs_value = fabsf(data[i]);
        if (abs_value > peak) {
            peak = abs_value;
        }
    }
    
    return peak;
}

/**
 * @brief 计算波峰因子
 * @param data 数据数组
 * @param size 数组大小
 * @retval 波峰因子
 */
float FaultDiag_CalculateCrestFactor(const float *data, uint16_t size)
{
    float rms = FaultDiag_CalculateRMS(data, size);
    float peak = FaultDiag_CalculatePeak(data, size);
    
    if (rms > 0) {
        return peak / rms;
    }
    
    return 0.0f;
}

/**
 * @brief 计算峭度
 * @param data 数据数组
 * @param size 数组大小
 * @retval 峭度值
 */
float FaultDiag_CalculateKurtosis(const float *data, uint16_t size)
{
    if (data == NULL || size < 4) {
        return 0.0f;
    }
    
    // 计算均值
    float mean = 0.0f;
    for (uint16_t i = 0; i < size; i++) {
        mean += data[i];
    }
    mean /= size;
    
    // 计算标准差和四阶矩
    float variance = 0.0f;
    float fourth_moment = 0.0f;
    
    for (uint16_t i = 0; i < size; i++) {
        float diff = data[i] - mean;
        variance += diff * diff;
        fourth_moment += diff * diff * diff * diff;
    }
    
    variance /= size;
    fourth_moment /= size;
    
    float std_dev = sqrtf(variance);
    if (std_dev > 0) {
        return fourth_moment / (std_dev * std_dev * std_dev * std_dev) - 3.0f;
    }
    
    return 0.0f;
} 