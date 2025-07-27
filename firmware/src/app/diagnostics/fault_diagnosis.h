/**
  ******************************************************************************
  * @file    fault_diagnosis.h
  * @brief   故障诊断系统
  * @author  Industrial Motion Control Team
  * @version 2.0
  * @date    2025-01-27
  ******************************************************************************
  * @attention
  * 
  * 本模块实现了工业级CNC控制系统的故障诊断功能：
  * - 实时故障检测和识别
  * - 预测性维护算法
  * - 振动分析和频谱诊断
  * - 电机状态监控
  * - 温度监控和热管理
  * - 故障历史记录和分析
  * 
  ******************************************************************************
  */

#ifndef __FAULT_DIAGNOSIS_H
#define __FAULT_DIAGNOSIS_H

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
#define DIAG_MAX_AXES                   8       // 最大轴数
#define DIAG_MAX_SENSORS                16      // 最大传感器数
#define DIAG_HISTORY_SIZE               1000    // 故障历史记录大小
#define DIAG_VIBRATION_SAMPLES          1024    // 振动采样点数
#define DIAG_FFT_SIZE                   512     // FFT变换大小
#define DIAG_TEMPERATURE_SENSORS        8       // 温度传感器数量
#define DIAG_CURRENT_SENSORS            8       // 电流传感器数量
#define DIAG_THRESHOLD_LEVELS           5       // 阈值等级数

/* 枚举类型定义 --------------------------------------------------------------*/

/**
 * @brief 故障类型
 */
typedef enum {
    FAULT_TYPE_NONE = 0,            // 无故障
    FAULT_TYPE_OVERVOLTAGE,         // 过压故障
    FAULT_TYPE_UNDERVOLTAGE,        // 欠压故障
    FAULT_TYPE_OVERCURRENT,         // 过流故障
    FAULT_TYPE_OVERTEMPERATURE,     // 过温故障
    FAULT_TYPE_ENCODER_ERROR,       // 编码器故障
    FAULT_TYPE_POSITION_ERROR,      // 位置误差过大
    FAULT_TYPE_VELOCITY_ERROR,      // 速度误差过大
    FAULT_TYPE_VIBRATION_ABNORMAL,  // 振动异常
    FAULT_TYPE_BEARING_WEAR,        // 轴承磨损
    FAULT_TYPE_MOTOR_UNBALANCE,     // 电机不平衡
    FAULT_TYPE_GEARBOX_FAULT,       // 齿轮箱故障
    FAULT_TYPE_COUPLING_LOOSE,      // 联轴器松动
    FAULT_TYPE_COMMUNICATION_LOSS,  // 通讯丢失
    FAULT_TYPE_EMERGENCY_STOP,      // 急停触发
    FAULT_TYPE_LIMIT_SWITCH,        // 限位开关触发
    FAULT_TYPE_SERVO_ALARM,         // 伺服报警
    FAULT_TYPE_SPINDLE_FAULT,       // 主轴故障
    FAULT_TYPE_COOLANT_FAULT,       // 冷却液故障
    FAULT_TYPE_PNEUMATIC_FAULT      // 气压故障
} FaultType_t;

/**
 * @brief 故障严重级别
 */
typedef enum {
    FAULT_SEVERITY_INFO = 0,        // 信息级别
    FAULT_SEVERITY_WARNING,         // 警告级别
    FAULT_SEVERITY_MINOR,           // 轻微故障
    FAULT_SEVERITY_MAJOR,           // 严重故障
    FAULT_SEVERITY_CRITICAL         // 致命故障
} FaultSeverity_t;

/**
 * @brief 诊断状态
 */
typedef enum {
    DIAG_STATE_NORMAL = 0,          // 正常状态
    DIAG_STATE_WARNING,             // 警告状态
    DIAG_STATE_FAULT,               // 故障状态
    DIAG_STATE_MAINTENANCE,         // 维护状态
    DIAG_STATE_DISABLED             // 禁用状态
} DiagState_t;

/**
 * @brief 传感器类型
 */
typedef enum {
    SENSOR_TYPE_TEMPERATURE = 0,    // 温度传感器
    SENSOR_TYPE_VIBRATION,          // 振动传感器
    SENSOR_TYPE_CURRENT,            // 电流传感器
    SENSOR_TYPE_VOLTAGE,            // 电压传感器
    SENSOR_TYPE_PRESSURE,           // 压力传感器
    SENSOR_TYPE_POSITION,           // 位置传感器
    SENSOR_TYPE_SPEED               // 速度传感器
} SensorType_t;

/**
 * @brief 维护类型
 */
typedef enum {
    MAINTENANCE_LUBRICATION = 0,    // 润滑维护
    MAINTENANCE_BEARING_REPLACE,    // 轴承更换
    MAINTENANCE_MOTOR_CLEAN,        // 电机清洁
    MAINTENANCE_CALIBRATION,        // 校准维护
    MAINTENANCE_SOFTWARE_UPDATE,    // 软件更新
    MAINTENANCE_MECHANICAL_CHECK    // 机械检查
} MaintenanceType_t;

/* 结构体定义 ----------------------------------------------------------------*/

/**
 * @brief 传感器配置
 */
typedef struct {
    uint8_t sensor_id;              // 传感器ID
    char sensor_name[32];           // 传感器名称
    SensorType_t sensor_type;       // 传感器类型
    uint8_t axis_id;                // 关联轴号
    float min_value;                // 最小值
    float max_value;                // 最大值
    float warning_threshold;        // 警告阈值
    float fault_threshold;          // 故障阈值
    bool enabled;                   // 使能状态
    
    // 校准参数
    float offset;                   // 偏移量
    float scale_factor;             // 比例因子
    bool calibrated;                // 校准状态
} SensorConfig_t;

/**
 * @brief 传感器数据
 */
typedef struct {
    float current_value;            // 当前值
    float min_value;                // 最小值
    float max_value;                // 最大值
    float average_value;            // 平均值
    float rms_value;                // 有效值
    uint32_t sample_count;          // 采样计数
    uint32_t last_update;           // 最后更新时间
    bool valid;                     // 数据有效标志
} SensorData_t;

/**
 * @brief 振动分析结构
 */
typedef struct {
    float time_domain[DIAG_VIBRATION_SAMPLES];      // 时域数据
    float frequency_domain[DIAG_FFT_SIZE];          // 频域数据
    float rms_value;                                // 有效值
    float peak_value;                               // 峰值
    float crest_factor;                             // 波峰因子
    float kurtosis;                                 // 峭度
    float dominant_frequency;                       // 主频率
    float amplitude_at_1x;                          // 1倍频幅值
    float amplitude_at_2x;                          // 2倍频幅值
    uint32_t sample_rate;                           // 采样率
    bool analysis_valid;                            // 分析有效标志
} VibrationAnalysis_t;

/**
 * @brief 故障记录
 */
typedef struct {
    uint32_t fault_id;              // 故障ID
    FaultType_t fault_type;         // 故障类型
    FaultSeverity_t severity;       // 严重级别
    uint8_t axis_id;                // 轴号
    uint8_t sensor_id;              // 传感器ID
    uint32_t occurrence_time;       // 发生时间
    uint32_t clear_time;            // 清除时间
    uint32_t duration;              // 持续时间
    float fault_value;              // 故障值
    float threshold_value;          // 阈值
    char description[128];          // 故障描述
    bool acknowledged;              // 确认状态
    bool cleared;                   // 清除状态
} FaultRecord_t;

/**
 * @brief 轴诊断状态
 */
typedef struct {
    uint8_t axis_id;                // 轴号
    DiagState_t state;              // 诊断状态
    
    // 温度监控
    float motor_temperature;        // 电机温度
    float driver_temperature;       // 驱动器温度
    float bearing_temperature;      // 轴承温度
    
    // 电流监控
    float motor_current;            // 电机电流
    float peak_current;             // 峰值电流
    float average_current;          // 平均电流
    
    // 振动监控
    VibrationAnalysis_t vibration_x; // X向振动
    VibrationAnalysis_t vibration_y; // Y向振动
    VibrationAnalysis_t vibration_z; // Z向振动
    
    // 位置精度
    float position_error;           // 位置误差
    float repeatability_error;      // 重复性误差
    float backlash_error;           // 反向间隙误差
    
    // 运行统计
    uint32_t total_runtime;         // 总运行时间
    uint32_t cycle_count;           // 循环计数
    uint32_t fault_count;           // 故障计数
    float efficiency;               // 效率
    
    // 预测性维护
    uint32_t next_maintenance;      // 下次维护时间
    MaintenanceType_t recommended_maintenance; // 推荐维护类型
    float health_index;             // 健康指数 (0-100)
} AxisDiagnostics_t;

/**
 * @brief 频谱分析结果
 */
typedef struct {
    float fundamental_frequency;    // 基频
    float harmonics[10];            // 谐波分量
    float sidebands[20];            // 边频分量
    float noise_floor;              // 噪声底
    float peak_frequency;           // 峰值频率
    float peak_amplitude;           // 峰值幅度
    bool bearing_fault_detected;    // 检测到轴承故障
    bool unbalance_detected;        // 检测到不平衡
    bool misalignment_detected;     // 检测到不对中
} SpectrumAnalysis_t;

/**
 * @brief 趋势分析结构
 */
typedef struct {
    float data_points[100];         // 数据点
    uint8_t point_count;            // 数据点数量
    float slope;                    // 趋势斜率
    float correlation;              // 相关性
    bool increasing_trend;          // 上升趋势
    bool decreasing_trend;          // 下降趋势
    float predicted_value;          // 预测值
    uint32_t prediction_time;       // 预测时间
} TrendAnalysis_t;

/**
 * @brief 故障诊断系统
 */
typedef struct {
    // 传感器配置
    SensorConfig_t sensor_configs[DIAG_MAX_SENSORS];
    SensorData_t sensor_data[DIAG_MAX_SENSORS];
    uint8_t sensor_count;
    
    // 轴诊断状态
    AxisDiagnostics_t axis_diag[DIAG_MAX_AXES];
    uint8_t axis_count;
    
    // 故障记录
    FaultRecord_t fault_history[DIAG_HISTORY_SIZE];
    uint16_t fault_head;            // 故障记录头指针
    uint16_t fault_count;           // 故障总数
    uint32_t next_fault_id;         // 下一个故障ID
    
    // 全局状态
    DiagState_t system_state;       // 系统诊断状态
    bool diag_enabled;              // 诊断使能
    uint32_t diag_cycle_time;       // 诊断周期时间
    
    // 阈值配置
    float temperature_thresholds[DIAG_THRESHOLD_LEVELS];
    float current_thresholds[DIAG_THRESHOLD_LEVELS];
    float vibration_thresholds[DIAG_THRESHOLD_LEVELS];
    
    // 性能统计
    uint32_t total_diag_cycles;     // 总诊断周期
    uint32_t fault_detection_count; // 故障检测次数
    uint32_t false_alarm_count;     // 误报次数
    float diag_accuracy;            // 诊断准确率
} FaultDiagnosisSystem_t;

/* 全局变量声明 --------------------------------------------------------------*/
extern FaultDiagnosisSystem_t g_fault_diag_system;

/* 函数声明 ------------------------------------------------------------------*/

/* 系统初始化和管理 */
HAL_StatusTypeDef FaultDiag_Init(void);
HAL_StatusTypeDef FaultDiag_Reset(void);
HAL_StatusTypeDef FaultDiag_Enable(bool enable);
HAL_StatusTypeDef FaultDiag_Update(void);

/* 传感器管理 */
HAL_StatusTypeDef FaultDiag_AddSensor(const SensorConfig_t *sensor_config);
HAL_StatusTypeDef FaultDiag_RemoveSensor(uint8_t sensor_id);
HAL_StatusTypeDef FaultDiag_UpdateSensorData(uint8_t sensor_id, float value);
SensorData_t* FaultDiag_GetSensorData(uint8_t sensor_id);
HAL_StatusTypeDef FaultDiag_CalibrateSensor(uint8_t sensor_id, 
                                           float reference_value);

/* 故障检测 */
HAL_StatusTypeDef FaultDiag_CheckTemperature(uint8_t axis_id);
HAL_StatusTypeDef FaultDiag_CheckCurrent(uint8_t axis_id);
HAL_StatusTypeDef FaultDiag_CheckVibration(uint8_t axis_id);
HAL_StatusTypeDef FaultDiag_CheckPosition(uint8_t axis_id);
HAL_StatusTypeDef FaultDiag_CheckCommunication(void);
FaultType_t FaultDiag_DetectFault(uint8_t axis_id);

/* 振动分析 */
HAL_StatusTypeDef FaultDiag_PerformFFT(const float *time_data,
                                      float *frequency_data,
                                      uint16_t size);
HAL_StatusTypeDef FaultDiag_AnalyzeVibration(uint8_t axis_id,
                                            const float *vibration_data,
                                            uint16_t data_size);
HAL_StatusTypeDef FaultDiag_DetectBearingFault(const VibrationAnalysis_t *vibration);
HAL_StatusTypeDef FaultDiag_DetectUnbalance(const VibrationAnalysis_t *vibration);
HAL_StatusTypeDef FaultDiag_DetectMisalignment(const VibrationAnalysis_t *vibration);

/* 频谱分析 */
HAL_StatusTypeDef FaultDiag_PerformSpectrumAnalysis(const float *frequency_data,
                                                   uint16_t size,
                                                   float sample_rate,
                                                   SpectrumAnalysis_t *result);
HAL_StatusTypeDef FaultDiag_FindPeaks(const float *spectrum,
                                     uint16_t size,
                                     float *peak_frequencies,
                                     float *peak_amplitudes,
                                     uint8_t max_peaks);

/* 趋势分析 */
HAL_StatusTypeDef FaultDiag_UpdateTrend(TrendAnalysis_t *trend, float new_value);
HAL_StatusTypeDef FaultDiag_PredictValue(const TrendAnalysis_t *trend,
                                        uint32_t future_time,
                                        float *predicted_value);
HAL_StatusTypeDef FaultDiag_AnalyzeTrend(const TrendAnalysis_t *trend,
                                        bool *degradation_detected);

/* 故障记录管理 */
HAL_StatusTypeDef FaultDiag_LogFault(FaultType_t fault_type,
                                    FaultSeverity_t severity,
                                    uint8_t axis_id,
                                    uint8_t sensor_id,
                                    float fault_value,
                                    const char *description);
HAL_StatusTypeDef FaultDiag_ClearFault(uint32_t fault_id);
HAL_StatusTypeDef FaultDiag_AcknowledgeFault(uint32_t fault_id);
FaultRecord_t* FaultDiag_GetFaultRecord(uint32_t fault_id);
uint16_t FaultDiag_GetActiveFaults(FaultRecord_t *active_faults,
                                   uint16_t max_count);

/* 预测性维护 */
HAL_StatusTypeDef FaultDiag_CalculateHealthIndex(uint8_t axis_id,
                                                float *health_index);
HAL_StatusTypeDef FaultDiag_PredictMaintenanceTime(uint8_t axis_id,
                                                  uint32_t *maintenance_time);
HAL_StatusTypeDef FaultDiag_RecommendMaintenance(uint8_t axis_id,
                                                MaintenanceType_t *maintenance_type);
HAL_StatusTypeDef FaultDiag_UpdateMaintenanceRecord(uint8_t axis_id,
                                                   MaintenanceType_t maintenance_type);

/* 阈值管理 */
HAL_StatusTypeDef FaultDiag_SetTemperatureThresholds(const float *thresholds);
HAL_StatusTypeDef FaultDiag_SetCurrentThresholds(const float *thresholds);
HAL_StatusTypeDef FaultDiag_SetVibrationThresholds(const float *thresholds);
HAL_StatusTypeDef FaultDiag_GetThresholds(SensorType_t sensor_type,
                                         float *thresholds);

/* 诊断报告 */
HAL_StatusTypeDef FaultDiag_GenerateReport(uint8_t axis_id,
                                          char *report_buffer,
                                          uint16_t buffer_size);
HAL_StatusTypeDef FaultDiag_GetSystemStatus(DiagState_t *system_state,
                                           uint16_t *active_fault_count,
                                           float *overall_health);
HAL_StatusTypeDef FaultDiag_ExportFaultHistory(char *export_buffer,
                                              uint32_t buffer_size);

/* 校准和配置 */
HAL_StatusTypeDef FaultDiag_LoadConfiguration(void);
HAL_StatusTypeDef FaultDiag_SaveConfiguration(void);
HAL_StatusTypeDef FaultDiag_ResetConfiguration(void);

/* 工具函数 */
float FaultDiag_CalculateRMS(const float *data, uint16_t size);
float FaultDiag_CalculatePeak(const float *data, uint16_t size);
float FaultDiag_CalculateCrestFactor(const float *data, uint16_t size);
float FaultDiag_CalculateKurtosis(const float *data, uint16_t size);
HAL_StatusTypeDef FaultDiag_FilterData(const float *input_data,
                                      float *output_data,
                                      uint16_t size,
                                      float cutoff_frequency);

/* 统计函数 */
HAL_StatusTypeDef FaultDiag_GetStatistics(uint32_t *total_faults,
                                         uint32_t *active_faults,
                                         float *mtbf,
                                         float *mttr);
HAL_StatusTypeDef FaultDiag_ResetStatistics(void);

#ifdef __cplusplus
}
#endif

#endif /* __FAULT_DIAGNOSIS_H */ 