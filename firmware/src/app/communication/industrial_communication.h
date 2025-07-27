/**
  ******************************************************************************
  * @file    industrial_communication.h
  * @brief   工业通讯协议栈
  * @author  Industrial Motion Control Team
  * @version 2.0
  * @date    2025-01-27
  ******************************************************************************
  * @attention
  * 
  * 本模块实现了工业级CNC控制系统的通讯协议栈：
  * - EtherCAT实时以太网协议
  * - Modbus TCP/RTU协议
  * - CAN总线协议
  * - 工业以太网协议
  * - 实时数据交换
  * - 网络诊断和监控
  * 
  ******************************************************************************
  */

#ifndef __INDUSTRIAL_COMMUNICATION_H
#define __INDUSTRIAL_COMMUNICATION_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "motion_control.h"
// #include "lwip.h" // 暂时注释掉，简化版本不依赖LwIP
#include <stdint.h>
#include <stdbool.h>

/* 宏定义 --------------------------------------------------------------------*/
#define COMM_MAX_DEVICES                32      // 最大设备数
#define COMM_MAX_CHANNELS               8       // 最大通道数
#define COMM_BUFFER_SIZE                2048    // 通讯缓冲区大小
#define COMM_TIMEOUT_MS                 1000    // 通讯超时时间 (ms)
#define COMM_RETRY_COUNT                3       // 重试次数

// EtherCAT配置
#define ETHERCAT_MAX_SLAVES             64      // 最大从站数
#define ETHERCAT_CYCLE_TIME_US          1000    // 周期时间 (μs)
#define ETHERCAT_PDO_SIZE               256     // PDO数据大小
#define ETHERCAT_SDO_TIMEOUT_MS         5000    // SDO超时时间

// Modbus配置
#define MODBUS_MAX_REGISTERS            1000    // 最大寄存器数
#define MODBUS_MAX_COILS                1000    // 最大线圈数
#define MODBUS_TCP_PORT                 502     // Modbus TCP端口
#define MODBUS_RTU_BAUDRATE             115200  // Modbus RTU波特率

// CAN配置
#define CAN_MAX_NODES                   32      // 最大CAN节点数
#define CAN_BAUDRATE                    1000000 // CAN波特率 (1Mbps)
#define CAN_TX_BUFFER_SIZE              64      // 发送缓冲区大小
#define CAN_RX_BUFFER_SIZE              64      // 接收缓冲区大小

/* 枚举类型定义 --------------------------------------------------------------*/

/**
 * @brief 通讯协议类型
 */
typedef enum {
    COMM_PROTOCOL_ETHERCAT = 0,     // EtherCAT协议
    COMM_PROTOCOL_MODBUS_TCP,       // Modbus TCP协议
    COMM_PROTOCOL_MODBUS_RTU,       // Modbus RTU协议
    COMM_PROTOCOL_CAN_OPEN,         // CANopen协议
    COMM_PROTOCOL_ETHERNET_IP,      // EtherNet/IP协议
    COMM_PROTOCOL_PROFINET,         // PROFINET协议
    COMM_PROTOCOL_CUSTOM            // 自定义协议
} CommProtocol_t;

/**
 * @brief 通讯状态
 */
typedef enum {
    COMM_STATE_DISCONNECTED = 0,   // 断开连接
    COMM_STATE_CONNECTING,          // 连接中
    COMM_STATE_CONNECTED,           // 已连接
    COMM_STATE_OPERATIONAL,         // 运行状态
    COMM_STATE_ERROR,               // 错误状态
    COMM_STATE_TIMEOUT              // 超时状态
} CommState_t;

/**
 * @brief 数据类型
 */
typedef enum {
    DATA_TYPE_BOOL = 0,             // 布尔型
    DATA_TYPE_INT8,                 // 8位整型
    DATA_TYPE_UINT8,                // 8位无符号整型
    DATA_TYPE_INT16,                // 16位整型
    DATA_TYPE_UINT16,               // 16位无符号整型
    DATA_TYPE_INT32,                // 32位整型
    DATA_TYPE_UINT32,               // 32位无符号整型
    DATA_TYPE_FLOAT,                // 单精度浮点
    DATA_TYPE_DOUBLE                // 双精度浮点
} DataType_t;

/**
 * @brief EtherCAT从站状态
 */
typedef enum {
    ETHERCAT_STATE_INIT = 1,        // 初始化状态
    ETHERCAT_STATE_PREOP = 2,       // 预操作状态
    ETHERCAT_STATE_SAFEOP = 4,      // 安全操作状态
    ETHERCAT_STATE_OP = 8           // 操作状态
} EtherCATState_t;

/**
 * @brief Modbus功能码
 */
typedef enum {
    MODBUS_FC_READ_COILS = 1,           // 读线圈
    MODBUS_FC_READ_DISCRETE_INPUTS = 2, // 读离散输入
    MODBUS_FC_READ_HOLDING_REGS = 3,    // 读保持寄存器
    MODBUS_FC_READ_INPUT_REGS = 4,      // 读输入寄存器
    MODBUS_FC_WRITE_SINGLE_COIL = 5,    // 写单个线圈
    MODBUS_FC_WRITE_SINGLE_REG = 6,     // 写单个寄存器
    MODBUS_FC_WRITE_MULTIPLE_COILS = 15,    // 写多个线圈
    MODBUS_FC_WRITE_MULTIPLE_REGS = 16      // 写多个寄存器
} ModbusFunctionCode_t;

/* 结构体定义 ----------------------------------------------------------------*/

/**
 * @brief 通讯设备信息
 */
typedef struct {
    uint8_t device_id;              // 设备ID
    char device_name[32];           // 设备名称
    CommProtocol_t protocol;        // 通讯协议
    CommState_t state;              // 连接状态
    uint32_t ip_address;            // IP地址
    uint16_t port;                  // 端口号
    uint32_t baudrate;              // 波特率
    
    // 统计信息
    uint32_t tx_count;              // 发送计数
    uint32_t rx_count;              // 接收计数
    uint32_t error_count;           // 错误计数
    uint32_t last_activity;         // 最后活动时间
} CommDevice_t;

/**
 * @brief 数据映射表项
 */
typedef struct {
    uint16_t address;               // 地址
    DataType_t data_type;           // 数据类型
    uint8_t data_size;              // 数据大小
    void *data_ptr;                 // 数据指针
    bool readable;                  // 可读标志
    bool writable;                  // 可写标志
    char description[64];           // 描述信息
} DataMapEntry_t;

/**
 * @brief EtherCAT从站配置
 */
typedef struct {
    uint16_t station_address;       // 从站地址
    uint32_t vendor_id;             // 厂商ID
    uint32_t product_code;          // 产品代码
    uint32_t revision;              // 版本号
    uint32_t serial_number;         // 序列号
    EtherCATState_t state;          // 从站状态
    
    // PDO配置
    uint8_t pdo_input[ETHERCAT_PDO_SIZE];   // 输入PDO
    uint8_t pdo_output[ETHERCAT_PDO_SIZE];  // 输出PDO
    uint16_t pdo_input_size;                // 输入PDO大小
    uint16_t pdo_output_size;               // 输出PDO大小
    
    // 状态信息
    bool online;                    // 在线状态
    uint16_t working_counter;       // 工作计数器
    uint32_t cycle_count;           // 周期计数
} EtherCATSlave_t;

/**
 * @brief EtherCAT主站配置
 */
typedef struct {
    bool enabled;                   // 使能状态
    uint32_t cycle_time_us;         // 周期时间 (μs)
    uint16_t slave_count;           // 从站数量
    EtherCATSlave_t slaves[ETHERCAT_MAX_SLAVES]; // 从站列表
    
    // 状态信息
    uint32_t total_cycles;          // 总周期数
    uint32_t error_cycles;          // 错误周期数
    uint32_t max_cycle_time;        // 最大周期时间
    bool all_slaves_operational;    // 所有从站运行状态
} EtherCATMaster_t;

/**
 * @brief Modbus寄存器映射
 */
typedef struct {
    // 线圈 (0x区域)
    bool coils[MODBUS_MAX_COILS];
    
    // 离散输入 (1x区域)
    bool discrete_inputs[MODBUS_MAX_COILS];
    
    // 输入寄存器 (3x区域)
    uint16_t input_registers[MODBUS_MAX_REGISTERS];
    
    // 保持寄存器 (4x区域)
    uint16_t holding_registers[MODBUS_MAX_REGISTERS];
    
    // 数据映射表
    DataMapEntry_t data_map[256];
    uint16_t data_map_count;
} ModbusDataMap_t;

/**
 * @brief Modbus通讯配置
 */
typedef struct {
    bool tcp_enabled;               // TCP使能
    bool rtu_enabled;               // RTU使能
    uint16_t tcp_port;              // TCP端口
    uint32_t rtu_baudrate;          // RTU波特率
    uint8_t slave_address;          // 从站地址
    
    // 数据映射
    ModbusDataMap_t data_map;
    
    // 统计信息
    uint32_t request_count;         // 请求计数
    uint32_t response_count;        // 响应计数
    uint32_t exception_count;       // 异常计数
} ModbusConfig_t;

/**
 * @brief CAN消息结构
 */
typedef struct {
    uint32_t id;                    // CAN ID
    uint8_t data[8];                // 数据
    uint8_t length;                 // 数据长度
    bool extended_id;               // 扩展ID标志
    bool remote_frame;              // 远程帧标志
    uint32_t timestamp;             // 时间戳
} CANMessage_t;

/**
 * @brief CAN节点配置
 */
typedef struct {
    uint8_t node_id;                // 节点ID
    char node_name[32];             // 节点名称
    bool online;                    // 在线状态
    uint32_t heartbeat_time;        // 心跳时间
    uint32_t last_heartbeat;        // 最后心跳
    
    // PDO配置
    uint32_t tpdo_id[4];            // 发送PDO ID
    uint32_t rpdo_id[4];            // 接收PDO ID
    uint8_t tpdo_data[4][8];        // 发送PDO数据
    uint8_t rpdo_data[4][8];        // 接收PDO数据
} CANNode_t;

/**
 * @brief CAN通讯配置
 */
typedef struct {
    bool enabled;                   // 使能状态
    uint32_t baudrate;              // 波特率
    uint8_t node_count;             // 节点数量
    CANNode_t nodes[CAN_MAX_NODES]; // 节点列表
    
    // 消息缓冲区
    CANMessage_t tx_buffer[CAN_TX_BUFFER_SIZE];
    CANMessage_t rx_buffer[CAN_RX_BUFFER_SIZE];
    uint8_t tx_head, tx_tail;       // 发送缓冲区指针
    uint8_t rx_head, rx_tail;       // 接收缓冲区指针
    
    // 统计信息
    uint32_t tx_count;              // 发送计数
    uint32_t rx_count;              // 接收计数
    uint32_t error_count;           // 错误计数
} CANConfig_t;

/**
 * @brief 工业通讯控制器
 */
typedef struct {
    // 设备列表
    CommDevice_t devices[COMM_MAX_DEVICES];
    uint8_t device_count;
    
    // 协议配置
    EtherCATMaster_t ethercat;      // EtherCAT配置
    ModbusConfig_t modbus;          // Modbus配置
    CANConfig_t can;                // CAN配置
    
    // 全局状态
    bool global_enable;             // 全局使能
    uint32_t cycle_counter;         // 周期计数器
    uint32_t error_counter;         // 错误计数器
    
    // 性能统计
    uint32_t total_tx_bytes;        // 总发送字节数
    uint32_t total_rx_bytes;        // 总接收字节数
    float network_utilization;      // 网络利用率
} IndustrialCommController_t;

/* 全局变量声明 --------------------------------------------------------------*/
extern IndustrialCommController_t g_comm_controller;

/* 函数声明 ------------------------------------------------------------------*/

/* 系统初始化和管理 */
HAL_StatusTypeDef CommInit(void);
HAL_StatusTypeDef CommReset(void);
HAL_StatusTypeDef CommEnable(bool enable);
HAL_StatusTypeDef CommUpdate(void);

/* 设备管理 */
HAL_StatusTypeDef CommAddDevice(const CommDevice_t *device);
HAL_StatusTypeDef CommRemoveDevice(uint8_t device_id);
CommDevice_t* CommGetDevice(uint8_t device_id);
CommState_t CommGetDeviceState(uint8_t device_id);

/* EtherCAT协议 */
HAL_StatusTypeDef EtherCAT_Init(uint32_t cycle_time_us);
HAL_StatusTypeDef EtherCAT_ScanSlaves(void);
HAL_StatusTypeDef EtherCAT_ConfigureSlave(uint16_t slave_address,
                                         uint32_t vendor_id,
                                         uint32_t product_code);
HAL_StatusTypeDef EtherCAT_SetSlaveState(uint16_t slave_address,
                                        EtherCATState_t state);
HAL_StatusTypeDef EtherCAT_WritePDO(uint16_t slave_address,
                                   const uint8_t *data,
                                   uint16_t size);
HAL_StatusTypeDef EtherCAT_ReadPDO(uint16_t slave_address,
                                  uint8_t *data,
                                  uint16_t *size);
HAL_StatusTypeDef EtherCAT_ProcessCycle(void);

/* Modbus协议 */
HAL_StatusTypeDef Modbus_Init(uint16_t tcp_port, uint32_t rtu_baudrate);
HAL_StatusTypeDef Modbus_SetSlaveAddress(uint8_t address);
HAL_StatusTypeDef Modbus_MapData(uint16_t address,
                                DataType_t data_type,
                                void *data_ptr,
                                bool readable,
                                bool writable);
HAL_StatusTypeDef Modbus_ReadCoils(uint16_t start_address,
                                  uint16_t quantity,
                                  bool *values);
HAL_StatusTypeDef Modbus_WriteCoils(uint16_t start_address,
                                   uint16_t quantity,
                                   const bool *values);
HAL_StatusTypeDef Modbus_ReadHoldingRegisters(uint16_t start_address,
                                             uint16_t quantity,
                                             uint16_t *values);
HAL_StatusTypeDef Modbus_WriteHoldingRegisters(uint16_t start_address,
                                              uint16_t quantity,
                                              const uint16_t *values);
HAL_StatusTypeDef Modbus_ProcessRequest(void);

/* CAN协议 */
HAL_StatusTypeDef CAN_Init(uint32_t baudrate);
HAL_StatusTypeDef CAN_AddNode(uint8_t node_id, const char *node_name);
HAL_StatusTypeDef CAN_RemoveNode(uint8_t node_id);
HAL_StatusTypeDef CAN_SendMessage(const CANMessage_t *message);
HAL_StatusTypeDef CAN_ReceiveMessage(CANMessage_t *message);
HAL_StatusTypeDef CAN_SendPDO(uint8_t node_id, uint8_t pdo_number,
                             const uint8_t *data, uint8_t length);
HAL_StatusTypeDef CAN_ReceivePDO(uint8_t node_id, uint8_t pdo_number,
                                uint8_t *data, uint8_t *length);
HAL_StatusTypeDef CAN_SendHeartbeat(uint8_t node_id);
HAL_StatusTypeDef CAN_ProcessMessages(void);

/* 数据交换 */
HAL_StatusTypeDef CommReadData(uint8_t device_id, uint16_t address,
                              DataType_t data_type, void *data);
HAL_StatusTypeDef CommWriteData(uint8_t device_id, uint16_t address,
                               DataType_t data_type, const void *data);
HAL_StatusTypeDef CommBulkRead(uint8_t device_id, uint16_t start_address,
                              uint16_t count, void *data);
HAL_StatusTypeDef CommBulkWrite(uint8_t device_id, uint16_t start_address,
                               uint16_t count, const void *data);

/* 网络诊断 */
HAL_StatusTypeDef CommDiagnoseNetwork(char *diagnosis_buffer,
                                     uint16_t buffer_size);
HAL_StatusTypeDef CommGetNetworkStats(uint32_t *tx_bytes,
                                     uint32_t *rx_bytes,
                                     float *utilization);
HAL_StatusTypeDef CommTestConnectivity(uint8_t device_id);

/* 错误处理 */
HAL_StatusTypeDef CommHandleError(uint8_t device_id, uint32_t error_code);
HAL_StatusTypeDef CommClearErrors(uint8_t device_id);
uint32_t CommGetErrorCount(uint8_t device_id);

/* 配置管理 */
HAL_StatusTypeDef CommSaveConfig(void);
HAL_StatusTypeDef CommLoadConfig(void);
HAL_StatusTypeDef CommResetConfig(void);

/* 工具函数 */
uint16_t CommCalculateCRC16(const uint8_t *data, uint16_t length);
uint8_t CommCalculateLRC(const uint8_t *data, uint16_t length);
HAL_StatusTypeDef CommConvertEndian(void *data, DataType_t data_type);

#ifdef __cplusplus
}
#endif

#endif /* __INDUSTRIAL_COMMUNICATION_H */ 