# 技术架构文档

## 概述

本文档详细描述了ATK-DMF407步进电机控制系统的技术架构，包括软件架构设计、硬件抽象、实时性保证、模块化设计等关键技术决策。

## 系统架构

### 分层架构设计

```
┌─────────────────────────────────────────────┐
│           应用层 (Application Layer)         │
│  - UI界面  - G代码执行  - 运动规划          │
├─────────────────────────────────────────────┤
│         中间件层 (Middleware Layer)          │
│  - 运动控制  - 插补算法  - 通信协议         │
├─────────────────────────────────────────────┤
│           驱动层 (Driver Layer)              │
│  - 电机驱动  - LCD驱动  - 传感器驱动        │
├─────────────────────────────────────────────┤
│      硬件抽象层 (HAL - STM32 HAL)          │
│  - GPIO  - Timer  - FSMC  - UART  - I2C     │
└─────────────────────────────────────────────┘
```

### 模块化设计

#### 1. 运动控制模块
```c
// 模块接口设计
typedef struct {
    // 初始化函数
    void (*init)(void);
    // 运动控制函数
    void (*move)(int32_t position, uint32_t speed);
    // 状态查询函数
    MotionStatus (*get_status)(void);
    // 紧急停止
    void (*emergency_stop)(void);
} MotionControlInterface;
```

#### 2. 通信模块
```c
// 统一的通信接口
typedef struct {
    void (*init)(void);
    int (*send)(uint8_t* data, uint32_t len);
    int (*receive)(uint8_t* buffer, uint32_t max_len);
    bool (*is_connected)(void);
} CommunicationInterface;
```

#### 3. 插补算法模块
```c
// 插补器接口
typedef struct {
    void (*init)(InterpolatorType type);
    bool (*calculate_next_point)(void);
    void (*set_parameters)(void* params);
    InterpolationResult (*get_result)(void);
} InterpolatorInterface;
```

## 实时性设计

### 1. 任务优先级分配

| 任务名称 | 优先级 | 周期 | 功能描述 |
|---------|--------|------|----------|
| 运动控制任务 | 最高(5) | 1ms | 步进脉冲生成、位置控制 |
| 插补计算任务 | 高(4) | 2ms | 轨迹插补计算 |
| 通信任务 | 中(3) | 10ms | 命令接收与响应 |
| UI更新任务 | 低(2) | 50ms | 界面刷新 |
| 诊断任务 | 最低(1) | 1000ms | 系统健康监测 |

### 2. 中断优先级管理

```c
// 中断优先级定义
#define MOTION_TIMER_IRQ_PRIORITY    0  // 最高优先级
#define ENCODER_IRQ_PRIORITY         1
#define UART_IRQ_PRIORITY           2
#define I2C_IRQ_PRIORITY            3
#define SYSTICK_IRQ_PRIORITY        15 // 最低优先级
```

### 3. 临界区保护

```c
// 临界区保护宏
#define ENTER_CRITICAL() portENTER_CRITICAL()
#define EXIT_CRITICAL()  portEXIT_CRITICAL()

// 使用示例
void update_position(int32_t new_pos) {
    ENTER_CRITICAL();
    current_position = new_pos;
    EXIT_CRITICAL();
}
```

## 内存管理

### 1. 静态内存分配
- 所有关键数据结构使用静态分配
- 避免动态内存分配导致的碎片化
- 预定义缓冲区大小

```c
// 静态缓冲区定义
static uint8_t uart_rx_buffer[UART_BUFFER_SIZE];
static uint8_t gcode_buffer[GCODE_BUFFER_SIZE];
static MotionCommand motion_queue[MOTION_QUEUE_SIZE];
```

### 2. DMA优化
- UART收发使用DMA
- LCD数据传输使用DMA
- 减少CPU占用率

```c
// DMA传输配置
HAL_UART_Receive_DMA(&huart1, uart_rx_buffer, UART_BUFFER_SIZE);
```

## 性能优化

### 1. 算法优化

#### 查表法优化
```c
// 预计算的S曲线加速度表
static const float s_curve_table[S_CURVE_TABLE_SIZE] = {
    // 预计算值...
};

// 使用查表代替实时计算
float get_acceleration(uint32_t index) {
    return s_curve_table[index];
}
```

#### 定点数运算
```c
// 使用定点数提高运算速度
typedef int32_t fixed_point_t;
#define FIXED_POINT_SCALE 1000

fixed_point_t fp_multiply(fixed_point_t a, fixed_point_t b) {
    return (a * b) / FIXED_POINT_SCALE;
}
```

### 2. 编译器优化
```makefile
# 编译优化选项
CFLAGS += -O2              # 优化级别2
CFLAGS += -ffast-math      # 快速数学运算
CFLAGS += -ffunction-sections -fdata-sections  # 函数和数据分段
LDFLAGS += -Wl,--gc-sections  # 移除未使用的段
```

## 错误处理机制

### 1. 分级错误处理
```c
typedef enum {
    ERROR_NONE = 0,
    ERROR_WARNING,     // 警告级别
    ERROR_RECOVERABLE, // 可恢复错误
    ERROR_CRITICAL,    // 严重错误
    ERROR_FATAL       // 致命错误
} ErrorLevel;

typedef struct {
    ErrorLevel level;
    uint32_t code;
    const char* message;
    void (*handler)(void);
} ErrorInfo;
```

### 2. 看门狗保护
```c
// 独立看门狗配置
void watchdog_init(void) {
    IWDG->KR = 0x5555;    // 使能寄存器访问
    IWDG->PR = 4;         // 预分频器
    IWDG->RLR = 4095;     // 重载值
    IWDG->KR = 0xCCCC;    // 启动看门狗
}

// 定期喂狗
void watchdog_feed(void) {
    IWDG->KR = 0xAAAA;
}
```

## 通信协议设计

### 1. 帧格式定义
```
┌──────┬──────┬──────┬────────┬──────────┬──────┐
│ SOF  │ LEN  │ CMD  │ DATA   │ CHECKSUM │ EOF  │
│ 1B   │ 2B   │ 1B   │ 0-255B │ 2B       │ 1B   │
└──────┴──────┴──────┴────────┴──────────┴──────┘
```

### 2. 协议状态机
```c
typedef enum {
    STATE_IDLE,
    STATE_SOF,
    STATE_LENGTH,
    STATE_COMMAND,
    STATE_DATA,
    STATE_CHECKSUM,
    STATE_EOF
} ProtocolState;
```

## 调试和诊断

### 1. 调试输出系统
```c
// 分级调试输出
#define DEBUG_LEVEL_ERROR   1
#define DEBUG_LEVEL_WARNING 2
#define DEBUG_LEVEL_INFO    3
#define DEBUG_LEVEL_DEBUG   4

#define DEBUG_PRINT(level, fmt, ...) \
    do { \
        if (level <= DEBUG_CURRENT_LEVEL) { \
            printf("[%s] " fmt "\n", debug_level_str[level], ##__VA_ARGS__); \
        } \
    } while(0)
```

### 2. 运行时统计
```c
typedef struct {
    uint32_t task_execution_time[TASK_COUNT];
    uint32_t cpu_usage;
    uint32_t free_heap;
    uint32_t max_stack_usage[TASK_COUNT];
    uint32_t error_count[ERROR_TYPE_COUNT];
} SystemStatistics;
```

## 扩展性设计

### 1. 插件式架构
```c
// 功能模块注册机制
typedef struct {
    const char* name;
    void (*init)(void);
    void (*process)(void);
    void (*deinit)(void);
} ModuleDescriptor;

// 模块注册表
static ModuleDescriptor* module_table[MAX_MODULES];

// 注册新模块
void register_module(ModuleDescriptor* module);
```

### 2. 配置管理
```c
// 参数存储结构
typedef struct {
    uint32_t magic;      // 魔术字，验证有效性
    uint32_t version;    // 配置版本
    MotionParams motion; // 运动参数
    CommParams comm;     // 通信参数
    UIParams ui;        // 界面参数
    uint32_t checksum;  // 校验和
} SystemConfig;

// 参数保存和加载
void config_save(SystemConfig* cfg);
bool config_load(SystemConfig* cfg);
```

## 安全性考虑

### 1. 边界检查
```c
// 所有数组访问都进行边界检查
#define SAFE_ARRAY_ACCESS(array, index, size) \
    ((index) < (size) ? (array)[index] : handle_out_of_bounds())
```

### 2. 输入验证
```c
// 命令参数验证
bool validate_motion_command(MotionCommand* cmd) {
    if (cmd->position > MAX_POSITION || 
        cmd->position < MIN_POSITION) {
        return false;
    }
    if (cmd->speed > MAX_SPEED) {
        return false;
    }
    return true;
}
```

## 最佳实践

### 1. 代码规范
- 使用一致的命名规则
- 添加详细的函数注释
- 保持函数简短（<50行）
- 避免深层嵌套（<4层）

### 2. 版本管理
- 使用语义化版本号
- 维护详细的更新日志
- 标记重要的发布版本

### 3. 测试策略
- 单元测试覆盖核心算法
- 集成测试验证模块交互
- 压力测试检验系统稳定性
- 边界测试确保安全性

## 性能指标

### 系统性能
- **最大脉冲频率**: 500kHz
- **插补周期**: 1ms
- **位置分辨率**: 0.01mm
- **速度范围**: 0.1-1000mm/s
- **加速度范围**: 1-10000mm/s²

### 资源使用
- **Flash使用**: <256KB
- **RAM使用**: <64KB
- **CPU占用**: <70%
- **实时响应**: <1ms

## 未来发展方向

1. **网络功能**
   - 以太网通信支持
   - Web配置界面
   - 远程监控功能

2. **高级算法**
   - 机器学习优化
   - 自适应控制
   - 预测性维护

3. **扩展支持**
   - 更多电机类型
   - 视觉系统集成
   - 多轴联动优化

---

*本架构设计遵循工业控制系统的最佳实践，确保系统的可靠性、实时性和可扩展性。* 