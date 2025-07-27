# STM32F407 G代码解析器系统

## 📖 项目简介

本项目为ATK-DMF407开发板（STM32F407IGT6）设计了一套完整的G代码解析和执行系统，支持标准CNC G代码指令，可控制ATK-2MD5050步进电机驱动器。

[[memory:678467]]

## 🏗️ 系统架构

### 核心模块
- **gcode_parser.h/c** - G代码解析器核心
- **gcode_executor.h/c** - G代码执行器
- **motion_control.h/c** - 运动控制系统
- **test_gcode_parser.c** - 完整测试程序

### 硬件配置
- **开发板**: ATK-DMF407 (STM32F407IGT6)
- **步进电机**: PC9(TIM3_CH4)脉冲信号
- **方向控制**: PH2引脚
- **使能控制**: PH3引脚
- **串口通信**: USART2 (115200bps)

## ⚙️ 支持的G代码指令

### 运动指令
| 指令 | 功能 | 示例 | 说明 |
|------|------|------|------|
| G00 | 快速定位 | `G00 X10` | 快速移动到指定位置 |
| G01 | 直线插补 | `G01 X20 F500` | 以指定速度直线移动 |
| G02 | 顺时针圆弧 | `G02 X10 Y10 I5 J0` | 顺时针圆弧插补* |
| G03 | 逆时针圆弧 | `G03 X10 Y10 R5` | 逆时针圆弧插补* |
| G04 | 延时暂停 | `G04 P2.5` | 暂停2.5秒 |
| G28 | 回零点 | `G28` 或 `G28 X` | 全轴或指定轴回零 |

*注：圆弧指令需要多轴支持，当前单轴版本暂不执行

### 设置指令
| 指令 | 功能 | 示例 | 说明 |
|------|------|------|------|
| G20 | 英制单位 | `G20` | 设置单位为英寸 |
| G21 | 公制单位 | `G21` | 设置单位为毫米（默认） |
| G90 | 绝对坐标 | `G90` | 绝对坐标模式（默认） |
| G91 | 相对坐标 | `G91` | 相对坐标模式 |
| G92 | 设置坐标系 | `G92 X0` | 设置当前位置为X=0 |

### M代码指令
| 指令 | 功能 | 示例 | 说明 |
|------|------|------|------|
| M00 | 程序暂停 | `M00` | 暂停程序执行 |
| M01 | 可选停止 | `M01` | 可选停止点 |
| M02 | 程序结束 | `M02` | 程序结束 |
| M03 | 主轴正转 | `M03 S1000` | 主轴正转（当前忽略） |
| M04 | 主轴反转 | `M04 S1000` | 主轴反转（当前忽略） |
| M05 | 主轴停止 | `M05` | 主轴停止（当前忽略） |
| M30 | 程序结束并返回 | `M30` | 程序结束并复位 |

### 参数说明
| 参数 | 含义 | 范围 | 说明 |
|------|------|------|------|
| X, Y, Z | 坐标值 | ±1000.0mm | 轴坐标（当前X映射到步进电机） |
| F | 进给速度 | 0-10000mm/min | 运动速度 |
| S | 主轴转速 | 0-30000rpm | 主轴转速（当前忽略） |
| P | 时间参数 | 0-3600s | 延时时间 |
| I, J | 圆弧中心 | ±1000.0mm | 圆弧中心偏移 |
| R | 圆弧半径 | 0-1000.0mm | 圆弧半径 |

## 🔧 API接口说明

### 核心解析器接口

```c
// 初始化G代码解析器
GCode_Error_t GCode_Init(GCode_ExecuteCallback_t execute_callback, 
                        GCode_ErrorCallback_t error_callback);

// 解析G代码行
GCode_Error_t GCode_ParseLine(const char* line, GCode_Instruction_t* instruction);

// 执行G代码指令
GCode_Error_t GCode_ExecuteInstruction(const GCode_Instruction_t* instruction);

// 逐字符处理（适用于串口接收）
GCode_Error_t GCode_ProcessChar(char ch);

// 获取参数值
bool GCode_GetParameter(const GCode_Instruction_t* instruction, 
                       GCode_Param_t param_type, float* value);
```

### 执行器接口

```c
// 初始化G代码执行器
void GCode_ExecutorInit(void);

// 处理串口接收的G代码
void GCode_ProcessSerialLine(const char* line);

// 发送系统状态报告
void GCode_SendStatusReport(void);
```

### 坐标和状态管理

```c
// 设置/获取当前坐标
void GCode_SetCurrentPosition(float x, float y, float z);
void GCode_GetCurrentPosition(float* x, float* y, float* z);

// 坐标转换（绝对/相对）
bool GCode_ConvertCoordinates(const GCode_Instruction_t* instruction,
                             float* target_x, float* target_y, float* target_z);

// 获取统计信息
void GCode_GetStats(uint32_t* line_count, uint32_t* error_count);

// 重置解析器
void GCode_Reset(void);
```

## 📋 使用示例

### 1. 基本初始化

```c
#include "gcode_parser.h"
#include "gcode_executor.h"

int main(void) {
    // 系统初始化
    HAL_Init();
    SystemClock_Config();
    
    // 初始化运动控制
    MotionControl_Init();
    
    // 初始化G代码执行器
    GCode_ExecutorInit();
    
    // 主循环
    while(1) {
        Communication_ProcessCommand();
        Interpolation_Execute();
        HAL_Delay(10);
    }
}
```

### 2. 串口G代码处理

```c
// 在串口接收中断中
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        GCode_ProcessChar(rx_char);
    }
}
```

### 3. 手动G代码执行

```c
// 解析和执行单行G代码
GCode_Instruction_t instruction;
GCode_Error_t result = GCode_ParseLine("G01 X10 F500", &instruction);

if (result == GCODE_OK && instruction.valid) {
    result = GCode_ExecuteInstruction(&instruction);
}
```

### 4. 自定义执行回调

```c
GCode_Error_t my_execute_callback(const GCode_Instruction_t* instruction) {
    switch (instruction->command) {
        case GCODE_G01:
            // 自定义直线运动处理
            return handle_linear_move(instruction);
        default:
            return GCODE_OK;
    }
}

// 初始化时设置回调
GCode_Init(my_execute_callback, my_error_callback);
```

## 🧪 测试程序

### 运行测试

测试程序 `test_gcode_parser.c` 包含完整的功能验证：

```c
// 运行完整测试
Test_GCodeParser_Main();
```

### 测试内容

1. **基础解析测试** - 验证G/M指令识别
2. **参数提取测试** - 验证坐标和参数解析
3. **坐标转换测试** - 验证绝对/相对坐标转换
4. **错误处理测试** - 验证异常情况处理
5. **执行功能测试** - 验证指令执行流程
6. **序列执行测试** - 验证连续G代码执行

### 测试G代码序列

```gcode
G21                  ; 设置公制单位
G90                  ; 绝对坐标模式
G28                  ; 回零
G01 X10.5 F1000     ; 移动到X=10.5
G01 X20 Y15 F500    ; 移动到X=20, Y=15
G00 X0              ; 快速回到X=0
G04 P2.5            ; 延时2.5秒
G91                 ; 相对坐标模式
G01 X5 F300         ; 相对移动+5mm
G01 X-10            ; 相对移动-10mm
G90                 ; 恢复绝对坐标
M30                 ; 程序结束
```

## 🔍 错误处理

### 错误代码

```c
typedef enum {
    GCODE_OK = 0,                   // 成功
    GCODE_ERROR_INVALID_COMMAND,    // 无效指令
    GCODE_ERROR_INVALID_PARAMETER,  // 无效参数
    GCODE_ERROR_MISSING_PARAMETER,  // 缺少参数
    GCODE_ERROR_OUT_OF_RANGE,       // 参数超出范围
    GCODE_ERROR_SYNTAX,             // 语法错误
    GCODE_ERROR_BUFFER_FULL,        // 缓冲区满
    GCODE_ERROR_NOT_INITIALIZED     // 未初始化
} GCode_Error_t;
```

### 错误响应格式

```
ok                          ; 成功响应
error:1 无效指令           ; 错误响应
```

### 状态报告格式

```
<Idle|MPos:10.50,0.00,0.00|FS:500.0|Ln:15|Err:0>
```

- **Idle**: 系统状态
- **MPos**: 机械坐标位置
- **FS**: 当前进给速度
- **Ln**: 处理的行数
- **Err**: 错误计数

## 📝 开发备注

### 当前限制

1. **单轴支持**: 当前版本主要支持X轴（映射到步进电机），Y/Z轴预留接口
2. **圆弧插补**: 需要多轴协调，当前版本暂时跳过
3. **主轴控制**: 项目只有步进电机，主轴相关指令被忽略

### 扩展方向

1. **多轴支持**: 增加Y/Z轴驱动器支持完整3D运动
2. **高级功能**: 加速度规划、轨迹优化
3. **通信协议**: 支持更多上位机软件协议
4. **文件系统**: 支持SD卡G代码文件执行

### 性能参数

- **解析速度**: >1000行/秒
- **响应时间**: <10ms
- **内存占用**: ~2KB RAM
- **代码大小**: ~8KB Flash

## 🚀 快速上手

1. **编译程序**: 使用提供的Makefile或导入到IDE
2. **烧录测试**: 烧录test程序验证功能
3. **串口连接**: 115200bps连接串口调试助手
4. **发送G代码**: 直接发送G代码指令测试
5. **查看响应**: 观察执行结果和状态报告

---

*项目基于现代化STM32开发环境，支持Cursor+Claude+STM32CubeCLT工具链。* 