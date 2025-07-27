# 快速开始指南

## 硬件要求

- STM32F407IGT6开发板 (ATK-DMF407)
- ATK-2MD5050步进电机驱动器
- 步进电机
- ST-Link调试器

## 软件环境

### 1. 开发工具

- Cursor编辑器 + Claude AI
- ARM GCC工具链
- STM32CubeCLT
- Make构建系统

### 2. 安装步骤

```bash
# 1. 安装ARM GCC工具链
sudo apt-get install gcc-arm-none-eabi

# 2. 安装STM32CubeCLT
# 下载并安装STM32CubeCLT

# 3. 克隆项目
git clone <repository_url>
cd ATK_StepMotor
```

## 硬件连接

### 步进电机连接

参考ATK-2MD5050驱动器手册：

- **PUL+/PUL-**: 脉冲信号线
- **DIR+/DIR-**: 方向信号线
- **ENA+/ENA-**: 使能信号线

### 开发板接口

- 使用开发板电机接口4（默认配置）
- PC9: TIM3_CH4 PWM输出（脉冲信号）
- PH2: 方向控制
- PH3: 使能控制

## 编译和烧录

### 1. 基础编译

```bash
# 编译基础步进电机程序
make

# 或指定目标
make TARGET=main

# 清理编译文件
make clean
```

### 2. 专业功能编译

```bash
# 编译工业运动控制
./build_industrial_motion.sh

# 编译G代码解析器
./build_gcode_test.sh

# 编译高级运动控制
./build_advanced_motion.sh
```

### 3. 烧录程序

```bash
# 使用Make烧录
make flash

# 或使用st-flash
st-flash write build/ATK_StepMotor.bin 0x8000000
```

## 测试验证

### 1. 基础功能测试

编译并运行基础步进电机程序：

```bash
make TARGET=main
make flash
```

验证功能：
- 电机转动
- 方向控制
- 速度调节

### 2. 高级功能测试

```bash
# 运行工业控制测试
./build_industrial_motion.sh
make flash
```

## 硬件配置

### 步进电机参数

在`main.h`中配置电机参数：

```c
#define MOTOR_STEP_ANGLE     1.8f           // 步进角度 (度)
#define MOTOR_MICROSTEP      16             // 细分设置
#define PWM_FREQ_MEDIUM      200            // 默认频率 (Hz)
```

### GPIO配置

确认引脚配置：

```c
// 步进电机接口4引脚定义
#define MOTOR4_PUL_PIN       GPIO_PIN_9     // PC9 - TIM3_CH4 PWM输出
#define MOTOR4_DIR_PIN       GPIO_PIN_2     // PH2 - 方向控制  
#define MOTOR4_ENA_PIN       GPIO_PIN_3     // PH3 - 使能控制
```

## 功能模块

### 可用的程序模块

- `main.c` - 基础步进电机控制
- `main_simple.c` - 简化版电机控制
- `main_industrial_motion_card.c` - 工业运动控制卡
- `main_gcode_test.c` - G代码解析测试
- `main_advanced_motion_test.c` - 高级运动控制测试

### 代码结构

```
Core/Src/
├── main.c                              # 主程序
├── motion_control.c                    # 基础运动控制
├── advanced_interpolation.c            # 高级插补算法
├── industrial_motion_controller.c      # 工业运动控制器
├── fault_diagnosis.c                   # 故障诊断
├── gcode_parser.c                      # G代码解析器
└── ...
```

## 常见问题

### 1. 编译错误

**问题**: 找不到ARM GCC工具链
**解决**: 确保ARM GCC已正确安装并添加到PATH

### 2. 烧录失败

**问题**: 无法连接到目标设备
**解决**: 
- 检查ST-Link连接
- 确认开发板电源
- 检查调试接口

### 3. 电机不转

**问题**: 步进电机无响应
**解决**:
- 检查电机连接线
- 确认驱动器电源
- 验证使能信号

## 开发指南

### 添加新功能

1. 在`Core/Src/`目录创建源文件
2. 在`Core/Inc/`目录创建头文件
3. 修改Makefile添加新文件
4. 编译测试

### 调试技巧

```c
// 使用printf输出调试信息
printf("Motor status: %d\n", motor_status);

// 使用LED指示状态
HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
```

### 性能优化

- 使用DMA传输提高效率
- 合理配置定时器频率
- 优化中断处理函数

## 进阶功能

### 1. 多轴控制

配置多个电机轴：

```c
// 配置X、Y、Z轴
StepMotor_ConfigAxis(AXIS_X, &x_config);
StepMotor_ConfigAxis(AXIS_Y, &y_config);
StepMotor_ConfigAxis(AXIS_Z, &z_config);
```

### 2. 插补运动

```c
// 直线插补
LinearInterpolation_Start(&start_point, &end_point, feedrate);

// 圆弧插补
CircularInterpolation_Start(&start_point, &end_point, &center_point);
```

### 3. G代码执行

```c
// 解析G代码
GCode_ParseLine("G01 X10 Y20 F100", &gcode_line);

// 执行运动
GCode_ExecuteLine(&gcode_line);
```

## 技术支持

如需更详细的信息，请参考：

- [技术架构文档](TECHNICAL_ARCHITECTURE.md)
- [项目开发历史](PROJECT_DEVELOPMENT_HISTORY.md)
- [相关模块README文档](README_StepMotor.md)

## 更新日志

- v2.0.0 - 完整的工业运动控制功能
- v1.5.0 - 增加G代码解析和高级插补
- v1.0.0 - 基础步进电机控制 