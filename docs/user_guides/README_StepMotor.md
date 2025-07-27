# STM32F407步进电机控制系统 - 接口4配置

这个项目实现了使用STM32F407微控制器通过PWM信号控制ATK-2MD5050步进电机驱动器的功能，使用ATK-DMF407开发板的**步进电机接口4**。

## 硬件连接

### ATK-DMF407开发板步进电机接口4引脚分配
- **PC9 (TIM3_CH4)** - PWM脉冲信号输出 → 连接到ATK-2MD5050的PUL+
- **PH2** - 方向控制GPIO → 连接到ATK-2MD5050的DIR+
- **PH3** - 使能控制GPIO → 连接到ATK-2MD5050的ENA+

### ATK-2MD5050驱动器连接
| STM32F407引脚 | 功能描述 | ATK-2MD5050接口 | 说明 |
|---------------|----------|----------------|------|
| PC9 | PWM脉冲输出 | PUL+ | 脉冲信号（正） |
| GND | 公共地 | PUL- | 脉冲信号（负） |
| PH2 | 方向控制 | DIR+ | 方向信号（正） |
| GND | 公共地 | DIR- | 方向信号（负） |
| PH3 | 使能控制 | ENA+ | 使能信号（正） |
| GND | 公共地 | ENA- | 使能信号（负） |

## 系统特性

### 步进电机参数
- **步进角度**: 1.8°/步 (200步/转)
- **细分设置**: 16细分
- **总步数**: 3200步/转 (200 × 16)

### 速度等级
- `SPEED_VERY_SLOW`: 50 Hz (非常慢)
- `SPEED_SLOW`: 100 Hz (慢)
- `SPEED_MEDIUM`: 200 Hz (中等) - 默认速度
- `SPEED_FAST`: 500 Hz (快)
- `SPEED_VERY_FAST`: 1000 Hz (非常快)

### PWM配置
- **定时器**: TIM3 Channel 4 (PC9)
- **预分频器**: 99 (84MHz ÷ 100 = 840kHz)
- **基础频率**: 840kHz
- **占空比**: 50%
- **系统时钟**: 168MHz (使用外部晶振 + PLL)

## 核心功能API

### 基础控制函数
```c
void StepMotor_Init(void);                    // 初始化步进电机系统
void StepMotor_Enable(void);                  // 使能步进电机
void StepMotor_Disable(void);                 // 禁用步进电机
void StepMotor_SetDirection(MotorDirection_t direction);  // 设置方向
void StepMotor_SetSpeed(MotorSpeed_t speed);  // 设置速度等级
void StepMotor_Start(void);                   // 开始PWM输出
void StepMotor_Stop(void);                    // 停止PWM输出
```

### 运动控制函数
```c
// 移动指定步数
void StepMotor_MoveSteps(uint32_t steps, MotorDirection_t direction);

// 移动指定角度
void StepMotor_MoveAngle(float angle, MotorDirection_t direction);

// 旋转指定角度（正数顺时针，负数逆时针）
void StepMotor_RotateByAngle(float angle);

// 角度转换为步数
uint32_t StepMotor_AngleToSteps(float angle);
```

### 使用示例

#### 基本移动控制
```c
// 初始化
StepMotor_Init();

// 设置中等速度，顺时针旋转180度
StepMotor_SetSpeed(SPEED_MEDIUM);
StepMotor_RotateByAngle(180.0f);

// 设置快速，逆时针旋转180度
StepMotor_SetSpeed(SPEED_FAST);
StepMotor_RotateByAngle(-180.0f);
```

#### 精确步数控制
```c
// 移动1600步（相当于180度）
StepMotor_MoveSteps(1600, MOTOR_DIR_CW);   // 顺时针
StepMotor_MoveSteps(1600, MOTOR_DIR_CCW);  // 逆时针
```

## 技术规格

### 硬件规格
- **微控制器**: STM32F407IGT6
- **系统频率**: 168MHz
- **定时器频率**: 84MHz (APB1)
- **PWM分辨率**: 16位
- **GPIO输出**: 推挽输出，低速

### 驱动器规格
- **驱动器型号**: ATK-2MD5050
- **输入电压**: 18-50V DC
- **输出电流**: 1.0-5.0A
- **细分**: 1-128细分可调
- **脉冲频率**: 0-200kHz

## 演示程序说明

主程序包含一个完整的演示循环：

1. **低速测试**: 50Hz频率，顺时针旋转180°
2. **中速测试**: 200Hz频率，逆时针旋转180°  
3. **高速测试**: 500Hz频率，顺时针旋转360°
4. **精确控制**: 200Hz频率，步数控制测试

## 故障排除

### 常见问题
1. **电机不转动**
   - 检查电源连接
   - 确认使能信号 (PH3) 
   - 检查驱动器电源和连接

2. **转动不平滑**
   - 调整速度等级
   - 检查细分设置
   - 确认负载是否过重

3. **方向错误**
   - 检查DIR引脚 (PH2) 连接
   - 确认驱动器方向配置

### 调试工具
- 使用万用表测量PWM信号 (PC9)
- 检查使能和方向信号电平
- 监控驱动器指示灯状态

## 电气安全

⚠️ **注意事项**:
- 步进电机驱动器工作电压较高 (18-50V)
- 确保正确的接地连接
- 断电状态下进行所有连接
- 注意驱动器散热

## 扩展功能

### 可扩展特性
- 支持多轴控制 (TIM1, TIM3, TIM4等)
- 中断驱动的步数计数
- 编码器反馈集成
- 上位机通信接口

### 性能优化
- DMA驱动的PWM生成
- 硬件定时器中断
- 实时操作系统集成
- 运动轨迹规划

---

## 开发环境

本项目完全适配现代化嵌入式开发环境：
- **Cursor编辑器** + Claude AI助手
- **ARM GCC工具链** + 标准Makefile  
- **VS Code扩展集成** (IntelliSense, Cortex-Debug)
- **多种烧录工具** (STM32CubeClt, STM32CubeProgrammer, OpenOCD)

支持文件监听和自动编译的智能开发模式，提供完整的现代化嵌入式开发体验。 