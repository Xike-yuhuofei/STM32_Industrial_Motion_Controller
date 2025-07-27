# ATK-DMF407 伺服电机运动演示程序

## 概述

本程序是专为ATK-DMF407开发板设计的伺服电机运动演示程序，实现了伺服电机在当前位置正负方向各运动5cm的功能，并提供了多种运动模式和测试功能。

## 功能特性

### 🎯 核心功能
- **基础运动演示**: 伺服电机从当前位置正负方向各运动5cm
- **连续往返运动**: 可设置往返次数的连续运动
- **精确定位测试**: 测试不同距离的定位精度
- **位置管理**: 设置零点、查看当前位置
- **急停功能**: 紧急停止运动保护

### 📊 技术参数
- **移动距离**: ±5cm (±50mm)
- **移动速度**: 1000mm/min (可调)
- **定位精度**: 高精度步进控制
- **控制方式**: PWM脉冲控制
- **通信接口**: UART串口 (115200波特率)

## 硬件连接

### ATK-DMF407开发板引脚定义

| 功能 | 引脚 | 说明 |
|------|------|------|
| PWM脉冲 | PI5 (TIM8_CH1) | 伺服电机脉冲信号 PUL+ |
| 方向控制 | PF14 | 伺服电机方向信号 DIR+ |
| 使能控制 | PH3 | 伺服电机使能信号 ENA |
| 串口通信 | PA9/PA10 (USART1) | 调试和控制接口 |

### 伺服电机驱动器连接

```
ATK-DMF407     →    伺服驱动器
─────────────────────────────
PI5 (TIM8_CH1) →    PUL+ (脉冲+)
GND            →    PUL- (脉冲-)
PF14           →    DIR+ (方向+)
GND            →    DIR- (方向-)
PH3            →    ENA+ (使能+)
GND            →    ENA- (使能-)
```

## 软件架构

### 文件结构

```
Core/
├── Inc/
│   ├── main.h                    # 主头文件，包含引脚定义
│   ├── servo_motion_demo.h       # 伺服运动演示头文件
│   └── motion_control.h          # 运动控制系统头文件
└── Src/
    ├── main_servo_motion_demo.c  # 主程序文件
    ├── servo_motion_demo.c       # 伺服运动演示实现
    └── motion_control.c          # 运动控制系统实现
```

### 核心函数

#### 基础运动函数
```c
void ServoMotion_Demo(void);                    // 基础运动演示
void ServoMotion_ContinuousDemo(uint8_t cycles); // 连续往返运动
void ServoMotion_PositionTest(void);            // 精确定位测试
```

#### 位置管理函数
```c
float ServoMotion_GetCurrentPosition(void);     // 获取当前位置
void ServoMotion_SetZeroPosition(void);         // 设置位置零点
void ServoMotion_EmergencyStop(void);           // 急停功能
```

## 编译和烧录

### 方法1: 使用PowerShell脚本 (推荐)

```powershell
# 在项目根目录执行
.\build_servo_motion_demo.ps1
```

### 方法2: 手动编译

```bash
# 设置工具链路径
export PATH=/path/to/arm-none-eabi-gcc/bin:$PATH

# 编译
arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard \
  -DUSE_HAL_DRIVER -DSTM32F407xx -ICore/Inc -IDrivers/STM32F4xx_HAL_Driver/Inc \
  -Os -fdata-sections -ffunction-sections -Wall \
  Core/Src/main_servo_motion_demo.c Core/Src/servo_motion_demo.c \
  Core/Src/motion_control.c [其他源文件...] \
  -TSTM32F407IGTX_FLASH.ld -Wl,--gc-sections \
  -o build/ATK_ServoMotion_Demo.elf

# 生成HEX文件
arm-none-eabi-objcopy -O ihex build/ATK_ServoMotion_Demo.elf build/ATK_ServoMotion_Demo.hex
```

### 烧录到开发板

1. 使用ST-Link或J-Link烧录器
2. 烧录生成的HEX或BIN文件到ATK-DMF407开发板
3. 复位开发板启动程序

## 使用说明

### 1. 硬件准备
- 确保ATK-DMF407开发板正常供电
- 正确连接伺服电机驱动器
- 连接串口调试线 (可选，用于查看运行状态)

### 2. 软件操作

#### 串口菜单操作
连接串口 (115200, 8, N, 1)，程序启动后会显示菜单：

```
=================== 菜单 ===================
1. 基础运动演示 (±5cm单次运动)
2. 连续往返运动 (3次往返)
3. 连续往返运动 (10次往返)
4. 精确定位测试
5. 设置位置零点
6. 查看当前位置
0. 急停
===========================================
请选择功能 (0-6):
```

#### 运动模式说明

**1. 基础运动演示**
- 从当前位置正方向移动5cm
- 返回起始位置
- 负方向移动5cm
- 最终返回起始位置

**2. 连续往返运动**
- 在当前位置±5cm范围内连续往返
- 可选择3次或10次往返

**3. 精确定位测试**
- 测试10mm、25mm、50mm、75mm、100mm距离的定位精度
- 显示目标位置、实际位置和误差

### 3. 运动参数

| 参数 | 数值 | 说明 |
|------|------|------|
| 移动距离 | ±50mm | 正负方向各5cm |
| 移动速度 | 1000mm/min | 约16.7mm/s |
| 暂停时间 | 2秒 | 每个动作间的暂停 |
| 步进精度 | 17.777步/度 | 基于3200步/转配置 |

## 故障排除

### 常见问题

**Q: 伺服电机不动**
- 检查电源连接
- 确认使能信号 (ENA) 正确
- 检查驱动器参数设置

**Q: 运动不准确**
- 检查步进参数配置
- 确认机械传动比
- 校准位置零点

**Q: 串口无输出**
- 检查串口连接
- 确认波特率设置 (115200)
- 检查printf重定向配置

### 调试技巧

1. **使用串口监控**: 通过串口实时查看运动状态
2. **分步测试**: 先测试单轴运动，再测试复合运动
3. **参数调整**: 根据实际硬件调整速度和加速度参数

## 技术支持

### 开发环境
- **MCU**: STM32F407IGT6
- **开发板**: ATK-DMF407
- **编译器**: ARM GCC
- **HAL库**: STM32F4xx HAL Driver

### 相关文档
- [ATK-DMF407用户手册](ATK-DMF407IO以及接口分配表.xlsx)
- [STM32F4xx参考手册](https://www.st.com/resource/en/reference_manual/dm00031020.pdf)
- [运动控制系统架构](TECHNICAL_ARCHITECTURE.md)

## 版本历史

- **v1.0** (2025-01-27)
  - 初始版本发布
  - 实现基础±5cm运动功能
  - 添加连续往返和精确定位测试
  - 完善串口交互界面

## 许可证

本项目遵循MIT许可证，详见 [LICENSE](LICENSE) 文件。

---

**注意**: 使用前请确保正确连接硬件，避免因接线错误导致设备损坏。