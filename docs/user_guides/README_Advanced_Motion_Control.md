# STM32F407 高级运动控制算法系统

## 📋 项目概述

本项目为STM32F407微控制器实现了一套完整的高级运动控制算法，适用于CNC数控、3D打印、工业自动化等精密运动控制场景。

### 🎯 核心特性

- **🔶 梯形速度规划** - 加速度受限的平滑运动控制
- **🔶 S曲线速度规划** - Jerk限制减少振动和冲击
- **🔶 DDA直线插补** - 高精度多轴协调运动
- **🔶 逐点比较法圆弧插补** - 精确圆弧轨迹生成
- **🔶 NURBS样条曲线插补** - 复杂曲面加工支持
- **🔶 实时性能基准测试** - 算法性能验证

### 🛠 硬件平台

- **开发板**: ATK-DMF407 (STM32F407IGT6)
- **步进电机驱动器**: ATK-2MD5050
- **系统时钟**: 168MHz
- **插补频率**: 1000Hz
- **内存要求**: <10KB RAM, <50KB Flash

---

## 🏗 系统架构

### 算法模块架构

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   G代码解析器   │───▶│  运动路径规划   │───▶│   插补算法引擎  │
│   (Parser)      │    │  (Path Planner) │    │ (Interpolator)  │
└─────────────────┘    └─────────────────┘    └─────────────────┘
          │                       │                       │
          ▼                       ▼                       ▼
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│    指令缓冲     │    │   速度规划器    │    │   脉冲生成器    │
│   (Command      │    │  (Velocity      │    │  (Pulse Gen)    │
│    Buffer)      │    │   Planner)      │    │                 │
└─────────────────┘    └─────────────────┘    └─────────────────┘
```

### 硬件接口

```
STM32F407              ATK-2MD5050驱动器
┌─────────────┐        ┌─────────────┐
│  PC9(TIM3)  │───────▶│  PUL+ PUL-  │  脉冲信号
│  PH2        │───────▶│  DIR+ DIR-  │  方向信号  
│  PH3        │───────▶│  ENA+ ENA-  │  使能信号
└─────────────┘        └─────────────┘
```

---

## 📖 算法详解

### 1. 梯形速度规划

梯形速度规划通过限制加速度实现平滑的速度变化，包含三个阶段：

```
速度
 ▲     
 │    ┌─────────┐ ← 目标速度
 │   ╱│         │╲
 │  ╱ │ 恒速段  │ ╲
 │ ╱  │         │  ╲
 │╱   │         │   ╲
 └────┴─────────┴────╲──▶ 时间
    加速段      减速段
```

**算法特点**:
- 加速度恒定，速度线性变化
- 适用于大多数工业应用
- 计算简单，实时性好

**应用场景**:
- 点到点移动
- 直线切削
- 快速定位

### 2. S曲线速度规划

S曲线速度规划通过限制Jerk（加速度的导数）实现更平滑的运动：

```
速度
 ▲     
 │       ┌───┐
 │     ╱╱     ╲╲
 │   ╱╱         ╲╲  
 │ ╱╱             ╲╲
 │╱╱               ╲╲
 └───────────────────╲──▶ 时间
  ①②  ③   ④   ⑤  ⑥⑦
```

**7段式S曲线**:
1. **阶段①**: Jerk上升，加速度增加
2. **阶段②**: Jerk为0，恒定加速度
3. **阶段③**: Jerk下降，加速度减少
4. **阶段④**: 恒速段
5. **阶段⑤**: Jerk下降，开始减速
6. **阶段⑥**: Jerk为0，恒定减速度
7. **阶段⑦**: Jerk上升，减速度减少

**优势**:
- 减少机械振动
- 提高加工精度
- 延长设备寿命

### 3. DDA直线插补算法

数字差分分析器(Digital Differential Analyzer)算法实现多轴协调运动：

```c
// DDA算法核心
while (current_step < total_steps) {
    for (axis = 0; axis < MAX_AXES; axis++) {
        error[axis] += delta[axis];
        if (error[axis] >= total_steps) {
            error[axis] -= total_steps;
            step_output[axis] = 1;  // 该轴输出脉冲
        }
    }
    current_step++;
}
```

**算法特点**:
- 整数运算，无浮点误差
- 多轴同步插补
- 误差均匀分布

### 4. 逐点比较法圆弧插补

基于逐点误差判断的圆弧插补算法：

```c
// 逐点比较法核心
int32_t f = x*x + y*y - R*R;  // 偏差函数

if (f >= 0) {
    // 在圆外，向圆心方向移动
    move_towards_center();
} else {
    // 在圆内，沿切线方向移动
    move_tangentially();
}
```

**特点**:
- 硬件友好，易于实现
- 误差控制精度高
- 支持任意象限圆弧

### 5. NURBS样条曲线插补

非均匀有理B样条(NURBS)实现复杂曲面插补：

```
控制点: P₀, P₁, P₂, ..., Pₙ
权重:   w₀, w₁, w₂, ..., wₙ
节点向量: U = {u₀, u₁, ..., uₘ}

NURBS曲线: C(u) = Σ(Nᵢ,ₚ(u) × wᵢ × Pᵢ) / Σ(Nᵢ,ₚ(u) × wᵢ)
```

**优势**:
- 精确表示复杂曲线
- 局部修改特性
- 工业标准格式

---

## 📁 文件结构

```
ATK_StepMotor/
├── Core/
│   ├── Inc/
│   │   └── motion_control.h              # 运动控制头文件(v2.0)
│   └── Src/
│       ├── advanced_motion_control.c     # 高级算法实现
│       ├── test_advanced_motion.c        # 算法测试套件
│       ├── main_advanced_motion_test.c   # 测试主程序
│       ├── motion_control.c              # 基础运动控制
│       ├── gcode_parser.c                # G代码解析器
│       └── gcode_executor.c              # G代码执行器
├── build_advanced_motion.sh              # 自动化编译脚本
├── Makefile                              # 项目构建配置
└── README_Advanced_Motion_Control.md     # 本文档
```

---

## 🚀 快速开始

### 1. 环境准备

**必需工具**:
```bash
# macOS
brew install arm-none-eabi-gcc
brew install stlink

# Ubuntu/Debian
sudo apt-get install gcc-arm-none-eabi
sudo apt-get install stlink-tools

# Windows
# 下载安装 GNU Arm Embedded Toolchain
# 下载安装 STM32CubeProgrammer
```

**可选工具**:
- STM32CubeProgrammer (官方烧录工具)
- OpenOCD (开源调试工具)
- STM32CubeIDE (开发环境)

### 2. 编译和测试

```bash
# 查看项目信息
./build_advanced_motion.sh info

# 完整测试流程（推荐）
./build_advanced_motion.sh test

# 仅编译
./build_advanced_motion.sh build

# 仅烧录
./build_advanced_motion.sh flash

# 清理构建文件
./build_advanced_motion.sh clean
```

### 3. 硬件连接

**步进电机驱动器连接**:
```
STM32F407    │  ATK-2MD5050   │  功能说明
─────────────┼────────────────┼─────────────
PC9(TIM3_CH4)│  PUL+ PUL-     │  脉冲信号
PH2          │  DIR+ DIR-     │  方向信号
PH3          │  ENA+ ENA-     │  使能信号
+5V          │  VCC           │  电源正极
GND          │  GND           │  电源地
```

**电源连接**:
- 驱动器供电: 12-48V DC
- 逻辑电源: 5V DC
- 步进电机: 根据电机规格连接

---

## 🧪 测试说明

### 测试内容

系统包含6个主要测试模块：

1. **梯形速度规划测试**
   - 标准梯形速度曲线验证
   - 三角形速度曲线测试
   - 速度连续性检查

2. **S曲线速度规划测试**
   - 7段式S曲线验证
   - 平滑性测试
   - Jerk限制验证

3. **DDA插补算法测试**
   - 初始化参数验证
   - 步进精度测试
   - 多轴协调性测试

4. **圆弧插补测试**
   - 角度插补精度验证
   - 半径误差测试
   - 顺/逆时针方向测试

5. **逐点比较法测试**
   - 算法精度验证
   - 步进输出测试
   - 圆弧路径验证

6. **样条曲线插补测试**
   - NURBS基函数测试
   - 曲线连续性验证
   - 控制点插值测试

### 性能基准

**实时性能指标**:
- 速度规划: <1ms (1000次计算)
- DDA插补: <50ms (10000步)
- 圆弧插补: <0.1ms/点
- 样条插补: <0.5ms/点

**内存使用**:
- VelocityPlan_t: ~200字节
- DDAInterpolator_t: ~100字节
- ArcInterpolator_t: ~80字节
- SplineInterpolator_t: ~2KB
- MotionBlock_t: <2KB

---

## 🔧 API参考

### 速度规划API

```c
// 梯形速度规划
void VelocityPlanning_Trapezoidal(
    VelocityPlan_t *plan,      // 速度规划结构
    float distance,            // 运动距离(mm)
    float start_vel,           // 起始速度(mm/min)
    float target_vel,          // 目标速度(mm/min)
    float end_vel,             // 结束速度(mm/min)
    float max_vel,             // 最大速度(mm/min)
    float acceleration         // 加速度(mm/s²)
);

// S曲线速度规划
void VelocityPlanning_SCurve(
    VelocityPlan_t *plan,      // 速度规划结构
    float distance,            // 运动距离(mm)
    float start_vel,           // 起始速度(mm/min)
    float target_vel,          // 目标速度(mm/min)
    float end_vel,             // 结束速度(mm/min)
    float max_vel,             // 最大速度(mm/min)
    float acceleration,        // 最大加速度(mm/s²)
    float jerk                 // 最大加加速度(mm/s³)
);

// 获取当前时刻速度
float VelocityPlanning_GetCurrentVelocity(
    const VelocityPlan_t *plan, // 速度规划结构
    float time                   // 当前时间(s)
);
```

### 插补算法API

```c
// DDA插补初始化
void DDA_Init(
    DDAInterpolator_t *dda,    // DDA插补器
    float start[MAX_AXES],     // 起始位置
    float end[MAX_AXES]        // 结束位置
);

// DDA单步插补
uint8_t DDA_Step(
    DDAInterpolator_t *dda,    // DDA插补器
    int32_t step_output[MAX_AXES] // 输出脉冲
);

// 圆弧插补初始化
void Arc_Init(
    ArcInterpolator_t *arc,    // 圆弧插补器
    float start[2],            // 起始点[X,Y]
    float end[2],              // 结束点[X,Y]
    float center[2],           // 圆心[X,Y]
    uint8_t clockwise,         // 顺时针标志
    float resolution           // 角度分辨率
);

// 样条曲线初始化
void Spline_Init(
    SplineInterpolator_t *spline, // 样条插补器
    SplinePoint_t points[],       // 控制点数组
    uint8_t point_count,          // 控制点数量
    uint8_t degree                // 样条阶数
);
```

### 运动控制API

```c
// 系统初始化
void MotionControl_Init(void);

// 轴配置
void MotionControl_ConfigAxis(
    AxisNumber_t axis,         // 轴号
    TIM_HandleTypeDef *htim,   // 定时器句柄
    uint32_t channel,          // 定时器通道
    GPIO_TypeDef *dir_port,    // 方向GPIO端口
    uint16_t dir_pin,          // 方向GPIO引脚
    GPIO_TypeDef *ena_port,    // 使能GPIO端口
    uint16_t ena_pin,          // 使能GPIO引脚
    float steps_per_mm,        // 每毫米步数
    float max_speed,           // 最大速度
    float max_accel            // 最大加速度
);

// 运动指令
void Motion_LinearMove(float pos[MAX_AXES], float feed_rate);
void Motion_ArcMove(float pos[MAX_AXES], float center[2], 
                   float feed_rate, uint8_t clockwise);
void Motion_SplineMove(SplinePoint_t points[], uint8_t point_count, 
                      float feed_rate);
```

---

## 📊 性能优化

### 算法优化策略

1. **整数运算优化**
   ```c
   // 使用整数避免浮点运算
   int32_t pos_um = (int32_t)(pos_mm * 1000);  // 转换为微米
   ```

2. **查表优化**
   ```c
   // 三角函数查表
   static const float sin_table[360] = {...};
   float sin_deg(int degree) { return sin_table[degree % 360]; }
   ```

3. **内存池管理**
   ```c
   // 预分配运动块内存池
   static MotionBlock_t motion_pool[MOTION_BUFFER_SIZE];
   ```

### 实时性优化

1. **中断优先级配置**
   ```c
   HAL_NVIC_SetPriority(TIM3_IRQn, 1, 0);  // 最高优先级
   ```

2. **DMA传输优化**
   ```c
   // 使用DMA减少CPU负载
   HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_4, pulse_buffer, buffer_size);
   ```

3. **缓存优化**
   ```c
   // 数据缓存对齐
   __attribute__((aligned(32))) static uint32_t pulse_buffer[1024];
   ```

---

## 🐛 故障排除

### 常见问题

1. **编译错误**
   ```
   错误: arm-none-eabi-gcc: command not found
   解决: 安装ARM GCC工具链
   ```

2. **烧录失败**
   ```
   错误: Could not find ST-Link
   解决: 检查ST-Link连接，安装驱动
   ```

3. **运动不平滑**
   ```
   原因: 速度规划参数不当
   解决: 调整加速度和Jerk参数
   ```

4. **插补精度问题**
   ```
   原因: 步进分辨率设置错误
   解决: 检查steps_per_mm参数
   ```

### 调试建议

1. **使用串口调试**
   ```c
   printf("当前位置: X=%.3f, Y=%.3f\n", pos[0], pos[1]);
   ```

2. **LED状态指示**
   ```c
   HAL_GPIO_WritePin(LED_PORT, LED_PIN, motion_state);
   ```

3. **逻辑分析仪**
   - 监控脉冲信号频率
   - 验证方向信号时序
   - 检查使能信号状态

---

## 🤝 贡献指南

### 开发规范

1. **代码风格**
   - 遵循HAL库命名规范
   - 使用驼峰命名法
   - 添加详细注释

2. **测试要求**
   - 所有新功能需要测试用例
   - 性能测试通过
   - 内存使用在限制范围内

3. **文档更新**
   - 更新API文档
   - 添加使用示例
   - 更新性能基准

### 提交规范

```bash
# 功能添加
git commit -m "feat: 添加NURBS样条曲线插补算法"

# Bug修复
git commit -m "fix: 修复DDA插补精度问题"

# 文档更新
git commit -m "docs: 更新API文档"

# 性能优化
git commit -m "perf: 优化S曲线计算性能"
```

---

## 📄 许可证

本项目采用MIT许可证，详情请参见[LICENSE](LICENSE)文件。

---

## 🙏 致谢

- **STMicroelectronics** - STM32F407 HAL库
- **GNU Arm Embedded Toolchain** - 编译工具链
- **OpenSTM32 Community** - 开源支持
- **ALIENTEK** - 开发板和技术支持

---

## 📞 技术支持

- **项目主页**: [GitHub Repository](https://github.com/your-repo)
- **问题反馈**: [Issues](https://github.com/your-repo/issues)
- **技术讨论**: [Discussions](https://github.com/your-repo/discussions)
- **邮箱支持**: tech-support@example.com

---

*最后更新: 2025-01-27* 