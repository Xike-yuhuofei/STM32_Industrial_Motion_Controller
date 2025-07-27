# STM32 Industrial Motion Controller

[![Build Status](https://github.com/your-org/STM32_Industrial_Motion_Controller/workflows/STM32%20Firmware%20Build%20and%20Test/badge.svg)](https://github.com/your-org/STM32_Industrial_Motion_Controller/actions)
[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)

基于STM32F407IGT6的工业级步进电机运动控制系统，提供完整的工业运动控制解决方案。

## 🚀 主要特性

- **多层架构设计**: 应用层、核心层、平台层清晰分离
- **模块化组织**: 运动控制、G代码解析、通信、诊断独立模块
- **工业级功能**: 高级插补、同步控制、故障诊断
- **现代化工程**: CMake + Python构建系统、CI/CD支持
- **丰富的接口**: 支持多种工业通信协议

## 🏗️ 项目结构

```
STM32_Industrial_Motion_Controller/
├── firmware/           # 固件代码
│   ├── src/           # 源代码 (分层组织)
│   ├── drivers/       # STM32 HAL驱动
│   └── linker/        # 链接脚本
├── tests/             # 测试框架
├── tools/             # 开发工具和脚本
├── hardware/          # 硬件相关文档
├── docs/              # 项目文档
└── config/            # 配置文件
```

## 🛠️ 快速开始

### 环境要求

- ARM GCC工具链 (10.3+)
- Python 3.8+
- Make或CMake
- ST-Link调试器

### 构建固件

```bash
# 使用统一的Python构建脚本 (推荐)
python3 tools/scripts/build.py -m debug -t industrial

# 或使用传统Make
make MODE=industrial

# 烧录固件
python3 tools/scripts/build.py -f
```

### 构建模式

| 模式 | 描述 |
|------|------|
| `industrial` | 完整的工业运动控制功能 (默认) |
| `gcode` | G代码解析器测试 |
| `advanced` | 高级运动控制测试 |
| `basic` | 基础步进电机控制 |
| `simple` | 简单测试程序 |

### 硬件配置

- **主控**: STM32F407IGT6
- **开发板**: ATK-DMF407
- **电机驱动**: ATK-2MD5050
- **默认接口**: 电机接口4 (PI5:PWM, PF14:DIR, PH3:ENA)

## 📚 文档

- [快速开始指南](docs/user_guides/QUICK_START_GUIDE.md)
- [技术架构](docs/technical/TECHNICAL_ARCHITECTURE.md)
- [API文档](docs/api/)
- [硬件说明](hardware/datasheets/)

## 🧪 测试

```bash
# 运行单元测试
make test

# 使用Python脚本运行测试
python3 tools/scripts/build.py -m test
```

## 🤝 开发指南

### 代码组织

- **应用层** (`firmware/src/app/`): 按功能模块组织
- **核心层** (`firmware/src/core/`): 系统核心逻辑  
- **平台层** (`firmware/src/platform/`): 硬件抽象

### 构建系统

支持两种构建方式:
- **Python脚本**: `tools/scripts/build.py` (现代化，推荐)
- **传统Make**: `Makefile` (兼容性)

### 添加新功能

1. 在对应的模块目录下创建源文件
2. 更新构建配置
3. 添加对应的测试用例
4. 更新文档

## 📊 性能指标

- **最大脉冲频率**: 500kHz
- **插补周期**: 1ms  
- **位置分辨率**: 0.01mm
- **速度范围**: 0.1-1000mm/s
- **加速度范围**: 1-10000mm/s²

## 📋 路线图

- [ ] 增加以太网通信支持
- [ ] 实现机器学习优化算法
- [ ] 添加视觉系统集成
- [ ] 支持更多电机类型

## 🤝 贡献

欢迎提交Issue和Pull Request。请查看[贡献指南](CONTRIBUTING.md)了解详情。

## 📄 许可证

本项目采用 [MIT许可证](LICENSE)。

## 🙏 致谢

感谢ATK团队提供的开发板和驱动器支持。

---

**联系方式**: [your-email@example.com](mailto:your-email@example.com)
**项目主页**: [https://github.com/your-org/STM32_Industrial_Motion_Controller](https://github.com/your-org/STM32_Industrial_Motion_Controller)