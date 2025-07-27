# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is an STM32F407IGT6-based industrial stepper motor control system with comprehensive motion control capabilities. The project provides multiple operation modes from basic stepper control to advanced industrial motion control with G-code parsing, interpolation algorithms, and fault diagnosis.

### Key Features
- **Multi-axis synchronous control** with real-time coordination
- **Advanced interpolation algorithms** (Linear, Circular, Helical, NURBS)
- **G-code parsing and execution** for CNC-like operations
- **Fault diagnosis system** with predictive maintenance
- **Industrial communication protocols** (EtherCAT, Modbus, CAN)
- **Real-time performance** with 1ms interpolation cycle

## Build Commands

### Primary Build System
Use the Makefile with different modes:

```bash
# Build different modes
make MODE=industrial    # Industrial motion control (default)
make MODE=gcode        # G-code parser test
make MODE=advanced     # Advanced motion control test  
make MODE=basic        # Basic stepper motor control
make MODE=simple       # Simple test program
make MODE=ui           # UI test with LCD/Touch
make MODE=fsmc_test    # FSMC basic test
make MODE=led_test     # LED basic test

# Clean build
make clean

# Flash firmware
make flash MODE=<mode>

# Show available modes
make modes
```

### Build Scripts
Convenient shell scripts for common builds:

```bash
# Industrial motion control (comprehensive)
./build_industrial_motion.sh

# G-code parser testing
./build_gcode_test.sh

# Advanced motion control testing  
./build_advanced_motion.sh

# FSMC testing
./build_fsmc_test.sh

# PowerShell scripts (Windows)
./build_basic.ps1
./build_servo_motion_demo.ps1
```

### Build Script Features
The `build_industrial_motion.sh` script supports advanced options:
- `-c, --clean`: Clean build directory before building
- `-f, --flash`: Automatically flash after successful build
- `-v, --verbose`: Show detailed build information
- `-l, --list`: List available build modes

### Additional Build Modes Available
The Makefile supports additional testing modes:
```bash
make MODE=ui_simple        # Simple UI mode
make MODE=lcd_diagnosis    # LCD diagnosis test
make MODE=chip_id_test     # Chip ID test
make MODE=lcd_chip_detect  # LCD chip detection
```

## Architecture Overview

### Layered Architecture
- **Application Layer**: UI interface, G-code execution, motion planning
- **Middleware Layer**: Motion control, interpolation algorithms, communication protocols
- **Driver Layer**: Motor drivers, LCD drivers, sensor drivers  
- **HAL Layer**: STM32 HAL (GPIO, Timer, FSMC, UART, I2C)

### Key Modules

#### Motion Control (`motion_control.c/h`)
- Basic stepper motor control interface
- Position, velocity, acceleration control
- Emergency stop functionality

#### Advanced Interpolation (`advanced_interpolation.c/h`)
- Linear interpolation
- Circular interpolation  
- Helical interpolation
- NURBS curve interpolation

#### Industrial Motion Controller (`industrial_motion_controller.c/h`)
- Complete industrial motion control card functionality
- Multi-axis synchronous control
- Real-time motion planning

#### G-Code Parser (`gcode_parser.c/h`, `gcode_executor.c/h`)
- G-code command parsing
- Motion command execution
- Test suite for validation

#### Fault Diagnosis (`fault_diagnosis.c/h`)
- Real-time sensor monitoring
- Predictive maintenance algorithms
- Automatic protection mechanisms

#### Communication (`industrial_communication.c/h`, `communication.c`)
- Industrial communication protocols (EtherCAT, Modbus, CAN)
- Real-time data exchange

### Hardware Configuration
- **MCU**: STM32F407IGT6
- **Development Board**: ATK-DMF407
- **Motor Driver**: ATK-2MD5050 
- **Motor Interface**: Multi-axis control support
- **Default Motor Interface**: Interface 4 (PI5: PWM/TIM8_CH1, PF14: DIR, PH3: ENA)

### Pin Configuration Details
Motor 4 Interface (default):
```c
#define MOTOR4_PUL_PIN       GPIO_PIN_5     // PI5 - PWM output (TIM8_CH1)
#define MOTOR4_PUL_PORT      GPIOI
#define MOTOR4_DIR_PIN       GPIO_PIN_14    // PF14 - Direction control
#define MOTOR4_DIR_PORT      GPIOF  
#define MOTOR4_ENA_PIN       GPIO_PIN_3     // PH3 - Enable control
#define MOTOR4_ENA_PORT      GPIOH
```

## Real-time Design

### Task Priority Structure
1. **Motion Control Task**: Highest priority (1ms cycle) - Step pulse generation, position control
2. **Interpolation Task**: High priority (2ms cycle) - Trajectory interpolation
3. **Communication Task**: Medium priority (10ms cycle) - Command processing
4. **UI Update Task**: Low priority (50ms cycle) - Interface refresh
5. **Diagnosis Task**: Lowest priority (1000ms cycle) - System health monitoring

### Performance Targets
- **Max Pulse Frequency**: 500kHz
- **Interpolation Cycle**: 1ms
- **Position Resolution**: 0.01mm
- **Speed Range**: 0.1-1000mm/s
- **Acceleration Range**: 1-10000mm/s²

## Development Guidelines

### Code Structure Conventions
- Use static memory allocation for critical data structures
- Implement modular interfaces for extensibility
- Follow interrupt priority hierarchy for real-time guarantees
- Use DMA for UART and LCD data transfers to reduce CPU load

### Common Development Tasks

#### Adding New Motion Modes
1. Create source file in `Core/Src/`
2. Add corresponding header in `Core/Inc/`
3. Update Makefile with new MODE option and source files
4. Test with appropriate build script

#### Modifying Motor Parameters
Configure in `main.h`:
```c
#define MOTOR_STEP_ANGLE     1.8f    // Step angle in degrees
#define MOTOR_MICROSTEP      16      // Microstepping setting
#define PWM_FREQ_MEDIUM      200     // Default frequency (Hz)
```

#### Testing Specific Modules
Use dedicated main files for isolated testing (stored in `backup/legacy_main_files/`):
- `main_chip_id_test.c` - Chip ID verification
- `main_fsmc_test.c` - External memory interface testing
- `main_led_test.c` - Basic LED functionality
- `main_simple.c` - Simple motor test
- `main_servo_motion_demo.c` - Servo motion demonstration
- `main_industrial_motion_card.c` - Industrial motion control
- `main_gcode_test.c` - G-code parser testing
- `main_advanced_motion_test.c` - Advanced motion control testing

Note: These files are kept in backup for reference but the build system uses `MODE` parameter to select functionality.

### Safety and Error Handling
- All critical operations include boundary checking
- Watchdog protection for system reliability
- Hierarchical error handling (Warning/Recoverable/Critical/Fatal)
- Input validation for all motion commands

## Hardware Integration

### Motor Connection (ATK-2MD5050)
- **PUL+/PUL-**: Pulse signal lines
- **DIR+/DIR-**: Direction signal lines  
- **ENA+/ENA-**: Enable signal lines

### Debug and Flash Tools
- **ST-Link**: Primary debugging interface
- **STM32CubeProgrammer**: Alternative flashing tool
- **OpenOCD**: Open source debugging option

## Memory and Performance

### Resource Constraints
- **Flash Usage**: <256KB target
- **RAM Usage**: <64KB target  
- **CPU Usage**: <70% target
- **Real-time Response**: <1ms requirement

### Optimization Techniques
- Pre-computed lookup tables for S-curve acceleration
- Fixed-point arithmetic for performance
- Function/data section optimization with garbage collection
- Fast math compilation flags enabled

## Documentation References

Key documentation files in repository:
- `TECHNICAL_ARCHITECTURE.md` - Detailed system architecture
- `QUICK_START_GUIDE.md` - Hardware setup and basic usage
- `PROJECT_DEVELOPMENT_HISTORY.md` - Development timeline
- `README_*.md` files - Module-specific documentation

## Troubleshooting

### Common Build Issues
1. **ARM toolchain not found**: Ensure `arm-none-eabi-gcc` is in PATH
2. **Missing dependencies**: Install STM32CubeCLT or required toolchain components
3. **Permission errors**: Check file permissions for build scripts

### Hardware Debug Tips
1. **Motor not responding**: Check enable signal (ENA pin) and power supply
2. **Incorrect rotation**: Verify DIR pin wiring and direction settings
3. **Irregular motion**: Check pulse frequency settings and motor driver configuration

### File Organization Notes
- Main application files are in `Core/Src/` and `Core/Inc/`
- Legacy/backup files are stored in `backup/legacy_main_files/`
- Build outputs go to `build/` directory
- Use `make clean` to clear build artifacts before switching modes

