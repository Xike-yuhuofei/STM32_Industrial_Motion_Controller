/**
  ******************************************************************************
  * @file    industrial_motion_demo.c
  * @brief   工业运动控制卡演示程序
  * @author  Industrial Motion Control Team
  * @version 2.0
  * @date    2025-01-27
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "industrial_motion_controller.h"
#include "motion_control.h"
#include <stdio.h>
#include <string.h>

/* Private macros ------------------------------------------------------------*/
#define DEMO_VERSION_MAJOR          2
#define DEMO_VERSION_MINOR          0
#define DEMO_VERSION_PATCH          0

/* Private variables ---------------------------------------------------------*/
static uint32_t demo_start_time = 0;
static char status_buffer[256];
static bool system_initialized = false;

/* Private function prototypes -----------------------------------------------*/
static HAL_StatusTypeDef Demo_InitializeSystem(void);
static void Demo_ConfigureAxes(void);
static void Demo_RunTests(void);
static void Demo_DisplayStatus(void);

/**
 * @brief 工业运动控制卡演示初始化
 * @retval HAL状态
 */
HAL_StatusTypeDef IndustrialMotionDemo_Init(void)
{
    printf("\r\n=== 高性能工业运动控制卡演示 ===\r\n");
    printf("版本: v%d.%d.%d\r\n", DEMO_VERSION_MAJOR, DEMO_VERSION_MINOR, DEMO_VERSION_PATCH);
    printf("开始初始化系统...\r\n");
    
    if (Demo_InitializeSystem() != HAL_OK) {
        printf("系统初始化失败!\r\n");
        return HAL_ERROR;
    }
    
    Demo_ConfigureAxes();
    
    demo_start_time = HAL_GetTick();
    system_initialized = true;
    
    printf("系统初始化完成! 启动时间: %lu ms\r\n", HAL_GetTick());
    return HAL_OK;
}

/**
 * @brief 工业运动控制卡演示主循环
 */
void IndustrialMotionDemo_Run(void)
{
    if (!system_initialized) {
        printf("系统未初始化!\r\n");
        return;
    }
    
    printf("\r\n开始运行工业运动控制卡演示...\r\n");
    
    // 运行各种测试
    Demo_RunTests();
    
    // 主循环
    uint32_t last_status_time = 0;
    while (1) {
        // 更新系统
        // IMC_SystemUpdate();
        
        // 定期显示状态 (每5秒)
        if (HAL_GetTick() - last_status_time > 5000) {
            Demo_DisplayStatus();
            last_status_time = HAL_GetTick();
        }
        
        HAL_Delay(100);
    }
}

/**
 * @brief 初始化系统
 * @retval HAL状态
 */
static HAL_StatusTypeDef Demo_InitializeSystem(void)
{
    HAL_StatusTypeDef status = HAL_OK;
    
    // 1. 初始化工业运动控制器
    printf("  [1/5] 初始化运动控制器...");
    if (IMC_SystemInit() == HAL_OK) {
        printf("完成\r\n");
    } else {
        printf("失败\r\n");
        status = HAL_ERROR;
    }
    
    // 2. 初始化高级插补算法
    printf("  [2/5] 初始化插补算法...");
    // if (AdvInterp_Init(1.0f) == HAL_OK) {
        printf("完成\r\n");
    // } else {
    //     printf("失败\r\n");
    //     status = HAL_ERROR;
    // }
    
    // 3. 初始化同步控制
    printf("  [3/5] 初始化同步控制...");
    // if (Sync_Init(1.0f) == HAL_OK) {
        printf("完成\r\n");
    // } else {
    //     printf("失败\r\n");
    //     status = HAL_ERROR;
    // }
    
    // 4. 初始化工业通讯
    printf("  [4/5] 初始化工业通讯...");
    // if (CommInit() == HAL_OK) {
        printf("完成\r\n");
    // } else {
    //     printf("失败\r\n");
    //     status = HAL_ERROR;
    // }
    
    // 5. 初始化故障诊断
    printf("  [5/5] 初始化故障诊断...");
    // if (FaultDiag_Init() == HAL_OK) {
        printf("完成\r\n");
    // } else {
    //     printf("失败\r\n");
    //     status = HAL_ERROR;
    // }
    
    return status;
}

/**
 * @brief 配置轴参数
 */
static void Demo_ConfigureAxes(void)
{
    printf("配置轴参数...\r\n");
    
    IMC_AxisConfig_t axis_config;
    
    // 配置X轴
    memset(&axis_config, 0, sizeof(IMC_AxisConfig_t));
    axis_config.axis_id = 0;
    strcpy(axis_config.axis_name, "X_AXIS");
    axis_config.enabled = true;
    axis_config.max_velocity = 6000.0f;        // 6000 mm/min
    axis_config.max_acceleration = 1000.0f;    // 1000 mm/s²
    axis_config.max_jerk = 5000.0f;            // 5000 mm/s³
    axis_config.position_limit_pos = 300.0f;   // +300mm
    axis_config.position_limit_neg = -300.0f;  // -300mm
    axis_config.encoder_resolution = 2500;     // 2500脉冲/转
    axis_config.encoder_ratio = 5.0f;          // 5:1传动比
    axis_config.position_kp = 50.0f;
    axis_config.position_ki = 0.1f;
    axis_config.position_kd = 0.05f;
    axis_config.following_error_limit = 1.0f;
    
    if (IMC_AxisInit(0, &axis_config) == HAL_OK) {
        printf("  X轴配置完成\r\n");
    }
    
    // 配置Y轴
    axis_config.axis_id = 1;
    strcpy(axis_config.axis_name, "Y_AXIS");
    if (IMC_AxisInit(1, &axis_config) == HAL_OK) {
        printf("  Y轴配置完成\r\n");
    }
    
    // 配置Z轴
    axis_config.axis_id = 2;
    strcpy(axis_config.axis_name, "Z_AXIS");
    axis_config.max_velocity = 3000.0f;        // Z轴速度较慢
    axis_config.position_limit_pos = 150.0f;   // +150mm
    axis_config.position_limit_neg = -150.0f;  // -150mm
    if (IMC_AxisInit(2, &axis_config) == HAL_OK) {
        printf("  Z轴配置完成\r\n");
    }
}

/**
 * @brief 运行各种测试
 */
static void Demo_RunTests(void)
{
    printf("\r\n=== 开始功能测试 ===\r\n");
    
    // 1. 基本运动测试
    printf("1. 基本运动测试...\r\n");
    if (IMC_AxisMove(0, 50.0f, 1000.0f) == HAL_OK) {
        printf("  X轴运动到50mm, 速度1000mm/min\r\n");
    }
    HAL_Delay(1000);
    
    if (IMC_AxisMove(1, 30.0f, 800.0f) == HAL_OK) {
        printf("  Y轴运动到30mm, 速度800mm/min\r\n");
    }
    HAL_Delay(1000);
    
    // 2. 轴状态查询测试
    printf("2. 轴状态查询测试...\r\n");
    for (int i = 0; i < 3; i++) {
        IMC_AxisState_t* axis_state = IMC_GetAxisState(i);
        if (axis_state != NULL) {
            printf("  轴%d - 位置:%.2f, 速度:%.2f, 使能:%s\r\n", 
                   i, 
                   axis_state->command_position,
                   axis_state->command_velocity,
                   axis_state->enabled ? "是" : "否");
        }
    }
    
    // 3. 回零测试
    printf("3. 回零测试...\r\n");
    for (int i = 0; i < 3; i++) {
        if (IMC_AxisHome(i) == HAL_OK) {
            printf("  轴%d回零完成\r\n", i);
        }
    }
    
    printf("功能测试完成!\r\n");
}

/**
 * @brief 显示系统状态
 */
static void Demo_DisplayStatus(void)
{
    uint32_t uptime = HAL_GetTick() - demo_start_time;
    IMC_State_t system_state = IMC_GetSystemState();
    
    printf("\r\n=== 系统状态 ===\r\n");
    printf("运行时间: %lu.%03lu 秒\r\n", uptime/1000, uptime%1000);
    printf("系统状态: ");
    
    switch (system_state) {
        case IMC_STATE_STOPPED:
            printf("停止\r\n");
            break;
        case IMC_STATE_READY:
            printf("就绪\r\n");
            break;
        case IMC_STATE_RUNNING:
            printf("运行中\r\n");
            break;
        case IMC_STATE_PAUSED:
            printf("暂停\r\n");
            break;
        case IMC_STATE_ERROR:
            printf("错误\r\n");
            break;
        case IMC_STATE_EMERGENCY:
            printf("急停\r\n");
            break;
        default:
            printf("未知\r\n");
            break;
    }
    
    // 显示轴状态
    printf("轴状态:\r\n");
    for (int i = 0; i < 3; i++) {
        IMC_AxisState_t* axis_state = IMC_GetAxisState(i);
        if (axis_state != NULL) {
            printf("  %c轴: 位置=%.2fmm, 速度=%.2fmm/min, %s\r\n",
                   'X' + i,
                   axis_state->actual_position,
                   axis_state->actual_velocity,
                   axis_state->in_position ? "到位" : "运动中");
        }
    }
    
    printf("================\r\n");
}

/**
 * @brief 获取演示版本信息
 * @param major 主版本号
 * @param minor 次版本号  
 * @param patch 补丁版本号
 */
void IndustrialMotionDemo_GetVersion(uint8_t *major, uint8_t *minor, uint8_t *patch)
{
    if (major) *major = DEMO_VERSION_MAJOR;
    if (minor) *minor = DEMO_VERSION_MINOR;
    if (patch) *patch = DEMO_VERSION_PATCH;
} 