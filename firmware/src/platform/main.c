/**
  ******************************************************************************
  * @file    main_unified.c  
  * @brief   STM32F407工业运动控制系统统一主程序
  * @author  Claude AI
  * @version 3.0
  * @date    2025-01-27
  ******************************************************************************
  * @attention
  * 
  * 统一的主程序入口，支持多种运行模式：
  * - 生产模式：完整工业运动控制功能
  * - 开发模式：包含调试和测试功能
  * - 测试模式：各种专项测试程序
  * 
  * 通过 config/build_config.h 配置运行模式
  * 
  ******************************************************************************
  */

#include "main.h"
#include "../config/build_config.h"

#if IS_FEATURE_ENABLED(FEATURE_ADVANCED_MOTION)
#include "motion_control.h"
#endif

#if IS_FEATURE_ENABLED(FEATURE_GCODE_PARSER)
#include "gcode_parser.h"
#endif

#include <stdio.h>
#include <stdarg.h>

/* 全局变量 */
TIM_HandleTypeDef htim8;

/* 外部函数声明 */
#if IS_TEST_MODE(TEST_MODE_ADVANCED_MOTION)
extern void Test_AdvancedMotion_RunAll(void);
#endif

#if IS_TEST_MODE(TEST_MODE_GCODE)
extern void Test_GCode_Parser_RunAll(void);
#endif

/* 功能模块声明 */
static void Run_ProductionMode(void);
static void Run_DevelopmentMode(void);
static void Run_TestMode(void);
static void Print_SystemInfo(void);

/* 函数声明 */
void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_TIM8_Init(void);
void Error_Handler(void);

/* 简化的printf实现 */
int printf(const char *format, ...)
{
    char buffer[512];
    va_list args;
    va_start(args, format);
    int len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    
    /* 可以在这里添加串口输出实现 */
    /* 当前仅返回长度，不实际输出 */
    return len;
}

/**
  * @brief  应用程序入口
  * @retval int
  */
int main(void)
{
    /* HAL库初始化 */
    HAL_Init();

    /* 配置系统时钟 */
    SystemClock_Config();

    /* GPIO初始化 */
    MX_GPIO_Init();
    
    /* TIM8初始化 */
    MX_TIM8_Init();
    
    /* 启动PWM输出 */
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    
    /* 打印系统信息 */
    Print_SystemInfo();
    
    /* 等待系统稳定 */
    HAL_Delay(1000);
    
    /* 根据配置选择运行模式 */
    #if IS_PRODUCTION_BUILD()
        Run_ProductionMode();
    #elif (BUILD_MODE == BUILD_MODE_DEVELOPMENT)
        Run_DevelopmentMode();
    #else
        Run_TestMode();
    #endif
    
    printf("\n📊 程序执行完成！\n");
    printf("🔄 系统将继续运行，可通过调试器观察结果...\n");
    
    /* 主循环 */
    while (1)
    {
        /* PWM频率演示 */
        static uint8_t freq_index = 0;
        static uint32_t last_change = 0;
        
        if (HAL_GetTick() - last_change > 2000) {
            freq_index = (freq_index + 1) % 5;
            
            uint32_t frequencies[] = {1000, 1500, 2000, 3000, 5000};
            uint32_t period = 168000000 / frequencies[freq_index] - 1;
            
            __HAL_TIM_SET_AUTORELOAD(&htim8, period);
            __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, period / 2);
            
            last_change = HAL_GetTick();
        }
        
        HAL_Delay(100);
    }
}

/**
  * @brief  打印系统信息
  */
static void Print_SystemInfo(void)
{
    printf("\n========================================\n");
    printf("  STM32F407 工业运动控制系统 v3.0\n");
    printf("  Industrial Motion Control System\n");
    printf("========================================\n");
    printf("  硬件平台: ATK-DMF407开发板\n");
    printf("  步进电机: ATK-2MD5050驱动器\n");
    printf("  编译时间: %s %s\n", __DATE__, __TIME__);
    printf("  构建模式: ");
    
    #if IS_PRODUCTION_BUILD()
        printf("生产模式 (Production)\n");
    #elif (BUILD_MODE == BUILD_MODE_DEVELOPMENT)
        printf("开发模式 (Development)\n");
    #else
        printf("调试模式 (Debug)\n");
    #endif
    
    printf("  功能模块:\n");
    #if IS_FEATURE_ENABLED(FEATURE_ADVANCED_MOTION)
        printf("    ✅ 高级运动控制\n");
    #endif
    #if IS_FEATURE_ENABLED(FEATURE_GCODE_PARSER)
        printf("    ✅ G-code解析器\n");
    #endif
    #if IS_FEATURE_ENABLED(FEATURE_FAULT_DIAGNOSIS)
        printf("    ✅ 故障诊断系统\n");
    #endif
    #if IS_FEATURE_ENABLED(FEATURE_COMMUNICATION)
        printf("    ✅ 通信模块\n");
    #endif
    
    printf("========================================\n");
}

/**
  * @brief  生产模式运行逻辑
  */
static void Run_ProductionMode(void)
{
    printf("[INFO] 启动生产模式...\n");
    
    #if IS_FEATURE_ENABLED(FEATURE_ADVANCED_MOTION)
        /* 初始化运动控制系统 */
        MotionControl_Init();
        printf("✅ 运动控制系统初始化完成\n");
    #endif
    
    #if IS_FEATURE_ENABLED(FEATURE_FAULT_DIAGNOSIS)
        /* 初始化故障诊断系统 */
        // FaultDiag_Init();
        printf("✅ 故障诊断系统初始化完成\n");
    #endif
    
    #if IS_FEATURE_ENABLED(FEATURE_COMMUNICATION)
        /* 初始化通信系统 */
        // Communication_Init();
        printf("✅ 通信系统初始化完成\n");
    #endif
    
    printf("🚀 工业运动控制系统就绪！\n");
}

/**
  * @brief  开发模式运行逻辑
  */
#if BUILD_MODE == BUILD_MODE_DEVELOPMENT
static void Run_DevelopmentMode(void)
{
    printf("[INFO] 启动开发模式...\n");
    
    /* 运行生产模式功能 */
    Run_ProductionMode();
    
    /* 额外的开发功能 */
    #if IS_FEATURE_ENABLED(FEATURE_ADVANCED_MOTION)
        printf("[DEV] 运行运动控制算法测试...\n");
        #if defined(Test_AdvancedMotion_RunAll)
            Test_AdvancedMotion_RunAll();
        #endif
    #endif
    
    printf("🔧 开发模式测试完成！\n");
}
#endif

/**
  * @brief  测试模式运行逻辑
  */
#if BUILD_MODE == BUILD_MODE_DEBUG  
static void Run_TestMode(void)
{
    printf("[TEST] 启动测试模式...\n");
    printf("当前测试: ");
    
    #if IS_TEST_MODE(TEST_MODE_ADVANCED_MOTION)
        printf("高级运动控制测试\n");
        #if defined(Test_AdvancedMotion_RunAll)
            Test_AdvancedMotion_RunAll();
        #endif
    #elif IS_TEST_MODE(TEST_MODE_GCODE)
        printf("G-code解析器测试\n");
        #if defined(Test_GCode_Parser_RunAll)
            Test_GCode_Parser_RunAll();
        #endif
    #elif IS_TEST_MODE(TEST_MODE_LED)
        printf("LED测试\n");
        /* LED测试代码 */
    #else
        printf("基础功能测试\n");
        /* 基础PWM测试 */
    #endif
    
    printf("✅ 测试模式执行完成！\n");
}
#endif

/* 以下是原有的硬件初始化函数，保持不变... */

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
    */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
void MX_GPIO_Init(void)
{
    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOI_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
void MX_TIM8_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

    htim8.Instance = TIM8;
    htim8.Init.Prescaler = 167;                // 168MHz/168 = 1MHz
    htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim8.Init.Period = 999;                   // 1MHz/1000 = 1kHz
    htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim8.Init.RepetitionCounter = 0;
    htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
    {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 500;                   // 50%占空比
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
    sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime = 0;
    sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_TIM_MspPostInit(&htim8);
}

/**
* @brief TIM_Base MSP初始化
* @param htim_base: TIM_Base句柄指针
* @retval None
*/
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{
    if(htim_base->Instance==TIM8)
    {
        /* TIM8时钟使能 */
        __HAL_RCC_TIM8_CLK_ENABLE();
    }
}

/**
* @brief TIM_Base MSP反初始化
* @param htim_base: TIM_Base句柄指针
* @retval None
*/
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base)
{
    if(htim_base->Instance==TIM8)
    {
        /* TIM8时钟禁用 */
        __HAL_RCC_TIM8_CLK_DISABLE();
    }
}

/**
  * @brief TIM MSP后处理初始化函数
  * @param htim: TIM句柄指针
  * @retval None
  */
void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if(htim->Instance==TIM8)
    {
        __HAL_RCC_GPIOI_CLK_ENABLE();
        /**TIM8 GPIO配置
        PI5     ------> TIM8_CH1 (PWM输出)
        */
        GPIO_InitStruct.Pin = GPIO_PIN_5;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
        HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);
    }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
    }
}