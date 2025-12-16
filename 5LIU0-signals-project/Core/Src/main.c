#include "main.h"
#include "stm32g0xx_hal.h"
#include "stm32g0xx_hal_adc.h"
#include "stm32g0xx_hal_rcc.h"
#include "stm32g0xx_hal_tim.h"
#include "stm32g0xx_hal_uart.h"

// Define ADC handle
ADC_HandleTypeDef hadc1;
TIM_HandleTypeDef htim2;

// System Clock Configuration (adjust as needed)
void SystemClock_Config(void)
{
    // Assume your clock is set to 64 MHz
    // Adjust the system clock to match your configuration
    // CubeMX should have generated this for you
}

// Initialize ADC1
void MX_ADC1_Init(void)
{
    // Enable ADC1 Clock
    __HAL_RCC_ADC1_CLK_ENABLE();

    // Configure the ADC1 parameters
    hadc1.Instance = ADC1;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc1.Init.ContinuousConvMode = ENABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;  // TIM2 trigger
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 1;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    HAL_ADC_Init(&hadc1);
}

// Initialize TIM2 for 2.5 µs interval (400 kHz)
void MX_TIM2_Init(void)
{
    // Enable TIM2 clock
    __HAL_RCC_TIM2_CLK_ENABLE();

    // Configure TIM2 as a timer to trigger ADC at 400 kHz
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 0;  // Set this according to your clock setup
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 159;  // 400kHz => (1 / (Prescaler + 1) / (Period + 1)) => Period = (System Clock / 400000) - 1
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Base_Init(&htim2);

    // Configure TIM2 trigger output (TRGO) for ADC
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim2);

    // Start TIM2
    HAL_TIM_Base_Start(&htim2);
}

// Start ADC Conversion
void Start_ADC_Conversion(void)
{
    HAL_ADC_Start(&hadc1);
}

int main(void)
{
    // HAL Initialization
    HAL_Init();
    SystemClock_Config();

    // Initialize peripherals
    MX_ADC1_Init();
    MX_TIM2_Init();

    // Start ADC Conversion
    Start_ADC_Conversion();

    // Main loop
    while (1)
    {
        if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) == HAL_OK)
        {
            uint32_t adc_value = HAL_ADC_GetValue(&hadc1);
            // Process ADC value (e.g., transmit via UART or store)
        }
    }
}

// Error handler
void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
    }
}
