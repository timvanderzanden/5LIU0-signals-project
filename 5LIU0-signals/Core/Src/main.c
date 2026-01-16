/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdio.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
typedef struct {
    float real;
    float imag;
} complex_t;


#define ADC_BUF_LEN 256
#define FFT_BUF_LEN 128
#define PI 3.14159265358979323846f
#define TOP_FFT_FREQ_AMOUNT 3

int top_bins[TOP_FFT_FREQ_AMOUNT] = {-1, -1, -1};
uint16_t adcBuf[ADC_BUF_LEN];
complex_t fft_buffer[FFT_BUF_LEN];

#define ENERGY_WINDOW_TIME_S 1
#define SAMPLING_FREQUENCY_KHZ 44.138f
#define AVERAGE_ENERGY_BUFFER_LEN 10
uint32_t interrupt_amount_for_energy_window = (uint32_t)((SAMPLING_FREQUENCY_KHZ*1000)/(ADC_BUF_LEN/2));
uint32_t interrupt_amount_for_energy_window_count = 0;
float average_energy = 0.0;
float average_energy_buffer[AVERAGE_ENERGY_BUFFER_LEN];
uint8_t average_energy_count = 0;
float current_average_voltage = 0.0f;

float current_rms = 0;

volatile uint8_t adcDataReady = 0;
volatile uint8_t halfReady    = 0;

typedef struct __attribute__((packed)) {
    uint32_t sync_word;       // 0xABCDEFFF for packet alignment
    float total_energy;
    float average_energy;
    float current_rms;
    float avg_voltage;
    float fft_real_vals[FFT_BUF_LEN/2];
    float fft_img_vals[FFT_BUF_LEN/2];
} TelemetryPacket_t;

TelemetryPacket_t txPacket;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void process_adc(uint16_t *data, uint16_t length);
float calculate_discrete_energy(uint16_t *data, uint16_t length);
float calculate_rms(uint16_t *data, uint16_t length);
float adc_to_voltage(uint16_t adc_val);
void adc_to_fft_input(uint16_t* buffer, uint32_t length);
static unsigned int bit_reverse(unsigned int x, unsigned int log2n);
void calculate_fft_frequencies(uint16_t length);
void find_top3_fft_frequencies(int* top_bins);
void calculate_current_average_voltage(uint16_t *data, uint16_t length);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float total_energy = 0;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */



  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();

  HAL_TIM_Base_Start(&htim2);  // start trigger timer

  HAL_ADC_Start_DMA(&hadc1,
                    (uint32_t*)adcBuf,
                    ADC_BUF_LEN);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (adcDataReady)
    {
        adcDataReady = 0;
        process_adc(adcBuf + ADC_BUF_LEN/2, ADC_BUF_LEN/2);

        // Fill the packet
        txPacket.sync_word = 0xABCDEFFF;
        txPacket.total_energy = total_energy;
        txPacket.average_energy = average_energy;
        txPacket.current_rms = current_rms;
        txPacket.avg_voltage = current_average_voltage;

        // Calculate and fill all FFT magnitudes
        for (int i = 0; i < FFT_BUF_LEN/2; i++) {
            txPacket.fft_real_vals[i] = fft_buffer[i].real;
            txPacket.fft_img_vals[i] = fft_buffer[i].imag;
        }

        // Send the entire struct as raw bytes
        HAL_UART_Transmit(&huart2, (uint8_t*)&txPacket, sizeof(txPacket), 10);
    }

    if (halfReady)
    {
        halfReady = 0;
        process_adc(adcBuf, ADC_BUF_LEN/2);
    }
  }
  
  /* USER CODE END 3 */
}

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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T2_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 28;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 49;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 921600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_GREEN_Pin */
  GPIO_InitStruct.Pin = LED_GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LED_GREEN_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void process_adc(uint16_t *data, uint16_t length)
{
  calculate_discrete_energy(data, length);
  current_rms = calculate_rms(data, length);
  adc_to_fft_input(data, length);
  calculate_fft_frequencies(length);
  find_top3_fft_frequencies(top_bins);
  calculate_current_average_voltage(data, length);
}

void calculate_current_average_voltage(uint16_t *data, uint16_t length)
{
  uint32_t sum = 0;
  for (int i=0; i < length; i++)
  {
    sum+=data[i];
  }
  current_average_voltage = adc_to_voltage((int)(sum/length));
}

float adc_to_voltage(uint16_t adc_val)
{
    float Vref = 3.3;
    int ADC_bits = 12;
    float V = adc_val * Vref / ( (1 << ADC_bits) - 1 );
    return V;
}

void adc_to_fft_input(uint16_t* buffer, uint32_t length)
{
    for (uint32_t i = 0; i < length; i++) {
        fft_buffer[i].real = (float)buffer[i] - 2048.0f;  // center around 0
        fft_buffer[i].imag = 0.0f;
    }
}

static unsigned int bit_reverse(unsigned int x, unsigned int log2n)
{
    unsigned int n = 0;
    for (unsigned int i = 0; i < log2n; i++) {
        n <<= 1;
        n |= (x & 1);
        x >>= 1;
    }
    return n;
}

void find_top3_fft_frequencies(int* top_bins)
{
    const int size = FFT_BUF_LEN/2;
    float magnitude[size];

    for (int i = 0; i < FFT_BUF_LEN/2; i++) {
        magnitude[i] = sqrtf(fft_buffer[i].real * fft_buffer[i].real +
                            fft_buffer[i].imag * fft_buffer[i].imag);
    }

    float top_mags[TOP_FFT_FREQ_AMOUNT] = {0.0f, 0.0f, 0.0f};

    for (int i = 1; i < size - 1; i++)
    {
        // local maxima
        if (magnitude[i] > magnitude[i-1] && magnitude[i] > magnitude[i+1])
        {
            float mag = magnitude[i];

            // insert in top 3 if larger than any
            for (int j = 0; j < TOP_FFT_FREQ_AMOUNT; j++)
            {
                if (mag > top_mags[j])
                {
                    // shift smaller down
                    for (int k = TOP_FFT_FREQ_AMOUNT - 1; k > j; k--)
                    {
                        top_mags[k] = top_mags[k-1];
                        top_bins[k] = top_bins[k-1];
                    }
                    top_mags[j] = mag;
                    top_bins[j] = i;
                    break;
                }
            }
        }
    }
}

void calculate_fft_frequencies(uint16_t length)
{
    unsigned int log2n = 0;
    for (unsigned int temp = length; temp > 1; temp >>= 1)
        log2n++;

    /* Bit-reversal */
    for (unsigned int i = 0; i < length; i++) {
        unsigned int j = bit_reverse(i, log2n);
        if (j > i) {
            complex_t tmp = fft_buffer[i];
            fft_buffer[i] = fft_buffer[j];
            fft_buffer[j] = tmp;
        }
    }

    for (unsigned int s = 1; s <= log2n; s++) {
        unsigned int m = 1 << s;      // FFT size at this stage
        unsigned int m2 = m >> 1;     // Half-size
        float theta = -2.0f * PI / m;

        float w_real_init = cosf(theta);
        float w_imag_init = sinf(theta);

        for (unsigned int k = 0; k < length; k += m) {
            float w_real = 1.0f;
            float w_imag = 0.0f;

            for (unsigned int j = 0; j < m2; j++) {
                unsigned int t = k + j + m2;
                unsigned int u = k + j;

                /* t = w * x[t] */
                float tr = w_real * fft_buffer[t].real - w_imag * fft_buffer[t].imag;
                float ti = w_real * fft_buffer[t].imag + w_imag * fft_buffer[t].real;

                /* x[t] = x[u] - t */
                fft_buffer[t].real = fft_buffer[u].real - tr;
                fft_buffer[t].imag = fft_buffer[u].imag - ti;

                /* x[u] = x[u] + t */
                fft_buffer[u].real += tr;
                fft_buffer[u].imag += ti;

                /* w = w * wm */
                float tmp = w_real;
                w_real = tmp * w_real_init - w_imag * w_imag_init;
                w_imag = tmp * w_imag_init + w_imag * w_real_init;
            }
        }
  }
} 

float calculate_discrete_energy(uint16_t *data, uint16_t length)
{
    interrupt_amount_for_energy_window_count++;
    float Ts = 1.0 / (SAMPLING_FREQUENCY_KHZ*1000);

    float energy = 0.0;

    for (int i = 0; i < length; i++) {
        float V = adc_to_voltage(data[i]);
        energy += V * V * Ts;
    }
    if (interrupt_amount_for_energy_window_count == interrupt_amount_for_energy_window)
    {
      if (average_energy_count >= AVERAGE_ENERGY_BUFFER_LEN)
      {
        float sum_energy_total = 0;
        for (int i = 0; i < AVERAGE_ENERGY_BUFFER_LEN; i++)
        {
          sum_energy_total+=average_energy_buffer[i];
        }
        average_energy = sum_energy_total/AVERAGE_ENERGY_BUFFER_LEN;
        average_energy_count = 0;
      }
      average_energy_buffer[average_energy_count] = total_energy;
      average_energy_count++;
      total_energy = 0;
      interrupt_amount_for_energy_window_count = 0;
    } 
    total_energy += energy;
    return energy;
}

float calculate_rms(uint16_t *data, uint16_t length)
{
    double sum_squares = 0.0;
    for (int i = 0; i < length; i++) {
        float V = adc_to_voltage(data[i]);
        sum_squares += V * V;
    }
    return sqrt(sum_squares / length);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        adcDataReady = 1;
    }
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
    {
        halfReady = 1;
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
