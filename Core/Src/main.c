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
#include "adc.h"
#include "dma.h"
#include "eth.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
#include <stdio.h>
#include "string.h"

/* Private define ------------------------------------------------------------*/
#define ADC1_TIMEOUT 10
#define ADC_MAX_VALUE 4095.0f  // Max ADC value for 12-bit resolution
#define ADC_REFERENCE_VOLTAGE 5.0f  // Reference voltage in volts

/* Private variables ---------------------------------------------------------*/
uint8_t rx_buffer[10];
uint8_t tx_buffer[100];
const uint16_t buff_len = 10;

int8_t e_reg = 0;
int8_t u_reg = 0;
uint8_t set_val = 50;  // zmieniono na float
float actual_val = 0.0f;
float adc_voltage_tab[10] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float adc_voltage = 0.0f;

float Kp = 0.23f;  // zmieniono na float
float Ki = 0.074f;  // zmieniono na float
float Kd = 1.9f;  // zmieniono na float=

float integral = 0.0f;
float previous_error = 0.0f;
float pid_output = 0.0f;

const float ADC_TO_Temp_Coeff_A = 10;

float ADC_REG2VOLTAGE(uint32_t adc_value);
float convertADC_to_temp(float adc);
float calculatePID(float setpoint, float measured, float kp, float ki, float kd, float dt);
float calculateAverage();
void updateAdcVoltage(float adc_voltage);

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
 {
	if(huart == &huart3)
	{
		if(strncmp("Set ", (char*) rx_buffer, 4) == 0)
		{
			set_val = strtol((char*) &rx_buffer[4], 0, 10);
		}


		memset((char*)rx_buffer, "", sizeof(rx_buffer));

		HAL_UART_Receive_IT(&huart3, rx_buffer, buff_len);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim == &htim4)
    {
        HAL_ADC_Start(&hadc1);
        if (HAL_ADC_PollForConversion(&hadc1, ADC1_TIMEOUT) == HAL_OK)
        {
            adc_voltage = ADC_REG2VOLTAGE(HAL_ADC_GetValue(&hadc1));
            updateAdcVoltage(adc_voltage);

            actual_val = convertADC_to_temp(calculateAverage());
        }
        int tx_msg_len = sprintf((char*)tx_buffer, "SetPoint: %d, Measured: %.2f, Kp: %.3f, Ki: %.4f, Kd: %.3f, PID: %.2f \r\r", set_val, actual_val, Kp, Ki, Kd, pid_output);
        HAL_UART_Transmit(&huart3, tx_buffer, tx_msg_len, 100);
        memset((char*)tx_buffer, 0, sizeof(tx_buffer));
    }

    if (htim == &htim6)
    {
        HAL_ADC_Start(&hadc1);
        if (HAL_ADC_PollForConversion(&hadc1, ADC1_TIMEOUT) == HAL_OK)
        {
            adc_voltage = ADC_REG2VOLTAGE(HAL_ADC_GetValue(&hadc1));
            updateAdcVoltage(adc_voltage);

            actual_val = convertADC_to_temp(calculateAverage());
        }

        // Obliczanie wyjścia PID
        float dt = 0.01f; // Okres przerwania w sekundach (10 ms)
        pid_output = calculatePID(set_val, actual_val, Kp, Ki, Kd, dt);

        // Stosowanie wyjścia PID do PWM
        if (pid_output < 0) pid_output = 0;
        if (pid_output > 100) pid_output = 100;

        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint32_t)(pid_output * 10));
    }
}

void updateAdcVoltage(float adc_voltage) {
    // Przesunięcie elementów tablicy w prawo
    for (int j = 9; j > 0; j--) {
        adc_voltage_tab[j] = adc_voltage_tab[j - 1];
    }
    // Wstawienie nowej wartości na początek
    adc_voltage_tab[0] = adc_voltage;
}

// Funkcja obliczająca średnią
float calculateAverage() {
    float sum = 0.0;
    for (int i = 0; i < 10; i++) {
        sum += adc_voltage_tab[i];
    }
    return sum / 10; // Zwrócenie średniej
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

float ADC_REG2VOLTAGE(uint32_t adc_value) {
    return ((float) adc_value / ADC_MAX_VALUE) * ADC_REFERENCE_VOLTAGE;
}

float convertADC_to_temp(float adc)
{
    float temp = 0;
    temp = adc * 100;

    return temp;
}

float calculatePID(float setpoint, float measured, float kp, float ki, float kd, float dt) {
    if (setpoint < measured) return 0.0f;
    float error = setpoint - measured;
    integral += error * dt;
    float derivative = (error - previous_error) / dt;
    previous_error = error;
    return (kp * error) + (ki * integral) + (kd * derivative);
}


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
  MX_ETH_Init();
  MX_I2C1_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM14_Init();
  MX_ADC1_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start_IT(&htim4);
  HAL_TIM_Base_Start_IT(&htim6);

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_UART_Receive_IT(&huart3, rx_buffer, buff_len);
  HAL_ADC_Start(&hadc1);

//__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


	  if(set_val - actual_val<=-1){
		  HAL_GPIO_WritePin(GPIOB, Fun_out_Pin, 1);
	  }
	  if(set_val - actual_val>=-0.5){
		  HAL_GPIO_WritePin(GPIOB, Fun_out_Pin, 0);
	  }


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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

#ifdef  USE_FULL_ASSERT
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
