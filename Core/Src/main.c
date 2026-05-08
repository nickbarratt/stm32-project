/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>   // Required for printf
#include <string.h>  // Required for string operations
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

/* USER CODE BEGIN PV */
uint8_t k0_last_state = 1; // Assuming pull-up (1 = released)
uint8_t k1_last_state = 1;

uint32_t duty_cycle = 500;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
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
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  // This starts the hardware PWM on the specified channels
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); 
	printf("PWM System Active at 1kHz\r\n");
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	
	// Enable Clock for Port A
  __HAL_RCC_GPIOA_CLK_ENABLE();
  uint32_t lastBlinkTime = 0;
  uint8_t ledIsOn = 0;
  uint16_t dimmedLevel = 500; // Set your desired dimness here


// Clear the terminal screen and reset cursor (ANSI escape codes)
	printf("\033[2J\033[H"); 
	printf("=================================\r\n");
	printf("   JL32-F4VE System Initialised  \r\n");
	printf("   PWM Frequency: 1kHz  (PE9)    \r\n");
	printf("=================================\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  	
  	// 1. BLINK LOGIC (Non-blocking)
    if (HAL_GetTick() - lastBlinkTime >= 500) 
    {
        lastBlinkTime = HAL_GetTick();
        ledIsOn = !ledIsOn; // Flip the state

        if (ledIsOn) {
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, duty_cycle);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
        } else {
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, duty_cycle);
        }
    }
  	
  	// Check K0: Increase Duty
    uint8_t k0_current = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_4);
    if (k0_current == 0 && k0_last_state == 1) {
        if (duty_cycle <= 950) duty_cycle += 50; 
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty_cycle); // updates PE9
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, duty_cycle); // Updates Onboard LED
        printf("Duty Increased: %lu\r\n", duty_cycle);
        HAL_Delay(50); // Debounce
    }
    k0_last_state = k0_current;

    // Check K1: Decrease Duty
    uint8_t k1_current = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_3);
    if (k1_current == 0 && k1_last_state == 1) {
        if (duty_cycle >= 50) duty_cycle -= 50; 
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty_cycle); // updates PE9
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, duty_cycle); // Updates Onboard LED        
        printf("Duty Decreased: %lu\r\n", duty_cycle);
        HAL_Delay(50); // Debounce
    }
    k1_last_state = k1_current;
    
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
