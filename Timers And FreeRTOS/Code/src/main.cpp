/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "usart.h"
#include "gpio.h"
#include <string.h>
#include <stdio.h>
#include <cmsis_gcc.h>
#include <FreeRTOS.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#define TIM2_PSC_12_Pos 12

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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
volatile uint8_t count = 0;
const int MSGBUFSIZE = 80;
char msgBuf[MSGBUFSIZE];
char number[10];


extern "C" void TIM2_IRQHandler(void)
{
  TIM2->SR = ~TIM_SR_UIF;
  snprintf(msgBuf, MSGBUFSIZE, "%s", "TIM2 interrupt\r\n");
  HAL_UART_Transmit(&huart2, (uint8_t *)msgBuf, strlen(msgBuf), HAL_MAX_DELAY); // serial print
}


void SetTim2()
{
  TIM2->PSC = 7200; 
  TIM2->ARR = 10000;
  NVIC_EnableIRQ(TIM2_IRQn);
  TIM2->DIER |= (1 << TIM_DIER_UIE_Pos);
  TIM2->CR1 |= (1 << TIM_CR1_CEN_Pos);
  TIM2->EGR |= TIM_EGR_UG;
  TIM2->SR = ~TIM_SR_UIF;
}

void SetTim2_1()
{
  const int MSGBUFSIZE = 80;
  char msgBuf[MSGBUFSIZE];

  RCC->APB1ENR = (RCC->APB1ENR & ~RCC_APB1ENR_TIM2EN_Msk) | (RCC_APB1ENR_TIM2EN << RCC_APB1ENR_TIM2EN_Pos);
  TIM2->PSC = 7200;
  TIM2->SMCR = (TIM2->SMCR & ~TIM_SMCR_ECE_Msk) | (TIM_SMCR_ECE << TIM_SMCR_ECE_Pos);
  TIM2->CR1 = (TIM2->CR1 & ~TIM_CR1_CEN_Msk) | (TIM_CR1_CEN << TIM_CR1_CEN_Pos);

  GPIOA->MODER = (GPIOA->MODER & ~GPIO_MODER_MODER12_Msk) | (0b10 << GPIO_MODER_MODER12_Pos);
  GPIOA->AFR[1] = (GPIOA->AFR[1] & ~GPIO_AFRH_AFRH4_Msk) | (0b1011 << GPIO_AFRH_AFRH4_Pos);
}

void SetWatchDog()
{
  IWDG->KR  = 0x5555u;		// Enable write to PR, RLR.
  IWDG->PR  = 0b101;	// Init prescaler.
  IWDG->RLR = 0x5FFu;	// Init reload register.
  IWDG->KR  = 0xAAAAu;		// Reload the watchdog.
  IWDG->KR  = 0xCCCCu;		// Start the watchdog.
}


int main(void)
{
   HAL_Init();
  SCB->SCR &= (uint32_t)~((uint32_t)SCB_SCR_SLEEPDEEP_Msk);
  /* USER CODE BEGIN Init */

  // enable the clocks
  RCC->AHBENR |= (1 << RCC_AHBENR_GPIOAEN_Pos);
  RCC->AHBENR |= (1 << RCC_AHBENR_GPIOBEN_Pos);
  RCC->APB1ENR |= (1 << RCC_APB1ENR_TIM2EN_Pos); // enable tim2

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();

  /* Init scheduler */
  // ES Course Comments: Uncomment the three lines below to enable FreeRTOS.
  osKernelInitialize(); /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();
  osKernelStart(); /* Start scheduler */


/* We should never get here as control is now taken by the scheduler */

  SetWatchDog();
  SetTim2();
  
  snprintf(msgBuf, MSGBUFSIZE, "%s", "IWDG\r\n");
  HAL_UART_Transmit(&huart2, (uint8_t *)msgBuf, strlen(msgBuf), HAL_MAX_DELAY);
  IWDG->KR  = 0xAAAAu;		// Reload the watchdog.

  // print  the time to the serial port
  snprintf(msgBuf, MSGBUFSIZE, "%d milliseconds have passed\n", HAL_GetTick());
  HAL_UART_Transmit(&huart2, (uint8_t *)msgBuf, strlen(msgBuf), HAL_MAX_DELAY); // serial print

  while (1)
  {
    IWDG->KR  = 0xAAAAu;
    // low power
    __WFI();
  }
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM17 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM17)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
