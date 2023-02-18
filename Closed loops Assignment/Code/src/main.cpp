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
#include "parallaxServo.h"
#include "pwmInput.h"
#include "timer.h"
#include "usart.h"
#include "gpio.h"
#include <string.h>
#include <stdio.h>

#define ALTERNATE_FUNCTION_MODE 0b10
#define STACK_COUNT 0x14
#define MSG_SIZE 120 

typedef struct{
  float p;
  float i;
  int angle;
}Message;

const int MSGBUFSIZE = 80;
char msgBuf[MSGBUFSIZE];

void SystemClock_Config(void);
void MX_FREERTOS_Init(void);

void UartQueue(void *argument);
void TurnServo(void *argument);

osThreadId_t UART;
osThreadId_t SERVO;


osMessageQueueId_t MsgQueue;
ParallaxServo servo(TIM2, PRESCALER, ARR_REG, MAX_ROTATION, MIN_CYCLE, MAX_CYCLE);


void SetAlternateFunctions()
{
  GPIOA->MODER = (GPIOA->MODER & ~GPIO_MODER_MODER0_Msk) | (ALTERNATE_FUNCTION_MODE << GPIO_MODER_MODER0_Pos); //PA0 - timer 2
  GPIOA->AFR[0] = (GPIOA->AFR[0] & ~GPIO_AFRL_AFRL0_Msk) | (0b01 << GPIO_AFRL_AFRL0_Pos);

  GPIOA->MODER = (GPIOA->MODER & ~GPIO_MODER_MODER6_Msk) | (ALTERNATE_FUNCTION_MODE << GPIO_MODER_MODER6_Pos); //PA6 - timer 3
  GPIOA->AFR[0] = (GPIOA->AFR[0] & ~GPIO_AFRL_AFRL6) | (GPIO_AF2_TIM3 << GPIO_AFRL_AFRL6_Pos);
}


/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART2_UART_Init();

  MsgQueue = osMessageQueueNew(STACK_COUNT, sizeof(Message) * 3, NULL);
  
  servo.InitialisePwmInput(TIM3, RCC_APB1ENR_TIM3EN);
  servo.InitialisePwmOutputLowerRegister(RCC_APB1ENR_TIM2EN);
  servo.StartServo();

  SetAlternateFunctions();

  const osThreadAttr_t thread_attr = {
    name : "TurnServo",
    .attr_bits = osThreadDetached,
    .cb_mem = NULL,
    .cb_size = 0,
    .stack_mem = NULL,
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
    .tz_module = 0,
    .reserved = 0
  };
  
  UART = osThreadNew(UartQueue, NULL, &thread_attr);
  SERVO = osThreadNew(TurnServo, NULL, &thread_attr);

  osKernelInitialize(); /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();
  osKernelStart(); /* Start scheduler */

  while (1)
  {
    
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


void TurnServo(void* arg)
{
  Message msg;
  msg.angle = 30;
  char buf[10];
  while(1)
  {
    osStatus_t status = osMessageQueueGet(MsgQueue, &msg, NULL, 0);

    servo.SetPosition(msg.angle - 180);
    sprintf(buf, "Error: %d\n", servo.GetError());
    HAL_UART_Transmit(&huart2, (uint8_t *)buf, strlen(buf), HAL_MAX_DELAY);

    if(status == osOK)
    {
      servo.SetKi(msg.i);
      servo.SetKp(msg.p);
    }
    osDelay(20);
  }
}

void UartQueue(void *arg)
{
  HAL_StatusTypeDef status;
  int index = 0;
  uint8_t character;
  Message msg;
  float p, i;
  int angle;
  osStatus_t stat;

  while (1)
  {
    USART2->ICR |= USART_ICR_ORECF;
    status = HAL_UART_Receive(&huart2, &character, sizeof(character), 0);
    if (status == HAL_OK)
    {
      if (character == ENTER)
      {
        HAL_UART_Transmit(&huart2, (uint8_t*)msgBuf, strlen(msgBuf), HAL_MAX_DELAY);
        sscanf(msgBuf, "%f %f %d", &p, &i, &angle);

        msg.p = p;
        msg.i = i;
        msg.angle = angle;

        stat = osMessageQueuePut(MsgQueue, &msg, 0U, 0U);
        bzero(msgBuf, sizeof(msgBuf));
        index = 0;
      }
      else
      {
        msgBuf[index] = character;
        index++;
      }
    }
    osDelay(1);
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
