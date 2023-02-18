/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include <string.h>
#include <stdio.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
/* USER CODE BEGIN Variables */
extern UART_HandleTypeDef huart2;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
osThreadId_t defaultTaskHandle1;
osThreadId_t defaultTaskHandle2;

const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .attr_bits = osThreadDetached,
    .cb_mem = NULL,
    .cb_size = 0,
    .stack_mem = NULL,
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
    .tz_module = 0,
    .reserved = 0};


osMutexId_t mutex_id;  

static int sharedVariable = 10;
char var[100];
uint32_t     flagsPrintFunc;
uint32_t     flags;


/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void AddToCounter(void* argument);
void PrintToSerial(void* argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void)
{
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  mutex_id = osMutexNew(NULL);
  

  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
  
  defaultTaskHandle1 = osThreadNew(AddToCounter, NULL, &defaultTask_attributes);

  defaultTaskHandle2 = osThreadNew(PrintToSerial, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for (;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}



void AddToCounter(void* argument)
{
  for(;;)
  {
    osMutexAcquire(mutex_id, osWaitForever);
    sharedVariable += 50;
    sprintf(var, "%d\n", sharedVariable);
    HAL_UART_Transmit(&huart2, (uint8_t *)var, strlen(var), HAL_MAX_DELAY); // serial print
    if(sharedVariable >= 1000)
    {
      osMutexRelease(mutex_id);
      flags = osThreadFlagsSet(defaultTaskHandle2, 0x0001U);
      osDelay(2u);
    }
    else
    {
      osMutexRelease(mutex_id);
    }
  }
}


void PrintToSerial(void* argument)
{
  for(;;)
  {
    flagsPrintFunc = osThreadFlagsWait(0x0001U, osFlagsWaitAny, osWaitForever);
    osMutexAcquire(mutex_id, osWaitForever);
    osDelay(200);
    sharedVariable -= 900; 
    osMutexRelease(mutex_id);
  }
}






/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
