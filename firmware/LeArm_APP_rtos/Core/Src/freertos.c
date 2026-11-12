/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ArmTask.h"
#include "ChassisTask.h"
#include "DefaultTask.h"
#include "HealthTask.h"
#include "RobotHealth.h"
#include "RobotRtos.h"
#include "RobotSafety.h"
#include "SerialTask.h"

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

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for chassisTask */
osThreadId_t chassisTaskHandle;
const osThreadAttr_t chassisTask_attributes = {
  .name = "chassisTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for serialTask */
osThreadId_t serialTaskHandle;
const osThreadAttr_t serialTask_attributes = {
  .name = "serialTask",
  .stack_size = 576 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for armTask */
osThreadId_t armTaskHandle;
const osThreadAttr_t armTask_attributes = {
  .name = "armTask",
  .stack_size = 320 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for healthTask */
osThreadId_t healthTaskHandle;
const osThreadAttr_t healthTask_attributes = {
  .name = "healthTask",
  .stack_size = 160 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for chassisCommandQueue */
osMessageQueueId_t chassisCommandQueueHandle;
const osMessageQueueAttr_t chassisCommandQueue_attributes = {
  .name = "chassisCommandQueue"
};
/* Definitions for armRequestQueue */
osMessageQueueId_t armRequestQueueHandle;
const osMessageQueueAttr_t armRequestQueue_attributes = {
  .name = "armRequestQueue"
};
/* Definitions for i2cMutexutex01 */
osMutexId_t i2cMutexutex01Handle;
const osMutexAttr_t i2cMutexutex01_attributes = {
  .name = "i2cMutexutex01"
};
/* Definitions for servoMutex */
osMutexId_t servoMutexHandle;
const osMutexAttr_t servoMutex_attributes = {
  .name = "servoMutex"
};
/* Definitions for robotEventFlags */
osEventFlagsId_t robotEventFlagsHandle;
const osEventFlagsAttr_t robotEventFlags_attributes = {
  .name = "robotEventFlags"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void chassisTask_entry(void *argument);
void serialTask_entry(void *argument);
void armTask_entry(void *argument);
void healthTask_entry(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
  (void)xTask;
  (void)pcTaskName;
  robot_health_reset_with_reason(0xE002U);
}
/* USER CODE END 4 */

/* USER CODE BEGIN 5 */
void vApplicationMallocFailedHook(void)
{
  robot_health_reset_with_reason(0xE001U);
}
/* USER CODE END 5 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of i2cMutexutex01 */
  i2cMutexutex01Handle = osMutexNew(&i2cMutexutex01_attributes);

  /* creation of servoMutex */
  servoMutexHandle = osMutexNew(&servoMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* Ready-state setup: no worker can access safety before this succeeds. */
  if (!robot_safety_init(HAL_GetTick())) {
    Error_Handler();
  }
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of chassisCommandQueue */
  chassisCommandQueueHandle = osMessageQueueNew (1, sizeof(ChassisCommand_t), &chassisCommandQueue_attributes);

  /* creation of armRequestQueue */
  armRequestQueueHandle = osMessageQueueNew (4, sizeof(ArmCommand_t), &armRequestQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of chassisTask */
  chassisTaskHandle = osThreadNew(chassisTask_entry, NULL, &chassisTask_attributes);

  /* creation of serialTask */
  serialTaskHandle = osThreadNew(serialTask_entry, NULL, &serialTask_attributes);

  /* creation of armTask */
  armTaskHandle = osThreadNew(armTask_entry, NULL, &armTask_attributes);

  /* creation of healthTask */
  healthTaskHandle = osThreadNew(healthTask_entry, NULL, &healthTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* creation of robotEventFlags */
  robotEventFlagsHandle = osEventFlagsNew(&robotEventFlags_attributes);

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
  default_task_run(argument);
  osThreadExit();
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_chassisTask_entry */
/**
* @brief Function implementing the chassisTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_chassisTask_entry */
void chassisTask_entry(void *argument)
{
  /* USER CODE BEGIN chassisTask_entry */
  chassis_task_run(argument);
  osThreadExit();
  /* USER CODE END chassisTask_entry */
}

/* USER CODE BEGIN Header_serialTask_entry */
/**
* @brief Function implementing the serialTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_serialTask_entry */
void serialTask_entry(void *argument)
{
  /* USER CODE BEGIN serialTask_entry */
  serial_task_run(argument);
  osThreadExit();
  /* USER CODE END serialTask_entry */
}

/* USER CODE BEGIN Header_armTask_entry */
/**
* @brief Function implementing the armTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_armTask_entry */
void armTask_entry(void *argument)
{
  /* USER CODE BEGIN armTask_entry */
  arm_task_run(argument);
  osThreadExit();
  /* USER CODE END armTask_entry */
}

/* USER CODE BEGIN Header_healthTask_entry */
/**
* @brief Function implementing the healthTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_healthTask_entry */
void healthTask_entry(void *argument)
{
  /* USER CODE BEGIN healthTask_entry */
  health_task_run(argument);
  osThreadExit();
  /* USER CODE END healthTask_entry */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

