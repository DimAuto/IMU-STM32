/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "lsm6_gyro.h"
#include "gps_neoM9N.h"
#include "uart.h"
#include "ring_buffer.h"
#include "Fusion/Fusion.h"
#include "message_handler.h"
#include "ellipsoid_fit.h"



/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
osThreadId_t calcHeadingTaskHandle;
osThreadId_t readMemsTaskHandle;
osThreadId_t printOutTaskHandle;
osThreadId_t getCoorsTaskHandle;
osThreadId_t readMessageTaskHandle;
osThreadId_t sendMessageTaskHandle;
osThreadId_t gyroCalibrationTaskHandle;
osThreadId_t magnCalibrationTaskHandle;
osThreadId_t accCalibrationTaskHandle;
osThreadId_t checkforInterruptsTaskHandle;


osSemaphoreId_t binSemHandle;
const osSemaphoreAttr_t binSem_attributes = {
  .name = "binSem"
};

const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

const osThreadAttr_t printOutTask_attributes = {
  .name = "printOutTask",
  .stack_size = 128 * 10,
  .priority = (osPriority_t) osPriorityLow,
};

const osThreadAttr_t calcHeadingTask_attributes = {
  .name = "calcHeading",
  .stack_size = 128 * 8,
  .priority = (osPriority_t) osPriorityNormal,
};

const osThreadAttr_t readMemsTask_attributes = {
  .name = "readMems",
  .stack_size = 128 * 8,
  .priority = (osPriority_t) osPriorityHigh,
};

const osThreadAttr_t getCoorsTask_attributes = {
  .name = "getCoors",
  .stack_size = 128 * 8,
  .priority = (osPriority_t) osPriorityLow,
};

const osThreadAttr_t readMessageTaskHandle_attributes = {
  .name = "readMessage",
  .stack_size = 128 * 8,
  .priority = (osPriority_t) osPriorityHigh,
};

const osThreadAttr_t sendMessageTaskHandle_attributes = {
  .name = "sendMessage",
  .stack_size = 128 * 8,
  .priority = (osPriority_t) osPriorityNormal,
};

const osThreadAttr_t gyroCalibrationTaskHandle_attributes = {
  .name = "gyro_calibration",
  .stack_size = 128 * 8,
  .priority = (osPriority_t) osPriorityHigh,
};

const osThreadAttr_t magnCalibrationTaskHandle_attributes = {
  .name = "magn_calibration",
  .stack_size = 128 * 8,
  .priority = (osPriority_t) osPriorityHigh,
};

const osThreadAttr_t accCalibrationTaskHandle_attributes = {
  .name = "acc_calibration",
  .stack_size = 128 * 8,
  .priority = (osPriority_t) osPriorityHigh,
};

const osThreadAttr_t checkforInterruptTaskHandle_attributes = {
  .name = "check_for_Interrupt",
  .stack_size = 128 * 8,
  .priority = (osPriority_t) osPriorityLow,
};

/* Definitions for myMutex01 */
osMutexId_t debugUartMutex;
const osMutexAttr_t uartMutex_attributes = {
  .name = "debugUartMutex"
};

osMutexId_t i2cMutex;
const osMutexAttr_t i2cMutex_attributes = {
  .name = "i2cMutex"
};

/* Definitions for memsQueue */
osMessageQueueId_t memsQueueHandle;
const osMessageQueueAttr_t memsQueue_attributes = {
  .name = "memsQueue"
};

osMessageQueueId_t outputQueueHandle;
const osMessageQueueAttr_t outputQueue_attributes = {
  .name = "outputQueue"
};

osMessageQueueId_t messageQueueHandle;
const osMessageQueueAttr_t messageQueue_attributes = {
  .name = "messageQueue"
};

osMessageQueueId_t coorsQueueHandle;
const osMessageQueueAttr_t coorsQueue_attributes = {
  .name = "coorsQueue"
};

//osMessageQueueId_t magnVectorQueueHandle;
//const osMessageQueueAttr_t magnVectorQueue_attributes = {
//  .name = "magnVectorQueue"
//};


// Ack receive event flag
osEventFlagsId_t ack_rcvd;
osEventFlagsId_t wait_for_ack;
osEventFlagsId_t magnetic_interf;
osEventFlagsId_t magnCalibStart;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);

void StartDefaultTask(void *argument);
void calcHeadingTask(void *argument);
void readMemsTask(void *argument);
void printOutTask(void *argument);
void getCoorsTask(void *argument);
void readMessageTask(void *argument);
void sendMessageTask(void *argument);
void gyroCalibrationTask(void *argument);
void magnCalibrationTask(void *argument);
void accCalibrationTask(void *argument);
void checkForInterrupt(void *argument);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();



  MX_USART1_UART_Init();

  MX_UART4_Init();

  FusionInit();


  if (lsm6_bus_init() != 0){
	  uint8_t Test[] = "Failed to init I2C bus\r\n";
	  uart_write_debug(Test, 10);
  }
  else{
		  if (lsm6_acc_init() != HAL_OK){
			  uint8_t Test[] = "Failed to init LSM6 acc\r\n";
			  uart_write_debug(Test, 10);
		  }
		  if (gyro_init() != HAL_OK){
			  uint8_t Test[] = "Failed to init LSM6 gyro\r\n";
			  uart_write_debug(Test, 10);
		  }
		  if (magn_init() != HAL_OK){
			  uint8_t Test[] = "Failed to init LIS3 magn\r\n";
			  uart_write_debug(Test, 10);
		  }
  }
  if (ublox_i2c_bus_init() != HAL_OK){
	  uart_write_debug("Failed to Initialize ublox bus\r\n", 10);
  }
  else{
	  UBLOX_transResult res;
	  res = ubloxInit();
	  if (res != UBX_ACK){
		  uart_write_debug("Failed to Initialize UBLOX\r\n", 10);
	  }
	  else{
		  uart_write_debug("Ublox Initialized!\r\n", 10);
	  }
  }

  /* Init scheduler */
  osKernelInitialize();
  /* USER CODE BEGIN RTOS_MUTEX */
  debugUartMutex = osMutexNew(&uartMutex_attributes);
  i2cMutex = osMutexNew(&i2cMutex_attributes);
  /* USER CODE END RTOS_MUTEX */
  memsQueueHandle = osMessageQueueNew (8, sizeof(mems_data_t), &memsQueue_attributes);
  outputQueueHandle = osMessageQueueNew (4, sizeof(FusionEuler), &outputQueue_attributes);
  messageQueueHandle = osMessageQueueNew (8, RB_SIZE, &messageQueue_attributes);
  coorsQueueHandle = osMessageQueueNew (8, sizeof(gps_data_t), &coorsQueue_attributes);
//  magnVectorQueueHandle = osMessageQueueNew (8, sizeof(double), &magnVectorQueue_attributes);

  /* EVENT FLAG FOR ACK RECEIVE */
  ack_rcvd = osEventFlagsNew(NULL);
  wait_for_ack = osEventFlagsNew(NULL);
  magnetic_interf = osEventFlagsNew(NULL);
  magnCalibStart = osEventFlagsNew(NULL);
  //							//

  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  readMemsTaskHandle = osThreadNew(readMemsTask, NULL, &readMemsTask_attributes);

//  calcHeadingTaskHandle = osThreadNew(calcHeadingTask, NULL, &calcHeadingTask_attributes);

  printOutTaskHandle = osThreadNew(printOutTask, NULL, &printOutTask_attributes);

//  getCoorsTaskHandle = osThreadNew(getCoorsTask, NULL, &getCoorsTask_attributes);

//  sendMessageTaskHandle = osThreadNew(sendMessageTask, NULL, &sendMessageTaskHandle_attributes);

  readMessageTaskHandle = osThreadNew(readMessageTask, NULL, &readMessageTaskHandle_attributes);

  gyroCalibrationTaskHandle = osThreadNew(gyroCalibrationTask, NULL, &gyroCalibrationTaskHandle_attributes);

  magnCalibrationTaskHandle = osThreadNew(magnCalibrationTask, NULL, &magnCalibrationTaskHandle_attributes);

  checkforInterruptsTaskHandle = osThreadNew(checkForInterrupt, NULL, &checkforInterruptTaskHandle_attributes);

  /*Suspend the gyro-calibration task*/
  osThreadSuspend(gyroCalibrationTaskHandle);
  osThreadSuspend(magnCalibrationTaskHandle);

  /* Start scheduler */
  osKernelStart();
  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  }
}


/* CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
//	uint32_t magneticCalibStart = 0;
	for(;;)
	{
		HAL_GPIO_TogglePin(GPIOB,LED2_Pin);
		uart_receive_it(UART_NYX);
		osDelay(500);
	}
	/* USER CODE END 5 */
}

void checkForInterrupt(void *argument){
	for(;;){
		//Check for enabled interrupt flags
		if (osEventFlagsWait(magnCalibStart, 0x00000001U, osFlagsWaitAny, 10) == 1){
			osThreadResume(magnCalibrationTaskHandle);
			osEventFlagsSet(magnCalibStart, 0x00000000U);
		}
		osDelay(1000);
	}
}

void calcHeadingTask(void *argument)
{
	mems_data_t mems_data;
	FusionEuler euler;
	osStatus_t status;
	FusionInit();

	for(;;)
	{
		status = osMessageQueueGet(memsQueueHandle, &mems_data, NULL, 5U);   // wait for message
	    if (status == osOK) {
	    	FusionCalcAngle(&mems_data, &euler);
	    	osMessageQueuePut(outputQueueHandle, &euler, 0U, 5U);
	    }
		osDelay(30);
	}
}

void readMemsTask(void *argument)
{
	mems_data_t mems_data;
	FusionEuler euler;
	double magn_vector;
	for(;;)
	{
		tick_gyro(&mems_data);
		FusionCalcHeading(&mems_data, &euler);
		osMessageQueuePut(outputQueueHandle, &euler, 0U, 0U);
		osDelay(MEMS_SR);
	}
}


void printOutTask(void *argument)
{
	mems_data_t mems_data;
	FusionEuler euler;
	int trans_field_flag;
	uint8_t text[50] = "";
	osStatus_t status, status2;
	uint32_t magnetic_rej_flag = 0;

	for(;;)
	{
		status = osMessageQueueGet(outputQueueHandle, &euler, NULL, 5U);   // wait for message
		if (status == osOK) {
			magnetic_rej_flag = osEventFlagsWait(magnetic_interf, 0x00000001U, osFlagsWaitAny, 10);
			trans_field_flag = get_magn_transient_field();
			if (magnetic_rej_flag == 1){
				if (trans_field_flag != 0){
					sprintf(text, "HEAD: %f * #\r\n", euler.angle.yaw);
				}
				else{
					sprintf(text, "HEAD: %f *\r\n", euler.angle.yaw);
				}
			}else{
				if (trans_field_flag != 0){
					sprintf(text, "HEAD: %f #\r\n", euler.angle.yaw);
				}
				else{
					sprintf(text, "HEAD: %f\r\n", euler.angle.yaw);
				}
			}
			uart_write_debug(text,50);
			memset(text,0,sizeof(text));
		}
		osDelay(140);
	}
}


void getCoorsTask(void *argument){
	gps_data_t data;
	for(;;)
	{
		ublox_tick();
		osMessageQueuePut(coorsQueueHandle, &data, 0U, 0U);
		osDelay(1700);
	}
}

void readMessageTask(void *argument){
	osStatus_t status;
	uint32_t ack_flag, wait_flag;
	uint8_t message_buffer[RB_SIZE] = {0};
	for(;;){
		status = osMessageQueueGet(messageQueueHandle, message_buffer, NULL, osWaitForever);   // wait for message
		if (status == osOK) {
			tick_Handler(message_buffer);
			wait_flag = osEventFlagsWait(wait_for_ack, 0x00000001U, osFlagsWaitAny, 20);
			if (wait_flag == 1){
				ack_flag = osEventFlagsWait(ack_rcvd, ACK_FLAG, osFlagsWaitAny, 150);
				if (ack_flag != 1){
					tick_Handler(message_buffer);
				}
				osEventFlagsSet(wait_for_ack, 0x00000000U);
			}
		}
		osDelay(200);
	}
}

void gyroCalibrationTask(void *argument){
	mems_data_t mems_data;
	osThreadSuspend(readMemsTaskHandle);
	osThreadSuspend(printOutTaskHandle);
	osDelay(100);
	uart_write_debug("Gyro Calibration: Hold the device still\r\n", 50);
	for(;;){
		if (gyro_offset_calculation(&mems_data) == 0){
			uart_write_debug("Gyro Calibration: Finished!\r\n", 50);
			osThreadResume(readMemsTaskHandle);
			osThreadResume(printOutTaskHandle);
			osThreadSuspend(gyroCalibrationTaskHandle);
		}
		osDelay(10);
	}
}

void magnCalibrationTask(void *argument){
	mems_data_t mems_data;
	uint8_t mag_sample_res, err_coef_res = 1;
	osThreadSuspend(readMemsTaskHandle);
	osThreadSuspend(printOutTaskHandle);
	uart_write_debug("Magnetometer Calibration: Rotate the device multiple times on each axis\r\n", 100);
	for(;;){
		if (magneto_sample(&mems_data) == 0){
			SetMagnCalibratingFlag(false);
			uart_write_debug("Magnetometer Calibration: Finished!\r\n", 50);
			osDelay(200);
			osThreadResume(readMemsTaskHandle);
			osThreadResume(printOutTaskHandle);
			osDelay(200);
			osThreadSuspend(magnCalibrationTaskHandle);
		}
		osDelay(50);
	}
}

void accCalibrationTask(void *argument){
	mems_data_t mems_data;
	osThreadSuspend(readMemsTaskHandle);
	osThreadSuspend(printOutTaskHandle);
	uart_write_debug("Accelerometer Calibration: Rotate the device multiple times on each axis\r\n", 100);
	for(;;){
		if (acc_sample(&mems_data) == 0){
			uart_write_debug("Accelerometer Calibration: Finished!\r\n", 50);
			osThreadResume(readMemsTaskHandle);
			osThreadResume(printOutTaskHandle);
			osThreadTerminate(accCalibrationTaskHandle);
		}
		osDelay(50);
	}
}

void magnCalStart(void){
	magnCalibrationTaskHandle = osThreadNew(magnCalibrationTask, NULL, &magnCalibrationTaskHandle_attributes);
//	osThreadResume(magnCalibrationTaskHandle);
}

void accCalStart(){
	accCalibrationTaskHandle = osThreadNew(accCalibrationTask, NULL, &accCalibrationTaskHandle_attributes);
}

//////////// SYSTEM CONFIG ///////////////////


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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 60;
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
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

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
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, ST25DV04K_RF_DISABLE_Pin|ISM43362_RST_Pin|ISM43362_SPI3_CSN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ARD_D10_Pin|ARD_D4_Pin|ARD_D7_Pin|SPBTLE_RF_RST_Pin
                          |ARD_D9_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ARD_D8_Pin|ISM43362_BOOT0_Pin|ISM43362_WAKEUP_Pin|LED2_Pin
                          |SPSGRF_915_SDN_Pin|ARD_D5_Pin|SPSGRF_915_SPI3_CSN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, SPBTLE_RF_SPI3_CSN_Pin|PMOD_RESET_Pin|PMOD_SPI2_SCK_Pin|STSAFE_A110_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, VL53L0X_XSHUT_Pin|LED3_WIFI__LED4_BLE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : ST25DV04K_RF_DISABLE_Pin ISM43362_RST_Pin ISM43362_SPI3_CSN_Pin */
  GPIO_InitStruct.Pin = ST25DV04K_RF_DISABLE_Pin|ISM43362_RST_Pin|ISM43362_SPI3_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_OTG_FS_OVRCR_EXTI3_Pin ST25DV04K_GPO_Pin SPSGRF_915_GPIO3_EXTI5_Pin SPBTLE_RF_IRQ_EXTI6_Pin
                           ISM43362_DRDY_EXTI1_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_OVRCR_EXTI3_Pin|ST25DV04K_GPO_Pin|SPSGRF_915_GPIO3_EXTI5_Pin|SPBTLE_RF_IRQ_EXTI6_Pin
                          |ISM43362_DRDY_EXTI1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : BUTTON_EXTI13_Pin VL53L0X_GPIO1_EXTI7_Pin LSM3MDL_DRDY_EXTI8_Pin */
  GPIO_InitStruct.Pin = BUTTON_EXTI13_Pin|VL53L0X_GPIO1_EXTI7_Pin|LSM3MDL_DRDY_EXTI8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_A5_Pin ARD_A4_Pin ARD_A3_Pin ARD_A2_Pin
                           ARD_A1_Pin ARD_A0_Pin */
  GPIO_InitStruct.Pin = ARD_A5_Pin|ARD_A4_Pin|ARD_A3_Pin|ARD_A2_Pin
                          |ARD_A1_Pin|ARD_A0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D1_Pin ARD_D0_Pin */
  GPIO_InitStruct.Pin = ARD_D1_Pin|ARD_D0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D10_Pin ARD_D4_Pin ARD_D7_Pin SPBTLE_RF_RST_Pin
                           ARD_D9_Pin */
  GPIO_InitStruct.Pin = ARD_D10_Pin|ARD_D4_Pin|ARD_D7_Pin|SPBTLE_RF_RST_Pin
                          |ARD_D9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D13_Pin ARD_D12_Pin ARD_D11_Pin */
  GPIO_InitStruct.Pin = ARD_D13_Pin|ARD_D12_Pin|ARD_D11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D3_Pin */
  GPIO_InitStruct.Pin = ARD_D3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ARD_D3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D6_Pin */
  GPIO_InitStruct.Pin = ARD_D6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(ARD_D6_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D8_Pin ISM43362_BOOT0_Pin ISM43362_WAKEUP_Pin LED2_Pin
                           SPSGRF_915_SDN_Pin ARD_D5_Pin SPSGRF_915_SPI3_CSN_Pin */
  GPIO_InitStruct.Pin = ARD_D8_Pin|ISM43362_BOOT0_Pin|ISM43362_WAKEUP_Pin|LED2_Pin
                          |SPSGRF_915_SDN_Pin|ARD_D5_Pin|SPSGRF_915_SPI3_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : DFSDM1_DATIN2_Pin DFSDM1_CKOUT_Pin */
  GPIO_InitStruct.Pin = DFSDM1_DATIN2_Pin|DFSDM1_CKOUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_DFSDM1;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : QUADSPI_CLK_Pin QUADSPI_NCS_Pin OQUADSPI_BK1_IO0_Pin QUADSPI_BK1_IO1_Pin
                           QUAD_SPI_BK1_IO2_Pin QUAD_SPI_BK1_IO3_Pin */
  GPIO_InitStruct.Pin = QUADSPI_CLK_Pin|QUADSPI_NCS_Pin|OQUADSPI_BK1_IO0_Pin|QUADSPI_BK1_IO1_Pin
                          |QUAD_SPI_BK1_IO2_Pin|QUAD_SPI_BK1_IO3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OCTOSPIM_P1;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : INTERNAL_UART3_TX_Pin INTERNAL_UART3_RX_Pin */
  GPIO_InitStruct.Pin = INTERNAL_UART3_TX_Pin|INTERNAL_UART3_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : LPS22HB_INT_DRDY_EXTI10_Pin LSM6DSL_INT1_EXTI11_Pin USB_OTG_FS_PWR_EN_Pin ARD_D2_Pin
                           HTS221_DRDY_EXTI15_Pin PMOD_IRQ_EXTI2_Pin */
  GPIO_InitStruct.Pin = LPS22HB_INT_DRDY_EXTI10_Pin|LSM6DSL_INT1_EXTI11_Pin|USB_OTG_FS_PWR_EN_Pin|ARD_D2_Pin
                          |HTS221_DRDY_EXTI15_Pin|PMOD_IRQ_EXTI2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : SPBTLE_RF_SPI3_CSN_Pin PMOD_RESET_Pin PMOD_SPI2_SCK_Pin STSAFE_A110_RESET_Pin */
  GPIO_InitStruct.Pin = SPBTLE_RF_SPI3_CSN_Pin|PMOD_RESET_Pin|PMOD_SPI2_SCK_Pin|STSAFE_A110_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : VL53L0X_XSHUT_Pin LED3_WIFI__LED4_BLE_Pin */
  GPIO_InitStruct.Pin = VL53L0X_XSHUT_Pin|LED3_WIFI__LED4_BLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OTG_FS_VBUS_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_VBUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OTG_FS_VBUS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : INTERNAL_SPI3_SCK_Pin INTERNAL_SPI3_MISO_Pin INTERNAL_SPI3_MOSI_Pin */
  GPIO_InitStruct.Pin = INTERNAL_SPI3_SCK_Pin|INTERNAL_SPI3_MISO_Pin|INTERNAL_SPI3_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PMOD_UART2_CTS_Pin PMOD_UART2_RTS_Pin PMOD_UART2_TX_Pin PMOD_UART2_RX_Pin */
  GPIO_InitStruct.Pin = PMOD_UART2_CTS_Pin|PMOD_UART2_RTS_Pin|PMOD_UART2_TX_Pin|PMOD_UART2_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

//  /*Configure GPIO pins : ARD_D15_Pin ARD_D14_Pin */
//  GPIO_InitStruct.Pin = ARD_D15_Pin|ARD_D14_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
//  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}


/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
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
	  HAL_GPIO_TogglePin(GPIOB,LED2_Pin);
	  HAL_Delay(100);
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


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == GPIO_PIN_13) // If The INT Source Is EXTI Line9 (A9 Pin)
    {
//    	magnCalStart();
    	osEventFlagsSet(magnCalibStart, 0x00000001U);
    }
}
