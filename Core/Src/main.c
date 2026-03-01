/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body with SSCMA AI integration
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
#include <stdio.h>
#include <string.h>
#include "Seeed_Arduino_SSCMA.h"
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
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef huart4;

/* USER CODE BEGIN PV */
// SSCMA device structure
SSCMA_t sscma;

// I2C Scanner variables
uint8_t Buffer[25] = {0};
uint8_t Space[] = " - ";
uint8_t StartMSG[] = "Starting I2C Scanning: \r\n";
uint8_t EndMSG[] = "Done! \r\n\r\n";


volatile uint32_t systick_counter = 0;

// JPEG save state
static uint32_t last_face_time = 0;
static uint8_t  jpeg_saving    = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_UART4_Init(void);
static void MX_I2C2_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  uint8_t i = 0, ret;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  SCB->VTOR = 0x08000000;

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_UART4_Init();
  MX_I2C2_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  if (HAL_I2C_IsDeviceReady(&hi2c1, (0x62 << 1), 3, 100) == HAL_OK)
  {
      HAL_UART_Transmit(&huart4, (uint8_t*)"I2C device found at 0x62\r\n", 27, 1000);
  }
  else
  {
      HAL_UART_Transmit(&huart4, (uint8_t*)"I2C device NOT found\r\n", 22, 1000);
  }

  if (SysTick->CTRL & SysTick_CTRL_ENABLE_Msk)
  {
      HAL_UART_Transmit(&huart4, (uint8_t*)"SysTick enabled\r\n", 17, 1000);
  }
  else
  {
      HAL_UART_Transmit(&huart4, (uint8_t*)"SysTick NOT enabled\r\n", 21, 1000);
  }
  char msg[50];
  sprintf(msg, "VTOR: 0x%08lX\r\n", SCB->VTOR);
  HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), 1000);



  // Welcome message
  HAL_UART_Transmit(&huart4, (uint8_t*)"\r\n========================================\r\n", 43, 1000);
  HAL_UART_Transmit(&huart4, (uint8_t*)"  STM32 + Grove AI Vision Module\r\n", 35, 1000);
  HAL_UART_Transmit(&huart4, (uint8_t*)"========================================\r\n", 43, 1000);


//  char msg[50];
//  sprintf(msg, "SysTick count: %lu\r\n", systick_counter);
//  HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), 1000);

//  __enable_irq();
//  HAL_Delay(2000);

  // Initialize SSCMA with I2C
  HAL_UART_Transmit(&huart4, (uint8_t*)"Initializing SSCMA device...\r\n", 31, 1000);

  if (SSCMA_Init_I2C(&sscma, &hi2c1, NULL, 0, I2C_ADDRESS, 2)) {
      HAL_UART_Transmit(&huart4, (uint8_t*)"[OK] SSCMA initialized!\r\n", 26, 1000);

      // Get device name
      char *name = SSCMA_GetName(&sscma, false);
      if (name) {
          char msg[100];
          snprintf(msg, sizeof(msg), "[INFO] Device: %s\r\n", name);
          HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), 1000);
      }

      // Get device ID
      char *id = SSCMA_GetID(&sscma, false);
      if (id) {
          char msg[100];
          snprintf(msg, sizeof(msg), "[INFO] ID: %s\r\n", id);
          HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), 1000);
      }
  } else {
      HAL_UART_Transmit(&huart4, (uint8_t*)"[ERROR] SSCMA initialization failed!\r\n", 39, 1000);
      HAL_UART_Transmit(&huart4, (uint8_t*)"[INFO] Check I2C connections\r\n", 31, 1000);
  }

//  char *info = SSCMA_GetInfo(&sscma, false);
//  if (info) {
//      HAL_UART_Transmit(&huart4, (uint8_t*)info, strlen(info), 1000);
//      HAL_UART_Transmit(&huart4, (uint8_t*)"\r\n", 2, 1000);
//  }
//  else{
//	  HAL_UART_Transmit(&huart4, (uint8_t*)"No model loaded into grove\r\n", 30, 1000);
//  }
//
//  HAL_UART_Transmit(&huart4, (uint8_t*)"Starting runtime...\r\n", 21, 1000);
//
//  SSCMA_Run(&sscma);
//  SSCMA_QueryModel(&sscma);


//  // After successful SSCMA_Init_I2C and GetInfo
//
//  HAL_UART_Transmit(&huart4, (uint8_t*)"Checking model...\r\n", 19, 1000);
//
//  SSCMA_GetModelInfo(&sscma);
//  HAL_UART_Transmit(&huart4, (uint8_t*)"Listing models...\r\n", 19, 1000);
//  SSCMA_Write(&sscma, "AT+HELP\r\n", 14);
//
////  SSCMA_QueryModel(&sscma);

//  HAL_UART_Transmit(&huart4, (uint8_t*)"Checking status...\r\n", 20, 1000);
//  SSCMA_QueryStatus(&sscma);
//
//
//  HAL_UART_Transmit(&huart4, (uint8_t*)"Selecting model 1...\r\n", 22, 1000);
//
//  SSCMA_SelectModel(&sscma, 1);
//  SSCMA_Invoke(&sscma, 0, false, false);
//
//  HAL_UART_Transmit(&huart4, (uint8_t*)"Running inference...\r\n", 22, 1000);
//
//  SSCMA_InvokeMulti(&sscma);
//  SSCMA_Write(&sscma, "AT+INVOKE=0,1,1\r\n", 16);

  SSCMA_SelectModel(&sscma, 1);
  HAL_Delay(300);

  HAL_UART_Transmit(&huart4, (uint8_t*)"========================================\r\n", 43, 1000);
  HAL_UART_Transmit(&huart4, (uint8_t*)"Starting main loop...\r\n\r\n", 25, 1000);




  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	    if (SSCMA_InvokeOnce(&sscma) == CMD_OK)
	    {
	        if (sscma.box_detected)
	        {//B9
	        	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_9);
	            HAL_UART_Transmit(&huart4,
	                (uint8_t*)"Face detected\r\n",
	                15, 1000);

	            last_face_time = HAL_GetTick();

	            if (!jpeg_saving)
	            {
	                if (SSCMA_SaveJPEG(&sscma) == CMD_OK)
	                {
	                    jpeg_saving = 1;
	                    HAL_UART_Transmit(&huart4,
	                        (uint8_t*)"[JPEG] Saving enabled\r\n",
	                        23, 1000);
	                }
	            }
	        }
	        else if (jpeg_saving && (HAL_GetTick() - last_face_time > 3000))
	        {
	            SSCMA_CleanActions(&sscma);
	            jpeg_saving = 0;
	            HAL_UART_Transmit(&huart4,
	                (uint8_t*)"[JPEG] Saving disabled\r\n",
	                24, 1000);
	        }
	    }

	    HAL_Delay(150);
//	  SSCMA_Fetch(&sscma, NULL);
//	  HAL_Delay(50);
    /* USER CODE END WHILE */

//    /* USER CODE BEGIN 3 */
//
//    // ========================================
//    // Run AI Inference
//    // ========================================
//	HAL_UART_Transmit(&huart4, (uint8_t*)"Invoking...\r\n", 13, 1000);
//
//
//    if (SSCMA_Invoke(&sscma, 1, false, false) == CMD_OK) {//should be 1, true, false
//
//        // Check for detected objects (bounding boxes)
//        if (sscma.boxes_count > 0) {
//            char msg[150];
//            snprintf(msg, sizeof(msg), "\r\n[AI] Detected %d object(s):\r\n", sscma.boxes_count);
//            HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), 1000);
//
//            for (int j = 0; j < sscma.boxes_count && j < 5; j++) {  // Limit to 5 objects
//                snprintf(msg, sizeof(msg),
//                        "  [%d] x=%d y=%d w=%d h=%d score=%d target=%d\r\n",
//                        j + 1,
//                        sscma.boxes[j].x,
//                        sscma.boxes[j].y,
//                        sscma.boxes[j].w,
//                        sscma.boxes[j].h,
//                        sscma.boxes[j].score,
//                        sscma.boxes[j].target);
//                HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), 1000);
//            }
//        }
//
//        // Check for classifications
//        if (sscma.classes_count > 0) {
//            char msg[100];
//            snprintf(msg, sizeof(msg),
//                    "[AI] Classification: target=%d score=%d\r\n",
//                    sscma.classes[0].target,
//                    sscma.classes[0].score);
//            HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), 1000);
//        }
//
//        // Show performance metrics
//        char perf_msg[150];
//        snprintf(perf_msg, sizeof(perf_msg),
//                "[PERF] Preprocess=%dms Inference=%dms Postprocess=%dms\r\n",
//                sscma.perf.preprocess,
//                sscma.perf.inference,
//                sscma.perf.postprocess);
//        HAL_UART_Transmit(&huart4, (uint8_t*)perf_msg, strlen(perf_msg), 1000);
//    }


//
//    // Small delay between AI inferences
//    HAL_Delay(10000); //change this back to 500 ????????????????????????????
//
//    // ========================================
//    // I2C Scanner (runs every 2 seconds)
//    // ========================================
//    HAL_UART_Transmit(&huart4, StartMSG, sizeof(StartMSG), 1000);
//
//    for(i = 1; i < 128; i++)
//    {
//        ret = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(i<<1), 3, 5);
//        if (ret != HAL_OK) {
//            // No device at this address
//            HAL_UART_Transmit(&huart4, Space, sizeof(Space), 1000);
//        }
//        else if(ret == HAL_OK) {
//            // Device found!
//            sprintf((char*)Buffer, "0x%02X", i);
//            HAL_UART_Transmit(&huart4, Buffer, strlen((char*)Buffer), 1000);
//        }
//    }
//
//    HAL_UART_Transmit(&huart4, EndMSG, sizeof(EndMSG), 1000);
//
//    // ========================================
//    // Toggle LED
//    // ========================================
//    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_9);
//
//    // Wait before next cycle
//    HAL_Delay(1500);

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

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00503D58;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00503D58;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
