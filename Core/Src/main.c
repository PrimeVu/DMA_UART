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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "math.h"
#include "stdlib.h"
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
UART_HandleTypeDef huart5;
DMA_HandleTypeDef hdma_uart5_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_UART5_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define RxData_SIZE		9
#define MainBuf_SIZE	128

uint8_t RxData[RxData_SIZE];
uint8_t MainBuf[MainBuf_SIZE];
uint8_t TxData[]={0x5A, 0xA5, 0x05, 0x82, 0x50, 0x00, 0x00, 0x00};

volatile uint32_t count_DMA=0;

uint16_t oldPos = 0;
uint16_t newPos = 0;

int isOK = 0;

int change_to_page1=0;
int tedone=0, predone=0;

int TEMP , pressure;
uint8_t highByte, lowByte;

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
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */
	
	TEMP = 926;
	pressure = 139;
	
	highByte = (uint8_t)((TEMP >>8) & 0xFF);
	lowByte = (uint8_t)((TEMP >>0) & 0xFF);
	
	TxData[6] = highByte;
	TxData[7] = lowByte;
	
	HAL_UART_Transmit(&huart5, TxData, sizeof(TxData),10);
	
	HAL_UARTEx_ReceiveToIdle_DMA(&huart5, MainBuf, MainBuf_SIZE);
	__HAL_DMA_DISABLE_IT(&hdma_uart5_rx, DMA_IT_HT);
	
//	HAL_UART_Receive_DMA(&huart5, MainBuf, MainBuf_SIZE);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		while(change_to_page1!=1)
		{
				if(TxData[4] == 0x50)
				{
					tedone=1;
					predone=0;
				}
				else if(TxData[4] == 0x51)
				{
					tedone=0;
					predone=1;
				}
				
				if(tedone==1 && predone==0)
				{
					TxData[4]=0x51;
					highByte = (uint8_t)((pressure >>8) & 0xFF);
					lowByte = (uint8_t)((pressure >>0) & 0xFF);
				
					TxData[6] = highByte;
					TxData[7] = lowByte;
				
					HAL_UART_Transmit(&huart5, TxData, sizeof(TxData),10);
				}
				else if(tedone==0 && predone==1)
				{
					TxData[4]=0x50;
					highByte = (uint8_t)((TEMP >>8) & 0xFF);
					lowByte = (uint8_t)((TEMP >>0) & 0xFF);
				
					TxData[6] = highByte;
					TxData[7] = lowByte;
				
					HAL_UART_Transmit(&huart5, TxData, sizeof(TxData),10);
				}
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

//void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
//{
//	if(huart->Instance == UART5)
//	{
////		oldPos = newPos;  // Update the last position before copying new data

////		/* If the data in large and it is about to exceed the buffer size, we have to route it to the start of the buffer
////		 * This is to maintain the circular buffer
////		 * The old data in the main buffer will be overlapped
////		 */
////		if (oldPos+Size > MainBuf_SIZE)  // If the current position + new data size is greater than the main buffer
////		{
////			uint16_t datatocopy = MainBuf_SIZE-oldPos;  // find out how much space is left in the main buffer
////			memcpy ((uint8_t *)MainBuf+oldPos, RxData, datatocopy);  // copy data in that remaining space

////			oldPos = 0;  // point to the start of the buffer
////			memcpy ((uint8_t *)MainBuf, (uint8_t *)RxData+datatocopy, (Size-datatocopy));  // copy the remaining data
////			newPos = (Size-datatocopy);  // update the position
////		}

////		/* if the current position + new data size is less than the main buffer
////		 * we will simply copy the data into the buffer and update the position
////		 */
////		else
////		{
////			memcpy ((uint8_t *)MainBuf+oldPos, RxData, Size);
////			newPos = Size+oldPos;
////		}
//		
//		for(int i = 0; i<MainBuf_SIZE; i++)
//		{
//			if(MainBuf[i]==0x5A && MainBuf[i+3]==0x83 && MainBuf[i+4]==0x11 && MainBuf[i+5]==0x01)
//			{
//				change_to_page1=1;
//			}
//			else if(MainBuf[i]==0x5A && MainBuf[i+3]==0x83 && MainBuf[i+4]==0x11 && MainBuf[i+5]==0x02)
//				change_to_page1=0;
//		}
//		
//		/* Start DMA again */
////		HAL_UARTEx_ReceiveToIdle_DMA(&huart5, RxData, RxData_SIZE);
////		__HAL_DMA_DISABLE_IT(&hdma_uart5_rx, DMA_IT_HT);
//	}
//}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
//	if(huart->Instance == UART5)
//	{
		count_DMA++;
		for(int i = 0; i<MainBuf_SIZE; i++)
		{
			if(i<=120)
			{
				if(MainBuf[i]==0x5A && MainBuf[i+3]==0x83 && MainBuf[i+4]==0x11 && MainBuf[i+5]==0x01)
					change_to_page1=1;
			}
//			else /*if(MainBuf[i]==0x5A && MainBuf[i+3]==0x83 && MainBuf[i+4]==0x11 && MainBuf[i+5]==0x02)*/
//					change_to_page1=0;
		}
//	}
	
		/* Start DMA again */
		HAL_UARTEx_ReceiveToIdle_DMA(&huart5, MainBuf, MainBuf_SIZE);
		__HAL_DMA_DISABLE_IT(&hdma_uart5_rx, DMA_IT_HT);
}

//void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
//{
//	while(change_to_page1!=1)
//		{
//			int i=0;
//			i++;
//			//HAL_UART_Transmit(&huart5, TxData, sizeof(TxData),10);
//			if(i==1000)
//			{
//				if(TxData[4] == 0x50)
//				{
//					tedone=1;
//					predone=0;
//				}
//				else if(TxData[4] == 0x51)
//				{
//					tedone=0;
//					predone=1;
//				}
//				
//				if(tedone==1 && predone==0)
//				{
//					TxData[4]=0x51;
//					highByte = (uint8_t)((pressure >>8) & 0xFF);
//					lowByte = (uint8_t)((pressure >>0) & 0xFF);
//				
//					TxData[6] = highByte;
//					TxData[7] = lowByte;
//				
//					HAL_UART_Transmit_IT(&huart5, TxData, sizeof(TxData));
//				}
//				else if(tedone==0 && predone==1)
//				{
//					TxData[4]=0x50;
//					highByte = (uint8_t)((TEMP >>8) & 0xFF);
//					lowByte = (uint8_t)((TEMP >>0) & 0xFF);
//				
//					TxData[6] = highByte;
//					TxData[7] = lowByte;
//				
//					HAL_UART_Transmit_IT(&huart5, TxData, sizeof(TxData));
//				}
//				i=0;
//			}
//		}
//}
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
