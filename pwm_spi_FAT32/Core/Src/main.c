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
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "file_app.h"
#include "spi_sca100T.h"
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
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
FIL file;													/* 文件对象 */
uint8_t rxbuff[1]= {0};		// 用来接收串口1发�?�的数据
uint8_t rxbuff2[1]= {0};		// 用来接收串口2发�?�的数据
volatile BYTE  fie_buffer[512];
char sbuf[1024]={0};
uint32_t sum=0;
uint8_t _1s_flag=0;
volatile uint16_t fie_buffer_sum=0;

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
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_FATFS_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  //60°
  // 200HZ
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_UART_Receive_IT(&huart1,rxbuff,1);		// 重新使能串口1接收中断
  HAL_UART_Receive_IT(&huart2,(uint8_t *)rxbuff2,sizeof(rxbuff2));		// 重新使能串口2接收中断

  uint8_t flag = 0;
  uint8_t autoload = 143;
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
  An_Tran_Init();
  app_initfile();
  pwm_frequency(230);


  printf("test\r\n");

  float X=0,Y=0;
  uint8_t uart2_commd = 0X4F;
//app_FATFS_Run(file);
//app_writefile(file);
//app_readfile(file, 56);

//app_writefile(file);
//app_readfile(file, 56);
//  test_sd_code();1400
int i;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(_1s_flag == 1)
	  {
		  HAL_NVIC_DisableIRQ(USART2_IRQn);
		  _1s_flag = 0;
		  fie_buffer_sum= 0 ;
		//  app_FATFS_Run(file,sbuf);
	//	  app_FATFS_Run(file,fie_buffer);
		//  HAL_UART_Transmit(&huart2,&uart2_commd,1,100);
		//  printf("%d,",sum);
		  printf("----------\r\n");
	  }
	//  X = AngularConvert(RDAX);
//	  Y = AngularConvert(RDAY);



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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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


void pwm_frequency(uint32_t fre)
{
	/*  频率
	  Fpwm = 100M / ((arr+1)*(psc+1))(单位：Hz)
	  500hz =72M /((143+1)*(999+1))
	  arr 是计数�??
	  psc 是预分频�?????????????????
	  如：
	  3. 主频=100M
	  4. arr=100
	  5. psc=1000
	  100,000,000/100/1000=1000Hz
	  */
	//fre = 72000000/((fre+1)*(999+1));
	uint32_t  prescale =71;
	uint32_t autoload = (72000000 / fre) / (prescale + 1);
	autoload = autoload-1;
	__HAL_TIM_SET_PRESCALER(&htim1,prescale);
	__HAL_TIM_SET_AUTORELOAD(&htim1,autoload);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,autoload/2);
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  char a[5];
  /* Prevent unused argument(s) compilation warning */
	if(huart->Instance == USART1)	// 判断是由哪个串口触发的中�???????????????????
	{
		HAL_UART_Transmit(&huart1,rxbuff,1,100);	// 接收到数据马上使用串�???????????????????1发�?�出�???????????????????
		HAL_UART_Receive_IT(&huart1,rxbuff,1);		// 重新使能串口1接收中断
	}
	else if(huart->Instance == USART2)
	{
	//	HAL_UART_Transmit(&huart1,rxbuff2,3,100);
	//	sum++;

	//	printf("rx:%d -%d",fie_buffer[fie_buffer_sum],fie_buffer_sum);
		fie_buffer[fie_buffer_sum]  = rxbuff2[0];
		sprintf(a,"%d ",fie_buffer[fie_buffer_sum]);
  //      strcat(sbuf,a);

		printf("%d,%s\r\n",rxbuff2[0],a);
		fie_buffer_sum++;
	//	sprintf(cr,"%s",rxbuff2[0]);

	//	if(i == 512) i=0;
		HAL_UART_Receive_IT(&huart2,(uint8_t *)rxbuff2,sizeof(rxbuff2));		// 重新使能串口2接收中断
	}
  /* NOTE: This function should not be modified, when the callback is needed,
           the HAL_UART_RxCpltCallback could be implemented in the user file
   */
}

/* USER CODE END 4 */

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
  if (htim->Instance == TIM3) {
	  //100ms 中断1次
	  if(sum++ == 10)
	  {
		  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);
		  _1s_flag = 1;
		  sum = 0;
	  }

  }

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
