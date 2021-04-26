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
#include "adc.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_hid.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define uar_printf(...)  HAL_UART_Transmit(&huart1,\
																				(uint8_t *)u_buf,\
																				sprintf((char*)u_buf,__VA_ARGS__),\
																				0xffff)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t u_buf[256];
uint32_t adc_value[4];
uint8_t MouseData[4];
extern USBD_HandleTypeDef hUsbDeviceFS;
uint16_t angle_get;										//角度值保存														
uint16_t volage_get;
//int flag=1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void set_data();
void Read_ADC();
void Date_analyse();
uint16_t ADC_Get_Average(uint8_t ch,uint8_t times);
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
	int i;
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
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
//	HAL_ADC_Start_DMA((ADC_HandleTypeDef*) &hadc1, (uint32_t*) adc_value, 4);
		HAL_ADCEx_Calibration_Start(&hadc1);	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		for(i=0;i<4;i++)
		{
			HAL_ADC_Start(&hadc1);
			HAL_ADC_PollForConversion(&hadc1,0xffff);
			adc_value[i]=HAL_ADC_GetValue(&hadc1);
			uar_printf("------ch:%d--%d-------\r\n",i,adc_value[i]);//串口打印
		}
		HAL_ADC_Stop(&hadc1);
		set_data();
		USBD_HID_SendReport(&hUsbDeviceFS,(uint8_t*)MouseData,sizeof(MouseData));
		HAL_Delay(10);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
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
  /** Initializes the CPU, AHB and APB busses clocks 
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void set_data()
{
	MouseData[0]=127-(float)adc_value[1]/4095*127;
	MouseData[1]=127-(float)adc_value[0]/4095*127;
	if(abs((int)MouseData[0])<5)
		MouseData[0]=0;
	if(abs((int)MouseData[1])<5)
		MouseData[1]=0;
//	uar_printf("------ch:%d--%d-------\r\n",MouseData[0],MouseData[1]);
//	uar_printf("------ch:%d--%d-------\r\n",adc_value[0],adc_value[1]);
	
}
//ch(1-6Chnnal),times(读取次数)
uint16_t ADC_Get_Average(uint8_t ch,uint8_t times)
{
	ADC_ChannelConfTypeDef sConfig;		//通道初始化
	uint32_t value_sum=0;	
	uint8_t i;
	switch(ch)							//选择ADC通道
	{
		case 0:sConfig.Channel = ADC_CHANNEL_0;break;	
		case 1:sConfig.Channel = ADC_CHANNEL_1;break;	
		case 2:sConfig.Channel = ADC_CHANNEL_2;break;
		case 3:sConfig.Channel = ADC_CHANNEL_3;break;
		case 4:sConfig.Channel = ADC_CHANNEL_4;break;
		case 5:sConfig.Channel = ADC_CHANNEL_5;break;
		case 6:sConfig.Channel = ADC_CHANNEL_6;break;
	}
	sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;		//采用周期239.5周期
	sConfig.Rank = 1;
	HAL_ADC_ConfigChannel(&hadc1,&sConfig);											
	for(i=0;i<times;i++)
	{
		HAL_ADC_Start(&hadc1);								//启动转换
		HAL_ADC_PollForConversion(&hadc1,30);				//等待转化结束
		value_sum += HAL_ADC_GetValue(&hadc1);				//求和					
		HAL_ADC_Stop(&hadc1);								//停止转换
	}
	return value_sum/times;									//返回平均值
}


/* USER CODE END 4 */

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
