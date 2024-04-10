/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "adc.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd.h"
#include "key.h"
#include "stdio.h"
#include "string.h"
#include "i2c_hal.h"
#include "led.h"


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
float vr37;

__IO uint32_t lcd_uwtick;
__IO uint32_t key_uwtick;


uint8_t key_val,key_up,key_down,key_old;
uint8_t lcd_str[22];
uint8_t tr_str[50];
uint8_t rx;

uint8_t str1[5]={0x11,0x22,0x33,0x44,0x55};
uint8_t str2[5]={0};

uint8_t number = 8;
uint8_t number_new;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void lcd_proc(void);
void key_proc(void);
float get_adc(void);


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
  MX_ADC2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  LCD_Init();
  LCD_Clear(Black);
  LCD_SetBackColor(Black);
  LCD_SetTextColor(White);
  
  led_disp(0x00);
  
//  LCD_DisplayStringLine(Line0,"hleoo");

	HAL_UART_Receive_IT(&huart1,&rx,1);
	
	I2CInit();		
	iic_read(&number_new,0,1);
//	iic_write(str1,0,5);
//	HAL_Delay(1);
//	iic_read(str2,0,5);
	

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		lcd_proc();
//	  	vr37=get_adc();
		key_proc();
		
	  
	  
	  
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 20;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(rx=='X')
	{
		sprintf((char*)lcd_str,"      UART TEST      ");
		LCD_DisplayStringLine(Line4,lcd_str);
	}
	else
	{
		sprintf((char*)lcd_str,"       ERROR         ");
		LCD_DisplayStringLine(Line4,lcd_str);
	}
	HAL_UART_Receive_IT(&huart1,&rx,1);
}



void key_proc(void)
{
	if(uwTick-key_uwtick<50) return;
	key_uwtick=uwTick;
	
	key_val = key_scan();
	key_down  = key_val&(key_val^key_old);
	key_up  = ~key_val&(key_val^key_old);
	key_old = key_val;
	
//	if(key_down)
//	{
//		sprintf((char*)lcd_str,"key:%d      ",key_down);
//		LCD_DisplayStringLine(Line1,lcd_str);
//	}
//	if(key_down==4)
//	{
//		
//		sprintf((char*)tr_str,"VR37:%.2fV\r\n",vr37);
//		HAL_UART_Transmit(&huart1,tr_str,strlen((char*)tr_str),50);
//	}
	
	
	if(key_down==1)
	{
		number++;
		I2CInit();
		iic_write(&number,0,1);
		HAL_Delay(1);
		iic_read(&number_new,0,1);
	}
	else if(key_down==2)
	{
		number--;
		I2CInit();
		iic_write(&number,0,1);
		HAL_Delay(1);
		iic_read(&number_new,0,1);
	}
}
void lcd_proc(void)
{
	if(uwTick-lcd_uwtick<500) return;
	lcd_uwtick=uwTick;
	
//	sprintf((char*)lcd_str,"E:%x,%x,%x,%x,%x",str1[0],str1[1],str1[2],str1[3],str1[4]);
//	LCD_DisplayStringLine(Line3,lcd_str);
//	sprintf((char*)lcd_str,"E:%x,%x,%x,%x,%x",str2[0],str2[1],str2[2],str2[3],str2[4]);
//	LCD_DisplayStringLine(Line4,lcd_str);
	
	
//	sprintf((char*)lcd_str,"        DATA        ");
//	LCD_DisplayStringLine(Line3,lcd_str);
//	vr37=get_adc();
//	sprintf((char*)lcd_str,"     VR37:%.2fV      ",vr37);
//	LCD_DisplayStringLine(Line4,lcd_str);
	
	sprintf((char*)lcd_str,"        DATA        ");
	LCD_DisplayStringLine(Line3,lcd_str);
	sprintf((char*)lcd_str,"      Number:%d    ",number);
	LCD_DisplayStringLine(Line4,lcd_str);
	
	
//	sprintf((char*)lcd_str,"      Number:%d    ",number_new);
//	LCD_DisplayStringLine(Line5,lcd_str);
}

float get_adc(void)
{
	float v;
	uint16_t adc;
	HAL_ADC_Start(&hadc2);
	adc=HAL_ADC_GetValue(&hadc2);
	v=3.3*adc/4096;
	return v;
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
