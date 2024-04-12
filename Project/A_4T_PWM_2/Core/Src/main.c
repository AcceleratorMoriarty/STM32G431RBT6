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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd.h"
#include "string.h"
#include "stdio.h"
#include "key.h"
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
__IO uint32_t lcd_uwtick;
__IO uint32_t key_uwtick;
uint8_t lcd_str[22];

uint8_t key_val,key_up,key_down,key_old;

int f=1000;

int pwm1_count,pwm2_count;
float duty;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void lcd_proc(void);
void key_proc(void);


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
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  LCD_Init();
  LCD_Clear(Black);
  LCD_SetBackColor(Black);
  LCD_SetTextColor(White);
  
//  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  key_proc();
	  lcd_proc();
	  
	  
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
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
//	if(htim->Instance==TIM3)
//	{
//		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
//		{
//			pwm1_count = HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_1)+1;
//			duty = (float)pwm2_count/pwm1_count;
//		}
//		else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
//		{
//			pwm2_count = HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_2)+1;
//		}
//	}
	
	if(htim->Instance==TIM3)
	{
		if(htim->Channel==HAL_TIM_ACTIVE_CHANNEL_1)
		{
			pwm1_count=HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_1)+1; 
			pwm2_count=HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_2)+1;
			duty=(float)pwm2_count/pwm1_count;
			__HAL_TIM_SetCounter(htim,0);
		}
//	else if(htim->Channel==HAL_TIM_ACTIVE_CHANNEL_2)
//	{
//	
////		pwm2_count=HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_2)+1;
////		__HAL_TIM_SetCounter(htim,0);
//	}
	}
}

void lcd_proc(void)
{
	if(uwTick-lcd_uwtick<100) return;
	lcd_uwtick = uwTick;
	
	sprintf((char*)lcd_str,"        DATA   ");
	LCD_DisplayStringLine(Line3,lcd_str);
	sprintf((char*)lcd_str,"     PR39:%05dHz      ",1000000/pwm1_count);
	LCD_DisplayStringLine(Line4,lcd_str);
	sprintf((char*)lcd_str,"     duty:%4.2f      ",duty*100);
	LCD_DisplayStringLine(Line5,lcd_str);
	sprintf((char*)lcd_str,"     PR39:%05dHz      ",1000000/pwm2_count);
	LCD_DisplayStringLine(Line6,lcd_str);
	
}
void key_proc(void)
{
	if(uwTick-key_uwtick<50) return;
	key_uwtick = uwTick;
	
	key_val = key_scan();
	key_down = key_val&(key_val^key_old);
	key_up = key_val&(key_val^key_old);
	key_old = key_val;
	
//	if(key_down==1)
//	{
//		//80 000 000/80/ARR = 1000  ARR=1000
//		//80 000 000/80/500 = 2000
//		//80 000 000/80/    = 3000 
//		if(f<10000)
//		{
//			f+=1000;
//			
//		}
//		else
//		{
//			f=1000;
//		}
//		__HAL_TIM_SET_AUTORELOAD(&htim3,1.0*1000000/f);
//		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,0.1*1000000/f);
//	}
	
//	if(key_down)
//	{
//		sprintf((char*)lcd_str,"key: %d",key_down);
//	LCD_DisplayStringLine(Line1,lcd_str);
//	}
	
	
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