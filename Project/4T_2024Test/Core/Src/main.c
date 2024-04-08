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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd.h"
#include "Led.h"
#include "stdio.h"
#include "string.h"
#include "Key.h"
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
uint8_t lcd_str[22];
__IO uint32_t lcd_uwtick;

uint8_t key_val;
uint8_t key_d;
uint8_t key_u;
uint8_t key_old;
__IO uint32_t key_uwtick;
__IO uint32_t led_uwtick=0;
uint8_t rx;
char str[50];
uint8_t rx_str[20];
uint8_t rx_len=0;
uint8_t menu_flag=0;
uint8_t std_flag=0;
uint8_t led1_flag=0;
uint8_t led2_flag=0;
float R37_up=2.2,R37_down=1.2,R38_up=3.0,R38_down=1.4;

int R37_all=0;
int R38_all=0;

float R37_num=0;
float R38_num=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void LCD_Proc(void);
void key_Proc(void);
void led_Proc(void);
void menu1(void);
void menu2(void);
void menu3(void);
uint16_t get_ADC(ADC_HandleTypeDef hadc);

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
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */
	LCD_Init();
	LCD_Clear(Black);
	LCD_SetBackColor(Black);
	LCD_SetTextColor(White);
	led_disp(0x00);
	
	HAL_UART_Receive_IT(&huart1,&rx,1);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
	
	__HAL_TIM_SET_AUTORELOAD(&htim3,499);//频率
	// 50/500 = 10%占空比
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,100);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//		LCD_DisplayStringLine(Line0,(uint8_t*)"hello");
//	 	changeLed(GPIO_PIN_8,GPIO_PIN_RESET);
//		HAL_Delay(500);
//		changeLed(GPIO_PIN_8,GPIO_PIN_SET);
//		HAL_Delay(600);
		
	  
		LCD_Proc();
		key_Proc();
		led_Proc();
	  
	  
//	  //80 000 000 /(79+1)/(499+1)= 2000Hz
//		__HAL_TIM_SET_AUTORELOAD(&htim3,499);//频率
//	  // 50/500 = 10%占空比
//		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,50);
		
		
//		HAL_UART_Transmit(&huart1,(uint8_t*)"hello",strlen("hello"),50);
		

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

void LCD_Proc(void)
{
	if(uwTick-lcd_uwtick<500)return;
	lcd_uwtick=uwTick;
	switch(menu_flag)
	{
		case 0:menu1();break;
		case 1:menu2();break;
		case 2:menu3();break;
		default:break;	
	}
//	sprintf((char*)lcd_str,"hello");
//	LCD_DisplayStringLine(Line1,lcd_str);
	
}

void led_Proc(void)
{
	if(menu_flag==0)
	{
		if((led1_flag==0)&&(led2_flag==0))
			led_disp(0x04);
		else if(led1_flag==1)
		{
			led_disp(0x05);
			if(uwTick-led_uwtick>1000)				
				led1_flag=0;
		}
		else if(led2_flag==1)
		{
			led_disp(0x06);
			if(uwTick-led_uwtick>1000)
				led2_flag=0;
		}
	}
			
	else if(menu_flag==1)
		led_disp(0x08);
	else if(menu_flag==2)
		led_disp(0x10);
}

void key_Proc(void)
{
	if(uwTick-key_uwtick<50)return;
	key_uwtick=uwTick;
	key_val = key_scan();
	key_d=key_val&(key_old^key_val);
	key_u=~key_val&(key_old^key_val);
	key_old=key_val;
	
	switch(key_d)
	{
		case 1:
			std_flag=0;
			if(menu_flag<2) menu_flag++;
			else menu_flag=0;
			break;
		case 2:
			if(menu_flag==1)
			{
				if(std_flag<3) std_flag++;
				else std_flag=0;
			}
			else if(menu_flag==0)
			{
				R37_all++;
				if(3.3*get_ADC(hadc2)/4096<=R37_up&&3.3*get_ADC(hadc2)/4096>=R37_down)
				{	
					R37_num++;
					led1_flag=1;
					led_uwtick=uwTick;
				}
			}
			break;
		case 3:
			if(menu_flag==1)
			{
				R38_all=0;
				R38_num=0;
				R37_all=0;
				R37_num=0;
				switch(std_flag)
				{
					case 0:
						if(R37_up<3.0f)
							R37_up=R37_up+0.2f;
						else R37_up=2.2;
					break;
					case 1:
						if(R37_down<2.0f)
							R37_down+=0.2f;
						else R37_down=1.2;
					break;
					case 2:
						if(R38_up<3.0f)
							R38_up+=0.2f;
						else R38_up=2.2;
					break;
					case 3:
						if(R38_down<2.0f)
							R38_down+=0.2f;
						else R38_down=1.2;
					break;
				}
			}
			else if(menu_flag==0)
			{
				std_flag=0;
				R38_all++;
				if(3.3*get_ADC(hadc1)/4096<=R38_up&&3.3*get_ADC(hadc1)/4096>=R38_down)
				{
					R38_num++;
					led2_flag=1;
					led_uwtick=uwTick;
				}
			}
			break;
		case 4:
			if(menu_flag==1)
			{
				R38_all=0;
				R38_num=0;
				R37_all=0;
				R37_num=0;
				switch(std_flag)
				{
					case 0:
						if(R37_up>2.2f)
							R37_up-=0.2f;
						else R37_up=3.0;
					break;
					case 1:
						if(R37_down>1.2f)
							R37_down-=0.2f;
						else R37_down=2.0;
					break;
					case 2:
						if(R38_up>2.2f)
							R38_up-=0.2f;
						else R38_up=3.0;
					break;
					case 3:
						if(R38_down>1.2f)
							R38_down-=0.2f;
						else R38_down=2.0;
					break;
				}
			}
			else if(menu_flag==2)
			{
				std_flag=0;
				R38_all=0;
				R38_num=0;
				R37_all=0;
				R37_num=0;
			}
			break;
	}
//	if(key_d!=0)
//	{
//			sprintf((char*)lcd_str,"ked:%d",key_d);
//			LCD_DisplayStringLine(Line0,lcd_str);
//	}
	
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	
	rx_str[rx_len++]=rx;
	if(rx_len>=3)
	{
		rx_len=0;
		if(rx_str[0]=='R'&&rx_str[1]=='3'&&rx_str[2]=='7')
		{
			sprintf(str,"R37:%d,%d,%.1f%%\n",R37_all,(int)R37_num,R37_num/R37_all*100);
			HAL_UART_Transmit(&huart1,(uint8_t*)str,strlen(str),50);
		}
		if(rx_str[0]=='R'&&rx_str[1]=='3'&&rx_str[2]=='8')
		{
			sprintf(str,"R38:%d,%d,%.1f%%\n",R38_all,(int)R38_num,R38_num/R38_all*100);
			HAL_UART_Transmit(&huart1,(uint8_t*)str,strlen(str),50);
		}
	}
	if(rx_str[0]=='R')
		rx_len=1;
	if(rx_str[1]=='3')
		rx_len=2;
//	if(rx=='a')
//	{
//		sprintf(str,"success\n");
//		
//	}
//	else
//	{
//		sprintf(str,"error\n");
//	}

	HAL_UART_Receive_IT(&huart1,&rx,1);
}


void menu1(void)
{
	sprintf((char*)lcd_str,"       GOODS      ");
	LCD_DisplayStringLine(Line1,lcd_str);
	sprintf((char*)lcd_str,"     R37:%.2fV    ",3.3*get_ADC(hadc2)/4096);
	LCD_DisplayStringLine(Line3,lcd_str);
	sprintf((char*)lcd_str,"     R38:%.2fV    ",3.3*get_ADC(hadc1)/4096);
	LCD_DisplayStringLine(Line4,lcd_str);
	
}
void menu2(void)
{
	sprintf((char*)lcd_str,"      STANDARD    ");
	LCD_DisplayStringLine(Line1,lcd_str);  
	sprintf((char*)lcd_str,"    SR37:%.1f-%.1f",R37_down,R37_up);
	LCD_DisplayStringLine(Line3,lcd_str);
	sprintf((char*)lcd_str,"    SR38:%.1f-%.1f",R38_down,R38_up);
	LCD_DisplayStringLine(Line4,lcd_str);
}
void menu3(void)
{
	sprintf((char*)lcd_str,"        PASS      ");
	LCD_DisplayStringLine(Line1,lcd_str);
	if(R37_all==0)
		sprintf((char*)lcd_str,"     PR37:0.0%%  ");
	else
		sprintf((char*)lcd_str,"     PR37:%.1f%%  ",R37_num/R37_all*100);
	LCD_DisplayStringLine(Line3,lcd_str);
	if(R38_all==0)
		sprintf((char*)lcd_str,"     PR38:0.0%%  ");
	else
		sprintf((char*)lcd_str,"     PR38:%.1f%%    ",R38_num/R38_all*100);
	LCD_DisplayStringLine(Line4,lcd_str);
}


uint16_t get_ADC(ADC_HandleTypeDef hadc)
{
	uint16_t adc=0;
	HAL_ADC_Start(&hadc);
	adc = HAL_ADC_GetValue(&hadc);
	return adc;
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
