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
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "led.h"
#include "lcd.h"
#include "key.h"
#include "i2c_hal.h"

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
char uart_str[50];
uint8_t rx_buffer;

//EEPROM
uint8_t EEPROM_1[3] = {0,1,2};

uint8_t EEPROM_2[3] = {0};

uint8_t MCP4017;

RTC_TimeTypeDef time;
RTC_DateTypeDef date;

uint16_t adc1,adc2;

int i=0;

int PA7_up,PA7_down;
float duty;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void lcd_proc(void);
void key_proc(void);
uint16_t get_adc1(void); 
uint16_t get_adc2(void); 

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
  MX_RTC_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM6_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  
  led_disp(0x00);
  
  LCD_Init();
  LCD_Clear(Black);
  LCD_SetBackColor(Black);
  LCD_SetTextColor(White);
  
  HAL_UART_Receive_IT(&huart1,&rx_buffer,1);
  
  I2CInit();
  
  //EEPROM²âÊÔ
  iic_write(EEPROM_1,0,3);
  HAL_Delay(10);
  iic_read(EEPROM_2,0,3);
//  iic_read(EEPROM_2,0,1);
//  EEPROM_2[0]++;
//  iic_write(EEPROM_2,0,1);
  
  //MCP_4017²âÊÔ
  mcp_write(0x0D);
  MCP4017=mcp_read();
  
  //TIM6 ¶¨Ê±Æ÷²âÊÔ
  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_Base_Start_IT(&htim3);
  
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
  HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_2);
  __HAL_TIM_SET_AUTORELOAD(&htim2,250-1);
  __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,25);
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  lcd_proc();
	  key_proc();	
	  //led²âÊÔ
//	  led_disp(0x23);
//	  HAL_Delay(500);
//	  led_disp(0x00);
//	  HAL_Delay(500);
//	    __HAL_TIM_SET_AUTORELOAD(&htim2,250-1);
//		__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,25);
//		 
//	  HAL_Delay(2000);
//	  
//	    __HAL_TIM_SET_AUTORELOAD(&htim2,500-1);
//		__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,250);
//	  HAL_Delay(2000);
	  
	  	HAL_RTC_GetTime(&hrtc,&time,RTC_FORMAT_BIN);
		HAL_RTC_GetDate(&hrtc,&date,RTC_FORMAT_BIN);

	  
	  
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
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
	if(htim->Instance==TIM3)
	{
		if(htim->Channel==HAL_TIM_ACTIVE_CHANNEL_1)
		{
			PA7_up=HAL_TIM_ReadCapturedValue(&htim3,TIM_CHANNEL_1)+1;
			
			duty = (float)PA7_down/PA7_up;
			__HAL_TIM_SetCounter(&htim3,0);
			
			  HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_1);
			
		}
		else if(htim->Channel==HAL_TIM_ACTIVE_CHANNEL_2)
		{
			PA7_down=HAL_TIM_ReadCapturedValue(&htim3,TIM_CHANNEL_2)+1;
//			__HAL_TIM_SetCounter(&htim3,0);
//			HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_2);
		}
		
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance==TIM6)
	{
		i++;
		HAL_TIM_Base_Start_IT(&htim6);
	}
}
uint16_t get_adc1(void)
{
	uint16_t adc;
	HAL_ADC_Start(&hadc1);
	adc = HAL_ADC_GetValue(&hadc1);
	return adc;
//	HAL_ADC_Start(&hadc2);
//	adc2 = HAL_ADC_GetValue(&hadc2);
}

uint16_t get_adc2(void)
{
	uint16_t adc;
	HAL_ADC_Start(&hadc2);
	adc = HAL_ADC_GetValue(&hadc2);
	return adc;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(rx_buffer=='a')
		sprintf(uart_str,"success receive a\r\n");
	else
		sprintf(uart_str,"error receive a\r\n");
	HAL_UART_Transmit(&huart1,(uint8_t*)uart_str,strlen(uart_str),50);
	HAL_UART_Receive_IT(&huart1,&rx_buffer,1);
}

void lcd_proc(void)
{
	if(uwTick-lcd_uwtick<100)return;
	lcd_uwtick = uwTick;
	
//	sprintf((char*)lcd_str,"E1:%x,%x,%x    E2:%x,%x,%x",EEPROM_1[0],EEPROM_1[1],EEPROM_1[2],EEPROM_2[0],EEPROM_2[1],EEPROM_2[2]);
//	LCD_DisplayStringLine(Line0,lcd_str);
//	sprintf((char*)lcd_str,"MCP4017:%.2fK,%.2fV",0.7874*MCP4017,3.3*(0.7874*MCP4017/(0.7874*MCP4017+10)));
//	LCD_DisplayStringLine(Line1,lcd_str);
//	sprintf((char*)lcd_str,"%02d-%02d-%02d    %02d:%02d:%02d",date.Year,date.Month,date.Date,time.Hours,time.Minutes,time.Seconds);
//	LCD_DisplayStringLine(Line2,lcd_str);
//	sprintf((char*)lcd_str,"R37:%.2f    R38:%.2f",3.3*get_adc2()/4096,3.3*get_adc1()/4096);
//	LCD_DisplayStringLine(Line3,lcd_str);
//	
//	sprintf((char*)lcd_str,"TIM6:%d",i);
//	LCD_DisplayStringLine(Line4,lcd_str);
//	
//	sprintf((char*)lcd_str,"PA7:%d,%d,%5.2f",1000000/PA7_up,1000000/PA7_down,duty*100);
//	LCD_DisplayStringLine(Line5,lcd_str);
	
	
	sprintf((char*)lcd_str,"        DATA      ");
	LCD_DisplayStringLine(Line3,lcd_str);
	sprintf((char*)lcd_str,"     PR39:%dHz   ", 1000000/PA7_up);
	LCD_DisplayStringLine(Line4,lcd_str);
}
void key_proc(void)
{
	if(uwTick-key_uwtick<50)return;
	key_uwtick = uwTick;
	
	key_val = key_scan();
	key_down = key_val&(key_old^key_val);
	key_up = ~key_val&(key_old^key_val);
	key_old = key_val;
	
	//keyµ÷ÊÔ
//	if(key_down)
//	{
//		sprintf((char*)lcd_str,"hello:%d    ",key_val);
//		LCD_DisplayStringLine(Line1,lcd_str);
//	}
	
	if(key_down==1)
	{
	    __HAL_TIM_SET_AUTORELOAD(&htim2,125-1);
		__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,25);
	}
	else if(key_down==2)
	{
	    __HAL_TIM_SET_AUTORELOAD(&htim2,250-1);
		__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,25);
	}
	
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
