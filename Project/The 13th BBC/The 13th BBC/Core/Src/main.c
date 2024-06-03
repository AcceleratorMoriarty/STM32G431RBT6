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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd.h"
#include "stdio.h"
#include "string.h"
#include "led.h" 
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

uint8_t lcd_str[22];

uint8_t lcd_flag=0;
float PA4,PA5,A4,A5,T4,T5,H4,H5,sum4,sum5;
int PA1;
uint8_t X=1,Y=1,N4,N5;
uint8_t key_val,key_down,key_up,key_old;
uint32_t count;
uint16_t ADC[2];

__IO uint32_t key_tick;

uint8_t str1[4]={0x01,0x22,0x33};
uint8_t str2[4]={0};

uint8_t mode_flag=0;
uint8_t led=0x00;
uint8_t clear_flag=0;
uint8_t led_flag=0;
uint8_t direct_flag=0;
uint8_t rx;
char uart_str[50];
char tx_str[50];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void lcd_proc(void);
void key_proc(void);

void get_ADC(void);
void led_proc(void);
void clear_proc(void);
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
  MX_DMA_Init();
  MX_ADC2_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  LCD_Init();
  LCD_Clear(Black);
  LCD_SetBackColor(Black);
  LCD_SetTextColor(White);
  
  led_disp(0x00);
  
  
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_2);
	
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);

  HAL_ADC_Start_DMA(&hadc2,(uint32_t*)ADC,4);
  
  I2CInit();
//  write(str1,0,4);
//  HAL_Delay(100);
//  read(str2,0,4);
  read(&Y,0,1);
  read(&X,1,1);
  
  HAL_UART_Receive_IT(&huart1,&rx,1);
  

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  
	  lcd_proc();
	  key_proc();	
	  led_proc();
	  clear_proc();
	  if(mode_flag==0)
	  {
		  __HAL_TIM_SET_AUTORELOAD(&htim3,80000000/80/(X*PA1));
		  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,80000000/80/(X*PA1)*0.5);
	  }
	  else
	  {
		  __HAL_TIM_SET_AUTORELOAD(&htim3,80000000/80/(PA1/X));
		  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,80000000/80/(PA1/X)*0.5);
	  }
	  if(mode_flag==0)
		{
			led = led & 0xFD;
			led = led | 0x01;
		}
		else
		{
			led = led & 0xFE;
			led = led | 0x02;
		}
		if(direct_flag==0)
		{
			 LCD_WriteReg(R1, 0x0000); 
			LCD_WriteReg(R96, 0x2700); 
		}
		else
		{
			LCD_WriteReg(R1, 0x0100); 
			LCD_WriteReg(R96, 0xA700); 
		}
		  led_disp(led);
		
		
		if(N4==1)
		{
			A4=PA4;
			T4=PA4;
		}
		else if(N4>1)
		{
			if(PA4>A4)A4=PA4;
			if(PA4<T4)T4=PA4;
		}
		
		
			if(N5==1)
		{
			A5=PA5;
			T5=PA5;
		}
		else if(N5>1)
		{
			if(PA5>A5)A5=PA5;
			if(PA5<T5)T5=PA5;
		}
	
		
		
	
//		get_ADC();
		
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
	static uint8_t i;
	if(rx=='X')
	{
		sprintf(tx_str,"X:%d",X);HAL_UART_Transmit(&huart1,(uint8_t*)tx_str,strlen(tx_str),50);
	}
	else if(rx=='Y'){
		sprintf(tx_str,"Y:%d",Y);HAL_UART_Transmit(&huart1,(uint8_t*)tx_str,strlen(tx_str),50);}
	else if(rx=='#')
	{
		LCD_Clear(Black);
		direct_flag = ~direct_flag;
	}
	else if(rx=='P')
	{
		uart_str[0]=rx;
		i=1;
	}
	else
	{
		uart_str[i]=rx;i++;
	}
	if(i>=3)
	{
		if((uart_str[1]=='A')&(uart_str[0]=='P'))
		{
			if(uart_str[2]=='1')
			{
				sprintf(tx_str,"PA1:%d",PA1);HAL_UART_Transmit(&huart1,(uint8_t*)tx_str,strlen(tx_str),50);}
			else if(uart_str[2]=='4')
			{
				sprintf(tx_str,"PA4:%.2f",PA4);HAL_UART_Transmit(&huart1,(uint8_t*)tx_str,strlen(tx_str),50);}
			else if(uart_str[2]=='5')
			{
				sprintf(tx_str,"PA5:%.2f",PA5);HAL_UART_Transmit(&huart1,(uint8_t*)tx_str,strlen(tx_str),50);}
		}
		i=0;
	}
		
//	if(rx=='a')
//		sprintf(uart_str,"hello");
//	else
//		sprintf(uart_str,"error");
//	HAL_UART_Transmit(&huart1,(uint8_t*)tx_str,strlen(tx_str),50);
	 HAL_UART_Receive_IT(&huart1,&rx,1);
}


void led_proc(void)
{
	static __IO uint32_t tick;
	if(uwTick-tick<100)return;
	tick = uwTick;
	
	if(PA4>PA5*Y)
	{
		if(led_flag==0)
		{
			led_flag=1;
			led=led|0x04;
			
		}
		else
		{
			led_flag=0;
			led=led&0xFB;
		}
	}
	else
		led=led&0xFB;
	
	
}
void get_ADC(void)
{
//	uint16_t adc;
//	
//	HAL_ADC_Start(&hadc2);
//	adc=HAL_ADC_GetValue(&hadc2);
//	PA4 = 3.3*adc/4096;
	PA4 = 3.3*ADC[1]/4096;
	PA5 = 3.3*ADC[0]/4096;
}
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance==TIM2)
	{
		count=HAL_TIM_ReadCapturedValue(htim,TIM_CHANNEL_2)+1;
		PA1=80000000/80/count;
		if(PA1>10000) PA1=10000;
		else if(PA1<100) PA1=100;
		__HAL_TIM_SetCounter(htim,0);
	}
		
}

void clear_proc(void)
{
	if(uwTick-key_tick>1000)
		clear_flag=0;
	

}
void key_proc(void)
{
	static __IO uint32_t tick;
	if(uwTick-tick<50)return;
	tick = uwTick;
	
	key_val =key_scan();
	key_down = key_val&(key_val^key_old);
	key_up = ~key_val&(key_val^key_old);
	key_old = key_val;
	sprintf((char*)lcd_str,"hello:%d",key_down);
	LCD_DisplayStringLine(Line0,lcd_str);
	switch(key_down)
	{
		case 1:
			LCD_Clear(Black);
			if(lcd_flag<2) lcd_flag++;
			else lcd_flag=0;
		break;
		case 2:
			if(lcd_flag==1)
			{
				if(X<4)X++;
				else X=1;
				write(&X,1,1);
			}
			
		break;
		case 3:
			if(lcd_flag==1)
			{
				if(Y<4)Y++;
				else Y=1;
				write(&Y,0,1);
			}
			
		break;
		case 4:
			if(lcd_flag==2)
			{
				clear_flag=1;
				key_tick=uwTick;

			}
			else if(lcd_flag==3)
			{
				clear_flag=1;
				key_tick=uwTick;

			}
			else if(lcd_flag==0)
			{
				get_ADC();
				N4++;
				N5++;
				sum4=sum4+PA4;
				sum5=sum5+PA5;
				H4=sum4/N4;
				H5=sum5/N5;
			}
			else if(lcd_flag==1)
			{
					if(mode_flag==0)mode_flag=1;
        			else mode_flag=0;
			}
			
		break;
			
				
	}
	if(key_up==4)
	{
		if(uwTick-key_tick<1000)
		{
			if(lcd_flag==2)
			{
				lcd_flag=3;
			}
			else if(lcd_flag==3)
			{
				lcd_flag=2;
			}
			clear_flag=0;
		}
		else
		{
			if(lcd_flag==2)
			{
				N4=0;A4=0;
					
					T4=H4=0;
			}
			else if(lcd_flag==3)
			{
				T5=H5=0;A5=0;N5=0;
			}
			clear_flag=0;
		}
	}
	


	if(key_down)
	{
		sprintf((char*)lcd_str,"hello:%d",key_down);
		LCD_DisplayStringLine(Line0,lcd_str);
	}
	
}

void lcd_proc(void)
{
	static __IO uint32_t tick;
	if(uwTick-tick<200)return;
	tick = uwTick;
	
	
	
//	
//	sprintf((char*)lcd_str,"E:%x,%x,%x,%x,%x,%x",str1[0],str1[1],str1[2],str2[0],str2[1],str2[2]);
//    LCD_DisplayStringLine(Line0,lcd_str);
//	
	
	
	switch(lcd_flag)
	{
		case 0: 
			sprintf((char*)lcd_str,"        DATA        ");
			LCD_DisplayStringLine(Line1,lcd_str);
			sprintf((char*)lcd_str,"     PA4=%.2f       ",PA4);
			LCD_DisplayStringLine(Line3,lcd_str);
			sprintf((char*)lcd_str,"     PA5=%.2f       ",PA5);
			LCD_DisplayStringLine(Line4,lcd_str);
			sprintf((char*)lcd_str,"     PA1=%d       ",PA1);
			LCD_DisplayStringLine(Line5,lcd_str);
		break;
		
		case 1: 
			sprintf((char*)lcd_str,"        PARA        ");
			LCD_DisplayStringLine(Line1,lcd_str);
			sprintf((char*)lcd_str,"     X=%d       ",X);
			LCD_DisplayStringLine(Line3,lcd_str);
			sprintf((char*)lcd_str,"     Y=%d       ",Y);
			LCD_DisplayStringLine(Line4,lcd_str);
		break;
		case 2: 
			sprintf((char*)lcd_str,"        REC-PA4    ");
			LCD_DisplayStringLine(Line1,lcd_str);
			sprintf((char*)lcd_str,"     N=%d       ",N4);
			LCD_DisplayStringLine(Line3,lcd_str);
			sprintf((char*)lcd_str,"     A=%.2f       ",A4);
			LCD_DisplayStringLine(Line4,lcd_str);
			sprintf((char*)lcd_str,"     T=%.2f       ",T4);
			LCD_DisplayStringLine(Line5,lcd_str);
			sprintf((char*)lcd_str,"     H=%.2f       ",H4);
			LCD_DisplayStringLine(Line6,lcd_str);
		break;
		case 3: 
			sprintf((char*)lcd_str,"        REC-PA5    ");
			LCD_DisplayStringLine(Line1,lcd_str);
			sprintf((char*)lcd_str,"     N=%d       ",N5);
			LCD_DisplayStringLine(Line3,lcd_str);
			sprintf((char*)lcd_str,"     A=%.2f       ",A5);
			LCD_DisplayStringLine(Line4,lcd_str);
			sprintf((char*)lcd_str,"     T=%.2f       ",T5);
			LCD_DisplayStringLine(Line5,lcd_str);
			sprintf((char*)lcd_str,"     H=%.2f       ",H5);
			LCD_DisplayStringLine(Line6,lcd_str);
		break;
		
	}
//	sprintf((char*)lcd_str,"hello");
//	LCD_DisplayStringLine(Line0,lcd_str);
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
