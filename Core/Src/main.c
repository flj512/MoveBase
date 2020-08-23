/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include<stdio.h>
#include<string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define CMD_LINE_CHAR	'\n'
#define CMD_ENTER_CHAR	'\r'

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(a) (sizeof(a)/sizeof((a)[0]))
#endif

#define DEBUG_INFO printf

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
static int errCode=0;
static int lastSpeed=-1;
static int lastDir[4];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM14_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void coarse_delay_ms(int ms) {
	volatile int i, j;
	for (i = 0; i < ms; i++)
		for (j = 0; j < 500; j++)
			;
}
static void setMotorSpeed(int speed) {
	TIM_OC_InitTypeDef sConfigOC = {0};

	HAL_TIM_PWM_Stop(&htim14, TIM_CHANNEL_1);
	if(speed>0){
		if(speed>100){
			speed=100;
		}
		sConfigOC.OCMode = TIM_OCMODE_PWM1;
		sConfigOC.Pulse = speed-1;
		sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
		sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
		if (HAL_TIM_PWM_ConfigChannel(&htim14, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
		{
			Error_Handler();
		}
		HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1);
	}
	lastSpeed=speed;
}
struct MotorCfg {
	uint16_t port1;
	uint16_t port2;
};
struct MotorCtl {
	GPIO_PinState value1;
	GPIO_PinState value2;
};
enum {
	DIR_POS = 0,
	DIR_NEG,
	DIR_STOP
};
static void setMotorPar(int idx, int dir) {
	static struct MotorCfg cfg[] = {
			{ GPIO_PIN_4,GPIO_PIN_5 },
			{ GPIO_PIN_0, GPIO_PIN_1 },
			{ GPIO_PIN_6, GPIO_PIN_7 },
			{ GPIO_PIN_9, GPIO_PIN_10 }
	};
	static struct MotorCtl ctl[] = {
			{ GPIO_PIN_RESET, GPIO_PIN_SET },
			{ GPIO_PIN_SET, GPIO_PIN_RESET },
			{ GPIO_PIN_RESET, GPIO_PIN_RESET }
	};
	HAL_GPIO_WritePin(GPIOA, cfg[idx].port1, ctl[dir].value1);
	HAL_GPIO_WritePin(GPIOA, cfg[idx].port2, ctl[dir].value2);
	lastDir[idx]=dir;
}
static void stopMotor()
{
	int i;
	setMotorSpeed(0);
	coarse_delay_ms(50);
	for(i=0;i<4;i++){
		setMotorPar(i, DIR_STOP);
	}
}
static int getCmd(char *cmd, int *size) {
	int i, ret;
	char last;
	for (i = 0; i < *size;) {
		__HAL_UART_CLEAR_OREFLAG(&huart1);
		__HAL_UART_CLEAR_NEFLAG(&huart1);
		if ((ret = HAL_UART_Receive(&huart1, (uint8_t*) cmd + i, 1, 1000))
				!= HAL_OK) {
			*size=i;
			errCode=ret;
			return -1;
		}

		if (cmd[i] == CMD_LINE_CHAR) {
			if (i > 0 && last == CMD_ENTER_CHAR) {
				i--;
			}
			break;
		}

		last = cmd[i++];
	}
	*size=i;
	return 0;
}
static int getDir(char d)
{
	if(d=='0'){
		return DIR_POS;
	}else if(d=='1'){
		return DIR_NEG;
	}else{
		return DIR_STOP;
	}
}
//"OFF",stop motor
//S[speed]D[dir0]D[dir1]D[dir2]D[dir3] -> "S100D0D0D0D0",0->DIR_POS,1->DIR_NEG
static void processCmd(char *cmd, int endIdx) {
	int i,speed,changed;
	char d[4];
	cmd[endIdx] = '\0';
	DEBUG_INFO("Receive command:%s\r\n",cmd);
	if(strcmp(cmd,"OFF")==0){
		stopMotor();
		return;
	}else if(cmd[0]=='S'&&sscanf(cmd,"S%dD%cD%cD%cD%c",&speed,&d[0],&d[1],&d[2],&d[3])==5){
		if(speed<0){
			goto invalid_cmd;
		}
		for(i=0;i<4;i++){
			if(d[i]!='0'&&d[i]!='1'&&d[i]!='2'){
				goto invalid_cmd;
			}
		}
		changed=0;
		if(lastSpeed<0||lastSpeed!=speed){
			changed=1;
		}else{
			for(i=0;i<4;i++){
				if(getDir(d[i])!=lastDir[i]){
					changed=1;
					break;
				}
			}
		}
		if(!changed){
			DEBUG_INFO("not change,unset\r\n");
			return;
		}
		setMotorSpeed(0);
		coarse_delay_ms(20);
		for(i=0;i<4;i++){
			setMotorPar(i, getDir(d[i]));
		}
		coarse_delay_ms(20);
		setMotorSpeed(speed);
		return;
	}
invalid_cmd:
	DEBUG_INFO("Invalid command:%s\r\n",cmd);
}
static void motorLoop() {
	int ret;
	char cmd[32];
	int size;

	while (1) {
		size=ARRAY_SIZE(cmd);
		ret = getCmd(cmd, &size);
		if(ret<0){
			DEBUG_INFO("receive error, reason=%d，size=%d \r\n",errCode,size);
			stopMotor(0);
			continue;
		}
		if (size == ARRAY_SIZE(cmd)) {
			DEBUG_INFO("Command overflow\r\n");
		} else {
			processCmd(cmd, size);
		}
	}
}
#ifdef DEBUG
static void testLoop()
{
	int idx=0;
	setMotorSpeed(100);
	while(1){
		setMotorPar(idx, DIR_POS);
		coarse_delay_ms(2000);
		setMotorPar(idx, DIR_STOP);
		coarse_delay_ms(2000);
		if(++idx==4){
			idx=0;
		}
	}
}
#endif
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
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */
  motorLoop();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
#ifdef DEBUG
	//testLoop();
#endif
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 2399;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 99;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 49;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim14, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */
  HAL_TIM_MspPostInit(&htim14);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 38400;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_9|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 PA4 PA5 
                           PA6 PA7 PA9 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
