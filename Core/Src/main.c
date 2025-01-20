/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include <stdlib.h>
#include <stdint.h>
//bras
#include <string.h>
#include <stdio.h>
#include "sensors_driver.h"
#include "nfc_sensor.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
//#define USE_PRINTF
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim21;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t current_section = 0;
#define TIMER_PERIOD 20000
ARM_STATE arm_state;
uint32_t current_rfid = 0x00;
bool login_successful = false;
uint8_t uartRxByte = 0x00;
uint8_t uartTxBuffer[128];

uint32_t adcValue1 = 0;
uint32_t adcValue2 = 0;
uint32_t adcValue3 = 0;
uint32_t adcValue4 = 0;

float tension1 = 0.0;
float tension2 = 0.0;
float tension3 = 0.0;
float tension4 = 0.0;

typedef enum {
	ROBOT_DIRECTION_STOP = 0x00,
	ROBOT_DIRECTION_FORWARD = 0x01,
	ROBOT_DIRECTION_BACKWARDS = 0x02,
	ROBOT_DIRECTION_TURN_LEFT = 0x03,
	ROBOT_DIRECTION_TURN_RIGHT = 0x04
} ROBOT_DIRECTION;

ROBOT_DIRECTION current_direction = ROBOT_DIRECTION_FORWARD;
bool is_follow_enabled = false;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM21_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void Set_Direction(uint8_t direction) {
    if (direction) {
    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
    } else {
    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
    }
}

void RotateToSection(uint8_t target_section) {

    if (target_section > 7) return;

    const uint16_t steps_per_section = 25; // 45° / 1.8° = 25

    int8_t section_diff = target_section - current_section;

    uint16_t steps_to_move = abs(section_diff) * steps_per_section;

    if (section_diff > 0) {
        Set_Direction(1);
    } else if (section_diff < 0) {
        Set_Direction(0);
    } else {
        return;
    }

    HAL_TIM_PWM_Start_IT(&htim21, TIM_CHANNEL_1);

    for (uint16_t i = 0; i < steps_to_move; i++) {
        HAL_Delay(50); //T = 0.05s
    }

    HAL_TIM_PWM_Stop_IT(&htim21, TIM_CHANNEL_1);

    current_section = target_section;
}

void ControlMotor(uint8_t direction, float angle) {
    if (angle < 0 || angle > 360) return;

    const float step_angle = 1.8;
    uint16_t steps_to_move = (uint16_t)(angle / step_angle + 0.5);

    if (direction == 1) {
        Set_Direction(1);
    } else if (direction == 0) {
        Set_Direction(0);
    } else {
        return;
    }

    HAL_TIM_PWM_Start_IT(&htim21, TIM_CHANNEL_1);

    for (uint16_t i = 0; i < steps_to_move; i++) {
        HAL_Delay(50);
    }

    HAL_TIM_PWM_Stop_IT(&htim21, TIM_CHANNEL_1);
}

//bras
uint16_t calculate_pwm_from_angle(float angle) {
    if (angle < 0.0f) angle = 0.0f;
    if (angle > 180.0f) angle = 180.0f;

    float duty_cycle = 2.5f + (angle / 180.0f) * (12.5f - 2.5f);

    return (uint16_t)(TIMER_PERIOD * duty_cycle / 100.0f);
}

float calculate_angle_from_pwm(uint16_t pwm) {
	if (pwm > TIMER_PERIOD) return 100.0f;

	float duty_cycle = 100.0f * ((float) pwm)/TIMER_PERIOD;
	float angle = (duty_cycle - 2.5f)/(12.5f - 2.5f) * 180.0f;

    return angle;
}


void control_servo(uint8_t motor_index, float angle) {
    if (motor_index < 1 || motor_index > 6) {
        return;
    }

    uint16_t pwm_value = calculate_pwm_from_angle(angle);

    switch (motor_index) {
        case 2:
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwm_value);
            break;
        case 3:
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pwm_value);
            break;
        case 4:
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, pwm_value);
            break;
        case 6:
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, pwm_value);
            break;
    }
    arm_state = ARM_STATE_CUSTOM;
}

float read_servo(uint8_t motor_index) {
    if (motor_index < 1 || motor_index > 6) {
        return 0.0f;
    }

    uint16_t pwm_value = 0;
    switch (motor_index) {
        case 2:
        	pwm_value = __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_1);
            break;
        case 3:
        	pwm_value = __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_2);
            break;
        case 4:
        	pwm_value = __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_3);
            break;
        case 6:
        	pwm_value = __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_4);
            break;
    }

    return calculate_angle_from_pwm(pwm_value);
}

void control_stop_yuan(){
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 400);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 600);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 2600);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 1000);
	arm_state = ARM_STATE_IDLE;
}

void control_stop(){
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 400);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 400);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 600);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 1000);
	arm_state = ARM_STATE_IDLE;
}

void get_ready(){
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 800);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 450);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 2500);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 900);
	arm_state = ARM_STATE_READY;
}

void catch(){
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 800);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 450);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 2500);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 1200);
	arm_state = ARM_STATE_GRAB;
}

void lift_up(){
	//__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 1200);//6
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 500);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 700);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 2100);
	arm_state = ARM_STATE_DISPLAY;
}
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
  MX_TIM21_Init();
  MX_TIM2_Init();
  MX_ADC_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_PWM_Start_IT(&htim21,TIM_CHANNEL_1);
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_PWM_Start_IT(&htim2,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start_IT(&htim2,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start_IT(&htim2,TIM_CHANNEL_3);
  HAL_TIM_PWM_Start_IT(&htim2,TIM_CHANNEL_4);

  init_nfc(&hspi1, SPI1_CS_GPIO_Port, SPI1_CS_Pin, NFC_RST_GPIO_Port, NFC_RST_Pin);
  HAL_UART_Receive_IT(&huart2, &uartRxByte, 1);

  printf("System initialized !\n");

  control_stop_yuan();
  RotateToSection(0);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  printf("Main loop started\n");
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  	  HAL_Delay(100);

	  if (!login_successful) {
		  if (get_nfc_id(&current_rfid) && current_rfid != 0) {
	            SENSOR_COMM_OPTIONS options = {
	            	.is_write = false,
					.is_answer = true,
					.echo_write = false,
					.is_spont_answ = true
	            };
				uint8_t answer[4];
				answer[0] = (current_rfid >> 0 * 8) & 0xFF;
				answer[1] = (current_rfid >> 1 * 8) & 0xFF;
				answer[2] = (current_rfid >> 2 * 8) & 0xFF;
				answer[3] = (current_rfid >> 3 * 8) & 0xFF;
				sensor_send_cmd(SENSOR_DEVICE_RFID, options, answer, 4);
				login_successful = true;
		  }
	  }


  	// Attendre que la conversion soit terminée

	  // Lancer les conversions
	   HAL_ADC_Start(&hadc);

	   // Attendre que la conversion soit terminée
	   if (HAL_ADC_PollForConversion(&hadc, 100) == HAL_OK)
	   {
	     adcValue1 = HAL_ADC_GetValue(&hadc);  // Canal 14

	  }

	   if (HAL_ADC_PollForConversion(&hadc, 100) == HAL_OK)
	   {
	     adcValue2 = HAL_ADC_GetValue(&hadc);  // Canal 15

	  }
	   if (HAL_ADC_PollForConversion(&hadc, 100) == HAL_OK)
	   {
	     adcValue3 = HAL_ADC_GetValue(&hadc);  // Canal 14

	  }

	   if (HAL_ADC_PollForConversion(&hadc, 100) == HAL_OK)
	   {
	     adcValue4 = HAL_ADC_GetValue(&hadc);  // Canal 15

	  }
	  HAL_ADC_Stop(&hadc);

	  // Convertir en tension
	  tension1 = (float)adcValue1 * 3.3 / 4095;
	  tension2 = (float)adcValue2 * 3.3 / 4095;
	  tension3 = (float)adcValue3 * 3.3 / 4095;
	  tension4 = (float)adcValue4 * 3.3 / 4095;



	  // Contrôler les LED en fonction des tensions
	  ROBOT_DIRECTION new_direction = ROBOT_DIRECTION_FORWARD;

	  if (tension1 > 2.75) {
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
		  new_direction = ROBOT_DIRECTION_STOP;
	  } else {
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
	  }

	  if (tension2 > 2.75) {
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
		  new_direction = ROBOT_DIRECTION_STOP;
	  } else {
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
	  }
	  if (tension3 > 2.75) {
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
		  new_direction = ROBOT_DIRECTION_STOP;
	  } else {
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
	  }

	  if (tension4 > 2.75) {
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
		  new_direction = ROBOT_DIRECTION_STOP;
	  } else {
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
	  }

	  if (new_direction != current_direction) {
		  SENSOR_COMM_OPTIONS options = {
			  .is_answer = true,
			  .is_write = false,
			  .echo_write = false,
			  .is_spont_answ = true
		  };
		  sensor_send_cmd(SENSOR_DEVICE_IR_TELEMETERS, options, &new_direction, 1);
		  current_direction = new_direction;
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.OversamplingMode = DISABLE;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.SamplingTime = ADC_SAMPLETIME_160CYCLES_5;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerFrequencyMode = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_14;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_15;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 32-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 20000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM21 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM21_Init(void)
{

  /* USER CODE BEGIN TIM21_Init 0 */

  /* USER CODE END TIM21_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM21_Init 1 */

  /* USER CODE END TIM21_Init 1 */
  htim21.Instance = TIM21;
  htim21.Init.Prescaler = 320-1;
  htim21.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim21.Init.Period = 5000-1;
  htim21.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim21.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim21) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim21, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim21) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim21, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 2500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim21, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM21_Init 2 */

  /* USER CODE END TIM21_Init 2 */
  HAL_TIM_MspPostInit(&htim21);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SPI1_CS_Pin|NFC_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DIR_CTRL_Pin|LED1_CTRL_Pin|LED2_CTRL_Pin|LED3_CTRL_Pin
                          |LED4_CTRL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : IR_SENSOR_IN_Pin */
  GPIO_InitStruct.Pin = IR_SENSOR_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IR_SENSOR_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_CS_Pin NFC_RST_Pin */
  GPIO_InitStruct.Pin = SPI1_CS_Pin|NFC_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DIR_CTRL_Pin LED1_CTRL_Pin LED2_CTRL_Pin LED3_CTRL_Pin
                           LED4_CTRL_Pin */
  GPIO_InitStruct.Pin = DIR_CTRL_Pin|LED1_CTRL_Pin|LED2_CTRL_Pin|LED3_CTRL_Pin
                          |LED4_CTRL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int __io_putchar(int ch) {
#ifdef USE_PRINTF
	HAL_UART_Transmit(&huart2, (uint8_t *) &ch, 1, 0xFFFF);
#endif
	return ch;
}

bool device_send_uart(uint8_t *bytes, uint8_t bytes_len) {
	memcpy(uartTxBuffer, bytes, bytes_len);
	return HAL_UART_Transmit_IT(&huart2, uartTxBuffer, bytes_len) == HAL_OK;
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	HAL_UART_Receive_IT(&huart2, &uartRxByte, 1);
	sensor_uart_rx_cb(&uartRxByte, 1);
}


// UART Callbacks

uint8_t sensor_hw_get_battery_level() {
	return 0;
}

uint32_t sensor_hw_get_rfid() {
	get_nfc_id(&current_rfid);
	return current_rfid;
}

uint8_t sensor_hw_get_set_ir_telemeter(bool set, uint8_t ir_telem_id, uint8_t ir_telem_distance) {
	if (set) {
		is_follow_enabled = (ir_telem_distance != 0);
	} else {
		return current_direction;
	}
}

bool sensor_hw_read_line_follower(uint8_t line_follow_id) {
	return 0;
}

uint8_t sensor_hw_get_set_led_value(bool set, uint8_t led_id, uint8_t led_value) {
	GPIO_TypeDef* led_port;
	uint32_t led_pin;
	switch (led_id) {
		case 1: {
			led_port = LED1_CTRL_GPIO_Port;
			led_pin = LED1_CTRL_Pin;
		}
		case 2: {
			led_port = LED2_CTRL_GPIO_Port;
			led_pin = LED2_CTRL_Pin;
		}
		case 3: {
			led_port = LED3_CTRL_GPIO_Port;
			led_pin = LED3_CTRL_Pin;
		}
		case 4: {
			led_port = LED4_CTRL_GPIO_Port;
			led_pin = LED4_CTRL_Pin;
		}
		default:
			return 0x00;
	}


	if (set) {
		HAL_GPIO_WritePin(led_port, led_pin, (led_value != 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	}

	return HAL_GPIO_ReadPin(led_port, led_pin) == GPIO_PIN_SET ? 1 : 0;
}

uint8_t sensor_hw_get_set_tool_selection(bool set, uint8_t tool_id) {
	if (set) {
		RotateToSection(tool_id);
	}

	return current_section;
}

uint8_t sensor_hw_robot_get_set_servo(bool set, uint8_t servo_id, uint8_t servo_angle) {
	if (set) {
		control_servo(servo_id, (float) servo_angle);
	}

	return read_servo(servo_id);
}

uint8_t sensor_hw_robot_get_set_state(bool set, ARM_STATE state) {
	if (set) {
		switch(state) {
			case ARM_STATE_CUSTOM: {
				break;
			}
			case ARM_STATE_IDLE: {
				control_stop();
			}
			case ARM_STATE_READY: {
				get_ready();
			}
			case ARM_STATE_GRAB: {
				catch();
			}
			case ARM_STATE_DISPLAY: {
				lift_up();
			}
			default: {
				break;
			}
		}
	}

	return arm_state;
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
