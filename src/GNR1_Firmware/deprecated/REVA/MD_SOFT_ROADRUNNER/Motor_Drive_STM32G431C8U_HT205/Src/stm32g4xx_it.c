/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32g4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32g4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

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
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DAC_HandleTypeDef hdac3;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim8;
extern UART_HandleTypeDef huart3;
extern TIM_HandleTypeDef htim6;

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/

/******************************************************************************/
/* STM32G4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g4xx.s).                    */
/******************************************************************************/

///**
//  * @brief This function handles TIM2 global interrupt.
//  */
//void TIM2_IRQHandler(void)
//{
//  /* USER CODE BEGIN TIM2_IRQn 0 */

//  /* USER CODE END TIM2_IRQn 0 */
//  HAL_TIM_IRQHandler(&htim2);
//  /* USER CODE BEGIN TIM2_IRQn 1 */

//  /* USER CODE END TIM2_IRQn 1 */
//}

///**
//  * @brief This function handles TIM3 global interrupt.
//  */
//void TIM3_IRQHandler(void)
//{
//  /* USER CODE BEGIN TIM3_IRQn 0 */

//  /* USER CODE END TIM3_IRQn 0 */
//  HAL_TIM_IRQHandler(&htim3);
//  /* USER CODE BEGIN TIM3_IRQn 1 */

//  /* USER CODE END TIM3_IRQn 1 */
//}

///**
//  * @brief This function handles TIM4 global interrupt.
//  */
//void TIM4_IRQHandler(void)
//{
//  /* USER CODE BEGIN TIM4_IRQn 0 */

//  /* USER CODE END TIM4_IRQn 0 */
//  HAL_TIM_IRQHandler(&htim4);
//  /* USER CODE BEGIN TIM4_IRQn 1 */

//  /* USER CODE END TIM4_IRQn 1 */
//}

///**
//  * @brief This function handles USART3 global interrupt / USART3 wake-up interrupt through EXTI line 28.
//  */
//void USART3_IRQHandler(void)
//{
//  /* USER CODE BEGIN USART3_IRQn 0 */

//  /* USER CODE END USART3_IRQn 0 */
//  HAL_UART_IRQHandler(&huart3);
//  /* USER CODE BEGIN USART3_IRQn 1 */

//  /* USER CODE END USART3_IRQn 1 */
//}

///**
//  * @brief This function handles TIM8 update interrupt.
//  */
//void TIM8_UP_IRQHandler(void)
//{
//  /* USER CODE BEGIN TIM8_UP_IRQn 0 */

//  /* USER CODE END TIM8_UP_IRQn 0 */
//  HAL_TIM_IRQHandler(&htim8);
//  /* USER CODE BEGIN TIM8_UP_IRQn 1 */

//  /* USER CODE END TIM8_UP_IRQn 1 */
//}

///**
//  * @brief This function handles TIM8 capture compare interrupt.
//  */
//void TIM8_CC_IRQHandler(void)
//{
//  /* USER CODE BEGIN TIM8_CC_IRQn 0 */

//  /* USER CODE END TIM8_CC_IRQn 0 */
//  HAL_TIM_IRQHandler(&htim8);
//  /* USER CODE BEGIN TIM8_CC_IRQn 1 */

//  /* USER CODE END TIM8_CC_IRQn 1 */
//}

/**
  * @brief This function handles TIM6 global interrupt, DAC1 and DAC3 channel underrun error interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */

  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  HAL_DAC_IRQHandler(&hdac3);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
