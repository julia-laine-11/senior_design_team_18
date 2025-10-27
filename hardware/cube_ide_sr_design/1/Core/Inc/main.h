/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LOW_SPEED_IN_Pin GPIO_PIN_14
#define LOW_SPEED_IN_GPIO_Port GPIOC
#define LOW_SPEED_OUT_Pin GPIO_PIN_15
#define LOW_SPEED_OUT_GPIO_Port GPIOC
#define HIGH_SPEED_IN_Pin GPIO_PIN_0
#define HIGH_SPEED_IN_GPIO_Port GPIOF
#define HIGH_SPEED_OUT_Pin GPIO_PIN_1
#define HIGH_SPEED_OUT_GPIO_Port GPIOF
#define CONTROLLER_X_Pin GPIO_PIN_0
#define CONTROLLER_X_GPIO_Port GPIOC
#define CONTROLLER_Y_Pin GPIO_PIN_1
#define CONTROLLER_Y_GPIO_Port GPIOC
#define LED_OE_Pin GPIO_PIN_2
#define LED_OE_GPIO_Port GPIOA
#define LED_LAT_Pin GPIO_PIN_3
#define LED_LAT_GPIO_Port GPIOA
#define LED_CLK_Pin GPIO_PIN_4
#define LED_CLK_GPIO_Port GPIOA
#define LED_D_Pin GPIO_PIN_5
#define LED_D_GPIO_Port GPIOA
#define LED_C_Pin GPIO_PIN_6
#define LED_C_GPIO_Port GPIOA
#define LED_B_Pin GPIO_PIN_7
#define LED_B_GPIO_Port GPIOA
#define LED_A_Pin GPIO_PIN_4
#define LED_A_GPIO_Port GPIOC
#define LED_B2_Pin GPIO_PIN_5
#define LED_B2_GPIO_Port GPIOC
#define LED_G2_Pin GPIO_PIN_0
#define LED_G2_GPIO_Port GPIOB
#define LED_R2_Pin GPIO_PIN_1
#define LED_R2_GPIO_Port GPIOB
#define LED_B1_Pin GPIO_PIN_2
#define LED_B1_GPIO_Port GPIOB
#define LED_G1_Pin GPIO_PIN_10
#define LED_G1_GPIO_Port GPIOB
#define LED_R1_Pin GPIO_PIN_11
#define LED_R1_GPIO_Port GPIOB
#define Step2_out3_Pin GPIO_PIN_12
#define Step2_out3_GPIO_Port GPIOB
#define Step2_out2_Pin GPIO_PIN_13
#define Step2_out2_GPIO_Port GPIOB
#define Step2_out1_Pin GPIO_PIN_14
#define Step2_out1_GPIO_Port GPIOB
#define Step2_input_Pin GPIO_PIN_15
#define Step2_input_GPIO_Port GPIOB
#define Step1_out3_Pin GPIO_PIN_6
#define Step1_out3_GPIO_Port GPIOC
#define Step1_out2_Pin GPIO_PIN_7
#define Step1_out2_GPIO_Port GPIOC
#define Step1_out1_Pin GPIO_PIN_8
#define Step1_out1_GPIO_Port GPIOC
#define Step1_input_Pin GPIO_PIN_9
#define Step1_input_GPIO_Port GPIOC
#define IR_IN_2_Pin GPIO_PIN_8
#define IR_IN_2_GPIO_Port GPIOA
#define IR_IN_1_Pin GPIO_PIN_9
#define IR_IN_1_GPIO_Port GPIOA
#define JST_SWDIO_IN_Pin GPIO_PIN_13
#define JST_SWDIO_IN_GPIO_Port GPIOA
#define JST_SWCLK_IN_Pin GPIO_PIN_14
#define JST_SWCLK_IN_GPIO_Port GPIOA
#define USB_USART5_TX_Pin GPIO_PIN_12
#define USB_USART5_TX_GPIO_Port GPIOC
#define USB_USART5_RX_Pin GPIO_PIN_2
#define USB_USART5_RX_GPIO_Port GPIOD
#define ADMIN_CONSOLE_BUTTON_Pin GPIO_PIN_4
#define ADMIN_CONSOLE_BUTTON_GPIO_Port GPIOB
#define ADMIN_JOYSTICK_BUTTON_Pin GPIO_PIN_5
#define ADMIN_JOYSTICK_BUTTON_GPIO_Port GPIOB
#define ADMIN_JOYSTICK_Y_Pin GPIO_PIN_6
#define ADMIN_JOYSTICK_Y_GPIO_Port GPIOB
#define ADMIN_JOYSTICK_X_Pin GPIO_PIN_7
#define ADMIN_JOYSTICK_X_GPIO_Port GPIOB
#define ADMIN_CONSOLE_DISP3_Pin GPIO_PIN_11
#define ADMIN_CONSOLE_DISP3_GPIO_Port GPIOF
#define ADMIN_CONSOLE_DISP2_Pin GPIO_PIN_8
#define ADMIN_CONSOLE_DISP2_GPIO_Port GPIOB
#define ADMIN_CONSOLE_DISP1_Pin GPIO_PIN_9
#define ADMIN_CONSOLE_DISP1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
