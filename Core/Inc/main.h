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
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct {
	int row;
	int col;
} KeyboardPos;

typedef struct {
	uint16_t Red;
	uint16_t Green;
	uint16_t Blue;
} Color;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LCD_RESET_Pin GPIO_PIN_5
#define LCD_RESET_GPIO_Port GPIOE
#define KBD_COL_1_Pin GPIO_PIN_1
#define KBD_COL_1_GPIO_Port GPIOC
#define KBD_ROW_B_Pin GPIO_PIN_0
#define KBD_ROW_B_GPIO_Port GPIOA
#define KBD_ROW_A_Pin GPIO_PIN_1
#define KBD_ROW_A_GPIO_Port GPIOA
#define KBD_ROW_C_Pin GPIO_PIN_2
#define KBD_ROW_C_GPIO_Port GPIOA
#define KBD_ROW_D_Pin GPIO_PIN_3
#define KBD_ROW_D_GPIO_Port GPIOA
#define KBD_ROW_E_Pin GPIO_PIN_6
#define KBD_ROW_E_GPIO_Port GPIOA
#define KBD_ROW_F_Pin GPIO_PIN_7
#define KBD_ROW_F_GPIO_Port GPIOA
#define KBD_COL_4_Pin GPIO_PIN_4
#define KBD_COL_4_GPIO_Port GPIOC
#define KBD_COL_5_Pin GPIO_PIN_5
#define KBD_COL_5_GPIO_Port GPIOC
#define KBD_COL_6_Pin GPIO_PIN_6
#define KBD_COL_6_GPIO_Port GPIOC
#define Led_Blue_Pin GPIO_PIN_8
#define Led_Blue_GPIO_Port GPIOC
#define KBD_COL_2_Pin GPIO_PIN_9
#define KBD_COL_2_GPIO_Port GPIOC
#define KBD_ROW_G_Pin GPIO_PIN_8
#define KBD_ROW_G_GPIO_Port GPIOA
#define KBD_ROW_H_Pin GPIO_PIN_10
#define KBD_ROW_H_GPIO_Port GPIOA
#define KBD_ROW_I_Pin GPIO_PIN_15
#define KBD_ROW_I_GPIO_Port GPIOA
#define KBD_COL_3_Pin GPIO_PIN_11
#define KBD_COL_3_GPIO_Port GPIOC
#define BACKLIGHT_Pin GPIO_PIN_3
#define BACKLIGHT_GPIO_Port GPIOD
#define Led_Red_Pin GPIO_PIN_4
#define Led_Red_GPIO_Port GPIOB
#define Led_Green_Pin GPIO_PIN_5
#define Led_Green_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define KEY_LEFT ((KeyboardPos){0, 0})
#define KEY_UP ((KeyboardPos){0, 1})
#define KEY_DOWN ((KeyboardPos){0, 2})
#define KEY_RIGHT ((KeyboardPos){0, 3})
#define KEY_OK ((KeyboardPos){0, 4})
#define KEY_BACK ((KeyboardPos){0, 5})
#define KEY_HOME ((KeyboardPos){1, 0})
#define KEY_ONOFF ((KeyboardPos){1, 2})
#define KEY_SHIFT ((KeyboardPos){2, 0})
#define KEY_ALPHA ((KeyboardPos){2, 1})
#define KEY_XNT ((KeyboardPos){2, 2})
#define KEY_VAR ((KeyboardPos){2, 3})
#define KEY_TOOLBOX ((KeyboardPos){2, 4})
#define KEY_BACKSPACE ((KeyboardPos){2, 5})
#define KEY_EXP ((KeyboardPos){3, 0})
#define KEY_LN ((KeyboardPos){3, 1})
#define KEY_LOG ((KeyboardPos){3, 2})
#define KEY_IMAGINARY ((KeyboardPos){3, 3})
#define KEY_COMMA ((KeyboardPos){3, 4})
#define KEY_POWER ((KeyboardPos){3, 5})
#define KEY_SINE ((KeyboardPos){4, 0})
#define KEY_COSINE ((KeyboardPos){4, 1})
#define KEY_TANGENT ((KeyboardPos){4, 2})
#define KEY_PI ((KeyboardPos){4, 3})
#define KEY_SQRT ((KeyboardPos){4, 4})
#define KEY_SQUARE ((KeyboardPos){4, 5})
#define KEY_SEVEN ((KeyboardPos){5, 0})
#define KEY_EIGHT ((KeyboardPos){5, 1})
#define KEY_NINE ((KeyboardPos){5, 2})
#define KEY_LEFTPARENTHESIS ((KeyboardPos){5, 3})
#define KEY_RIGHTPARENTHESIS ((KeyboardPos){5, 4})
#define KEY_FOUR ((KeyboardPos){6, 0})
#define KEY_FIVE ((KeyboardPos){6, 1})
#define KEY_SIX ((KeyboardPos){6, 2})
#define KEY_MULTIPLICATION ((KeyboardPos){6, 3})
#define KEY_DIVISION ((KeyboardPos){6, 4})
#define KEY_ONE ((KeyboardPos){7, 0})
#define KEY_TWO ((KeyboardPos){7, 1})
#define KEY_THREE ((KeyboardPos){7, 2})
#define KEY_PLUS ((KeyboardPos){7, 3})
#define KEY_MINUS ((KeyboardPos){7, 4})
#define KEY_ZERO ((KeyboardPos){8, 0})
#define KEY_DOT ((KeyboardPos){8, 1})
#define KEY_EE ((KeyboardPos){8, 2})
#define KEY_ANS ((KeyboardPos){8, 3})
#define KEY_EXE ((KeyboardPos){8, 4})
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
