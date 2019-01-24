/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define USER_BOTTON_Pin GPIO_PIN_0
#define USER_BOTTON_GPIO_Port GPIOA
#define USER_BOTTON_EXTI_IRQn EXTI0_IRQn
#define FALTA_COMIDA_Pin GPIO_PIN_11
#define FALTA_COMIDA_GPIO_Port GPIOE
#define COMIDA_LLENA_Pin GPIO_PIN_12
#define COMIDA_LLENA_GPIO_Port GPIOE
#define FALTA_AGUA_Pin GPIO_PIN_13
#define FALTA_AGUA_GPIO_Port GPIOE
#define AGUA_LLENA_Pin GPIO_PIN_14
#define AGUA_LLENA_GPIO_Port GPIOE
#define display_A_Pin GPIO_PIN_10
#define display_A_GPIO_Port GPIOB
#define display_B_Pin GPIO_PIN_11
#define display_B_GPIO_Port GPIOB
#define display_C_Pin GPIO_PIN_12
#define display_C_GPIO_Port GPIOB
#define display_D_Pin GPIO_PIN_13
#define display_D_GPIO_Port GPIOB
#define display_E_Pin GPIO_PIN_14
#define display_E_GPIO_Port GPIOB
#define display_F_Pin GPIO_PIN_15
#define display_F_GPIO_Port GPIOB
#define LED_PARO_Pin GPIO_PIN_13
#define LED_PARO_GPIO_Port GPIOD
#define LED_FUNCIONAMIENTO_Pin GPIO_PIN_15
#define LED_FUNCIONAMIENTO_GPIO_Port GPIOD
#define SENSOR_VACIO_Pin GPIO_PIN_7
#define SENSOR_VACIO_GPIO_Port GPIOC
#define SENSOR_VACIO_EXTI_IRQn EXTI9_5_IRQn
#define display_G_Pin GPIO_PIN_3
#define display_G_GPIO_Port GPIOB
#define display_DP_Pin GPIO_PIN_4
#define display_DP_GPIO_Port GPIOB
#define D1_Pin GPIO_PIN_5
#define D1_GPIO_Port GPIOB
#define D2_Pin GPIO_PIN_6
#define D2_GPIO_Port GPIOB
#define D3_Pin GPIO_PIN_7
#define D3_GPIO_Port GPIOB
#define D4_Pin GPIO_PIN_8
#define D4_GPIO_Port GPIOB

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
