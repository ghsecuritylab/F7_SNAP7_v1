/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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

#define DIGI_INPUT26_Pin GPIO_PIN_2
#define DIGI_INPUT26_GPIO_Port GPIOE
#define DIGI_INPUT27_Pin GPIO_PIN_3
#define DIGI_INPUT27_GPIO_Port GPIOE
#define DIGI_INPUT28_Pin GPIO_PIN_4
#define DIGI_INPUT28_GPIO_Port GPIOE
#define DIGI_INPUT29_Pin GPIO_PIN_5
#define DIGI_INPUT29_GPIO_Port GPIOE
#define DIGI_INPUT30_Pin GPIO_PIN_6
#define DIGI_INPUT30_GPIO_Port GPIOE
#define LD1_Pin GPIO_PIN_0
#define LD1_GPIO_Port GPIOB
#define DIGI_OUTPUT25_Pin GPIO_PIN_10
#define DIGI_OUTPUT25_GPIO_Port GPIOE
#define DIGI_OUTPUT24_Pin GPIO_PIN_11
#define DIGI_OUTPUT24_GPIO_Port GPIOE
#define DIGI_OUTPUT23_Pin GPIO_PIN_12
#define DIGI_OUTPUT23_GPIO_Port GPIOE
#define DIGI_OUTPUT22_Pin GPIO_PIN_13
#define DIGI_OUTPUT22_GPIO_Port GPIOE
#define DIGI_OUTPUT21_Pin GPIO_PIN_14
#define DIGI_OUTPUT21_GPIO_Port GPIOE
#define DIGI_OUTPUT20_Pin GPIO_PIN_15
#define DIGI_OUTPUT20_GPIO_Port GPIOE
#define DIGI_OUTPUT19_Pin GPIO_PIN_10
#define DIGI_OUTPUT19_GPIO_Port GPIOB
#define DIGI_OUTPUT18_Pin GPIO_PIN_11
#define DIGI_OUTPUT18_GPIO_Port GPIOB
#define DIGI_OUTPUT17_Pin GPIO_PIN_12
#define DIGI_OUTPUT17_GPIO_Port GPIOB
#define LD3_Pin GPIO_PIN_14
#define LD3_GPIO_Port GPIOB
#define DIGI_OUTPUT16_Pin GPIO_PIN_15
#define DIGI_OUTPUT16_GPIO_Port GPIOB
#define DIGI_OUTPUT15_Pin GPIO_PIN_10
#define DIGI_OUTPUT15_GPIO_Port GPIOD
#define DIGI_OUTPUT14_Pin GPIO_PIN_11
#define DIGI_OUTPUT14_GPIO_Port GPIOD
#define DIGI_OUTPUT13_Pin GPIO_PIN_12
#define DIGI_OUTPUT13_GPIO_Port GPIOD
#define DIGI_OUTPUT12_Pin GPIO_PIN_13
#define DIGI_OUTPUT12_GPIO_Port GPIOD
#define DIGI_OUTPUT11_Pin GPIO_PIN_14
#define DIGI_OUTPUT11_GPIO_Port GPIOD
#define DIGI_OUTPUT10_Pin GPIO_PIN_15
#define DIGI_OUTPUT10_GPIO_Port GPIOD
#define DIGI_OUTPUT9_Pin GPIO_PIN_2
#define DIGI_OUTPUT9_GPIO_Port GPIOG
#define DIGI_OUTPUT8_Pin GPIO_PIN_3
#define DIGI_OUTPUT8_GPIO_Port GPIOG
#define DIGI_OUTPUT7_Pin GPIO_PIN_4
#define DIGI_OUTPUT7_GPIO_Port GPIOG
#define DIGI_OUTPUT6_Pin GPIO_PIN_5
#define DIGI_OUTPUT6_GPIO_Port GPIOG
#define DIGI_OUTPUT5_Pin GPIO_PIN_8
#define DIGI_OUTPUT5_GPIO_Port GPIOC
#define DIGI_OUTPUT4_Pin GPIO_PIN_9
#define DIGI_OUTPUT4_GPIO_Port GPIOC
#define DIGI_OUTPUT3_Pin GPIO_PIN_8
#define DIGI_OUTPUT3_GPIO_Port GPIOA
#define DIGI_OUTPUT2_Pin GPIO_PIN_9
#define DIGI_OUTPUT2_GPIO_Port GPIOA
#define DIGI_OUTPUT1_Pin GPIO_PIN_10
#define DIGI_OUTPUT1_GPIO_Port GPIOA
#define DIGI_INPUT1_Pin GPIO_PIN_10
#define DIGI_INPUT1_GPIO_Port GPIOC
#define DIGI_INPUT2_Pin GPIO_PIN_11
#define DIGI_INPUT2_GPIO_Port GPIOC
#define DIGI_INPUT3_Pin GPIO_PIN_12
#define DIGI_INPUT3_GPIO_Port GPIOC
#define DIGI_INPUT4_Pin GPIO_PIN_0
#define DIGI_INPUT4_GPIO_Port GPIOD
#define DIGI_INPUT5_Pin GPIO_PIN_1
#define DIGI_INPUT5_GPIO_Port GPIOD
#define DIGI_INPUT6_Pin GPIO_PIN_2
#define DIGI_INPUT6_GPIO_Port GPIOD
#define DIGI_INPUT7_Pin GPIO_PIN_3
#define DIGI_INPUT7_GPIO_Port GPIOD
#define DIGI_INPUT8_Pin GPIO_PIN_4
#define DIGI_INPUT8_GPIO_Port GPIOD
#define DIGI_INPUT9_Pin GPIO_PIN_5
#define DIGI_INPUT9_GPIO_Port GPIOD
#define DIGI_INPUT10_Pin GPIO_PIN_6
#define DIGI_INPUT10_GPIO_Port GPIOD
#define DIGI_INPUT11_Pin GPIO_PIN_7
#define DIGI_INPUT11_GPIO_Port GPIOD
#define DIGI_INPUT12_Pin GPIO_PIN_9
#define DIGI_INPUT12_GPIO_Port GPIOG
#define DIGI_INPUT13_Pin GPIO_PIN_10
#define DIGI_INPUT13_GPIO_Port GPIOG
#define DIGI_INPUT15_Pin GPIO_PIN_12
#define DIGI_INPUT15_GPIO_Port GPIOG
#define DIGI_INPUT16_Pin GPIO_PIN_14
#define DIGI_INPUT16_GPIO_Port GPIOG
#define DIGI_INPUT17_Pin GPIO_PIN_15
#define DIGI_INPUT17_GPIO_Port GPIOG
#define DIGI_INPUT18_Pin GPIO_PIN_3
#define DIGI_INPUT18_GPIO_Port GPIOB
#define DIGI_INPUT19_Pin GPIO_PIN_4
#define DIGI_INPUT19_GPIO_Port GPIOB
#define DIGI_INPUT20_Pin GPIO_PIN_5
#define DIGI_INPUT20_GPIO_Port GPIOB
#define DIGI_INPUT21_Pin GPIO_PIN_6
#define DIGI_INPUT21_GPIO_Port GPIOB
#define LD2_Pin GPIO_PIN_7
#define LD2_GPIO_Port GPIOB
#define DIGI_INPUT22_Pin GPIO_PIN_8
#define DIGI_INPUT22_GPIO_Port GPIOB
#define DIGI_INPUT23_Pin GPIO_PIN_9
#define DIGI_INPUT23_GPIO_Port GPIOB
#define DIGI_INPUT24_Pin GPIO_PIN_0
#define DIGI_INPUT24_GPIO_Port GPIOE
#define DIGI_INPUT25_Pin GPIO_PIN_1
#define DIGI_INPUT25_GPIO_Port GPIOE

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
