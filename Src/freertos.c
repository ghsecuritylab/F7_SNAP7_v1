/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */     
#include "adc.h"
#include "modbus.h"
/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId SysTaskHandle;
osThreadId ModbusTaskHandle;
osMessageQId modbusTcpRxQueueHandle;
osMessageQId modbusRtuQueueHandle;
osMessageQId modbusAsciiQueueHandle;

/* USER CODE BEGIN Variables */
uint16_t button = 0;
uint8_t button_action = 0;
uint16_t adc_buffer[13];
/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartSysTask(void const * argument);
void StartModbusTask(void const * argument);

extern void MX_LWIP_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_13)
	{
		button++;
		if(button > 3) button = 0;
		button_action = 1;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	modbus_acsii_get_input(huart);
//	modbus_rtu_get_input(huart);
}

/* USER CODE END FunctionPrototypes */

/* Hook prototypes */

/* Init FreeRTOS */

void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of SysTask */
  osThreadDef(SysTask, StartSysTask, osPriorityNormal, 0, 128);
  SysTaskHandle = osThreadCreate(osThread(SysTask), NULL);

  /* definition and creation of ModbusTask */
//  osThreadDef(ModbusTask, StartModbusTask, osPriorityIdle, 0, 128);
//  ModbusTaskHandle = osThreadCreate(osThread(ModbusTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the queue(s) */
  /* definition and creation of modbusTcpRxQueue */
  osMessageQDef(modbusTcpRxQueue, 100, uint8_t);
  modbusTcpRxQueueHandle = osMessageCreate(osMessageQ(modbusTcpRxQueue), NULL);

  /* definition and creation of modbusRtuQueue */
  osMessageQDef(modbusRtuQueue, 100, uint8_t);
  modbusRtuQueueHandle = osMessageCreate(osMessageQ(modbusRtuQueue), NULL);

  /* definition and creation of modbusAsciiQueue */
  osMessageQDef(modbusAsciiQueue, 100, uint8_t);
  modbusAsciiQueueHandle = osMessageCreate(osMessageQ(modbusAsciiQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* StartSysTask function */
void StartSysTask(void const * argument)
{
  /* init code for LWIP */
//  MX_LWIP_Init();

  /* USER CODE BEGIN StartSysTask */
	modbus_register_init(1,100,50);																																				// register init
//	modbus_ascii_port_init(&huart4,115200,UART_WORDLENGTH_8B,UART_STOPBITS_1,UART_PARITY_NONE);						// modbus ascii init
//	modbus_rtu_port_init(&huart6,115200,UART_WORDLENGTH_8B,UART_STOPBITS_1,UART_PARITY_NONE);							// modbus rtu init
	modbus_tcp_port_init(TCP_CLIENT);																																			// modbus tcp init
	printf("modbus init complete\r\n");
	/* Start thread modbus */
	osThreadDef(ModbusTask, StartModbusTask, osPriorityIdle, 0, 128);
  ModbusTaskHandle = osThreadCreate(osThread(ModbusTask), NULL);
	printf("start task handle modbus\r\n");
	HAL_ADC_Start_DMA(&hadc3, (uint32_t*) adc_buffer, 13);
  /* Infinite loop */
  for(;;)
  {
		/* update register analog input */
		for(uint8_t count = 0; count < 13; count++)
		{
			modbus_register_30000[count] = adc_buffer[count];
		}
		
		if(button_action != 0)
		{
			modbus_tcp_writemultiple_register(1,0,13,adc_buffer);
			button_action = 0;
			osDelay(300);
			modbus_tcp_check_input();
		}
		
		modbus_tcp_readholding_register(1,0,13);
		osDelay(300);
		modbus_tcp_check_input();
		osDelay(1);
		
		HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_0);
    osDelay(1000);
  }
  /* USER CODE END StartSysTask */
}

/* StartModbusTask function */
void StartModbusTask(void const * argument)
{
  /* USER CODE BEGIN StartModbusTask */
  /* Infinite loop */
  for(;;)
  {
//		modbus_acsii_check_input();
//		modbus_rtu_check_input();
//		modbus_tcp_check_input();
    osDelay(100);
  }
  /* USER CODE END StartModbusTask */
}

/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
