/** @file
  * 
  * @brief modbus ascii
  *
  */
#ifndef _MODBUS_CONSTANT_H
#define _MODBUS_CONSTANT_H

/*********************************************************************************
 * INCLUDE
 */
 
#include "stm32f7xx_hal.h"
#include <stdio.h>
#include <stdbool.h>

/*********************************************************************************
 * TYPEDEFS
 */


/*********************************************************************************
 * DEFINE
 */
/* LED_TCP_ERROR */
#define LED_TCP_ERROR_PORT        GPIOB
#define LED_TCP_ERROR_PIN         GPIO_PIN_14

#define LED_TCP_ERROR_ON    			HAL_GPIO_WritePin(LED_TCP_ERROR_PORT, LED_TCP_ERROR_PIN, GPIO_PIN_SET);
#define LED_TCP_ERROR_OFF    			HAL_GPIO_WritePin(LED_TCP_ERROR_PORT, LED_TCP_ERROR_PIN, GPIO_PIN_RESET);


/* MODBUS DEFINE */
#define BROADCAST        0x00
#define TCP_CLIENT       0x01
#define TCP_SERVER       0x02

/* REGISTER SIZE */
#define MAX_BUFFER_RX     100
#define MAX_TX_MODBUS     200
#define MAX_COILS         200
#define MAX_REGISTES      200

/* FUNCTION */
#define FUNCTION_01      0x01
#define FUNCTION_02      0x02
#define FUNCTION_03      0x03
#define FUNCTION_04      0x04
#define FUNCTION_05      0x05
#define FUNCTION_06      0x06
#define FUNCTION_15      0x0F
#define FUNCTION_16      0x10

/* EXCEPTION RESPONSES */
#define ILLEGAL_FUNCTION                           0x01
#define ILLEGAL_DATA_ADDRESS                       0x02
#define ILLEGAL_DATA_VALUE                         0x03
#define SLAVE_DEVICE_FAILURE                       0x04
#define ACKNOWLEDGE                                0x05
#define SLAVE_DEVICE_BUSY                          0x06
#define NEGATIVE_ACKNOWLEDGE                       0x07
#define MEMORY_PARITY_ERROR                        0x08
#define GATEWAY_PATH_UNAVAILABLE	                 0x0A
#define GATEWAY_TARGET_DEVICE_FAILED_TORESPOND	   0x0B


/* FUNCTION MODE */
#define RECEIVE_REQUEST   0x01
#define RECEIVE_RESPONSE  0x02



#endif /* _MODBUS_CONSTANT_H */

