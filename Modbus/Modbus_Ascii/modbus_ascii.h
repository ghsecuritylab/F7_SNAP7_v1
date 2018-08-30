/** @file
  * 
  * @brief modbus ascii
  *
  */
#ifndef _MODBUS_ASCII_H
#define _MODBUS_ASCII_H

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

/**
 * @brief        
 * 
 * @param
 */
void modbus_ascii_port_init(UART_HandleTypeDef *huart,uint32_t BaudRate,uint32_t WordLength,uint32_t StopBits,uint32_t Parity);

/**
 * @brief        
 * 
 * @param
 */
void modbus_acsii_get_input(UART_HandleTypeDef *huart);

/**
 * @brief        
 * 
 * @param
 */
void modbus_acsii_check_input(void);

/**
 * @brief        
 * 
 * @param
 */
extern void modbus_acsii_read_coil(uint8_t add_slave_ascii, uint16_t add_start_data_ascii, uint16_t number_coils_ascii);

/**
 * @brief        
 * 
 * @param
 */
extern void modbus_ascii_readdiscrete_input(uint8_t add_slave_ascii, uint16_t add_start_data_ascii, uint16_t number_coils_ascii);

/**
 * @brief        
 * 
 * @param
 */
extern void modbus_acsii_readholding_register(uint8_t add_slave_ascii, uint16_t add_start_register, uint16_t number_register);

/**
 * @brief        
 * 
 * @param
 */
extern void modbus_ascii_read_input_register(uint8_t add_slave_ascii, uint16_t add_start_register_ascii, uint16_t number_register_ascii);

/**
 * @brief        
 * 
 * @param
 */ 
void modbus_ascii_writemultiple_coil(uint8_t add_slave_ascii, uint16_t add_start_data_ascii, uint16_t number_coils_ascii, uint8_t* p_data);
/**
 * @brief        
 * 
 * @param
 */
extern void modbus_acsii_writemultiple_register(uint8_t add_slave_ascii, uint16_t add_start_register, uint16_t number_register, uint8_t* p_data);

#endif /* _MODBUS_ASCII_H */
