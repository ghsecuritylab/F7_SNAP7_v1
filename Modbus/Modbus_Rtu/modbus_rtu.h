/** @file
  * 
  * @brief modbus ascii
  *
  */
#ifndef _MODBUS_RTU_H
#define _MODBUS_RTU_H

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
void modbus_rtu_port_init(UART_HandleTypeDef *huart,uint32_t BaudRate,uint32_t WordLength,uint32_t StopBits,uint32_t Parity);

/**
 * @brief        
 * 
 * @param
 */
void modbus_rtu_get_input(UART_HandleTypeDef *huart);

/**
 * @brief        
 * 
 * @param
 */
void modbus_rtu_check_input(void);

/**
 * @brief        
 * 
 * @param
 */
extern void modbus_rtu_read_coil(uint8_t add_slave_, uint16_t add_start_data_rtu, uint16_t number_coils_rtu);

 /**
 * @brief        
 * 
 * @param
 */
extern void modbus_rtu_readdiscrete_input(uint8_t add_slave_, uint16_t add_start_data_rtu, uint16_t number_coils_rtu);

/**
 * @brief        
 * 
 * @param
 */
extern void modbus_rtu_readholding_register(uint8_t add_slave_, uint16_t add_start_register, uint16_t number_register);

/**
 * @brief        
 * 
 * @param
 */
extern void modbus_rtu_read_input_register(uint8_t add_slave_, uint16_t add_start_register_rtu, uint16_t number_register_rtu);

/**
 * @brief        
 * 
 * @param
 */ 
extern void modbus_rtu_writemultiple_coil(uint8_t add_slave_, uint16_t add_start_data_rtu, uint16_t number_coils_rtu, uint8_t* p_data);
/**
 * @brief        
 * 
 * @param
 */
extern void modbus_rtu_writemultiple_register(uint8_t add_slave, uint16_t add_start_register, uint16_t number_register, uint8_t* p_data);

#endif /* _MODBUS_RTU_H */
