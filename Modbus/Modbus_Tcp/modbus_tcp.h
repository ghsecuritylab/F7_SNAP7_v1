/** @file
  * 
  * @brief modbus tcp
  *
  */
#ifndef _MODBUS_TCP_H
#define _MODBUS_TCP_H

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
void modbus_tcp_port_init(uint8_t tcp_modde);

/**
 * @brief        
 * 
 * @param
 */
void modbus_tcp_check_input(void);

/**
 * @brief        
 * 
 * @param
 */
extern void modbus_tcp_read_coil(uint8_t add_slave_, uint16_t add_start_data_tcp, uint16_t number_coils_tcp);
 /**
 * @brief        
 * 
 * @param
 */
extern void modbus_tcp_readdiscrete_input(uint8_t add_slave_, uint16_t add_start_data_tcp, uint16_t number_coils_tcp);

/**
 * @brief        
 * 
 * @param
 */
extern void modbus_tcp_readholding_register(uint8_t add_slave_, uint16_t add_start_register_tcp, uint16_t number_register_tcp);

/**
 * @brief        
 * 
 * @param
 */
extern void modbus_tcp_read_input_register(uint8_t add_slave_, uint16_t add_start_register_tcp, uint16_t number_register_tcp);

/**
 * @brief        
 * 
 * @param
 */
extern void modbus_tcp_writemultiple_coil(uint8_t add_slave_, uint16_t add_start_data_tcp, uint16_t number_coils_tcp, uint8_t* p_data);
/**
 * @brief        
 * 
 * @param
 */
extern void modbus_tcp_writemultiple_register(uint8_t add_slave_, uint16_t add_start_register_tcp, uint16_t number_register_tcp, uint16_t* p_data);
#endif /* _MODBUS_TCP_H */
