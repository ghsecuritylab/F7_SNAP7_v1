/** @file
  * 
  * @brief modbus TCP FUNCTION
  *
  */
#ifndef _MODBUS_TCP_FUNC_H
#define _MODBUS_TCP_FUNC_H

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

/*********************************************************************************
 * MACRO
 */
 
/*********************************************************************************
 * STATIC VARIABLE
 */

/*********************************************************************************
 * GLOBAL VARIABLE
 */

/*********************************************************************************
 * STATIC FUNCTION
 */

/*********************************************************************************
 * GLOBAL FUNCTION
 */
/**
 * @brief        
 * 
 * @param
 */
void modbus_tcp_read_coil(uint8_t add_slave_, uint16_t add_start_data_tcp, uint16_t number_coils_tcp);

/**
 * @brief        
 * 
 * @param
 */
void modbus_tcp_function1_response(uint16_t transaction, uint16_t add_start_data_tcp, uint16_t number_coils_tcp);

/**
 * @brief        
 * 
 * @param
 */
void modbus_tcp_readdiscrete_input(uint8_t add_slave_, uint16_t add_start_data_tcp, uint16_t number_coils_tcp);
/**
 * @brief        
 * 
 * @param
 */
void modbus_tcp_function2_response(uint16_t transaction, uint16_t add_start_data_tcp, uint16_t number_coils_tcp);

/**
 * @brief        
 * 
 * @param
 */
void modbus_tcp_readholding_register(uint8_t add_slave_, uint16_t add_start_register_tcp, uint16_t number_register_tcp);

/**
 * @brief        
 * 
 * @param
 */
void modbus_tcp_function3_response(uint16_t transaction, uint16_t add_start_register_tcp, uint16_t number_register_tcp);

/**
 * @brief        
 * 
 * @param
 */
void modbus_tcp_read_input_register(uint8_t add_slave_, uint16_t add_start_register_tcp, uint16_t number_register_tcp);
/**
 * @brief        
 * 
 * @param
 */
void modbus_tcp_function4_response(uint16_t transaction, uint16_t add_start_register_tcp, uint16_t number_register_tcp);

/**
 * @brief        
 * 
 * @param
 */ 
void modbus_tcp_writemultiple_coil(uint8_t add_slave_, uint16_t add_start_data_tcp, uint16_t number_coils_tcp, uint8_t* p_data);

/**
 * @brief        
 * 
 * @param
 */
void modbus_tcp_function15_response(uint16_t transaction, uint8_t add_slave_, uint16_t add_start_data_tcp, uint16_t number_coils_tcp);

/**
 * @brief        
 * 
 * @param
 */ 
void modbus_tcp_writemultiple_register(uint8_t add_slave_, uint16_t add_start_register_tcp, uint16_t number_register_tcp, uint16_t* p_data);

/**
 * @brief        
 * 
 * @param
 */
void modbus_tcp_function16_response(uint16_t transaction, uint8_t add_slave_, uint16_t add_start_register, uint16_t number_register);

/**
 * @brief        
 * 
 * @param
 */
uint16_t mb_tcp_read_transaction_id(void);

/**
 * @brief        
 * 
 * @param
 */
uint16_t mb_tcp_read_protocol_id(void);

/**
 * @brief        
 * 
 * @param
 */
uint16_t mb_tcp_read_message_length(void);
/**
 * @brief        
 * 
 * @param
 */
uint8_t mb_tcp_read_addslave(void);
/**
 * @brief        
 * 
 * @param
 */
uint8_t mb_tcp_read_function(void);

/**
 * @brief        
 * 
 * @param
 */
uint16_t mb_tcp_read_add_register_start(void);

/**
 * @brief        
 * 
 * @param
 */
uint16_t mb_tcp_read_number_register(void);

/**
 * @brief        
 * 
 * @param
 */
uint16_t mb_tcp_read_value_register(void);

/**
 * @brief        
 * 
 * @param
 */
uint8_t mb_tcp_read_byte_coil(void);

/**
 * @brief        
 * 
 * @param
 */
uint8_t mb_tcp_read_number_byte_follow(void);

/**
 * @brief        
 * 
 * @param
 */


#endif /* _MODBUS_ASCII_FUNC_H */
