/** @file
  * 
  * @brief modbus ascii
  *
  */
#ifndef _MODBUS_ASCII_FUNC_H
#define _MODBUS_ASCII_FUNC_H

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
void modbus_acsii_read_coil(uint8_t add_slave_ascii, uint16_t add_start_data_ascii, uint16_t number_coils_ascii);

/**
 * @brief        
 * 
 * @param
 */
void modbus_ascii_function1_response(uint16_t add_start_data_ascii, uint16_t number_coils_ascii);

/**
 * @brief        
 * 
 * @param
 */
void modbus_ascii_readdiscrete_input(uint8_t add_slave_ascii, uint16_t add_start_data_ascii, uint16_t number_coils_ascii);

/**
 * @brief        
 * 
 * @param
 */
void modbus_ascii_function2_response(uint16_t add_start_data_ascii, uint16_t number_coils_ascii);

/**
 * @brief        
 * 
 * @param
 */
void modbus_acsii_readholding_register(uint8_t add_slave_ascii, uint16_t add_start_register_ascii, uint16_t number_register_ascii);

/**
 * @brief        
 * 
 * @param
 */
void modbus_acsii_function3_response(uint16_t add_start_register_ascii, uint16_t number_register_ascii);

/**
 * @brief        
 * 
 * @param
 */
void modbus_ascii_read_input_register(uint8_t add_slave_ascii, uint16_t add_start_register_ascii, uint16_t number_register_ascii);

/**
 * @brief        
 * 
 * @param
 */
void modbus_ascii_function4_response(uint16_t add_start_register_ascii, uint16_t number_register_ascii);

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
void modbus_ascii_function15_response(uint8_t add_slave_ascii, uint16_t add_start_register_ascii, uint16_t number_register_ascii);

/**
 * @brief        
 * 
 * @param
 */
void modbus_acsii_writemultiple_register(uint8_t add_slave_ascii, uint16_t add_start_register_ascii, uint16_t number_register_ascii, uint8_t* p_data);

/**
 * @brief        
 * 
 * @param
 */
void modbus_acsii_function16_response(uint8_t add_slave_ascii, uint16_t add_start_register_ascii, uint16_t number_register_ascii);

/**
 * @brief        
 * 
 * @param
 */
uint8_t mb_ascii_read_addslave(void);
/**
 * @brief        
 * 
 * @param
 */
uint8_t mb_ascii_read_function(void);

/**
 * @brief        
 * 
 * @param
 */
uint16_t mb_ascii_read_add_register_start(void);

/**
 * @brief        
 * 
 * @param
 */
uint16_t mb_ascii_read_number_register(void);

/**
 * @brief        
 * 
 * @param
 */
uint8_t mb_ascii_read_number_byte_follow(void);

/**
 * @brief        
 * 
 * @param
 */
uint16_t mb_ascii_read_value_to_register(void);

/**
 * @brief        
 * 
 * @param
 */
uint8_t mb_ascii_read_byte_coil(void);

/**
 * @brief        
 * 
 * @param
 */
uint8_t mb_ascii_read_lrc(void);

#endif /* _MODBUS_ASCII_FUNC_H */
