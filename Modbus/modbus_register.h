/** @file
  * 
  * @brief modbus ascii
  *
  */
#ifndef _MODBUS_REGISTER_H
#define _MODBUS_REGISTER_H

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
void modbus_register_init(uint8_t add_slave_init, uint16_t number_register_init, uint16_t number_coils_init);

/**
 * @brief        
 * 
 * @param
 */
void modbus_register_input_update(uint16_t register_position, uint16_t value);

/**
 * @brief        
 * 
 * @param
 */
void modbus_register_output_update(uint16_t register_position, uint16_t value);

/**
 * @brief        
 * 
 * @param
 */
void modbus_coils_update(uint16_t coils_position, uint16_t value);

/**
 * @brief        
 * 
 * @param
 */
void modbus_discrete_input_update(uint16_t input_position, uint16_t value);
#endif /* _MODBUS_REGISTER_H */
