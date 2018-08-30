/** @file
  * 
  * @brief modbus ascii
  *
  */
#ifndef _MODBUS_RTU_FUNC_H
#define _MODBUS_RTU_FUNC_H

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
void modbus_rtu_read_coil(uint8_t add_slave_, uint16_t add_start_data_rtu, uint16_t number_coils_rtu);

/**
 * @brief        
 * 
 * @param
 */
void modbus_rtu_function1_response(uint16_t add_start_data_rtu,uint16_t number_coils_rtu);

 /**
 * @brief        
 * 
 * @param
 */
void modbus_rtu_readdiscrete_input(uint8_t add_slave_, uint16_t add_start_data_rtu, uint16_t number_coils_rtu);

/**
 * @brief        
 * 
 * @param
 */
void modbus_rtu_function2_response(uint16_t add_start_data_rtu,uint16_t number_coils_rtu);

/**
 * @brief        
 * 
 * @param
 */
void modbus_rtu_readholding_register(uint8_t add_slave_rtu_, uint16_t add_start_register_rtu, uint16_t number_register_rtu);

/**
 * @brief        
 * 
 * @param
 */
void modbus_rtu_function3_response(uint16_t add_start_register_rtu, uint16_t number_register_rtu);

/**
 * @brief        
 * 
 * @param
 */
void modbus_rtu_read_input_register(uint8_t add_slave_, uint16_t add_start_register_rtu, uint16_t number_register_rtu);

/**
 * @brief        
 * 
 * @param
 */
void modbus_rtu_function4_response(uint16_t add_start_register_rtu, uint16_t number_register_rtu);

/**
 * @brief        
 * 
 * @param
 */ 
void modbus_rtu_writemultiple_coil(uint8_t add_slave_, uint16_t add_start_data_rtu, uint16_t number_coils_rtu, uint8_t* p_data);

/**
 * @brief        
 * 
 * @param
 */
void modbus_rtu_function15_response(uint8_t add_slave_rtu_, uint16_t add_start_register_rtu, uint16_t number_register_rtu);

/**
 * @brief        
 * 
 * @param
 */
void modbus_rtu_writemultiple_register(uint8_t add_slave_, uint16_t add_start_register, uint16_t number_register, uint8_t* p_data);

/**
 * @brief        
 * 
 * @param
 */
void modbus_rtu_function16_response(uint8_t add_slave_rtu_, uint16_t add_start_register_rtu, uint16_t number_register_rtu);

/**
 * @brief        
 * 
 * @param
 */
uint8_t mb_rtu_read_addslave(void);

/**
 * @brief        
 * 
 * @param
 */
uint8_t mb_rtu_read_function(void);

/**
 * @brief        
 * 
 * @param
 */
uint16_t mb_rtu_read_add_register_start(void);

/**
 * @brief        
 * 
 * @param
 */
uint16_t mb_rtu_read_number_register(void);

/**
 * @brief        
 * 
 * @param
 */
uint16_t mb_rtu_read_value_register(void);

/**
 * @brief        
 * 
 * @param
 */
uint8_t mb_rtu_read_byte_coil(void);

/**
 * @brief        
 * 
 * @param
 */
uint8_t mb_rtu_read_byte_follow(void);
/**
 * @brief        
 * 
 * @param
 */
uint16_t mb_rtu_read_crc(void);
#endif /* _MODBUS_RTU_FUNC_H */
