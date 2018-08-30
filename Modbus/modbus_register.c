/** @file
  * 
  * @brief modbus tcp
  *
  */
	
/*********************************************************************************
 * INCLUDE
 */

/* user include */
#include "modbus_constant.h"
/*********************************************************************************
 * EXTERN
 */

/*********************************************************************************
 * MACRO
 */


/*********************************************************************************
 * 	GLOBAL VARIABLE
 */
/* add slave */
uint8_t add_slave;

/* number register */
uint16_t modbus_number_register;
uint16_t modbus_number_coil;
/* register modbus */
uint8_t modbus_register_00000[MAX_COILS];
uint8_t modbus_register_10000[MAX_COILS];
uint16_t modbus_register_40000[MAX_REGISTES];
uint16_t modbus_register_30000[MAX_REGISTES];

/**
 * @brief        
 * 
 * @param
 */
void modbus_register_init(uint8_t add_slave_init, uint16_t number_register_init, uint16_t number_coils_init)
{
	add_slave = add_slave_init;
	/* init number register */
	if(number_register_init > MAX_REGISTES)
	{
		/* error number register init */
		printf("error number register init\r\n");
	}
	else
	{
		modbus_number_register = number_register_init;
	}
	/* init number coils */
	if(number_coils_init > MAX_COILS)
	{
		/* error number register init */
		printf("error number coils init\r\n");
	}
	else
	{
		modbus_number_coil = number_coils_init;
	}
}
/**
 * @brief        
 * 
 * @param
 */
void modbus_register_input_update(uint16_t register_position, uint16_t value)
{
	modbus_register_30000[register_position] = value;
}

/**
 * @brief        
 * 
 * @param
 */
void modbus_register_output_update(uint16_t register_position, uint16_t value)
{
	modbus_register_40000[register_position] = value;
}

/**
 * @brief        
 * 
 * @param
 */
void modbus_coils_update(uint16_t coils_position, uint16_t value)
{
	modbus_register_00000[coils_position] = value;
}

/**
 * @brief        
 * 
 * @param
 */
void modbus_discrete_input_update(uint16_t input_position, uint16_t value)
{
	modbus_register_10000[input_position] = value;
}
