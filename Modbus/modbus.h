/** @file
  * 
  * @brief modbus ascii
  *
  */
#ifndef _MODBUS_H
#define _MODBUS_H

/*********************************************************************************
 * INCLUDE
 */
#include "modbus_ascii.h"
#include "modbus_rtu.h"
#include "modbus_tcp.h"
#include "modbus_constant.h"
#include "modbus_register.h"

#include "usart.h"

/*********************************************************************************
 * EXTERN
 */
extern uint8_t modbus_register_00000[MAX_COILS];
extern uint8_t modbus_register_10000[MAX_COILS];
extern uint16_t modbus_register_40000[MAX_REGISTES];
extern uint16_t modbus_register_30000[MAX_REGISTES];

#endif /* _MODBUS_H */
