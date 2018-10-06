/** @file
  * 
  * @brief modbus ascii
  *
  */
	
/*********************************************************************************
 * INCLUDE
 */
/* system include */
#include "usart.h"
//#include "cmsis_os.h"
/* user include */
#include "modbus_rtu.h"
#include "modbus_rtu_func.h"
#include "modbus_constant.h"

#include "circular_buffer.h"
/*********************************************************************************
 * MACRO
 */
 
/*********************************************************************************
 * STATIC VARIABLE
 */

/*********************************************************************************
 * EXTERN
 */
/* address slave */
extern uint8_t add_slave;
/* init register modbus */
extern uint16_t modbus_number_register;
/* init coil modbus */
extern uint16_t modbus_number_coil;
/* register of modbus */
extern uint8_t modbus_register_00000[MAX_REGISTES];
extern uint8_t modbus_register_10000[MAX_REGISTES];
extern uint16_t modbus_register_30000[MAX_REGISTES];
extern uint16_t modbus_register_40000[MAX_REGISTES];

extern UART_HandleTypeDef*  DEBUG;
extern UART_HandleTypeDef* uart_modbus_rtu;
extern circular_buf_t cbuf_rtu;

extern uint8_t buf_rx_rtu[MAX_BUFFER_RX];
/* flag receive request or response */
extern uint8_t flag_request_or_response_func1_rtu;
extern uint8_t flag_request_or_response_func2_rtu;
extern uint8_t flag_request_or_response_func3_rtu;
extern uint8_t flag_request_or_response_func4_rtu;
extern uint8_t flag_request_or_response_func15_rtu;
extern uint8_t flag_request_or_response_func16_rtu;
uint8_t buf_tx_modbus_rtu[MAX_TX_MODBUS];

/* register of modbus */

/*********************************************************************************
 * STATIC FUNCTION
 */
static uint8_t read_uint8_queue(void);
static uint16_t read_uint16_queue(void);
static uint16_t calculateCRC_out(uint8_t buf_size);
/*********************************************************************************
 * GLOBAL FUNCTION
 */

/**
 * @brief        
 * 
 * @param
 */
void modbus_rtu_read_coil(uint8_t add_slave_, uint16_t add_start_data_rtu, uint16_t number_coils_rtu)
{
	uint16_t count_data_tx_modbus_rtu = 0;
	uint16_t crc_output;
	/* byte add slave */
	buf_tx_modbus_rtu[count_data_tx_modbus_rtu++] = add_slave_;
	/* byte function */
	buf_tx_modbus_rtu[count_data_tx_modbus_rtu++] = 0x01;
	/* byte add start register */
	buf_tx_modbus_rtu[count_data_tx_modbus_rtu++] = add_start_data_rtu / 256;
	buf_tx_modbus_rtu[count_data_tx_modbus_rtu++] = (uint8_t) add_start_data_rtu % 256;
	/* byte number register */
	buf_tx_modbus_rtu[count_data_tx_modbus_rtu++] = number_coils_rtu / 256;;
	buf_tx_modbus_rtu[count_data_tx_modbus_rtu++] = (uint8_t) number_coils_rtu % 256;
	/* calulator CRC*/
	crc_output = calculateCRC_out(count_data_tx_modbus_rtu);
	buf_tx_modbus_rtu[count_data_tx_modbus_rtu++] = crc_output / 256;
	buf_tx_modbus_rtu[count_data_tx_modbus_rtu++] = (uint8_t) crc_output % 256;
	/* Send Data */
	HAL_UART_Transmit(uart_modbus_rtu,(uint8_t *)buf_tx_modbus_rtu,count_data_tx_modbus_rtu,10);
//	HAL_UART_Transmit(DEBUG,(uint8_t *)buf_tx_modbus_rtu,count_data_tx_modbus_rtu,10);
	flag_request_or_response_func1_rtu = RECEIVE_RESPONSE;
}

/**
 * @brief        
 * 
 * @param
 */
void modbus_rtu_function1_response(uint16_t add_start_data_rtu, uint16_t number_coils_rtu)
{
	uint16_t count_data_tx_modbus_rtu = 0;
	uint16_t buf_CRC = 0;
	uint16_t count_number_coils_rtu_send = 0;
	uint16_t add_start_data_send = 0;
	uint16_t number_byte_data_rtu = 0;
	uint16_t check_coil_rtu = 0;
	uint8_t byte_data_buf = 0;
	if(add_start_data_rtu > modbus_number_coil)
	{
		/* error add register start */
		printf("error add register start\r\n");
	}
	else
	{
		add_start_data_send = add_start_data_rtu;
		if(number_coils_rtu > modbus_number_coil)
		{
			/* error number register */
			printf("error number register\r\n");
		}
		else
		{
			if(add_start_data_rtu +  number_coils_rtu > modbus_number_coil)
			{
				/* error register */
				printf("error register\r\n");
			}
			else /* not error */
			{
				/* calutation number byte data */
				check_coil_rtu = (uint8_t) number_coils_rtu % 8;
				if(check_coil_rtu != 0) number_byte_data_rtu = ((number_coils_rtu / 8) + 1);
				else number_byte_data_rtu = (number_coils_rtu / 8);
				/* byte add slave */
				buf_tx_modbus_rtu[count_data_tx_modbus_rtu++] = add_slave;
				/* byte func */
				buf_tx_modbus_rtu[count_data_tx_modbus_rtu++] = 0x01;
				/* length data */
				buf_tx_modbus_rtu[count_data_tx_modbus_rtu++] = number_byte_data_rtu;
				/* add byte data */
				for(uint16_t count_data = 0; count_data < number_byte_data_rtu; count_data++)
				{
					byte_data_buf = 0;
					if(count_number_coils_rtu_send++ < number_coils_rtu && modbus_register_00000[add_start_data_send++] == 1) byte_data_buf = byte_data_buf | 0x01;
//					printf("count_number_coils_tcp_send is %d\r\n",count_number_coils_tcp_send);
					if(count_number_coils_rtu_send++ < number_coils_rtu && modbus_register_00000[add_start_data_send++] == 1) byte_data_buf = byte_data_buf | 0x02;
//					printf("count_number_coils_tcp_send is %d\r\n",count_number_coils_tcp_send);
					if(count_number_coils_rtu_send++ < number_coils_rtu && modbus_register_00000[add_start_data_send++] == 1) byte_data_buf = byte_data_buf | 0x04;
//					printf("count_number_coils_tcp_send is %d\r\n",count_number_coils_tcp_send);
					if(count_number_coils_rtu_send++ < number_coils_rtu && modbus_register_00000[add_start_data_send++] == 1) byte_data_buf = byte_data_buf | 0x08;
//					printf("count_number_coils_tcp_send is %d\r\n",count_number_coils_tcp_send);
					if(count_number_coils_rtu_send++ < number_coils_rtu && modbus_register_00000[add_start_data_send++] == 1) byte_data_buf = byte_data_buf | 0x10;
//					printf("count_number_coils_tcp_send is %d\r\n",count_number_coils_tcp_send);
					if(count_number_coils_rtu_send++ < number_coils_rtu && modbus_register_00000[add_start_data_send++] == 1) byte_data_buf = byte_data_buf | 0x20;
//					printf("count_number_coils_tcp_send is %d\r\n",count_number_coils_tcp_send);
					if(count_number_coils_rtu_send++ < number_coils_rtu && modbus_register_00000[add_start_data_send++] == 1) byte_data_buf = byte_data_buf | 0x40;
//					printf("count_number_coils_tcp_send is %d\r\n",count_number_coils_tcp_send);
					if(count_number_coils_rtu_send++ < number_coils_rtu && modbus_register_00000[add_start_data_send++] == 1) byte_data_buf = byte_data_buf | 0x80;
//					printf("count_number_coils_tcp_send is %d\r\n",count_number_coils_tcp_send);
					buf_tx_modbus_rtu[count_data_tx_modbus_rtu++] = byte_data_buf;
				}
				/* calulator CRC*/
				buf_CRC = calculateCRC_out(count_data_tx_modbus_rtu);
				buf_tx_modbus_rtu[count_data_tx_modbus_rtu++] = buf_CRC /256;
				buf_tx_modbus_rtu[count_data_tx_modbus_rtu++] = (uint8_t )buf_CRC % 256;

				HAL_UART_Transmit(uart_modbus_rtu,(uint8_t *)buf_tx_modbus_rtu,count_data_tx_modbus_rtu,10);
//				HAL_UART_Transmit(DEBUG,(uint8_t *)buf_tx_modbus_rtu,count_data_tx_modbus_rtu,10);
			}
		}
	}
}

/**
 * @brief        
 * 
 * @param
 */
void modbus_rtu_readdiscrete_input(uint8_t add_slave_, uint16_t add_start_data_rtu, uint16_t number_coils_rtu)
{
	uint16_t count_data_tx_modbus_rtu = 0;
	uint16_t crc_output;
	/* byte add slave */
	buf_tx_modbus_rtu[count_data_tx_modbus_rtu++] = add_slave_;
	/* byte function */
	buf_tx_modbus_rtu[count_data_tx_modbus_rtu++] = 0x02;
	/* byte add start register */
	buf_tx_modbus_rtu[count_data_tx_modbus_rtu++] = add_start_data_rtu / 256;
	buf_tx_modbus_rtu[count_data_tx_modbus_rtu++] = (uint8_t) add_start_data_rtu % 256;
	/* byte number register */
	buf_tx_modbus_rtu[count_data_tx_modbus_rtu++] = number_coils_rtu / 256;;
	buf_tx_modbus_rtu[count_data_tx_modbus_rtu++] = (uint8_t) number_coils_rtu % 256;
	/* calulator CRC*/
	crc_output = calculateCRC_out(count_data_tx_modbus_rtu);
	buf_tx_modbus_rtu[count_data_tx_modbus_rtu++] = crc_output / 256;
	buf_tx_modbus_rtu[count_data_tx_modbus_rtu++] = (uint8_t) crc_output % 256;
	/* Send Data */
	HAL_UART_Transmit(uart_modbus_rtu,(uint8_t *)buf_tx_modbus_rtu,count_data_tx_modbus_rtu,10);
//	HAL_UART_Transmit(DEBUG,(uint8_t *)buf_tx_modbus_rtu,count_data_tx_modbus_rtu,10);
	flag_request_or_response_func2_rtu = RECEIVE_RESPONSE;
}
/**
 * @brief        
 * 
 * @param
 */
void modbus_rtu_function2_response(uint16_t add_start_data_rtu,uint16_t number_coils_rtu)
{
	uint16_t count_data_tx_modbus_rtu = 0;
	uint16_t buf_CRC = 0;
	uint16_t count_number_coils_rtu_send = 0;
	uint16_t add_start_data_send = 0;
	uint16_t number_byte_data_rtu = 0;
	uint16_t check_coil_rtu = 0;
	uint8_t byte_data_buf = 0;
	if(add_start_data_rtu > modbus_number_coil)
	{
		/* error add register start */
		printf("error add register start\r\n");
	}
	else
	{
		add_start_data_send = add_start_data_rtu;
		if(number_coils_rtu > modbus_number_coil)
		{
			/* error number register */
			printf("error number register\r\n");
		}
		else
		{
			if(add_start_data_rtu +  number_coils_rtu > modbus_number_coil)
			{
				/* error register */
				printf("error register\r\n");
			}
			else /* not error */
			{
				/* calutation number byte data */
				check_coil_rtu = (uint8_t) number_coils_rtu % 8;
				if(check_coil_rtu != 0) number_byte_data_rtu = ((number_coils_rtu / 8) + 1);
				else number_byte_data_rtu = (number_coils_rtu / 8);
				/* byte add slave */
				buf_tx_modbus_rtu[count_data_tx_modbus_rtu++] = add_slave;
				/* byte func */
				buf_tx_modbus_rtu[count_data_tx_modbus_rtu++] = 0x02;
				/* length data */
				buf_tx_modbus_rtu[count_data_tx_modbus_rtu++] = number_byte_data_rtu;
				/* add byte data */
				for(uint16_t count_data = 0; count_data < number_byte_data_rtu; count_data++)
				{
					byte_data_buf = 0;
					if(count_number_coils_rtu_send++ < number_coils_rtu && modbus_register_10000[add_start_data_send++] == 1) byte_data_buf = byte_data_buf | 0x01;
//					printf("count_number_coils_tcp_send is %d\r\n",count_number_coils_tcp_send);
					if(count_number_coils_rtu_send++ < number_coils_rtu && modbus_register_10000[add_start_data_send++] == 1) byte_data_buf = byte_data_buf | 0x02;
//					printf("count_number_coils_tcp_send is %d\r\n",count_number_coils_tcp_send);
					if(count_number_coils_rtu_send++ < number_coils_rtu && modbus_register_10000[add_start_data_send++] == 1) byte_data_buf = byte_data_buf | 0x04;
//					printf("count_number_coils_tcp_send is %d\r\n",count_number_coils_tcp_send);
					if(count_number_coils_rtu_send++ < number_coils_rtu && modbus_register_10000[add_start_data_send++] == 1) byte_data_buf = byte_data_buf | 0x08;
//					printf("count_number_coils_tcp_send is %d\r\n",count_number_coils_tcp_send);
					if(count_number_coils_rtu_send++ < number_coils_rtu && modbus_register_10000[add_start_data_send++] == 1) byte_data_buf = byte_data_buf | 0x10;
//					printf("count_number_coils_tcp_send is %d\r\n",count_number_coils_tcp_send);
					if(count_number_coils_rtu_send++ < number_coils_rtu && modbus_register_10000[add_start_data_send++] == 1) byte_data_buf = byte_data_buf | 0x20;
//					printf("count_number_coils_tcp_send is %d\r\n",count_number_coils_tcp_send);
					if(count_number_coils_rtu_send++ < number_coils_rtu && modbus_register_10000[add_start_data_send++] == 1) byte_data_buf = byte_data_buf | 0x40;
//					printf("count_number_coils_tcp_send is %d\r\n",count_number_coils_tcp_send);
					if(count_number_coils_rtu_send++ < number_coils_rtu && modbus_register_10000[add_start_data_send++] == 1) byte_data_buf = byte_data_buf | 0x80;
//					printf("count_number_coils_tcp_send is %d\r\n",count_number_coils_tcp_send);
					buf_tx_modbus_rtu[count_data_tx_modbus_rtu++] = byte_data_buf;
				}
				/* calulator CRC*/
				buf_CRC = calculateCRC_out(count_data_tx_modbus_rtu);
				buf_tx_modbus_rtu[count_data_tx_modbus_rtu++] = buf_CRC /256;
				buf_tx_modbus_rtu[count_data_tx_modbus_rtu++] = (uint8_t )buf_CRC % 256;

				HAL_UART_Transmit(uart_modbus_rtu,(uint8_t *)buf_tx_modbus_rtu,count_data_tx_modbus_rtu,10);
//				HAL_UART_Transmit(DEBUG,(uint8_t *)buf_tx_modbus_rtu,count_data_tx_modbus_rtu,10);
			}
		}
	}
}
/**
 * @brief        
 * 
 * @param
 */
void modbus_rtu_readholding_register(uint8_t add_slave_rtu_, uint16_t add_start_register_rtu, uint16_t number_register_rtu)
{
	uint16_t count_data_tx_modbus_rtu = 0;
	uint16_t crc_output;
	/* byte add slave */
	buf_tx_modbus_rtu[count_data_tx_modbus_rtu++] = add_slave_rtu_;
	/* byte function */
	buf_tx_modbus_rtu[count_data_tx_modbus_rtu++] = 0x03;
	/* byte add start register */
	buf_tx_modbus_rtu[count_data_tx_modbus_rtu++] = add_start_register_rtu / 256;
	buf_tx_modbus_rtu[count_data_tx_modbus_rtu++] = (uint8_t) add_start_register_rtu % 256;
	/* byte number register */
	buf_tx_modbus_rtu[count_data_tx_modbus_rtu++] = number_register_rtu / 256;;
	buf_tx_modbus_rtu[count_data_tx_modbus_rtu++] = (uint8_t) number_register_rtu % 256;
	/* calulator CRC*/
	crc_output = calculateCRC_out(count_data_tx_modbus_rtu);
	buf_tx_modbus_rtu[count_data_tx_modbus_rtu++] = crc_output / 256;
	buf_tx_modbus_rtu[count_data_tx_modbus_rtu++] = (uint8_t) crc_output % 256;
	/* Send Data */
	HAL_UART_Transmit(uart_modbus_rtu,(uint8_t *)buf_tx_modbus_rtu,count_data_tx_modbus_rtu,10);
//	HAL_UART_Transmit(DEBUG,(uint8_t *)buf_tx_modbus_rtu,count_data_tx_modbus_rtu,10);
	flag_request_or_response_func3_rtu = RECEIVE_RESPONSE;
}
/**
 * @brief        
 * 
 * @param
 */
void modbus_rtu_function3_response(uint16_t add_start_register_rtu, uint16_t number_register_rtu)
{
	uint16_t count_data_tx_modbus_rtu = 0;
	uint16_t buf_CRC = 0;
	uint16_t add_start_register_send = 0;
	if(add_start_register_rtu > modbus_number_register)
	{
		/* error add register start */
		printf("error add register start\r\n");
	}
	else
	{
		add_start_register_send = add_start_register_rtu;
		if(number_register_rtu > modbus_number_register)
		{
			/* error number register */
			printf("error number register\r\n");
		}
		else
		{
			if(add_start_register_rtu +  number_register_rtu > modbus_number_register)
			{
				/* error register */
				printf("error register\r\n");
			}
			else /* not error */
			{
				/* byte add slave */
				buf_tx_modbus_rtu[count_data_tx_modbus_rtu++] = add_slave;
				/* byte func */
				buf_tx_modbus_rtu[count_data_tx_modbus_rtu++] = 0x03;
				/* length data */
				buf_tx_modbus_rtu[count_data_tx_modbus_rtu++] = number_register_rtu*2;
				/* byte data */
				for(uint16_t count_data = 0; count_data < number_register_rtu; count_data++)
				{
					buf_tx_modbus_rtu[count_data_tx_modbus_rtu++] = modbus_register_40000[add_start_register_send + count_data] / 256;
					buf_tx_modbus_rtu[count_data_tx_modbus_rtu++] = (uint8_t) modbus_register_40000[add_start_register_send + count_data] % 256;
				}
				/* calulator CRC*/
				buf_CRC = calculateCRC_out(count_data_tx_modbus_rtu);
				buf_tx_modbus_rtu[count_data_tx_modbus_rtu++] = buf_CRC /256;
				buf_tx_modbus_rtu[count_data_tx_modbus_rtu++] = (uint8_t )buf_CRC % 256;

				HAL_UART_Transmit(uart_modbus_rtu,(uint8_t *)buf_tx_modbus_rtu,count_data_tx_modbus_rtu,10);
//				HAL_UART_Transmit(DEBUG,(uint8_t *)buf_tx_modbus_rtu,count_data_tx_modbus_rtu,10);
			}
		}
	}
}

/**
 * @brief        
 * 
 * @param
 */
void modbus_rtu_read_input_register(uint8_t add_slave_, uint16_t add_start_register_rtu, uint16_t number_register_rtu)
{
	uint16_t count_data_tx_modbus_rtu = 0;
	uint16_t crc_output;
	/* byte add slave */
	buf_tx_modbus_rtu[count_data_tx_modbus_rtu++] = add_slave_;
	/* byte function */
	buf_tx_modbus_rtu[count_data_tx_modbus_rtu++] = 0x02;
	/* byte add start register */
	buf_tx_modbus_rtu[count_data_tx_modbus_rtu++] = add_start_register_rtu / 256;
	buf_tx_modbus_rtu[count_data_tx_modbus_rtu++] = (uint8_t) add_start_register_rtu % 256;
	/* byte number register */
	buf_tx_modbus_rtu[count_data_tx_modbus_rtu++] = number_register_rtu / 256;;
	buf_tx_modbus_rtu[count_data_tx_modbus_rtu++] = (uint8_t) number_register_rtu % 256;
	/* calulator CRC*/
	crc_output = calculateCRC_out(count_data_tx_modbus_rtu);
	buf_tx_modbus_rtu[count_data_tx_modbus_rtu++] = crc_output / 256;
	buf_tx_modbus_rtu[count_data_tx_modbus_rtu++] = (uint8_t) crc_output % 256;
	/* Send Data */
	HAL_UART_Transmit(uart_modbus_rtu,(uint8_t *)buf_tx_modbus_rtu,count_data_tx_modbus_rtu,10);
//	HAL_UART_Transmit(DEBUG,(uint8_t *)buf_tx_modbus_rtu,count_data_tx_modbus_rtu,10);
	flag_request_or_response_func4_rtu = RECEIVE_RESPONSE;
}

/**
 * @brief        
 * 
 * @param
 */
void modbus_rtu_function4_response(uint16_t add_start_register_rtu, uint16_t number_register_rtu)
{
	uint16_t count_data_tx_modbus_rtu = 0;
	uint16_t buf_CRC = 0;
	uint16_t add_start_register_send = 0;
	if(add_start_register_rtu > modbus_number_register)
	{
		/* error add register start */
		printf("error add register start\r\n");
	}
	else
	{
		add_start_register_send = add_start_register_rtu;
		if(number_register_rtu > modbus_number_register)
		{
			/* error number register */
			printf("error number register\r\n");
		}
		else
		{
			if(add_start_register_rtu +  number_register_rtu > modbus_number_register)
			{
				/* error register */
				printf("error register\r\n");
			}
			else /* not error */
			{
				/* byte add slave */
				buf_tx_modbus_rtu[count_data_tx_modbus_rtu++] = add_slave;
				/* byte func */
				buf_tx_modbus_rtu[count_data_tx_modbus_rtu++] = 0x04;
				/* length data */
				buf_tx_modbus_rtu[count_data_tx_modbus_rtu++] = number_register_rtu*2;
				/* byte data */
				for(uint16_t count_data = 0; count_data < number_register_rtu; count_data++)
				{
					buf_tx_modbus_rtu[count_data_tx_modbus_rtu++] = modbus_register_30000[add_start_register_send + count_data] / 256;
					buf_tx_modbus_rtu[count_data_tx_modbus_rtu++] = (uint8_t) modbus_register_30000[add_start_register_send + count_data] % 256;
				}
				/* calulator CRC*/
				buf_CRC = calculateCRC_out(count_data_tx_modbus_rtu);
				buf_tx_modbus_rtu[count_data_tx_modbus_rtu++] = buf_CRC /256;
				buf_tx_modbus_rtu[count_data_tx_modbus_rtu++] = (uint8_t )buf_CRC % 256;

				HAL_UART_Transmit(uart_modbus_rtu,(uint8_t *)buf_tx_modbus_rtu,count_data_tx_modbus_rtu,10);
//				HAL_UART_Transmit(DEBUG,(uint8_t *)buf_tx_modbus_rtu,count_data_tx_modbus_rtu,10);
			}
		}
	}
}

/**
 * @brief        
 * 
 * @param
 */ 
void modbus_rtu_writemultiple_coil(uint8_t add_slave_, uint16_t add_start_data_rtu, uint16_t number_coils_rtu, uint8_t* p_data)
{
	uint16_t count_data_tx_modbus_rtu = 0;
	uint16_t crc_output;
	uint16_t number_byte_data_rtu = 0;
	uint16_t check_coil_rtu = 0;
	/* byte add slave */
	buf_tx_modbus_rtu[count_data_tx_modbus_rtu++] = add_slave_;
	/* byte function */
	buf_tx_modbus_rtu[count_data_tx_modbus_rtu++] = 0x10;
	/* byte add start register */
	buf_tx_modbus_rtu[count_data_tx_modbus_rtu++] = add_start_data_rtu / 256;
	buf_tx_modbus_rtu[count_data_tx_modbus_rtu++] = (uint8_t) add_start_data_rtu % 256;
	/* byte number register */
	buf_tx_modbus_rtu[count_data_tx_modbus_rtu++] = number_coils_rtu / 256;;
	buf_tx_modbus_rtu[count_data_tx_modbus_rtu++] = (uint8_t) number_coils_rtu % 256;
	/* byte length */
	/* calutation number byte data */
	check_coil_rtu = (uint8_t) number_coils_rtu % 8;
	if(check_coil_rtu != 0) number_byte_data_rtu = ((number_coils_rtu / 8) + 1);
	else number_byte_data_rtu = (number_coils_rtu / 8);
	buf_tx_modbus_rtu[count_data_tx_modbus_rtu++] = number_byte_data_rtu;
	/* byte data*/
	for(uint16_t count_data = 0; count_data < number_byte_data_rtu; count_data++)
	{
		buf_tx_modbus_rtu[count_data_tx_modbus_rtu++] = (*p_data);
		p_data++;
	}
	/* calulator CRC*/
	crc_output = calculateCRC_out(count_data_tx_modbus_rtu);
	buf_tx_modbus_rtu[count_data_tx_modbus_rtu++] = crc_output / 256;
	buf_tx_modbus_rtu[count_data_tx_modbus_rtu++] = (uint8_t) crc_output % 256;
	/* Send Data */
	HAL_UART_Transmit(uart_modbus_rtu,(uint8_t *)buf_tx_modbus_rtu,count_data_tx_modbus_rtu,10);
//	HAL_UART_Transmit(DEBUG,(uint8_t *)buf_tx_modbus_rtu,count_data_tx_modbus_rtu,10);
	flag_request_or_response_func15_rtu = RECEIVE_RESPONSE;
}

/**
 * @brief        
 * 
 * @param
 */
void modbus_rtu_function15_response(uint8_t add_slave_rtu_, uint16_t add_start_register_rtu, uint16_t number_register_rtu)
{
	uint16_t count_data_tx_modbus_rtu = 0;
	uint16_t crc_output;
	/* byte add slave */
	buf_tx_modbus_rtu[count_data_tx_modbus_rtu++] = add_slave_rtu_;
	/* byte function */
	buf_tx_modbus_rtu[count_data_tx_modbus_rtu++] = 0x0F;
	/* byte add start register */
	buf_tx_modbus_rtu[count_data_tx_modbus_rtu++] = add_start_register_rtu / 256;
	buf_tx_modbus_rtu[count_data_tx_modbus_rtu++] = (uint8_t) add_start_register_rtu % 256;
	/* byte number register */
	buf_tx_modbus_rtu[count_data_tx_modbus_rtu++] = number_register_rtu / 256;;
	buf_tx_modbus_rtu[count_data_tx_modbus_rtu++] = (uint8_t) number_register_rtu % 256;
	/* calulator CRC*/
	crc_output = calculateCRC_out(count_data_tx_modbus_rtu);
	buf_tx_modbus_rtu[count_data_tx_modbus_rtu++] = crc_output / 256;
	buf_tx_modbus_rtu[count_data_tx_modbus_rtu++] = (uint8_t) crc_output % 256;
	/* Send Data */
	HAL_UART_Transmit(uart_modbus_rtu,(uint8_t *)buf_tx_modbus_rtu,count_data_tx_modbus_rtu,10);
//	HAL_UART_Transmit(DEBUG,(uint8_t *)buf_tx_modbus_rtu,count_data_tx_modbus_rtu,10);
}

/**
 * @brief        
 * 
 * @param
 */
void modbus_rtu_writemultiple_register(uint8_t add_slave_rtu_, uint16_t add_start_register_rtu, uint16_t number_register_rtu, uint8_t* p_data)
{
	uint16_t count_data_tx_modbus_rtu = 0;
	uint16_t crc_output;
	/* byte add slave */
	buf_tx_modbus_rtu[count_data_tx_modbus_rtu++] = add_slave_rtu_;
	/* byte function */
	buf_tx_modbus_rtu[count_data_tx_modbus_rtu++] = 0x10;
	/* byte add start register */
	buf_tx_modbus_rtu[count_data_tx_modbus_rtu++] = add_start_register_rtu / 256;
	buf_tx_modbus_rtu[count_data_tx_modbus_rtu++] = (uint8_t) add_start_register_rtu % 256;
	/* byte number register */
	buf_tx_modbus_rtu[count_data_tx_modbus_rtu++] = number_register_rtu / 256;;
	buf_tx_modbus_rtu[count_data_tx_modbus_rtu++] = (uint8_t) number_register_rtu % 256;
	/* byte length */
	buf_tx_modbus_rtu[count_data_tx_modbus_rtu++] = number_register_rtu * 2;
	/* byte data*/
	for(uint16_t count_data = 0; count_data < number_register_rtu; count_data++)
	{
		buf_tx_modbus_rtu[count_data_tx_modbus_rtu++] = (*p_data) / 256;
		buf_tx_modbus_rtu[count_data_tx_modbus_rtu++] = (uint8_t) (*p_data) % 256;
		p_data++;
	}
	/* calulator CRC*/
	crc_output = calculateCRC_out(count_data_tx_modbus_rtu);
	buf_tx_modbus_rtu[count_data_tx_modbus_rtu++] = crc_output / 256;
	buf_tx_modbus_rtu[count_data_tx_modbus_rtu++] = (uint8_t) crc_output % 256;
	/* Send Data */
	HAL_UART_Transmit(uart_modbus_rtu,(uint8_t *)buf_tx_modbus_rtu,count_data_tx_modbus_rtu,10);
//	HAL_UART_Transmit(DEBUG,(uint8_t *)buf_tx_modbus_rtu,count_data_tx_modbus_rtu,10);
	flag_request_or_response_func16_rtu = RECEIVE_RESPONSE;
}

/**
 * @brief        
 * 
 * @param
 */
void modbus_rtu_function16_response(uint8_t add_slave_rtu_, uint16_t add_start_register_rtu, uint16_t number_register_rtu)
{
	uint16_t count_data_tx_modbus_rtu = 0;
	uint16_t crc_output;
	/* byte add slave */
	buf_tx_modbus_rtu[count_data_tx_modbus_rtu++] = add_slave_rtu_;
	/* byte function */
	buf_tx_modbus_rtu[count_data_tx_modbus_rtu++] = 0x10;
	/* byte add start register */
	buf_tx_modbus_rtu[count_data_tx_modbus_rtu++] = add_start_register_rtu / 256;
	buf_tx_modbus_rtu[count_data_tx_modbus_rtu++] = (uint8_t) add_start_register_rtu % 256;
	/* byte number register */
	buf_tx_modbus_rtu[count_data_tx_modbus_rtu++] = number_register_rtu / 256;;
	buf_tx_modbus_rtu[count_data_tx_modbus_rtu++] = (uint8_t) number_register_rtu % 256;
	/* calulator CRC*/
	crc_output = calculateCRC_out(count_data_tx_modbus_rtu);
	buf_tx_modbus_rtu[count_data_tx_modbus_rtu++] = crc_output / 256;
	buf_tx_modbus_rtu[count_data_tx_modbus_rtu++] = (uint8_t) crc_output % 256;
	/* Send Data */
	HAL_UART_Transmit(uart_modbus_rtu,(uint8_t *)buf_tx_modbus_rtu,count_data_tx_modbus_rtu,10);
//	HAL_UART_Transmit(DEBUG,(uint8_t *)buf_tx_modbus_rtu,count_data_tx_modbus_rtu,10);
}

/**
 * @brief        
 * 
 * @param
 */
uint8_t mb_rtu_read_addslave(void)
{
	return read_uint8_queue();
}

/**
 * @brief        
 * 
 * @param
 */
uint8_t mb_rtu_read_function(void)
{
	return read_uint8_queue();
}

/**
 * @brief        
 * 
 * @param
 */
uint16_t mb_rtu_read_add_register_start(void)
{
	return read_uint16_queue();
}

/**
 * @brief        
 * 
 * @param
 */
uint16_t mb_rtu_read_number_register(void)
{
	return read_uint16_queue();
}

/**
 * @brief        
 * 
 * @param
 */
uint16_t mb_rtu_read_value_register(void)
{
	return read_uint16_queue();
}

/**
 * @brief        
 * 
 * @param
 */
uint8_t mb_rtu_read_byte_coil(void)
{
	return read_uint8_queue();
}

/**
 * @brief        
 * 
 * @param
 */
uint8_t mb_rtu_read_byte_follow(void)
{
	return read_uint8_queue();
}

/**
 * @brief        
 * 
 * @param
 */
uint16_t mb_rtu_read_crc(void)
{
	return read_uint16_queue();
}

static uint8_t read_uint8_queue(void)
{
	/* init variable */
	uint8_t data_return;
//	osEvent data_modbusRx_Queue;
	/* get first byte data */
//	data_modbusRx_Queue = osMessageGet(modbusRtuQueueHandle,1);
//	data_return = (uint8_t) data_modbusRx_Queue.value.v;
	circular_buf_get(&cbuf_rtu,&data_return);
	/* return value */
	return data_return;
}

static uint16_t read_uint16_queue(void)
{
  /* init variable */
	uint8_t data_1;
	uint8_t data_2;
	uint16_t data_return;
//	osEvent data_modbusRx_Queue;
	/* get first byte data */
//	data_modbusRx_Queue = osMessageGet(modbusRtuQueueHandle,1);
//	data_1 = (uint8_t) data_modbusRx_Queue.value.v;
//	data_modbusRx_Queue = osMessageGet(modbusRtuQueueHandle,1);
//	data_2 = (uint8_t) data_modbusRx_Queue.value.v;
	circular_buf_get(&cbuf_rtu,&data_1);
	circular_buf_get(&cbuf_rtu,&data_2);
	/* return value */
	data_return = data_1*256 + data_2;
	return data_return;
}

static uint16_t calculateCRC_out(uint8_t buf_size)
{
	uint16_t temp, temp2, flag;
  temp = 0xFFFF;
  for (uint8_t i = 0; i < buf_size; i++)
  {
    temp = temp ^ buf_tx_modbus_rtu[i];
    for (uint8_t j = 1; j <= 8; j++)
    {
      flag = temp & 0x0001;
      temp >>= 1;
      if (flag)
        temp ^= 0xA001;
    }
  }
  // Reverse byte order.
  temp2 = temp >> 8;
  temp = (temp << 8) | temp2;
  temp &= 0xFFFF;
  // the returned value is already swapped
  // crcLo byte is first & crcHi byte is last
  return temp;
}
