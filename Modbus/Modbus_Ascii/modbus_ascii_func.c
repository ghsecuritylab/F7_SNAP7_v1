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
#include "modbus_ascii.h"
#include "modbus_ascii_func.h"
#include "modbus_constant.h"

#include "circular_buffer.h"
/*********************************************************************************
 * MACRO
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

/*********************************************************************************
 * STATIC VARIABLE
 */

/*********************************************************************************
 * GLOBAL VARIABLE
 */
extern UART_HandleTypeDef*  DEBUG;
extern UART_HandleTypeDef* uart_modbus_ascii;
extern circular_buf_t cbuf_asc;

extern uint8_t buf_rx_ascii[MAX_BUFFER_RX];
extern uint8_t flag_request_or_response_func1_ascii;
extern uint8_t flag_request_or_response_func2_ascii;
extern uint8_t flag_request_or_response_func3_ascii;
extern uint8_t flag_request_or_response_func4_ascii;
extern uint8_t flag_request_or_response_func15_ascii;
extern uint8_t flag_request_or_response_func16_ascii;

uint8_t buf_tx_modbus_ascii[MAX_TX_MODBUS];

/* register of modbus */

/*********************************************************************************
 * STATIC FUNCTION
 */
static uint8_t mb_ascii_read_2byte(void);
static uint16_t mb_ascii_read_4byte(void);

static uint8_t ascii_to_hex(uint8_t byte_1, uint8_t byte_2);

static uint8_t hex_uint8_to_ascii_one(uint8_t hex_data);
static uint8_t hex_uint8_to_ascii_two(uint8_t hex_data);

static uint8_t hex_uint16_to_ascii_one(uint16_t hex_data);
static uint8_t hex_uint16_to_ascii_two(uint16_t hex_data);
static uint8_t hex_uint16_to_ascii_three(uint16_t hex_data);
static uint8_t hex_uint16_to_ascii_fourth(uint16_t hex_data);
/*********************************************************************************
 * GLOBAL FUNCTION
 */

/**
 * @brief        
 * 
 * @param
 */
void modbus_acsii_read_coil(uint8_t add_slave_ascii, uint16_t add_start_data_ascii, uint16_t number_coils_ascii)
{
	uint16_t count_data_tx_modbus_ascii = 0;
	uint8_t lrc_ouput_ascii = 0;
	/* byte start frame */
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = 0x3A;
	/* byte add slave */
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint8_to_ascii_one(add_slave_ascii);
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint8_to_ascii_two(add_slave_ascii);
	lrc_ouput_ascii += add_slave_ascii;
	/* byte function */
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = 0x30;
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = 0x31;
	lrc_ouput_ascii += 0x03;
	/* byte add start register */
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint16_to_ascii_one(add_start_data_ascii);
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint16_to_ascii_two(add_start_data_ascii);
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint16_to_ascii_three(add_start_data_ascii);
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint16_to_ascii_fourth(add_start_data_ascii);
	lrc_ouput_ascii += add_start_data_ascii / 256;
	lrc_ouput_ascii += (uint8_t) add_start_data_ascii % 256;
	/* byte number register */
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint16_to_ascii_one(number_coils_ascii);
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint16_to_ascii_two(number_coils_ascii);
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint16_to_ascii_three(number_coils_ascii);
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint16_to_ascii_fourth(number_coils_ascii);
	lrc_ouput_ascii += number_coils_ascii / 256;
	lrc_ouput_ascii += (uint8_t) number_coils_ascii % 256;
	/* calulator LRC*/
	lrc_ouput_ascii = (uint8_t ) lrc_ouput_ascii % 256;
	lrc_ouput_ascii = (uint8_t) 0x100 - lrc_ouput_ascii;
	/* byte LRC */
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint8_to_ascii_one(lrc_ouput_ascii);
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint8_to_ascii_two(lrc_ouput_ascii);
	/* byte end */
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = 0x0D;
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = 0x0A;
	/* Send Data */
	HAL_UART_Transmit(uart_modbus_ascii,(uint8_t *)buf_tx_modbus_ascii,count_data_tx_modbus_ascii,10);
//	HAL_UART_Transmit(DEBUG,(uint8_t *)buf_tx_modbus_ascii,count_data_tx_modbus_ascii,10);
  flag_request_or_response_func2_ascii = RECEIVE_RESPONSE;
//	printf("modbus_readholding_register\r\n");
}

/**
 * @brief        
 * 
 * @param
 */
void modbus_ascii_function1_response(uint16_t add_start_data_ascii, uint16_t number_coils_ascii)
{
	uint16_t count_data_tx_modbus_ascii = 0;
	uint8_t lrc_ouput_ascii = 0;
	uint16_t count_number_coils_ascii_send = 0;
	uint16_t add_start_data_send = 0;
	uint16_t number_byte_data_ascii = 0;
	uint16_t check_coil_ascii = 0;
	uint8_t byte_data_buf = 0;
	if(add_start_data_ascii > modbus_number_coil)
	{
		/* error add register start */
		printf("error add register start\r\n");
	}
	else
	{
		add_start_data_send = add_start_data_ascii;
		if(number_coils_ascii > modbus_number_coil)
		{
			/* error number register */
			printf("error number register\r\n");
		}
		else
		{
			if(add_start_data_ascii +  number_coils_ascii > modbus_number_coil)
			{
				/* error register */
				printf("error register\r\n");
			}
			else /* not error */
			{
				/* calutation number byte data */
				check_coil_ascii = (uint8_t) number_coils_ascii % 8;
				if(check_coil_ascii != 0) number_byte_data_ascii = ((number_coils_ascii / 8) + 1);
				else number_byte_data_ascii = (number_coils_ascii / 8);
				/* byte start frame */
				buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = 0x3A;
				/* byte add slave */
				buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint8_to_ascii_one(add_slave);
				buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint8_to_ascii_two(add_slave);
				lrc_ouput_ascii += add_slave;
				/* byte func */
				buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint8_to_ascii_one(0x01);
				buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint8_to_ascii_two(0x01);
				lrc_ouput_ascii += 0x01;
				/* The number of data bytes to follow */
				buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint8_to_ascii_one(number_byte_data_ascii);
				buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint8_to_ascii_two(number_byte_data_ascii);
				lrc_ouput_ascii += number_byte_data_ascii;
				/* add byte data */
				for(uint16_t count_data = 0; count_data < number_byte_data_ascii; count_data++)
				{
					byte_data_buf = 0;
					if(count_number_coils_ascii_send++ < number_byte_data_ascii && modbus_register_00000[add_start_data_send++] == 1) byte_data_buf = byte_data_buf | 0x01;
//					printf("count_number_coils_tcp_send is %d\r\n",count_number_coils_tcp_send);
					if(count_number_coils_ascii_send++ < number_byte_data_ascii && modbus_register_00000[add_start_data_send++] == 1) byte_data_buf = byte_data_buf | 0x02;
//					printf("count_number_coils_tcp_send is %d\r\n",count_number_coils_tcp_send);
					if(count_number_coils_ascii_send++ < number_byte_data_ascii && modbus_register_00000[add_start_data_send++] == 1) byte_data_buf = byte_data_buf | 0x04;
//					printf("count_number_coils_tcp_send is %d\r\n",count_number_coils_tcp_send);
					if(count_number_coils_ascii_send++ < number_byte_data_ascii && modbus_register_00000[add_start_data_send++] == 1) byte_data_buf = byte_data_buf | 0x08;
//					printf("count_number_coils_tcp_send is %d\r\n",count_number_coils_tcp_send);
					if(count_number_coils_ascii_send++ < number_byte_data_ascii && modbus_register_00000[add_start_data_send++] == 1) byte_data_buf = byte_data_buf | 0x10;
//					printf("count_number_coils_tcp_send is %d\r\n",count_number_coils_tcp_send);
					if(count_number_coils_ascii_send++ < number_byte_data_ascii && modbus_register_00000[add_start_data_send++] == 1) byte_data_buf = byte_data_buf | 0x20;
//					printf("count_number_coils_tcp_send is %d\r\n",count_number_coils_tcp_send);
					if(count_number_coils_ascii_send++ < number_byte_data_ascii && modbus_register_00000[add_start_data_send++] == 1) byte_data_buf = byte_data_buf | 0x40;
//					printf("count_number_coils_tcp_send is %d\r\n",count_number_coils_tcp_send);
					if(count_number_coils_ascii_send++ < number_byte_data_ascii && modbus_register_00000[add_start_data_send++] == 1) byte_data_buf = byte_data_buf | 0x80;
//					printf("count_number_coils_tcp_send is %d\r\n",count_number_coils_tcp_send);
					buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint8_to_ascii_one(byte_data_buf);
					buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint8_to_ascii_two(byte_data_buf);
					lrc_ouput_ascii += byte_data_buf;
				}
				/* calulator LRC*/
				lrc_ouput_ascii = (uint8_t ) lrc_ouput_ascii % 256;
				lrc_ouput_ascii = (uint8_t) 0x100 - lrc_ouput_ascii;
				buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint8_to_ascii_one(lrc_ouput_ascii);
				buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint8_to_ascii_two(lrc_ouput_ascii);
				/* byte end */
				buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = 0x0D;
				buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = 0x0A;
				/* send data */
				HAL_UART_Transmit(uart_modbus_ascii,(uint8_t *)buf_tx_modbus_ascii,count_data_tx_modbus_ascii,10);
				HAL_UART_Transmit(DEBUG,(uint8_t *)buf_tx_modbus_ascii,count_data_tx_modbus_ascii,10);
			}
		}
	}
}

/**
 * @brief        
 * 
 * @param
 */
void modbus_ascii_readdiscrete_input(uint8_t add_slave_ascii, uint16_t add_start_data_ascii, uint16_t number_coils_ascii)
{
	uint16_t count_data_tx_modbus_ascii = 0;
	uint8_t lrc_ouput_ascii = 0;
	/* byte start frame */
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = 0x3A;
	/* byte add slave */
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint8_to_ascii_one(add_slave_ascii);
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint8_to_ascii_two(add_slave_ascii);
	lrc_ouput_ascii += add_slave_ascii;
	/* byte function */
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = 0x30;
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = 0x32;
	lrc_ouput_ascii += 0x03;
	/* byte add start register */
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint16_to_ascii_one(add_start_data_ascii);
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint16_to_ascii_two(add_start_data_ascii);
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint16_to_ascii_three(add_start_data_ascii);
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint16_to_ascii_fourth(add_start_data_ascii);
	lrc_ouput_ascii += add_start_data_ascii / 256;
	lrc_ouput_ascii += (uint8_t) add_start_data_ascii % 256;
	/* byte number register */
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint16_to_ascii_one(number_coils_ascii);
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint16_to_ascii_two(number_coils_ascii);
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint16_to_ascii_three(number_coils_ascii);
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint16_to_ascii_fourth(number_coils_ascii);
	lrc_ouput_ascii += number_coils_ascii / 256;
	lrc_ouput_ascii += (uint8_t) number_coils_ascii % 256;
	/* calulator LRC*/
	lrc_ouput_ascii = (uint8_t ) lrc_ouput_ascii % 256;
	lrc_ouput_ascii = (uint8_t) 0x100 - lrc_ouput_ascii;
	/* byte LRC */
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint8_to_ascii_one(lrc_ouput_ascii);
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint8_to_ascii_two(lrc_ouput_ascii);
	/* byte end */
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = 0x0D;
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = 0x0A;
	/* Send Data */
	HAL_UART_Transmit(uart_modbus_ascii,(uint8_t *)buf_tx_modbus_ascii,count_data_tx_modbus_ascii,10);
//	HAL_UART_Transmit(DEBUG,(uint8_t *)buf_tx_modbus_ascii,count_data_tx_modbus_ascii,10);
  flag_request_or_response_func2_ascii = RECEIVE_RESPONSE;
//	printf("modbus_readholding_register\r\n");
}

/**
 * @brief        
 * 
 * @param
 */
void modbus_ascii_function2_response(uint16_t add_start_data_ascii, uint16_t number_coils_ascii)
{
	uint16_t count_data_tx_modbus_ascii = 0;
	uint8_t lrc_ouput_ascii = 0;
	uint16_t count_number_coils_ascii_send = 0;
	uint16_t add_start_data_send = 0;
	uint16_t number_byte_data_ascii = 0;
	uint16_t check_coil_ascii = 0;
	uint8_t byte_data_buf = 0;
	if(add_start_data_ascii > modbus_number_coil)
	{
		/* error add register start */
		printf("error add register start\r\n");
	}
	else
	{
		add_start_data_send = add_start_data_ascii;
		if(number_coils_ascii > modbus_number_coil)
		{
			/* error number register */
			printf("error number register\r\n");
		}
		else
		{
			if(add_start_data_ascii +  number_coils_ascii > modbus_number_coil)
			{
				/* error register */
				printf("error register\r\n");
			}
			else /* not error */
			{
				/* calutation number byte data */
				check_coil_ascii = (uint8_t) number_coils_ascii % 8;
				if(check_coil_ascii != 0) number_byte_data_ascii = ((number_coils_ascii / 8) + 1);
				else number_byte_data_ascii = (number_coils_ascii / 8);
				/* byte start frame */
				buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = 0x3A;
				/* byte add slave */
				buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint8_to_ascii_one(add_slave);
				buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint8_to_ascii_two(add_slave);
				lrc_ouput_ascii += add_slave;
				/* byte func */
				buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint8_to_ascii_one(0x02);
				buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint8_to_ascii_two(0x02);
				lrc_ouput_ascii += 0x01;
				/* The number of data bytes to follow */
				buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint8_to_ascii_one(number_byte_data_ascii);
				buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint8_to_ascii_two(number_byte_data_ascii);
				lrc_ouput_ascii += number_byte_data_ascii;
				/* add byte data */
				for(uint16_t count_data = 0; count_data < number_byte_data_ascii; count_data++)
				{
					byte_data_buf = 0;
					if(count_number_coils_ascii_send++ < number_byte_data_ascii && modbus_register_10000[add_start_data_send++] == 1) byte_data_buf = byte_data_buf | 0x01;
//					printf("count_number_coils_tcp_send is %d\r\n",count_number_coils_tcp_send);
					if(count_number_coils_ascii_send++ < number_byte_data_ascii && modbus_register_10000[add_start_data_send++] == 1) byte_data_buf = byte_data_buf | 0x02;
//					printf("count_number_coils_tcp_send is %d\r\n",count_number_coils_tcp_send);
					if(count_number_coils_ascii_send++ < number_byte_data_ascii && modbus_register_10000[add_start_data_send++] == 1) byte_data_buf = byte_data_buf | 0x04;
//					printf("count_number_coils_tcp_send is %d\r\n",count_number_coils_tcp_send);
					if(count_number_coils_ascii_send++ < number_byte_data_ascii && modbus_register_10000[add_start_data_send++] == 1) byte_data_buf = byte_data_buf | 0x08;
//					printf("count_number_coils_tcp_send is %d\r\n",count_number_coils_tcp_send);
					if(count_number_coils_ascii_send++ < number_byte_data_ascii && modbus_register_10000[add_start_data_send++] == 1) byte_data_buf = byte_data_buf | 0x10;
//					printf("count_number_coils_tcp_send is %d\r\n",count_number_coils_tcp_send);
					if(count_number_coils_ascii_send++ < number_byte_data_ascii && modbus_register_10000[add_start_data_send++] == 1) byte_data_buf = byte_data_buf | 0x20;
//					printf("count_number_coils_tcp_send is %d\r\n",count_number_coils_tcp_send);
					if(count_number_coils_ascii_send++ < number_byte_data_ascii && modbus_register_10000[add_start_data_send++] == 1) byte_data_buf = byte_data_buf | 0x40;
//					printf("count_number_coils_tcp_send is %d\r\n",count_number_coils_tcp_send);
					if(count_number_coils_ascii_send++ < number_byte_data_ascii && modbus_register_10000[add_start_data_send++] == 1) byte_data_buf = byte_data_buf | 0x80;
//					printf("count_number_coils_tcp_send is %d\r\n",count_number_coils_tcp_send);
					buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint8_to_ascii_one(byte_data_buf);
					buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint8_to_ascii_two(byte_data_buf);
					lrc_ouput_ascii += byte_data_buf;
				}
				/* calulator LRC*/
				lrc_ouput_ascii = (uint8_t ) lrc_ouput_ascii % 256;
				lrc_ouput_ascii = (uint8_t) 0x100 - lrc_ouput_ascii;
				buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint8_to_ascii_one(lrc_ouput_ascii);
				buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint8_to_ascii_two(lrc_ouput_ascii);
				/* byte end */
				buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = 0x0D;
				buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = 0x0A;
				/* send data */
				HAL_UART_Transmit(uart_modbus_ascii,(uint8_t *)buf_tx_modbus_ascii,count_data_tx_modbus_ascii,10);
				HAL_UART_Transmit(DEBUG,(uint8_t *)buf_tx_modbus_ascii,count_data_tx_modbus_ascii,10);
			}
		}
	}
}

/**
 * @brief        
 * 
 * @param
 */
void modbus_acsii_readholding_register(uint8_t add_slave_ascii, uint16_t add_start_register_ascii, uint16_t number_register_ascii)
{
	uint16_t count_data_tx_modbus_ascii = 0;
	uint8_t lrc_ouput_ascii = 0;
	/* byte start frame */
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = 0x3A;
	/* byte add slave */
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint8_to_ascii_one(add_slave_ascii);
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint8_to_ascii_two(add_slave_ascii);
	lrc_ouput_ascii += add_slave_ascii;
	/* byte function */
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = 0x30;
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = 0x33;
	lrc_ouput_ascii += 0x03;
	/* byte add start register */
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint16_to_ascii_one(add_start_register_ascii);
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint16_to_ascii_two(add_start_register_ascii);
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint16_to_ascii_three(add_start_register_ascii);
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint16_to_ascii_fourth(add_start_register_ascii);
	lrc_ouput_ascii += add_start_register_ascii / 256;
	lrc_ouput_ascii += (uint8_t) add_start_register_ascii % 256;
	/* byte number register */
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint16_to_ascii_one(number_register_ascii);
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint16_to_ascii_two(number_register_ascii);
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint16_to_ascii_three(number_register_ascii);
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint16_to_ascii_fourth(number_register_ascii);
	lrc_ouput_ascii += number_register_ascii / 256;
	lrc_ouput_ascii += (uint8_t) number_register_ascii % 256;
	/* calulator LRC*/
	lrc_ouput_ascii = (uint8_t ) lrc_ouput_ascii % 256;
	lrc_ouput_ascii = (uint8_t) 0x100 - lrc_ouput_ascii;
	/* byte LRC */
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint8_to_ascii_one(lrc_ouput_ascii);
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint8_to_ascii_two(lrc_ouput_ascii);
	/* byte end */
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = 0x0D;
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = 0x0A;
	/* Send Data */
	HAL_UART_Transmit(uart_modbus_ascii,(uint8_t *)buf_tx_modbus_ascii,count_data_tx_modbus_ascii,10);
//	HAL_UART_Transmit(DEBUG,(uint8_t *)buf_tx_modbus_ascii,count_data_tx_modbus_ascii,10);
  flag_request_or_response_func3_ascii = RECEIVE_RESPONSE;
//	printf("modbus_readholding_register\r\n");
}
/**
 * @brief        
 * 
 * @param
 */
void modbus_acsii_function3_response(uint16_t add_start_register_ascii, uint16_t number_register_ascii)
{
	uint16_t count_data_tx_modbus_ascii = 0;
	uint8_t lrc_ouput_ascii = 0;
	uint16_t add_start_register_send_ascii = 0;
	if(add_start_register_ascii > modbus_number_register)
	{
		/* error add register start */
		printf("error add register start\r\n");
	}
	else
	{
		add_start_register_send_ascii = add_start_register_ascii;
		if(number_register_ascii > modbus_number_register)
		{
			/* error number register */
			printf("error number register\r\n");
		}
		else
		{
			if(add_start_register_ascii +  number_register_ascii > modbus_number_register)
			{
				/* error register */
				printf("error register\r\n");
			}
			else /* not error */
			{
				/* byte start frame */
				buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = 0x3A;
				/* byte add slave */
				buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint8_to_ascii_one(add_slave);
				buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint8_to_ascii_two(add_slave);
				lrc_ouput_ascii += add_slave;
				/* byte func */
				buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint8_to_ascii_one(0x03);
				buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint8_to_ascii_two(0x03);
				lrc_ouput_ascii += 0x03;
				/* length data */
				buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint8_to_ascii_one(number_register_ascii*2);
				buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint8_to_ascii_two(number_register_ascii*2);
				lrc_ouput_ascii += (number_register_ascii*2);
				/* byte data */
				for(uint16_t count_data = 0; count_data < number_register_ascii; count_data++)
				{
					buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] =   hex_uint16_to_ascii_one   (modbus_register_40000[add_start_register_send_ascii + count_data]);
					buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] =   hex_uint16_to_ascii_two   (modbus_register_40000[add_start_register_send_ascii + count_data]);
					buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] =   hex_uint16_to_ascii_three (modbus_register_40000[add_start_register_send_ascii + count_data]);
					buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] =   hex_uint16_to_ascii_fourth(modbus_register_40000[add_start_register_send_ascii + count_data]);
					lrc_ouput_ascii += modbus_register_40000[add_start_register_send_ascii + count_data] / 256 ;
					lrc_ouput_ascii += modbus_register_40000[add_start_register_send_ascii + count_data] % 256 ;
				}
				/* calulator LRC*/
				lrc_ouput_ascii = (uint8_t ) lrc_ouput_ascii % 256;
	      lrc_ouput_ascii = (uint8_t) 0x100 - lrc_ouput_ascii;
				
				/* byte LRC */
				buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint8_to_ascii_one(lrc_ouput_ascii);
				buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint8_to_ascii_two(lrc_ouput_ascii);
				/* byte end */
				buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = 0x0D;
				buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = 0x0A;
				/* Send data */
//				HAL_UART_Transmit(DEBUG,(uint8_t *)buf_tx_modbus_ascii,count_data_tx_modbus_ascii,10);
				HAL_UART_Transmit(uart_modbus_ascii,(uint8_t *)buf_tx_modbus_ascii,count_data_tx_modbus_ascii,10);
			}
		}
	}
}

/**
 * @brief        
 * 
 * @param
 */
void modbus_ascii_read_input_register(uint8_t add_slave_ascii, uint16_t add_start_register_ascii, uint16_t number_register_ascii)
{
	uint16_t count_data_tx_modbus_ascii = 0;
	uint8_t lrc_ouput_ascii = 0;
	/* byte start frame */
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = 0x3A;
	/* byte add slave */
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint8_to_ascii_one(add_slave_ascii);
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint8_to_ascii_two(add_slave_ascii);
	lrc_ouput_ascii += add_slave_ascii;
	/* byte function */
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = 0x30;
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = 0x34;
	lrc_ouput_ascii += 0x03;
	/* byte add start register */
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint16_to_ascii_one(add_start_register_ascii);
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint16_to_ascii_two(add_start_register_ascii);
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint16_to_ascii_three(add_start_register_ascii);
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint16_to_ascii_fourth(add_start_register_ascii);
	lrc_ouput_ascii += add_start_register_ascii / 256;
	lrc_ouput_ascii += (uint8_t) add_start_register_ascii % 256;
	/* byte number register */
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint16_to_ascii_one(number_register_ascii);
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint16_to_ascii_two(number_register_ascii);
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint16_to_ascii_three(number_register_ascii);
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint16_to_ascii_fourth(number_register_ascii);
	lrc_ouput_ascii += number_register_ascii / 256;
	lrc_ouput_ascii += (uint8_t) number_register_ascii % 256;
	/* calulator LRC*/
	lrc_ouput_ascii = (uint8_t ) lrc_ouput_ascii % 256;
	lrc_ouput_ascii = (uint8_t) 0x100 - lrc_ouput_ascii;
	/* byte LRC */
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint8_to_ascii_one(lrc_ouput_ascii);
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint8_to_ascii_two(lrc_ouput_ascii);
	/* byte end */
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = 0x0D;
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = 0x0A;
	/* Send Data */
	HAL_UART_Transmit(uart_modbus_ascii,(uint8_t *)buf_tx_modbus_ascii,count_data_tx_modbus_ascii,10);
//	HAL_UART_Transmit(DEBUG,(uint8_t *)buf_tx_modbus_ascii,count_data_tx_modbus_ascii,10);
  flag_request_or_response_func4_ascii = RECEIVE_RESPONSE;
//	printf("modbus_readholding_register\r\n");
}

/**
 * @brief        
 * 
 * @param
 */
void modbus_ascii_function4_response(uint16_t add_start_register_ascii, uint16_t number_register_ascii)
{
	uint16_t count_data_tx_modbus_ascii = 0;
	uint8_t lrc_ouput_ascii = 0;
	uint16_t add_start_register_send_ascii = 0;
	if(add_start_register_ascii > modbus_number_register)
	{
		/* error add register start */
		printf("error add register start\r\n");
	}
	else
	{
		add_start_register_send_ascii = add_start_register_ascii;
		if(number_register_ascii > modbus_number_register)
		{
			/* error number register */
			printf("error number register\r\n");
		}
		else
		{
			if(add_start_register_ascii +  number_register_ascii > modbus_number_register)
			{
				/* error register */
				printf("error register\r\n");
			}
			else /* not error */
			{
				/* byte start frame */
				buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = 0x3A;
				/* byte add slave */
				buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint8_to_ascii_one(add_slave);
				buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint8_to_ascii_two(add_slave);
				lrc_ouput_ascii += add_slave;
				/* byte func */
				buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint8_to_ascii_one(0x04);
				buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint8_to_ascii_two(0x04);
				lrc_ouput_ascii += 0x03;
				/* length data */
				buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint8_to_ascii_one(number_register_ascii*2);
				buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint8_to_ascii_two(number_register_ascii*2);
				lrc_ouput_ascii += (number_register_ascii*2);
				/* byte data */
				for(uint16_t count_data = 0; count_data < number_register_ascii; count_data++)
				{
					buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] =   hex_uint16_to_ascii_one   (modbus_register_30000[add_start_register_send_ascii + count_data]);
					buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] =   hex_uint16_to_ascii_two   (modbus_register_30000[add_start_register_send_ascii + count_data]);
					buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] =   hex_uint16_to_ascii_three (modbus_register_30000[add_start_register_send_ascii + count_data]);
					buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] =   hex_uint16_to_ascii_fourth(modbus_register_30000[add_start_register_send_ascii + count_data]);
					lrc_ouput_ascii += modbus_register_40000[add_start_register_send_ascii + count_data] / 256 ;
					lrc_ouput_ascii += modbus_register_40000[add_start_register_send_ascii + count_data] % 256 ;
				}
				/* calulator LRC*/
				lrc_ouput_ascii = (uint8_t ) lrc_ouput_ascii % 256;
	      lrc_ouput_ascii = (uint8_t) 0x100 - lrc_ouput_ascii;
				
				/* byte LRC */
				buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint8_to_ascii_one(lrc_ouput_ascii);
				buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint8_to_ascii_two(lrc_ouput_ascii);
				/* byte end */
				buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = 0x0D;
				buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = 0x0A;
				/* Send data */ 
//				HAL_UART_Transmit(DEBUG,(uint8_t *)buf_tx_modbus_ascii,count_data_tx_modbus_ascii,10);
				HAL_UART_Transmit(uart_modbus_ascii,(uint8_t *)buf_tx_modbus_ascii,count_data_tx_modbus_ascii,10);
			}
		}
	}
}
/**
 * @brief        
 * 
 * @param
 */ 
void modbus_ascii_writemultiple_coil(uint8_t add_slave_ascii, uint16_t add_start_data_ascii, uint16_t number_coils_ascii, uint8_t* p_data)
{
	uint16_t count_data_tx_modbus_ascii = 0;
	uint8_t lrc_ouput_ascii = 0;
	uint16_t number_byte_data_ascii = 0;
	uint16_t check_coil_ascii = 0;
	/* byte start frame */
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = 0x3A;
	/* byte add slave */
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint8_to_ascii_one(add_slave_ascii);
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint8_to_ascii_two(add_slave_ascii);
	lrc_ouput_ascii += add_slave_ascii;
	/* byte function */
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = 0x30;
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = 0x46;
	lrc_ouput_ascii += 0x10;
	/* byte add start coil */
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint16_to_ascii_one(add_start_data_ascii);
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint16_to_ascii_two(add_start_data_ascii);
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint16_to_ascii_three(add_start_data_ascii);
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint16_to_ascii_fourth(add_start_data_ascii);
	lrc_ouput_ascii += add_start_data_ascii / 256;
	lrc_ouput_ascii += add_start_data_ascii % 256;
	/* byte number coil */
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint16_to_ascii_one(number_coils_ascii);
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint16_to_ascii_two(number_coils_ascii);
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint16_to_ascii_three(number_coils_ascii);
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint16_to_ascii_fourth(number_coils_ascii);
	lrc_ouput_ascii += number_coils_ascii / 256;
	/* calutation number byte data */
	check_coil_ascii = (uint8_t) number_coils_ascii % 8;
	if(check_coil_ascii != 0) number_byte_data_ascii = ((number_coils_ascii / 8) + 1);
	else number_byte_data_ascii = (number_coils_ascii / 8);
	/* byte follow */
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint8_to_ascii_one(number_byte_data_ascii);
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint8_to_ascii_two(number_byte_data_ascii);
	lrc_ouput_ascii += number_byte_data_ascii;
	/* byte data*/
	for(uint16_t count_data = 0; count_data < number_byte_data_ascii; count_data++)
	{
		buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] =   hex_uint8_to_ascii_one   (*p_data);
		buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] =   hex_uint8_to_ascii_two   (*p_data);
		lrc_ouput_ascii += *p_data;
		p_data++;
	}
	/* calulator LRC*/
	lrc_ouput_ascii = (uint8_t ) lrc_ouput_ascii % 256;
	lrc_ouput_ascii = (uint8_t) 0x100 - lrc_ouput_ascii;
	/* byte LRC */
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint8_to_ascii_one(lrc_ouput_ascii);
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint8_to_ascii_two(lrc_ouput_ascii);
	/* byte end */
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = 0x0D;
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = 0x0A;
	/* Send function 15*/
	HAL_UART_Transmit(uart_modbus_ascii,(uint8_t *)buf_tx_modbus_ascii,count_data_tx_modbus_ascii,10);
//	HAL_UART_Transmit(DEBUG,(uint8_t *)buf_tx_modbus_ascii,count_data_tx_modbus_ascii,10);
	flag_request_or_response_func15_ascii = RECEIVE_RESPONSE;
}

/**
 * @brief        
 * 
 * @param
 */
void modbus_ascii_function15_response(uint8_t add_slave_ascii, uint16_t add_start_register_ascii, uint16_t number_register_ascii)
{
	uint16_t count_data_tx_modbus_ascii = 0;
	uint8_t lrc_ouput_ascii = 0;
	/* byte start frame */
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = 0x3A;
	/* byte add slave */
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint8_to_ascii_one(add_slave_ascii);
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint8_to_ascii_two(add_slave_ascii);
	lrc_ouput_ascii += add_slave_ascii;
	/* byte function */
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = 0x30;
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = 0x46;
	lrc_ouput_ascii += 0x10;
	/* byte add start register */
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint16_to_ascii_one(add_start_register_ascii);
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint16_to_ascii_two(add_start_register_ascii);
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint16_to_ascii_three(add_start_register_ascii);
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint16_to_ascii_fourth(add_start_register_ascii);
	lrc_ouput_ascii += add_start_register_ascii / 256;
	lrc_ouput_ascii += add_start_register_ascii % 256;
	/* byte number register */
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint16_to_ascii_one(number_register_ascii);
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint16_to_ascii_two(number_register_ascii);
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint16_to_ascii_three(number_register_ascii);
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint16_to_ascii_fourth(number_register_ascii);
	lrc_ouput_ascii += number_register_ascii / 256;
	lrc_ouput_ascii += number_register_ascii % 256;
	/* calulator LRC*/
	lrc_ouput_ascii = (uint8_t ) lrc_ouput_ascii % 256;
	lrc_ouput_ascii = (uint8_t) 0x100 - lrc_ouput_ascii;
	/* byte LRC */
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint8_to_ascii_one(lrc_ouput_ascii);
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint8_to_ascii_two(lrc_ouput_ascii);
	/* byte end */
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = 0x0D;
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = 0x0A;
	/* Send function 15*/
	HAL_UART_Transmit(uart_modbus_ascii,(uint8_t *)buf_tx_modbus_ascii,count_data_tx_modbus_ascii,10);
//	HAL_UART_Transmit(DEBUG,(uint8_t *)buf_tx_modbus_ascii,count_data_tx_modbus_ascii,10);
}

/**
 * @brief        
 * 
 * @param
 */
void modbus_acsii_writemultiple_register(uint8_t add_slave_ascii, uint16_t add_start_register_ascii, uint16_t number_register_ascii, uint8_t* p_data)
{
	uint16_t count_data_tx_modbus_ascii = 0;
	uint8_t lrc_ouput_ascii = 0;
	/* byte start frame */
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = 0x3A;
	/* byte add slave */
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint8_to_ascii_one(add_slave_ascii);
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint8_to_ascii_two(add_slave_ascii);
	lrc_ouput_ascii += add_slave_ascii;
	/* byte function */
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = 0x31;
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = 0x30;
	lrc_ouput_ascii += 0x10;
	/* byte add start register */
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint16_to_ascii_one(add_start_register_ascii);
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint16_to_ascii_two(add_start_register_ascii);
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint16_to_ascii_three(add_start_register_ascii);
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint16_to_ascii_fourth(add_start_register_ascii);
	lrc_ouput_ascii += add_start_register_ascii / 256;
	lrc_ouput_ascii += add_start_register_ascii % 256;
	/* byte number register */
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint16_to_ascii_one(number_register_ascii);
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint16_to_ascii_two(number_register_ascii);
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint16_to_ascii_three(number_register_ascii);
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint16_to_ascii_fourth(number_register_ascii);
	lrc_ouput_ascii += number_register_ascii / 256;
	lrc_ouput_ascii += number_register_ascii % 256;
	/* byte length */
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint8_to_ascii_one(number_register_ascii*2);
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint8_to_ascii_two(number_register_ascii*2);
	lrc_ouput_ascii += (number_register_ascii*2);
	/* byte data*/
	for(uint16_t count_data = 0; count_data < number_register_ascii; count_data++)
	{
		buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] =   hex_uint16_to_ascii_one   (*p_data);
		buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] =   hex_uint16_to_ascii_two   (*p_data);
		buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] =   hex_uint16_to_ascii_three (*p_data);
		buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] =   hex_uint16_to_ascii_fourth(*p_data);
		lrc_ouput_ascii += *p_data / 256;
		lrc_ouput_ascii += *p_data % 256;
		p_data++;
	}
	/* calulator LRC*/
	lrc_ouput_ascii = (uint8_t ) lrc_ouput_ascii % 256;
	lrc_ouput_ascii = (uint8_t) 0x100 - lrc_ouput_ascii;
	/* byte LRC */
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint8_to_ascii_one(lrc_ouput_ascii);
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint8_to_ascii_two(lrc_ouput_ascii);
	/* byte end */
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = 0x0D;
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = 0x0A;
	/* Send function 16*/
	HAL_UART_Transmit(uart_modbus_ascii,(uint8_t *)buf_tx_modbus_ascii,count_data_tx_modbus_ascii,10);
//	HAL_UART_Transmit(DEBUG,(uint8_t *)buf_tx_modbus_ascii,count_data_tx_modbus_ascii,10);
	flag_request_or_response_func16_ascii = RECEIVE_RESPONSE;
}

/**
 * @brief        
 * 
 * @param
 */
void modbus_acsii_function16_response(uint8_t add_slave_ascii, uint16_t add_start_register_ascii, uint16_t number_register_ascii)
{
	uint16_t count_data_tx_modbus_ascii = 0;
	uint8_t lrc_ouput_ascii = 0;
	/* byte start frame */
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = 0x3A;
	/* byte add slave */
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint8_to_ascii_one(add_slave_ascii);
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint8_to_ascii_two(add_slave_ascii);
	lrc_ouput_ascii += add_slave_ascii;
	/* byte function */
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = 0x31;
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = 0x30;
	lrc_ouput_ascii += 0x10;
	/* byte add start register */
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint16_to_ascii_one(add_start_register_ascii);
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint16_to_ascii_two(add_start_register_ascii);
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint16_to_ascii_three(add_start_register_ascii);
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint16_to_ascii_fourth(add_start_register_ascii);
	lrc_ouput_ascii += add_start_register_ascii / 256;
	lrc_ouput_ascii += add_start_register_ascii % 256;
	/* byte number register */
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint16_to_ascii_one(number_register_ascii);
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint16_to_ascii_two(number_register_ascii);
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint16_to_ascii_three(number_register_ascii);
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint16_to_ascii_fourth(number_register_ascii);
	lrc_ouput_ascii += number_register_ascii / 256;
	lrc_ouput_ascii += number_register_ascii % 256;
	/* calulator LRC*/
	lrc_ouput_ascii = (uint8_t ) lrc_ouput_ascii % 256;
	lrc_ouput_ascii = (uint8_t) 0x100 - lrc_ouput_ascii;
	/* byte LRC */
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint8_to_ascii_one(lrc_ouput_ascii);
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = hex_uint8_to_ascii_two(lrc_ouput_ascii);
	/* byte end */
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = 0x0D;
	buf_tx_modbus_ascii[count_data_tx_modbus_ascii++] = 0x0A;
	/* Send function 16*/
	HAL_UART_Transmit(uart_modbus_ascii,(uint8_t *)buf_tx_modbus_ascii,count_data_tx_modbus_ascii,10);
//	HAL_UART_Transmit(DEBUG,(uint8_t *)buf_tx_modbus_ascii,count_data_tx_modbus_ascii,10);
}

/**
 * @brief        
 * 
 * @param
 */
uint8_t mb_ascii_read_addslave(void)
{
	return mb_ascii_read_2byte();
}
/**
 * @brief        
 * 
 * @param
 */
uint8_t mb_ascii_read_function(void)
{
	return mb_ascii_read_2byte();
}

/**
 * @brief        
 * 
 * @param
 */
uint16_t mb_ascii_read_add_register_start(void)
{
	return mb_ascii_read_4byte();
}

/**
 * @brief        
 * 
 * @param
 */
uint16_t mb_ascii_read_number_register(void)
{
	return mb_ascii_read_4byte();
}

/**
 * @brief        
 * 
 * @param
 */
uint8_t mb_ascii_read_byte_coil(void)
{
	return mb_ascii_read_2byte();
}

/**
 * @brief        
 * 
 * @param
 */
uint8_t mb_ascii_read_number_byte_follow(void)
{
	return mb_ascii_read_2byte();
}

/* @brief        
 * 
 * @param
 */
uint16_t mb_ascii_read_value_to_register(void)
{
	return mb_ascii_read_4byte();
}

/**
 * @brief        
 * 
 * @param
 */
uint8_t mb_ascii_read_lrc(void)
{
	return mb_ascii_read_2byte();
}

/*********************************************************************************
 * STATIC FUNCTION
 */
static uint8_t mb_ascii_read_2byte(void)
{
	/* init variable */
	uint8_t first_byte;
	uint8_t second_byte;
//	osEvent data_modbusRx_Queue;
	/* get first byte data */
//	data_modbusRx_Queue = osMessageGet(modbusAsciiQueueHandle,1);
//	first_byte = (uint8_t) data_modbusRx_Queue.value.v;
	circular_buf_get(&cbuf_asc,&first_byte);
	/* get second byte data */
//	data_modbusRx_Queue = osMessageGet(modbusAsciiQueueHandle,1);
//	second_byte = (uint8_t) data_modbusRx_Queue.value.v;
	circular_buf_get(&cbuf_asc,&second_byte);
	/* return 1 function add slave to 2 byte ascii*/
	return ascii_to_hex(first_byte, second_byte);
}

static uint16_t mb_ascii_read_4byte(void)
{
	/* init variable */
	uint8_t first_byte;
	uint8_t second_byte;
	uint8_t third_byte;
	uint8_t fourth_byte;
//	osEvent data_modbusRx_Queue;
	/* get first byte data */
//	data_modbusRx_Queue = osMessageGet(modbusAsciiQueueHandle,1);
//	first_byte = (uint8_t) data_modbusRx_Queue.value.v;
	circular_buf_get(&cbuf_asc,&first_byte);
	/* get second byte data */
//	data_modbusRx_Queue = osMessageGet(modbusAsciiQueueHandle,1);
//	second_byte = (uint8_t) data_modbusRx_Queue.value.v;
	circular_buf_get(&cbuf_asc,&second_byte);
	/* get third byte data */
//	data_modbusRx_Queue = osMessageGet(modbusAsciiQueueHandle,1);
//	third_byte = (uint8_t) data_modbusRx_Queue.value.v;
	circular_buf_get(&cbuf_asc,&third_byte);
	/* get fourth byte data */
//	data_modbusRx_Queue = osMessageGet(modbusAsciiQueueHandle,1);
//	fourth_byte = (uint8_t) data_modbusRx_Queue.value.v;
	circular_buf_get(&cbuf_asc,&fourth_byte);
	/* return 1 function add slave to 4 byte ascii*/
	return (256*ascii_to_hex(first_byte, second_byte) + ascii_to_hex(third_byte, fourth_byte));
}

static uint8_t ascii_to_hex(uint8_t byte_1, uint8_t byte_2)
{
	uint8_t buf;
	if(byte_1 > 0x39) byte_1 = byte_1 - 0x40 +0x09;
	else byte_1 -= 0x30;
	if(byte_2 > 0x39) byte_2 = byte_2 - 0x40 +0x09;
	else byte_2 -= 0x30;
	buf = byte_1*16+byte_2;
	return buf;
}

static uint8_t hex_uint8_to_ascii_one(uint8_t hex_data)
{
	uint8_t buf = 0;
	buf = (uint8_t)hex_data / 16;
	if(buf <= 9) buf += 0x30;
	else buf = buf + 0x40 - 0x09;
	return buf;
}

static uint8_t hex_uint8_to_ascii_two(uint8_t hex_data)
{
	uint8_t buf = 0;
	buf = (uint8_t )hex_data % 16;
	if(buf <= 9) buf += 0x30;
	else buf = buf + 0x40 - 0x09;
	return buf;
}

static uint8_t hex_uint16_to_ascii_one(uint16_t hex_data)
{
	uint8_t buf = 0;
	buf =  hex_data / 256;
	buf = (uint8_t) buf / 16;
	if(buf <= 9) buf += 0x30;
	else buf = buf + 0x40 - 0x09;
	return buf;
}
static uint8_t hex_uint16_to_ascii_two(uint16_t hex_data)
{
	uint8_t buf = 0;
	buf =  hex_data / 256;
	buf = (uint8_t) buf % 16;
	if(buf <= 9) buf += 0x30;
	else buf = buf + 0x40 - 0x09;
	return buf;
}
static uint8_t hex_uint16_to_ascii_three(uint16_t hex_data)
{
	uint8_t buf = 0;
	buf = (uint8_t) hex_data % 256;
	buf = (uint8_t) buf / 16;
	if(buf <= 9) buf += 0x30;
	else buf = buf + 0x40 - 0x09;
	return buf;
}
static uint8_t hex_uint16_to_ascii_fourth(uint16_t hex_data)
{
	uint8_t buf = 0;
	buf = (uint8_t) hex_data % 256;
	buf = (uint8_t) buf % 16;
	if(buf <= 9) buf += 0x30;
	else buf = buf + 0x40 - 0x09;
	return buf;
}
