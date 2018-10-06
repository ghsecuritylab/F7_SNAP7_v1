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

uint8_t buffer_data_RTU[100];
uint8_t clear_data_rtu;
circular_buf_t cbuf_rtu;
/*********************************************************************************
 * STATIC VARIABLE
 */
static uint8_t modbus_rx_buf_rtu = 0;
/*********************************************************************************
 * GLOBAL VARIABLE
 */
/* uart for modbus */
UART_HandleTypeDef* uart_modbus_rtu;

/* buffer for calulator lrc*/
uint8_t buf_rx_rtu[MAX_BUFFER_RX];
/* byte get address slave */
uint8_t byte_get_add_slave_rtu;
/* byte get function */
uint8_t byte_get_function_rtu;
/* byte get add register start */
uint16_t add_register_start_rtu;
/* byte get number register */
uint16_t number_register_rtu;
/* byte get number byte folow */
uint8_t number_byte_follow_rtu;
uint16_t buf_register_rtu[MAX_REGISTES];
/* byte lrc*/
uint16_t crc;
/* flag receive request or response */
uint8_t flag_request_or_response_func1_rtu = RECEIVE_REQUEST;
uint8_t flag_request_or_response_func2_rtu = RECEIVE_REQUEST;
uint8_t flag_request_or_response_func3_rtu = RECEIVE_REQUEST;
uint8_t flag_request_or_response_func4_rtu = RECEIVE_REQUEST;
uint8_t flag_request_or_response_func15_rtu = RECEIVE_REQUEST;
uint8_t flag_request_or_response_func16_rtu = RECEIVE_REQUEST;
/* count data receive for calulator */
uint16_t count_data_modbusRx_Queue_rtu = 0;
/*********************************************************************************
 * STATIC FUNCTION
 */

static uint16_t calculateCRC_input_rtu(uint8_t buf_size);
static void function1_handle_rtu(void);
static void function2_handle_rtu(void);
static void function3_handle_rtu(void);
static void function4_handle_rtu(void);
static void function15_handle_rtu(void);
static void function16_handle_rtu(void);
/*********************************************************************************
 * GLOBAL FUNCTION
 */


/**
 * @brief        
 * 
 * @param
 */
void modbus_rtu_port_init(UART_HandleTypeDef *huart, uint32_t BaudRate, uint32_t WordLength, uint32_t StopBits, uint32_t Parity)
{
	/* Init circular buffer */
	circular_buf_init(&cbuf_rtu,buffer_data_RTU,100);
	
	/* Init uart */
	uart_modbus_rtu = huart;
	uart_modbus_rtu->Init.BaudRate   = BaudRate;
	uart_modbus_rtu->Init.WordLength = WordLength;
	uart_modbus_rtu->Init.StopBits   = StopBits;
	uart_modbus_rtu->Init.Parity     = Parity;
	uart_modbus_rtu->Init.Mode = UART_MODE_TX_RX;
  uart_modbus_rtu->Init.HwFlowCtl = UART_HWCONTROL_NONE;
  uart_modbus_rtu->Init.OverSampling = UART_OVERSAMPLING_16;
		
	if (HAL_UART_Init(uart_modbus_rtu) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
	else
	{
		HAL_UART_Receive_IT(uart_modbus_rtu,(uint8_t *)&modbus_rx_buf_rtu,1);
	}
}

/**
 * @brief        
 * 
 * @param
 */
void modbus_rtu_get_input(UART_HandleTypeDef *huart)
{
	if(huart->Instance == uart_modbus_rtu->Instance)
	{
		circular_buf_put(&cbuf_rtu,modbus_rx_buf_rtu);
		HAL_UART_Receive_IT(huart,(uint8_t *)&modbus_rx_buf_rtu,1);
	}
}

/**
 * @brief        
 * 
 * @param
 */
void modbus_rtu_check_input(void)
{
  count_data_modbusRx_Queue_rtu = 0;
	/* get byte add slave */
	byte_get_add_slave_rtu = mb_rtu_read_addslave();
	buf_rx_rtu[count_data_modbusRx_Queue_rtu++] = byte_get_add_slave_rtu;
	/* check add slave */
	if (byte_get_add_slave_rtu == add_slave)
	{
		/* get byte function */
		byte_get_function_rtu = mb_rtu_read_function();
		buf_rx_rtu[count_data_modbusRx_Queue_rtu++] = byte_get_function_rtu;

		switch (byte_get_function_rtu)
		{
			case FUNCTION_01:
				function1_handle_rtu();
				break;
			case FUNCTION_02:
				function2_handle_rtu();
				break;
			case FUNCTION_03:
				function3_handle_rtu();
				break;
			case FUNCTION_04:
				function4_handle_rtu();
				break;
			case FUNCTION_15:
				function15_handle_rtu();
				break;
			case FUNCTION_16:
				function16_handle_rtu();
				break;
			default:
				break;
		}
	}
	else
	{
		/* error add slave */
		printf("modbus rtu error add slave\r\n");
		/* clear data on queue */
	}
}

/*********************************************************************************
 * STATIC FUNCTION
 */
static void function1_handle_rtu(void)
{
	if(flag_request_or_response_func1_rtu == RECEIVE_REQUEST)
	{
		printf("function1_handle_rtu RECEIVE_REQUEST\r\n");
		/* get data add register start */
		add_register_start_rtu = mb_rtu_read_add_register_start();
		buf_rx_rtu[count_data_modbusRx_Queue_rtu++] = add_register_start_rtu / 256;
		buf_rx_rtu[count_data_modbusRx_Queue_rtu++] = (uint8_t) add_register_start_rtu % 256;
		/* get data number register */
		number_register_rtu = mb_rtu_read_number_register();
		buf_rx_rtu[count_data_modbusRx_Queue_rtu++] = number_register_rtu / 256;
		buf_rx_rtu[count_data_modbusRx_Queue_rtu++] = (uint8_t) number_register_rtu % 256;
		crc = mb_rtu_read_crc();
		if(calculateCRC_input_rtu(count_data_modbusRx_Queue_rtu) == crc)
		{
			/* handle */
			modbus_rtu_function1_response(add_register_start_rtu, number_register_rtu);
		}
	}
	else
	{
		printf("function1_handle_rtu RECEIVE_REPONSE\r\n");
		/* The number of data bytes to follow */
		number_byte_follow_rtu = mb_rtu_read_byte_follow();
		buf_rx_rtu[count_data_modbusRx_Queue_rtu++] = number_byte_follow_rtu;
		for(uint16_t count = 0; count < number_byte_follow_rtu; count++)
		{
			buf_register_rtu[count] = mb_rtu_read_byte_coil();
			buf_rx_rtu[count_data_modbusRx_Queue_rtu++] = buf_register_rtu[count];
		}
		crc = mb_rtu_read_crc();
		if(calculateCRC_input_rtu(count_data_modbusRx_Queue_rtu) == crc)
		{
			for(uint16_t count = 0; count < number_byte_follow_rtu; count++)
			{
				printf("buf_register_rtu[%d] is %d\r\n",count,buf_register_rtu[count]);
			}
		}
		else
		{
			/* error */
			printf("error CRC function 1 receive reponse\r\n");
		}
		flag_request_or_response_func1_rtu = RECEIVE_REQUEST;
	}
}

static void function2_handle_rtu(void)
{
	if(flag_request_or_response_func2_rtu == RECEIVE_REQUEST)
	{
		printf("function2_handle RECEIVE_REQUEST\r\n");
		/* get data add register start */
		add_register_start_rtu = mb_rtu_read_add_register_start();
		buf_rx_rtu[count_data_modbusRx_Queue_rtu++] = add_register_start_rtu / 256;
		buf_rx_rtu[count_data_modbusRx_Queue_rtu++] = (uint8_t) add_register_start_rtu % 256;
		/* get data number register */
		number_register_rtu = mb_rtu_read_number_register();
		buf_rx_rtu[count_data_modbusRx_Queue_rtu++] = number_register_rtu / 256;
		buf_rx_rtu[count_data_modbusRx_Queue_rtu++] = (uint8_t) number_register_rtu % 256;
		crc = mb_rtu_read_crc();
		if(calculateCRC_input_rtu(count_data_modbusRx_Queue_rtu) == crc)
		{
			/* handle */
			modbus_rtu_function2_response(add_register_start_rtu, number_register_rtu);
		}
		else
		{
			/*error CRC*/
			printf("error CRC function 3 receive request\r\n");
		}
	}
	else
	{
		printf("function2_handle RECEIVE_REPONSE\r\n");
		/* The number of data bytes to follow */
		number_byte_follow_rtu = mb_rtu_read_byte_follow();
		buf_rx_rtu[count_data_modbusRx_Queue_rtu++] = number_byte_follow_rtu;
		for(uint16_t count = 0; count < number_byte_follow_rtu; count++)
		{
			buf_register_rtu[count] = mb_rtu_read_byte_coil();
			buf_rx_rtu[count_data_modbusRx_Queue_rtu++] = buf_register_rtu[count];
		}
		crc = mb_rtu_read_crc();
		if(calculateCRC_input_rtu(count_data_modbusRx_Queue_rtu) == crc)
		{
			for(uint16_t count = 0; count < number_byte_follow_rtu; count++)
			{
				printf("buf_register_rtu[%d] is %d\r\n",count,buf_register_rtu[count]);
			}
		}
		else
		{
			/* error CRC */
			printf("error CRC function 2 receive reponse\r\n");
		}
		flag_request_or_response_func2_rtu = RECEIVE_REQUEST;
	}
}


static void function3_handle_rtu(void)
{
	if(flag_request_or_response_func3_rtu == RECEIVE_REQUEST)
	{
		printf("function3_handle RECEIVE_REQUEST\r\n");
		add_register_start_rtu = mb_rtu_read_add_register_start();
		buf_rx_rtu[count_data_modbusRx_Queue_rtu++] = add_register_start_rtu / 256;
		buf_rx_rtu[count_data_modbusRx_Queue_rtu++] = (uint8_t) add_register_start_rtu % 256;
		number_register_rtu = mb_rtu_read_number_register();
		buf_rx_rtu[count_data_modbusRx_Queue_rtu++] = number_register_rtu / 256;
		buf_rx_rtu[count_data_modbusRx_Queue_rtu++] = (uint8_t) number_register_rtu % 256;
		crc = mb_rtu_read_crc();
		if(calculateCRC_input_rtu(count_data_modbusRx_Queue_rtu) == crc)
		{
			modbus_rtu_function3_response(add_register_start_rtu,number_register_rtu);
		}
		else
		{
			/*error CRC*/
			printf("error CRC function 3 receive request\r\n");
		}
	}
	else
	{
		printf("function3_handle RECEIVE_REPONSE\r\n");
		/* The number of data bytes to follow */
		number_byte_follow_rtu = mb_rtu_read_byte_follow();
		buf_rx_rtu[count_data_modbusRx_Queue_rtu++] = number_byte_follow_rtu;
		for(uint16_t count = 0; count < (number_byte_follow_rtu / 2); count++)
		{
			buf_register_rtu[count] = mb_rtu_read_value_register();
			buf_rx_rtu[count_data_modbusRx_Queue_rtu++] = buf_register_rtu[count] / 256;
			buf_rx_rtu[count_data_modbusRx_Queue_rtu++] = (uint8_t) buf_register_rtu[count] % 256;
		}
		crc = mb_rtu_read_crc();
		if(calculateCRC_input_rtu(count_data_modbusRx_Queue_rtu) == crc)
		{
			for(uint16_t count = 0; count < (number_byte_follow_rtu / 2); count++)
			{
				printf("buf_register_rtu[%d] is %d\r\n",count,buf_register_rtu[count]);
			}
		}
		else
		{
			/* error CRC */
			printf("error CRC function 3 receive reponse\r\n");
		}
		flag_request_or_response_func3_rtu = RECEIVE_REQUEST;
	}
}

static void function4_handle_rtu(void)
{
	if(flag_request_or_response_func4_rtu == RECEIVE_REQUEST)
	{
		printf("function4_handle RECEIVE_REQUEST\r\n");
		add_register_start_rtu = mb_rtu_read_add_register_start();
		buf_rx_rtu[count_data_modbusRx_Queue_rtu++] = add_register_start_rtu / 256;
		buf_rx_rtu[count_data_modbusRx_Queue_rtu++] = (uint8_t) add_register_start_rtu % 256;
		number_register_rtu = mb_rtu_read_number_register();
		buf_rx_rtu[count_data_modbusRx_Queue_rtu++] = number_register_rtu / 256;
		buf_rx_rtu[count_data_modbusRx_Queue_rtu++] = (uint8_t) number_register_rtu % 256;
		crc = mb_rtu_read_crc();
		if(calculateCRC_input_rtu(count_data_modbusRx_Queue_rtu) == crc)
		{
			modbus_rtu_function4_response(add_register_start_rtu,number_register_rtu);
		}
		else
		{
			/*error CRC*/
			printf("error CRC function 4 receive request\r\n");
		}
	}
	else
	{
		printf("function4_handle RECEIVE_REPONSE\r\n");
		/* The number of data bytes to follow */
		number_byte_follow_rtu = mb_rtu_read_byte_follow();
		buf_rx_rtu[count_data_modbusRx_Queue_rtu++] = number_byte_follow_rtu;
		for(uint16_t count = 0; count < (number_byte_follow_rtu / 2); count++)
		{
			buf_register_rtu[count] = mb_rtu_read_value_register();
			buf_rx_rtu[count_data_modbusRx_Queue_rtu++] = buf_register_rtu[count] / 256;
			buf_rx_rtu[count_data_modbusRx_Queue_rtu++] = (uint8_t) buf_register_rtu[count] % 256;
		}
		crc = mb_rtu_read_crc();
		if(calculateCRC_input_rtu(count_data_modbusRx_Queue_rtu) == crc)
		{
			for(uint16_t count = 0; count < (number_byte_follow_rtu / 2); count++)
			{
				printf("buf_register_rtu[%d] is %d\r\n",count,buf_register_rtu[count]);
			}
		}
		else
		{
			/* error CRC */
			printf("error CRC function 4 receive reponse\r\n");
		}
		flag_request_or_response_func4_rtu = RECEIVE_REQUEST;
	}
}

static void function15_handle_rtu(void)
{
	uint16_t number_byte_data_rtu = 0;
	uint16_t check_coil_rtu = 0;
	if(flag_request_or_response_func15_rtu == RECEIVE_REQUEST)
	{
		printf("function15_handle RECEIVE_REQUEST\r\n");
		add_register_start_rtu = mb_rtu_read_add_register_start();
		buf_rx_rtu[count_data_modbusRx_Queue_rtu++] = add_register_start_rtu / 256;
		buf_rx_rtu[count_data_modbusRx_Queue_rtu++] = (uint8_t) add_register_start_rtu % 256;
		number_register_rtu = mb_rtu_read_number_register();
		buf_rx_rtu[count_data_modbusRx_Queue_rtu++] = number_register_rtu / 256;
		buf_rx_rtu[count_data_modbusRx_Queue_rtu++] = (uint8_t) number_register_rtu % 256;
		/* The number of data bytes to follow */
		number_byte_follow_rtu = mb_rtu_read_byte_follow();
		buf_rx_rtu[count_data_modbusRx_Queue_rtu++] = number_byte_follow_rtu;
		if(add_register_start_rtu > modbus_number_coil || number_register_rtu > modbus_number_coil \
			 || add_register_start_rtu +  number_register_rtu > modbus_number_coil)
		{
			/* error -- clear queue */
		}
		else
		{
			/*check number byte */
			check_coil_rtu = (uint8_t) number_register_rtu % 8;
			if(check_coil_rtu != 0) number_byte_data_rtu = ((number_register_rtu / 8) + 1);
			else number_byte_data_rtu = (number_register_rtu / 8);
			
			for(uint16_t count = 0; count < number_byte_data_rtu; count++)
			{
				buf_register_rtu[count] = mb_rtu_read_byte_coil();
				buf_rx_rtu[count_data_modbusRx_Queue_rtu++] = buf_register_rtu[count];
			}
			crc = mb_rtu_read_crc();
			if(calculateCRC_input_rtu(count_data_modbusRx_Queue_rtu) == crc)
			{
				/*send Response */
				modbus_rtu_function15_response(add_slave, add_register_start_rtu, number_register_rtu);
				/* update register  */
				for(uint16_t count = add_register_start_rtu; count < number_register_rtu; count++)
				{
					modbus_register_00000[count] =buf_register_rtu[count- add_register_start_rtu];
					printf("modbus_register_rtu[%d] is %d\r\n",count,modbus_register_00000[count]);
				}
			}
			else
			{
				/*error LRC*/
				printf("error CRC function 15 requet\r\n");
			}
		}
	}
	else
	{
		/* co the doc ve de xu ly */
		/* hien tai thi khong lam gi voi du lieu nay*/
		/* clead data */
		printf("function15_handle RECEIVE_RESPONSE\r\n");
		add_register_start_rtu = mb_rtu_read_add_register_start();
		number_register_rtu = mb_rtu_read_number_register();
		crc = mb_rtu_read_crc();
		flag_request_or_response_func15_rtu = RECEIVE_REQUEST;
	}
}

static void function16_handle_rtu(void)
{
	if(flag_request_or_response_func16_rtu == RECEIVE_REQUEST)
	{
		printf("function16_handle RECEIVE_REQUEST\r\n");
		add_register_start_rtu = mb_rtu_read_add_register_start();
		buf_rx_rtu[count_data_modbusRx_Queue_rtu++] = add_register_start_rtu / 256;
		buf_rx_rtu[count_data_modbusRx_Queue_rtu++] = (uint8_t) add_register_start_rtu % 256;
		number_register_rtu = mb_rtu_read_number_register();
		buf_rx_rtu[count_data_modbusRx_Queue_rtu++] = number_register_rtu / 256;
		buf_rx_rtu[count_data_modbusRx_Queue_rtu++] = (uint8_t) number_register_rtu % 256;
		/* The number of data bytes to follow */
		number_byte_follow_rtu = mb_rtu_read_byte_follow();
		buf_rx_rtu[count_data_modbusRx_Queue_rtu++] = number_byte_follow_rtu;
		if(add_register_start_rtu > modbus_number_register || number_register_rtu > modbus_number_register \
			 || add_register_start_rtu +  number_register_rtu > modbus_number_register)
		{
			/* error -- clear queue */
		}
		else
		{
			for(uint16_t count = 0; count < number_register_rtu; count++)
			{
				buf_register_rtu[count] = mb_rtu_read_value_register();
				buf_rx_rtu[count_data_modbusRx_Queue_rtu++] = buf_register_rtu[count] / 256;
				buf_rx_rtu[count_data_modbusRx_Queue_rtu++] = (uint8_t) buf_register_rtu[count] % 256;
			}
			crc = mb_rtu_read_crc();
			if(calculateCRC_input_rtu(count_data_modbusRx_Queue_rtu) == crc)
			{
				/*send Response */
				modbus_rtu_function16_response(add_slave, add_register_start_rtu, number_register_rtu);
				/* update register  */
				for(uint16_t count = add_register_start_rtu; count < number_register_rtu; count++)
				{
					modbus_register_40000[count] =buf_register_rtu[count- add_register_start_rtu];
					printf("modbus_register_rtu[%d] is %d\r\n",count,modbus_register_40000[count]);
				}
			}
			else
			{
				/*error LRC*/
				printf("error CRC function 16 requet\r\n");
			}
		}
	}
	else
	{
		/* co the doc ve de xu ly */
		/* hien tai thi khong lam gi voi du lieu nay*/
		/* clead data */
		printf("function16_handle RECEIVE_RESPONSE\r\n");
		add_register_start_rtu = mb_rtu_read_add_register_start();
		number_register_rtu = mb_rtu_read_number_register();
		crc = mb_rtu_read_crc();
		flag_request_or_response_func16_rtu = RECEIVE_REQUEST;
	}
}



static uint16_t calculateCRC_input_rtu(uint8_t buf_size)
{
	uint16_t temp, temp2, flag;
  temp = 0xFFFF;
  for (uint8_t i = 0; i < buf_size; i++)
  {
    temp = temp ^ buf_rx_rtu[i];
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
