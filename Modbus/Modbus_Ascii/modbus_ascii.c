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
#include "cmsis_os.h"
/* user include */
#include "modbus_ascii.h"
#include "modbus_ascii_func.h"
#include "modbus_constant.h"
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
extern  osMessageQId modbusAsciiQueueHandle;

/*********************************************************************************
 * STATIC VARIABLE
 */
static uint8_t modbus_rx_buf_ascii = 0;
static uint8_t flag_receive_sucess_ascii = 0;

/*********************************************************************************
 * GLOBAL VARIABLE
 */
/* uart for modbus */
UART_HandleTypeDef* uart_modbus_ascii;

/* buffer for calulator lrc*/
uint8_t buf_rx_ascii[MAX_BUFFER_RX];
/* byte get address slave */
uint8_t byte_get_add_slave_ascii;
/* byte get function */
uint8_t byte_get_function_ascii;
/* byte get add register start */
uint16_t add_register_start_ascii;
/* byte get number register */
uint16_t number_register_ascii;
/* byte get number byte folow */
uint8_t number_byte_follow_ascii;
uint16_t buf_register_ascii[MAX_REGISTES];
/* byte lrc*/
uint8_t lrc;
/* flag receive request or response */
uint8_t flag_request_or_response_func1_ascii = RECEIVE_REQUEST;
uint8_t flag_request_or_response_func2_ascii = RECEIVE_REQUEST;
uint8_t flag_request_or_response_func3_ascii = RECEIVE_REQUEST;
uint8_t flag_request_or_response_func4_ascii = RECEIVE_REQUEST;
uint8_t flag_request_or_response_func15_ascii = RECEIVE_REQUEST;
uint8_t flag_request_or_response_func16_ascii = RECEIVE_REQUEST;
/* count data receive for calulator */
uint16_t count_data_modbusRx_Queue_ascii = 0;
/*********************************************************************************
 * STATIC FUNCTION
 */

static uint8_t calulator_lrc_input_ascii(uint16_t length);
static void clear_modbus_frame(void);
static void function1_handle_ascii(void);
static void function2_handle_ascii(void);
static void function3_handle_ascii(void);
static void function4_handle_ascii(void);
static void function15_handle_ascii(void);
static void function16_handle_ascii(void);
/*********************************************************************************
 * GLOBAL FUNCTION
 */

/**
 * @brief        
 * 
 * @param
 */
void modbus_ascii_port_init(UART_HandleTypeDef *huart, uint32_t BaudRate, uint32_t WordLength, uint32_t StopBits, uint32_t Parity)
{
	/* Init uart */
	uart_modbus_ascii = huart;
	uart_modbus_ascii->Init.BaudRate   = BaudRate;
	uart_modbus_ascii->Init.WordLength = WordLength;
	uart_modbus_ascii->Init.StopBits   = StopBits;
	uart_modbus_ascii->Init.Parity     = Parity;
	uart_modbus_ascii->Init.Mode = UART_MODE_TX_RX;
	uart_modbus_ascii->Init.HwFlowCtl = UART_HWCONTROL_NONE;
	uart_modbus_ascii->Init.OverSampling = UART_OVERSAMPLING_16;
		
	if (HAL_UART_Init(uart_modbus_ascii) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
	else
	{
		HAL_UART_Receive_IT(uart_modbus_ascii,(uint8_t *)&modbus_rx_buf_ascii,1);
	}
}

/**
 * @brief        
 * 
 * @param
 */
void modbus_acsii_get_input(UART_HandleTypeDef *huart)
{
	if(huart->Instance == uart_modbus_ascii->Instance)
	{
		osMessagePut(modbusAsciiQueueHandle,modbus_rx_buf_ascii,1);
		HAL_UART_Receive_IT(huart,(uint8_t *)&modbus_rx_buf_ascii,1);
		if(modbus_rx_buf_ascii == 0x0A) flag_receive_sucess_ascii++;
	}
}

/**
 * @brief        
 * 
 * @param
 */
void modbus_acsii_check_input(void)
{
	osEvent check_modbusRx_Queue_ascii;
	count_data_modbusRx_Queue_ascii = 0;
	if(flag_receive_sucess_ascii != 0)
	{
		/* get byte start */
		check_modbusRx_Queue_ascii = osMessageGet(modbusAsciiQueueHandle,1);

		if(check_modbusRx_Queue_ascii.value.v == 0x3A)
		{
			/* get byte add slave */
			byte_get_add_slave_ascii = mb_ascii_read_addslave();
			buf_rx_ascii[count_data_modbusRx_Queue_ascii++] = byte_get_add_slave_ascii;
			/* check add slave */
			if(byte_get_add_slave_ascii == add_slave)
			{
				/* get byte function */
				byte_get_function_ascii = mb_ascii_read_function();
				buf_rx_ascii[count_data_modbusRx_Queue_ascii++] = byte_get_function_ascii;
				
				switch(byte_get_function_ascii)
				{
					case FUNCTION_01:
						function1_handle_ascii();
						flag_receive_sucess_ascii -= 1;
						break;
					case FUNCTION_02:
						function2_handle_ascii();
						flag_receive_sucess_ascii -= 1;
						break;
					case FUNCTION_03:
						function3_handle_ascii();
						flag_receive_sucess_ascii -= 1;
						break;
					case FUNCTION_04:
						function4_handle_ascii();
						flag_receive_sucess_ascii -= 1;
						break;
					case FUNCTION_15:
						function15_handle_ascii();
						flag_receive_sucess_ascii -= 1;
						break;
					case FUNCTION_16:
						function16_handle_ascii();
						flag_receive_sucess_ascii -= 1;
						break;
					default:
						break;
				}
				
			}
			else
			{
				/* error add slave */
				printf("modbus ascii error add slave\r\n");
				/* clear data on queue */
				clear_modbus_frame();
			}
		}
	}
}

static void function1_handle_ascii(void)
{
	if(flag_request_or_response_func1_ascii == RECEIVE_REQUEST)
	{
		printf("function1_handle_ascii RECEIVE_REQUEST\r\n");
		/* get data add register start */
		add_register_start_ascii = mb_ascii_read_add_register_start();
		buf_rx_ascii[count_data_modbusRx_Queue_ascii++] = add_register_start_ascii / 256;
		buf_rx_ascii[count_data_modbusRx_Queue_ascii++] = (uint8_t) add_register_start_ascii % 256;
		/* get data number register */
		number_register_ascii = mb_ascii_read_number_register();
		buf_rx_ascii[count_data_modbusRx_Queue_ascii++] = number_register_ascii / 256;
		buf_rx_ascii[count_data_modbusRx_Queue_ascii++] = (uint8_t) number_register_ascii % 256;
		lrc = mb_ascii_read_lrc();
		if(calulator_lrc_input_ascii(count_data_modbusRx_Queue_ascii) == lrc)
		{
			/* handle */
			modbus_ascii_function1_response(add_register_start_ascii, number_register_ascii);
		}
		else
		{
			/*error LRC*/
			printf("error LRC function 1 receive request\r\n");
		}
	}
	else
	{
		printf("function1_handle_ascii RECEIVE_REPONSE\r\n");
		/* The number of data bytes to follow */
		number_byte_follow_ascii = mb_ascii_read_number_byte_follow();
		buf_rx_ascii[count_data_modbusRx_Queue_ascii++] = number_byte_follow_ascii;
		for(uint16_t count = 0; count < number_byte_follow_ascii; count++)
		{
			buf_register_ascii[count] = mb_ascii_read_byte_coil();
			buf_rx_ascii[count_data_modbusRx_Queue_ascii++] = buf_register_ascii[count];
		}
		lrc = mb_ascii_read_lrc();
		if(calulator_lrc_input_ascii(count_data_modbusRx_Queue_ascii) == lrc)
		{
			for(uint16_t count = 0; count < number_byte_follow_ascii; count++)
			{
				printf("buf_register_ascii[%d] is %d\r\n",count,buf_register_ascii[count]);
			}
		}
		else
		{
			/* error */
			printf("error LRC function 1 receive reponse\r\n");
		}
		flag_request_or_response_func1_ascii = RECEIVE_REQUEST;
	}
}

static void function2_handle_ascii(void)
{
	if(flag_request_or_response_func2_ascii == RECEIVE_REQUEST)
	{
		printf("function2_handle RECEIVE_REQUEST\r\n");
		/* get data add register start */
		add_register_start_ascii = mb_ascii_read_add_register_start();
		buf_rx_ascii[count_data_modbusRx_Queue_ascii++] = add_register_start_ascii / 256;
		buf_rx_ascii[count_data_modbusRx_Queue_ascii++] = (uint8_t) add_register_start_ascii % 256;
		/* get data number register */
		number_register_ascii = mb_ascii_read_number_register();
		buf_rx_ascii[count_data_modbusRx_Queue_ascii++] = number_register_ascii / 256;
		buf_rx_ascii[count_data_modbusRx_Queue_ascii++] = (uint8_t) number_register_ascii % 256;
		lrc = mb_ascii_read_lrc();
		if(calulator_lrc_input_ascii(count_data_modbusRx_Queue_ascii) == lrc)
		{
			/* handle */
			modbus_ascii_function2_response(add_register_start_ascii, number_register_ascii);
		}
		else
		{
			/*error CRC*/
			printf("error LRC function 2 receive request\r\n");
		}
	}
	else
	{
		printf("function2_handle RECEIVE_REPONSE\r\n");
		/* The number of data bytes to follow */
		number_byte_follow_ascii = mb_ascii_read_number_byte_follow();
		buf_rx_ascii[count_data_modbusRx_Queue_ascii++] = number_byte_follow_ascii;
		for(uint16_t count = 0; count < number_byte_follow_ascii; count++)
		{
			buf_register_ascii[count] = mb_ascii_read_byte_coil();
			buf_rx_ascii[count_data_modbusRx_Queue_ascii++] = buf_register_ascii[count];
		}
		lrc = mb_ascii_read_lrc();
		/* clear 2 byte end frame modbus */
		osMessageGet(modbusAsciiQueueHandle,1);
		osMessageGet(modbusAsciiQueueHandle,1);
		if(calulator_lrc_input_ascii(count_data_modbusRx_Queue_ascii) == lrc)
		{
			for(uint16_t count = 0; count < number_byte_follow_ascii; count++)
			{
				printf("buf_register[%d] is %d\r\n",count,buf_register_ascii[count]);
			}
		}
		else
		{
			/* error LRC */
			printf("error LRC function 2 receive reponse\r\n");
//			clear_modbus_frame();
		}
		flag_request_or_response_func2_ascii = RECEIVE_REQUEST;
	}
}

static void function3_handle_ascii(void)
{
	if(flag_request_or_response_func3_ascii == RECEIVE_REQUEST)
	{
		printf("function3_handle RECEIVE_REQUEST\r\n");
		add_register_start_ascii = mb_ascii_read_add_register_start();
		buf_rx_ascii[count_data_modbusRx_Queue_ascii++] = add_register_start_ascii / 256;
		buf_rx_ascii[count_data_modbusRx_Queue_ascii++] = (uint8_t) add_register_start_ascii % 256;
		number_register_ascii = mb_ascii_read_number_register();
		buf_rx_ascii[count_data_modbusRx_Queue_ascii++] = number_register_ascii / 256;
		buf_rx_ascii[count_data_modbusRx_Queue_ascii++] = (uint8_t) number_register_ascii % 256;
		lrc = mb_ascii_read_lrc();
		/* clear 2 byte end frame modbus */
		osMessageGet(modbusAsciiQueueHandle,1);
		osMessageGet(modbusAsciiQueueHandle,1);
		if(calulator_lrc_input_ascii(count_data_modbusRx_Queue_ascii) == lrc)
		{
			modbus_acsii_function3_response(add_register_start_ascii,number_register_ascii);
		}
		else
		{
			/*error LRC*/
			printf("error LRC function 3 receive request\r\n");
		}
	}
	else
	{
		printf("function3_handle RECEIVE_REPONSE\r\n");
		/* The number of data bytes to follow */
		number_byte_follow_ascii = mb_ascii_read_number_byte_follow();
		buf_rx_ascii[count_data_modbusRx_Queue_ascii++] = number_byte_follow_ascii;
		for(uint16_t count = 0; count < (number_byte_follow_ascii /2); count++)
		{
			buf_register_ascii[count] = mb_ascii_read_value_to_register();
			buf_rx_ascii[count_data_modbusRx_Queue_ascii++] = buf_register_ascii[count] / 256;
			buf_rx_ascii[count_data_modbusRx_Queue_ascii++] = (uint8_t) buf_register_ascii[count] % 256;
		}
		lrc = mb_ascii_read_lrc();
		/* clear 2 byte end frame modbus */
		osMessageGet(modbusAsciiQueueHandle,1);
		osMessageGet(modbusAsciiQueueHandle,1);
		if(calulator_lrc_input_ascii(count_data_modbusRx_Queue_ascii) == lrc)
		{
			for(uint16_t count = 0; count < (number_byte_follow_ascii / 2); count++)
			{
				printf("buf_register[%d] is %d\r\n",count,buf_register_ascii[count]);
			}
		}
		else
		{
			/* error LRC */
			printf("error LRC function 3 receive reponse\r\n");
//			clear_modbus_frame();
		}
		flag_request_or_response_func3_ascii = RECEIVE_REQUEST;
	}
}

static void function4_handle_ascii(void)
{
	if(flag_request_or_response_func4_ascii == RECEIVE_REQUEST)
	{
		printf("function4_handle RECEIVE_REQUEST\r\n");
		add_register_start_ascii = mb_ascii_read_add_register_start();
		buf_rx_ascii[count_data_modbusRx_Queue_ascii++] = add_register_start_ascii / 256;
		buf_rx_ascii[count_data_modbusRx_Queue_ascii++] = (uint8_t) add_register_start_ascii % 256;
		number_register_ascii = mb_ascii_read_number_register();
		buf_rx_ascii[count_data_modbusRx_Queue_ascii++] = number_register_ascii / 256;
		buf_rx_ascii[count_data_modbusRx_Queue_ascii++] = (uint8_t) number_register_ascii % 256;
		lrc = mb_ascii_read_lrc();
		/* clear 2 byte end frame modbus */
		osMessageGet(modbusAsciiQueueHandle,1);
		osMessageGet(modbusAsciiQueueHandle,1);
		if(calulator_lrc_input_ascii(count_data_modbusRx_Queue_ascii) == lrc)
		{
			modbus_ascii_function4_response(add_register_start_ascii,number_register_ascii);
		}
		else
		{
			/*error LRC*/
			printf("error LRC function 4 receive request\r\n");
		}
	}
	else
	{
		printf("function4_handle RECEIVE_REPONSE\r\n");
		/* The number of data bytes to follow */
		number_byte_follow_ascii = mb_ascii_read_number_byte_follow();
		buf_rx_ascii[count_data_modbusRx_Queue_ascii++] = number_byte_follow_ascii;
		for(uint16_t count = 0; count < (number_byte_follow_ascii /2); count++)
		{
			buf_register_ascii[count] = mb_ascii_read_value_to_register();
			buf_rx_ascii[count_data_modbusRx_Queue_ascii++] = buf_register_ascii[count] / 256;
			buf_rx_ascii[count_data_modbusRx_Queue_ascii++] = (uint8_t) buf_register_ascii[count] % 256;
		}
		lrc = mb_ascii_read_lrc();
		/* clear 2 byte end frame modbus */
		osMessageGet(modbusAsciiQueueHandle,1);
		osMessageGet(modbusAsciiQueueHandle,1);
		if(calulator_lrc_input_ascii(count_data_modbusRx_Queue_ascii) == lrc)
		{
			for(uint16_t count = 0; count < (number_byte_follow_ascii / 2); count++)
			{
				printf("buf_register[%d] is %d\r\n",count,buf_register_ascii[count]);
			}
		}
		else
		{
			/* error LRC */
			printf("error LRC function 4 receive reponse\r\n");
//			clear_modbus_frame();
		}
		flag_request_or_response_func4_ascii = RECEIVE_REQUEST;
	}
}

static void function15_handle_ascii(void)
{
	uint16_t number_byte_data_ascii = 0;
	uint16_t check_coil_ascii = 0;
	if(flag_request_or_response_func15_ascii == RECEIVE_REQUEST)
	{
		printf("function15_handle RECEIVE_REQUEST\r\n");
		add_register_start_ascii = mb_ascii_read_add_register_start();
		buf_rx_ascii[count_data_modbusRx_Queue_ascii++] = add_register_start_ascii / 256;
		buf_rx_ascii[count_data_modbusRx_Queue_ascii++] = (uint8_t) add_register_start_ascii % 256;
		number_register_ascii = mb_ascii_read_number_register();
		buf_rx_ascii[count_data_modbusRx_Queue_ascii++] = number_register_ascii / 256;
		buf_rx_ascii[count_data_modbusRx_Queue_ascii++] = (uint8_t) number_register_ascii % 256;
		/* The number of data bytes to follow */
		number_byte_follow_ascii = mb_ascii_read_number_byte_follow();
		buf_rx_ascii[count_data_modbusRx_Queue_ascii++] = number_byte_follow_ascii;
		if(add_register_start_ascii > modbus_number_register || number_register_ascii> modbus_number_register \
			 || add_register_start_ascii +  number_register_ascii > modbus_number_register)
		{
			/* error */
		}
		else
		{
			/*check number byte */
			check_coil_ascii = (uint8_t) number_register_ascii % 8;
			if(check_coil_ascii != 0) number_byte_data_ascii = ((number_register_ascii / 8) + 1);
			else number_byte_data_ascii = (number_register_ascii / 8);
			
			for(uint16_t count = 0; count < number_byte_data_ascii; count++)
			{
				buf_register_ascii[count] = mb_ascii_read_byte_coil();
				buf_rx_ascii[count_data_modbusRx_Queue_ascii++] = buf_register_ascii[count];
			}
			lrc = mb_ascii_read_lrc();
			/* clear 2 byte end frame modbus */
			osMessageGet(modbusAsciiQueueHandle,1);
			osMessageGet(modbusAsciiQueueHandle,1);
			if(calulator_lrc_input_ascii(count_data_modbusRx_Queue_ascii) == lrc)
			{
				/*send Response */
				modbus_ascii_function15_response(add_slave, add_register_start_ascii, number_register_ascii);
				/* update register  */
				for(uint16_t count = add_register_start_ascii; count < number_byte_data_ascii; count++)
				{
					modbus_register_00000[count] = buf_register_ascii[count- add_register_start_ascii];
					printf("modbus_register[%d] is %d\r\n",count,modbus_register_00000[count]);
				}
			}
			else
			{
				/*error LRC*/
				printf("error LRC function 15 requet\r\n");
			}
		}
	}
	else
	{
		/* co the doc ve de xu ly */
		/* hien tai thi khong lam gi voi du lieu nay*/
		/* clead data */
		printf("function15_handle RECEIVE_RESPONSE\r\n");
		add_register_start_ascii = mb_ascii_read_add_register_start();
		number_register_ascii = mb_ascii_read_number_register();
		lrc = mb_ascii_read_lrc();
		/* clear 2 byte end frame modbus */
		osMessageGet(modbusAsciiQueueHandle,1);
		osMessageGet(modbusAsciiQueueHandle,1);
		
		flag_request_or_response_func15_ascii = RECEIVE_REQUEST;
	}
}


static void function16_handle_ascii(void)
{
	if(flag_request_or_response_func16_ascii == RECEIVE_REQUEST)
	{
		printf("function16_handle RECEIVE_REQUEST\r\n");
		add_register_start_ascii = mb_ascii_read_add_register_start();
		buf_rx_ascii[count_data_modbusRx_Queue_ascii++] = add_register_start_ascii / 256;
		buf_rx_ascii[count_data_modbusRx_Queue_ascii++] = (uint8_t) add_register_start_ascii % 256;
		number_register_ascii = mb_ascii_read_number_register();
		buf_rx_ascii[count_data_modbusRx_Queue_ascii++] = number_register_ascii / 256;
		buf_rx_ascii[count_data_modbusRx_Queue_ascii++] = (uint8_t) number_register_ascii % 256;
		/* The number of data bytes to follow */
		number_byte_follow_ascii = mb_ascii_read_number_byte_follow();
		buf_rx_ascii[count_data_modbusRx_Queue_ascii++] = number_byte_follow_ascii;
		if(add_register_start_ascii > modbus_number_register || number_register_ascii> modbus_number_register \
			 || add_register_start_ascii +  number_register_ascii > modbus_number_register)
		{
			/* error */
		}
		else
		{
			for(uint16_t count = 0; count < number_register_ascii; count++)
			{
				buf_register_ascii[count] = mb_ascii_read_value_to_register();
				buf_rx_ascii[count_data_modbusRx_Queue_ascii++] = buf_register_ascii[count] / 256;
				buf_rx_ascii[count_data_modbusRx_Queue_ascii++] = (uint8_t) buf_register_ascii[count] % 256;
			}
			lrc = mb_ascii_read_lrc();
			/* clear 2 byte end frame modbus */
			osMessageGet(modbusAsciiQueueHandle,1);
			osMessageGet(modbusAsciiQueueHandle,1);
			if(calulator_lrc_input_ascii(count_data_modbusRx_Queue_ascii) == lrc)
			{
				/*send Response */
				modbus_acsii_function16_response(add_slave, add_register_start_ascii, number_register_ascii);
				/* update register  */
				for(uint16_t count = add_register_start_ascii; count < number_register_ascii; count++)
				{
					modbus_register_40000[count] = buf_register_ascii[count- add_register_start_ascii];
					printf("modbus_register[%d] is %d\r\n",count,modbus_register_40000[count]);
				}
			}
			else
			{
				/*error LRC*/
				printf("error LRC function 16 requet\r\n");
			}
		}
	}
	else
	{
		/* co the doc ve de xu ly */
		/* hien tai thi khong lam gi voi du lieu nay*/
		/* clead data */
		printf("function16_handle RECEIVE_RESPONSE\r\n");
		add_register_start_ascii = mb_ascii_read_add_register_start();
		number_register_ascii = mb_ascii_read_number_register();
		lrc = mb_ascii_read_lrc();
		/* clear 2 byte end frame modbus */
		osMessageGet(modbusAsciiQueueHandle,1);
		osMessageGet(modbusAsciiQueueHandle,1);
		
		flag_request_or_response_func16_ascii = RECEIVE_REQUEST;
	}
}

static void clear_modbus_frame(void)
{
	/* Init get data queue */
	osEvent clear_queue;
	/* clear frame */
	do
	{
	  clear_queue = osMessageGet(modbusAsciiQueueHandle,1);
	}while(clear_queue.value.v != 0x0A);
}

static uint8_t calulator_lrc_input_ascii(uint16_t length)
{
	uint8_t calulator_lrc = 0;
	for(uint16_t count = 0; count <= length; count++)
	{
		calulator_lrc += buf_rx_ascii[count];
//		printf("buf_rx[%d] is %d\r\n",count,buf_rx[count]);
	}
	calulator_lrc = (uint8_t ) calulator_lrc % 256;
	calulator_lrc = (uint8_t) 0x100 - calulator_lrc;
	return calulator_lrc;
}
