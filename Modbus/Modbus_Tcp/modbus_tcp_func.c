/** @file
  * 
  * @brief modbus TCP FUNCTION
  *
  */
	
/*********************************************************************************
 * INCLUDE
 */
/* system include */
#include "usart.h"
//#include "cmsis_os.h"
/* user include */
#include "modbus_constant.h"
#include "circular_buffer.h"
/*********************************************************************************
 * MACRO
 */

/*********************************************************************************
 * EXTERN
 */
 
extern circular_buf_t cbuf_tcp;


extern uint8_t add_slave;
extern uint16_t modbus_number_register;
extern uint16_t modbus_number_coil;

extern uint8_t modbus_register_00000[MAX_REGISTES];
extern uint8_t modbus_register_10000[MAX_REGISTES];
extern uint16_t modbus_register_40000[MAX_REGISTES];
extern uint16_t modbus_register_30000[MAX_REGISTES];

extern uint8_t flag_request_or_response_func1_tcp;
extern uint8_t flag_request_or_response_func2_tcp;
extern uint8_t flag_request_or_response_func3_tcp;
extern uint8_t flag_request_or_response_func4_tcp;
extern uint8_t flag_request_or_response_func15_tcp;
extern uint8_t flag_request_or_response_func16_tcp;

//extern  osMessageQId modbusTcpRxQueueHandle;
extern void tcp_sendata(uint8_t *pData, uint16_t length);
/*********************************************************************************
 * STATIC VARIABLE
 */
static uint16_t transaction_id_trasnsmit;
/*********************************************************************************
 * GLOBAL VARIABLE
 */

uint8_t buf_tx_modbus_tcp[MAX_TX_MODBUS];
/*********************************************************************************
 * STATIC FUNCTION
 */
static uint8_t read_uint8_queue_tcp(void);
static uint16_t read_uint16_queue_tcp(void);
/*********************************************************************************
 * GLOBAL FUNCTION
 */
/**
 * @brief        
 * 
 * @param
 */
void modbus_tcp_read_coil(uint8_t add_slave_, uint16_t add_start_data_tcp, uint16_t number_coils_tcp)
{
	uint16_t count_data_tx_modbus_tcp = 0;
	/* add transaction id */
	buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = transaction_id_trasnsmit / 256;
	buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = transaction_id_trasnsmit / 256;
	transaction_id_trasnsmit++;
	/* byte protocol id */
	buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = 0x00;
	buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = 0x00;
	/* byte message length */
	buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = 0x00;
	buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = 0x06;
	/* byte Unit id */
	buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = add_slave_;
	/* byte function */
	buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = 0x01;
	/* byte add start register */
	buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = add_start_data_tcp / 256;
	buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = (uint8_t) add_start_data_tcp % 256;
	/* byte number register */
	buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = number_coils_tcp / 256;
	buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = (uint8_t) number_coils_tcp % 256;
	/*transmit data over tcp */
	printf("modbus_tcp_read_coil 1\r\n");
	tcp_sendata(buf_tx_modbus_tcp,count_data_tx_modbus_tcp);
	flag_request_or_response_func1_tcp = RECEIVE_RESPONSE;
}
/**
 * @brief        
 * 
 * @param
 */
void modbus_tcp_function1_response(uint16_t transaction, uint16_t add_start_data_tcp, uint16_t number_coils_tcp)
{
	uint16_t count_data_tx_modbus_tcp = 0;
	uint16_t count_number_coils_tcp_send = 0;
	uint16_t add_start_coil_send = 0;
	uint16_t number_byte_data = 0;
	uint16_t check_coil_tcp = 0;
	uint8_t byte_data_buf = 0;
	if(add_start_data_tcp > modbus_number_coil)
	{
		/* error add register start */
		printf("error add register start\r\n");
	}
	else
	{
		add_start_coil_send = add_start_data_tcp;
		if(number_coils_tcp > modbus_number_coil)
		{
			/* error number register */
			printf("error number register\r\n");
		}
		else
		{
			if(add_start_data_tcp +  number_coils_tcp > modbus_number_coil)
			{
				/* error register */
				printf("error register\r\n");
			}
			else /* not error */
			{
				/* calutation number byte data */
				check_coil_tcp = (uint8_t) number_coils_tcp % 8;
				if(check_coil_tcp != 0) number_byte_data = ((number_coils_tcp / 8) + 1);
				else number_byte_data = (number_coils_tcp / 8);
				/* byte transaction id */
//				printf("transaction is %d\r\n",transaction);
				buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = transaction / 256;
				buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = transaction % 256;
				/* byte protocol id */
				buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = 0x00;
				buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = 0x00;
				/* message length */
//				printf("message length is %d\r\n",number_register_tcp*2 + 2);
				buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = (number_byte_data + 3) / 256;           /*******/  
				buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = (number_byte_data + 3) % 256;
				/* byte Unit id */
//				printf("add_slave_tcp is %d\r\n",add_slave_tcp);
				buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = add_slave;
				/* byte function id */
//				printf("afunction id is %d\r\n",0x02);
				buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = 0x01;
				/* byte byte folow */
//				printf("byte folow is %d\r\n",number_register_tcp*2);
				buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = number_byte_data;
				/* set data coil */
				for(uint16_t count_data = 0; count_data < number_byte_data; count_data++)
				{
					byte_data_buf = 0;
					if(count_number_coils_tcp_send++ < number_coils_tcp && modbus_register_00000[add_start_coil_send++] == 1) byte_data_buf = byte_data_buf | 0x01;
//					printf("count_number_coils_tcp_send is %d\r\n",count_number_coils_tcp_send);
					if(count_number_coils_tcp_send++ < number_coils_tcp && modbus_register_00000[add_start_coil_send++] == 1) byte_data_buf = byte_data_buf | 0x02;
//					printf("count_number_coils_tcp_send is %d\r\n",count_number_coils_tcp_send);
					if(count_number_coils_tcp_send++ < number_coils_tcp && modbus_register_00000[add_start_coil_send++] == 1) byte_data_buf = byte_data_buf | 0x04;
//					printf("count_number_coils_tcp_send is %d\r\n",count_number_coils_tcp_send);
					if(count_number_coils_tcp_send++ < number_coils_tcp && modbus_register_00000[add_start_coil_send++] == 1) byte_data_buf = byte_data_buf | 0x08;
//					printf("count_number_coils_tcp_send is %d\r\n",count_number_coils_tcp_send);
					if(count_number_coils_tcp_send++ < number_coils_tcp && modbus_register_00000[add_start_coil_send++] == 1) byte_data_buf = byte_data_buf | 0x10;
//					printf("count_number_coils_tcp_send is %d\r\n",count_number_coils_tcp_send);
					if(count_number_coils_tcp_send++ < number_coils_tcp && modbus_register_00000[add_start_coil_send++] == 1) byte_data_buf = byte_data_buf | 0x20;
//					printf("count_number_coils_tcp_send is %d\r\n",count_number_coils_tcp_send);
					if(count_number_coils_tcp_send++ < number_coils_tcp && modbus_register_00000[add_start_coil_send++] == 1) byte_data_buf = byte_data_buf | 0x40;
//					printf("count_number_coils_tcp_send is %d\r\n",count_number_coils_tcp_send);
					if(count_number_coils_tcp_send++ < number_coils_tcp && modbus_register_00000[add_start_coil_send++] == 1) byte_data_buf = byte_data_buf | 0x80;
//					printf("count_number_coils_tcp_send is %d\r\n",count_number_coils_tcp_send);
					buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = byte_data_buf;
				}
				
				/*transmit data over tcp */
				tcp_sendata(buf_tx_modbus_tcp,count_data_tx_modbus_tcp);
			}
		}
	}
}
/**
 * @brief        
 * 
 * @param
 */
void modbus_tcp_readdiscrete_input(uint8_t add_slave_, uint16_t add_start_data_tcp, uint16_t number_coils_tcp)
{
	uint16_t count_data_tx_modbus_tcp = 0;
	/* add transaction id */
	buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = transaction_id_trasnsmit / 256;
	buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = transaction_id_trasnsmit / 256;
	transaction_id_trasnsmit++;
	/* byte protocol id */
	buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = 0x00;
	buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = 0x00;
	/* byte message length */
	buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = 0x00;
	buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = 0x06;
	/* byte Unit id */
	buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = add_slave_;
	/* byte function */
	buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = 0x02;
	/* byte add start register */
	buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = add_start_data_tcp / 256;
	buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = (uint8_t) add_start_data_tcp % 256;
	/* byte number register */
	buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = number_coils_tcp / 256;
	buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = (uint8_t) number_coils_tcp % 256;
	/*transmit data over tcp */
	printf("modbus_tcp_readdiscrete_input 2\r\n");
	tcp_sendata(buf_tx_modbus_tcp,count_data_tx_modbus_tcp);
	flag_request_or_response_func2_tcp = RECEIVE_RESPONSE;
}

/**
 * @brief        
 * 
 * @param
 */
void modbus_tcp_function2_response(uint16_t transaction, uint16_t add_start_data_tcp, uint16_t number_coils_tcp)
{
	uint16_t count_data_tx_modbus_tcp = 0;
	uint16_t count_number_coils_tcp_send = 0;
	uint16_t add_start_coil_send = 0;
	uint16_t number_byte_data = 0;
	uint16_t check_coil_tcp = 0;
	uint8_t byte_data_buf = 0;
	if(add_start_data_tcp > modbus_number_coil)
	{
		/* error add register start */
		printf("error add register start\r\n");
	}
	else
	{
		add_start_coil_send = add_start_data_tcp;
		if(add_start_data_tcp > modbus_number_coil)
		{
			/* error number register */
			printf("error number register\r\n");
		}
		else
		{
			if(add_start_data_tcp +  number_coils_tcp > modbus_number_coil)
			{
				/* error register */
				printf("error register\r\n");
			}
			else /* not error */
			{
				/* calutation number byte data */
				check_coil_tcp = (uint8_t) number_coils_tcp % 8;
				if(check_coil_tcp != 0) number_byte_data = ((number_coils_tcp / 8) + 1);
				else number_byte_data = (number_coils_tcp / 8);
				/* byte transaction id */
//				printf("transaction is %d\r\n",transaction);
				buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = transaction / 256;
				buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = transaction % 256;
				/* byte protocol id */
				buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = 0x00;
				buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = 0x00;
				/* message length */
//				printf("message length is %d\r\n",number_register_tcp*2 + 2);
				buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = (number_byte_data + 3) / 256;           /*******/  
				buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = (number_byte_data + 3) % 256;
				/* byte Unit id */
//				printf("add_slave_tcp is %d\r\n",add_slave_tcp);
				buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = add_slave;
				/* byte function id */
//				printf("afunction id is %d\r\n",0x02);
				buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = 0x02;
				/* byte byte folow */
//				printf("byte folow is %d\r\n",number_register_tcp*2);
				buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = number_byte_data;
				/* set data coil */
				for(uint16_t count_data = 0; count_data < number_byte_data; count_data++)
				{
					byte_data_buf = 0;
					if(count_number_coils_tcp_send++ < number_coils_tcp && modbus_register_10000[add_start_coil_send++] == 1) byte_data_buf = byte_data_buf | 0x01;
//					printf("count_number_coils_tcp_send is %d\r\n",count_number_coils_tcp_send);
					if(count_number_coils_tcp_send++ < number_coils_tcp && modbus_register_10000[add_start_coil_send++] == 1) byte_data_buf = byte_data_buf | 0x02;
//					printf("count_number_coils_tcp_send is %d\r\n",count_number_coils_tcp_send);
					if(count_number_coils_tcp_send++ < number_coils_tcp && modbus_register_10000[add_start_coil_send++] == 1) byte_data_buf = byte_data_buf | 0x04;
//					printf("count_number_coils_tcp_send is %d\r\n",count_number_coils_tcp_send);
					if(count_number_coils_tcp_send++ < number_coils_tcp && modbus_register_10000[add_start_coil_send++] == 1) byte_data_buf = byte_data_buf | 0x08;
//					printf("count_number_coils_tcp_send is %d\r\n",count_number_coils_tcp_send);
					if(count_number_coils_tcp_send++ < number_coils_tcp && modbus_register_10000[add_start_coil_send++] == 1) byte_data_buf = byte_data_buf | 0x10;
//					printf("count_number_coils_tcp_send is %d\r\n",count_number_coils_tcp_send);
					if(count_number_coils_tcp_send++ < number_coils_tcp && modbus_register_10000[add_start_coil_send++] == 1) byte_data_buf = byte_data_buf | 0x20;
//					printf("count_number_coils_tcp_send is %d\r\n",count_number_coils_tcp_send);
					if(count_number_coils_tcp_send++ < number_coils_tcp && modbus_register_10000[add_start_coil_send++] == 1) byte_data_buf = byte_data_buf | 0x40;
//					printf("count_number_coils_tcp_send is %d\r\n",count_number_coils_tcp_send);
					if(count_number_coils_tcp_send++ < number_coils_tcp && modbus_register_10000[add_start_coil_send++] == 1) byte_data_buf = byte_data_buf | 0x80;
//					printf("count_number_coils_tcp_send is %d\r\n",count_number_coils_tcp_send);
					buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = byte_data_buf;
				}
				
				/*transmit data over tcp */
				tcp_sendata(buf_tx_modbus_tcp,count_data_tx_modbus_tcp);
			}
		}
	}
}
/**
 * @brief        
 * 
 * @param
 */
void modbus_tcp_readholding_register(uint8_t add_slave_, uint16_t add_start_register_tcp, uint16_t number_register_tcp)
{
	uint16_t count_data_tx_modbus_tcp = 0;
	/* add transaction id */
	buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = transaction_id_trasnsmit / 256;
	buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = transaction_id_trasnsmit / 256;
	transaction_id_trasnsmit++;
	/* byte protocol id */
	buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = 0x00;
	buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = 0x00;
	/* byte message length */
	buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = 0x00;
	buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = 0x06;
	/* byte Unit id */
	buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = add_slave_;
	/* byte function */
	buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = 0x03;
	/* byte add start register */
	buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = add_start_register_tcp / 256;
	buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = (uint8_t) add_start_register_tcp % 256;
	/* byte number register */
	buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = number_register_tcp / 256;
	buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = (uint8_t) number_register_tcp % 256;
	/*transmit data over tcp */
	printf("modbus_tcp_readholding_register\r\n");
	tcp_sendata(buf_tx_modbus_tcp,count_data_tx_modbus_tcp);
	flag_request_or_response_func3_tcp = RECEIVE_RESPONSE;
}
/**
 * @brief        
 * 
 * @param
 */
void modbus_tcp_function3_response(uint16_t transaction, uint16_t add_start_register_tcp, uint16_t number_register_tcp)
{
	uint16_t count_data_tx_modbus_tcp = 0;
	uint16_t add_start_register_send = 0;
	if(add_start_register_tcp > modbus_number_register)
	{
		/* error add register start */
		printf("error add register start\r\n");
	}
	else
	{
		add_start_register_send = add_start_register_tcp;
		if(number_register_tcp > modbus_number_register)
		{
			/* error number register */
			printf("error number register\r\n");
		}
		else
		{
			if(add_start_register_tcp +  number_register_tcp > modbus_number_register)
			{
				/* error register */
				printf("error register\r\n");
			}
			else /* not error */
			{
				/* byte transaction id */
//				printf("transaction is %d\r\n",transaction);
				buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = transaction / 256;
				buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = transaction % 256;
				/* byte protocol id */
				buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = 0x00;
				buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = 0x00;
				/* message length */
//				printf("message length is %d\r\n",number_register_tcp*2 + 2);
				buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = (number_register_tcp*2 + 3) / 256;
				buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = (number_register_tcp*2 + 3) % 256;
				/* byte Unit id */
//				printf("add_slave_tcp is %d\r\n",add_slave_tcp);
				buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = add_slave;
				/* byte function id */
//				printf("afunction id is %d\r\n",0x03);
				buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = 0x03;
				/* byte byte folow */
//				printf("byte folow is %d\r\n",number_register_tcp*2);
				buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = (number_register_tcp*2);
				for(uint16_t count_data = 0; count_data < number_register_tcp; count_data++)
				{
					buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = modbus_register_40000[add_start_register_send + count_data] / 256;
					buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = (uint8_t) modbus_register_40000[add_start_register_send + count_data] % 256;
				}
				/*transmit data over tcp */
				tcp_sendata(buf_tx_modbus_tcp,count_data_tx_modbus_tcp);
			}
		}
	}
}

/**
 * @brief        
 * 
 * @param
 */
void modbus_tcp_read_input_register(uint8_t add_slave_, uint16_t add_start_register_tcp, uint16_t number_register_tcp)
{
	uint16_t count_data_tx_modbus_tcp = 0;
	/* add transaction id */
	buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = transaction_id_trasnsmit / 256;
	buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = transaction_id_trasnsmit / 256;
	transaction_id_trasnsmit++;
	/* byte protocol id */
	buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = 0x00;
	buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = 0x00;
	/* byte message length */
	buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = 0x00;
	buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = 0x06;
	/* byte Unit id */
	buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = add_slave_;
	/* byte function */
	buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = 0x04;
	/* byte add start register */
	buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = add_start_register_tcp / 256;
	buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = (uint8_t) add_start_register_tcp % 256;
	/* byte number register */
	buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = number_register_tcp / 256;
	buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = (uint8_t) number_register_tcp % 256;
	/*transmit data over tcp */
	printf("modbus_tcp_read_input_register\r\n");
	tcp_sendata(buf_tx_modbus_tcp,count_data_tx_modbus_tcp);
	flag_request_or_response_func4_tcp = RECEIVE_RESPONSE;
}

/**
 * @brief        
 * 
 * @param
 */
void modbus_tcp_function4_response(uint16_t transaction, uint16_t add_start_register_tcp, uint16_t number_register_tcp)
{
	uint16_t count_data_tx_modbus_tcp = 0;
	uint16_t add_start_register_send = 0;
	if(add_start_register_tcp > modbus_number_register)
	{
		/* error add register start */
		printf("error add register start\r\n");
	}
	else
	{
		add_start_register_send = add_start_register_tcp;
		if(number_register_tcp > modbus_number_register)
		{
			/* error number register */
			printf("error number register\r\n");
		}
		else
		{
			if(add_start_register_tcp +  number_register_tcp > modbus_number_register)
			{
				/* error register */
				printf("error register\r\n");
			}
			else /* not error */
			{
				/* byte transaction id */
//				printf("transaction is %d\r\n",transaction);
				buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = transaction / 256;
				buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = transaction % 256;
				/* byte protocol id */
				buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = 0x00;
				buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = 0x00;
				/* message length */
//				printf("message length is %d\r\n",number_register_tcp*2 + 2);
				buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = (number_register_tcp*2 + 3) / 256;
				buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = (number_register_tcp*2 + 3) % 256;
				/* byte Unit id */
//				printf("add_slave_tcp is %d\r\n",add_slave_tcp);
				buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = add_slave;
				/* byte function id */
//				printf("afunction id is %d\r\n",0x03);
				buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = 0x04;
				/* byte byte folow */
//				printf("byte folow is %d\r\n",number_register_tcp*2);
				buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = (number_register_tcp*2);
				for(uint16_t count_data = 0; count_data < number_register_tcp; count_data++)
				{
					buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = modbus_register_30000[add_start_register_send + count_data] / 256;
					buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = (uint8_t) modbus_register_30000[add_start_register_send + count_data] % 256;
				}
				/*transmit data over tcp */
				printf("transmit data over tcp\r\n");
				tcp_sendata(buf_tx_modbus_tcp,count_data_tx_modbus_tcp);
			}
		}
	}
}

/**
 * @brief        
 * 
 * @param
 */ 
void modbus_tcp_writemultiple_coil(uint8_t add_slave_, uint16_t add_start_data_tcp, uint16_t number_coils_tcp, uint8_t* p_data)
{
	uint16_t count_data_tx_modbus_tcp = 0;
	uint16_t length = 0;
	uint16_t number_byte_data = 0;
	uint16_t check_coil_tcp = 0;
	/* add transaction id */
	buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = transaction_id_trasnsmit / 256;
	buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = transaction_id_trasnsmit / 256;
	transaction_id_trasnsmit++;
	/* byte protocol id */
	buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = 0x00;
	buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = 0x00;
	/* byte message length */
	check_coil_tcp = (uint8_t) number_coils_tcp % 8;
	if(check_coil_tcp != 0) number_byte_data = ((number_coils_tcp / 8) + 1);
	else number_byte_data = (number_coils_tcp / 8);
	length = 7 + number_byte_data;
	buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = length / 256;
	buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = (uint8_t) length % 256;
	/* byte Unit id */
	buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = add_slave_;
	/* byte function */
	buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = 0x0F;
	/* byte add start register */
	buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = add_start_data_tcp / 256;
	buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = (uint8_t) add_start_data_tcp % 256;
	/* byte number register */
	buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = number_coils_tcp / 256;
	buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = (uint8_t) number_coils_tcp % 256;
	/* byte length */
	buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = number_byte_data;
	/* byte data*/
	for(uint16_t count_data = 0; count_data < number_byte_data; count_data++)
	{
		buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = *p_data ;
		p_data++;
	}
//	printf("modbus_tcp_writemultiple_coils\r\n");
	tcp_sendata(buf_tx_modbus_tcp,count_data_tx_modbus_tcp);
	flag_request_or_response_func15_tcp = RECEIVE_RESPONSE;
}

/**
 * @brief        
 * 
 * @param
 */
void modbus_tcp_function15_response(uint16_t transaction, uint8_t add_slave_, uint16_t add_start_data_tcp, uint16_t number_coils_tcp)
{
	uint16_t count_data_tx_modbus_tcp = 0;
	/* byte transaction id */
	buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = transaction / 256;
	buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = (uint8_t) transaction % 256;
	/* byte protocol id */
	buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = 0x00;
	buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = 0x00;
	/* byte message length */
	buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = 0x00;
	buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = 0x06;
	/* byte Unit id */
	buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = add_slave_;
	/* byte function */
	buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = 0x0F;
	/* byte add start register */
	buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = add_start_data_tcp / 256;
	buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = (uint8_t) add_start_data_tcp % 256;
	/* byte number register */
	buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = number_coils_tcp / 256;
	buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = (uint8_t) number_coils_tcp % 256;
	/*transmit data over tcp */
	printf("modbus_tcp_function15_response\r\n");
	tcp_sendata(buf_tx_modbus_tcp,count_data_tx_modbus_tcp);
}
/**
 * @brief        
 * 
 * @param
 */
void modbus_tcp_writemultiple_register(uint8_t add_slave_, uint16_t add_start_register_tcp, uint16_t number_register_tcp, uint16_t* p_data)
{
	uint16_t count_data_tx_modbus_tcp = 0;
	uint16_t length = 0;
	/* add transaction id */
	buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = transaction_id_trasnsmit / 256;
	buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = transaction_id_trasnsmit / 256;
	transaction_id_trasnsmit++;
	/* byte protocol id */
	buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = 0x00;
	buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = 0x00;
	/* byte message length */
	length = 7 + number_register_tcp*2;
	buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = length / 256;
	buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = (uint8_t) length % 256;
	/* byte Unit id */
	buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = add_slave_;
	/* byte function */
	buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = 0x10;
	/* byte add start register */
	buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = add_start_register_tcp / 256;
	buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = (uint8_t) add_start_register_tcp % 256;
	/* byte number register */
	buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = number_register_tcp / 256;
	buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = (uint8_t) number_register_tcp % 256;
	/* byte length */
	buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = number_register_tcp * 2;
	/* byte data*/
	for(uint16_t count_data = 0; count_data < number_register_tcp; count_data++)
	{
		buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = (*p_data) / 256;
		buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = (uint8_t) (*p_data) % 256;
		p_data++;
	}
	/*transmit data over tcp */
//	printf("modbus_tcp_writemultiple_register\r\n");
	tcp_sendata(buf_tx_modbus_tcp,count_data_tx_modbus_tcp);
	flag_request_or_response_func16_tcp = RECEIVE_RESPONSE;
}

/**
 * @brief        
 * 
 * @param
 */
void modbus_tcp_function16_response(uint16_t transaction, uint8_t add_slave_, uint16_t add_start_register, uint16_t number_register)
{
	uint16_t count_data_tx_modbus_tcp = 0;
	/* byte transaction id */
	buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = transaction / 256;
	buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = (uint8_t) transaction % 256;
	/* byte protocol id */
	buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = 0x00;
	buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = 0x00;
	/* byte message length */
	buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = 0x00;
	buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = 0x06;
	/* byte Unit id */
	buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = add_slave_;
	/* byte function */
	buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = 0x10;
	/* byte add start register */
	buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = add_start_register / 256;
	buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = (uint8_t) add_start_register % 256;
	/* byte number register */
	buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = number_register / 256;
	buf_tx_modbus_tcp[count_data_tx_modbus_tcp++] = (uint8_t) number_register % 256;
	/*transmit data over tcp */
	printf("modbus_tcp_function16_response\r\n");
	tcp_sendata(buf_tx_modbus_tcp,count_data_tx_modbus_tcp);
}
/**
 * @brief        
 * 
 * @param
 */
uint16_t mb_tcp_read_transaction_id(void)
{
	return read_uint16_queue_tcp();
}

/**
 * @brief        
 * 
 * @param
 */
uint16_t mb_tcp_read_protocol_id(void)
{
	return read_uint16_queue_tcp();
}

/**
 * @brief        
 * 
 * @param
 */
uint16_t mb_tcp_read_message_length(void)
{
	return read_uint16_queue_tcp();
}
/**
 * @brief        
 * 
 * @param
 */
uint8_t mb_tcp_read_addslave(void)
{
	return read_uint8_queue_tcp();
}
/**
 * @brief        
 * 
 * @param
 */
uint8_t mb_tcp_read_function(void)
{
	return read_uint8_queue_tcp();
}

/**
 * @brief        
 * 
 * @param
 */
uint16_t mb_tcp_read_add_register_start(void)
{
	return read_uint16_queue_tcp();
}

/**
 * @brief        
 * 
 * @param
 */
uint16_t mb_tcp_read_number_register(void)
{
	return read_uint16_queue_tcp();
}

/**
 * @brief        
 * 
 * @param
 */
uint16_t mb_tcp_read_value_register(void)
{
	return read_uint16_queue_tcp();
}

/**
 * @brief        
 * 
 * @param
 */
uint8_t mb_tcp_read_byte_coil(void)
{
	return read_uint8_queue_tcp();
}

/**
 * @brief        
 * 
 * @param
 */
uint8_t mb_tcp_read_number_byte_follow(void)
{
	return read_uint8_queue_tcp();
}

/*********************************************************************************
 * STATIC FUNCTION
 */
static uint8_t read_uint8_queue_tcp(void)
{
	/* init variable */
	uint8_t data_return;
//	osEvent data_modbusRx_Queue;
	/* get first byte data */
//	data_modbusRx_Queue = osMessageGet(modbusTcpRxQueueHandle,1);
//	data_return = (uint8_t) data_modbusRx_Queue.value.v;
	circular_buf_get(&cbuf_tcp,&data_return);
	/* return value */
	return data_return;
}

static uint16_t read_uint16_queue_tcp(void)
{
  /* init variable */
	uint8_t data_1;
	uint8_t data_2;
	uint16_t data_return;
//	osEvent data_modbusRx_Queue;
	/* get first byte data */
//	data_modbusRx_Queue = osMessageGet(modbusTcpRxQueueHandle,1);
//	data_1 = (uint8_t) data_modbusRx_Queue.value.v;
//	data_modbusRx_Queue = osMessageGet(modbusTcpRxQueueHandle,1);
//	data_2 = (uint8_t) data_modbusRx_Queue.value.v;
	circular_buf_get(&cbuf_tcp,&data_1);
	circular_buf_get(&cbuf_tcp,&data_2);
	/* return value */
	data_return = data_1*256 + data_2;
	return data_return;
}
