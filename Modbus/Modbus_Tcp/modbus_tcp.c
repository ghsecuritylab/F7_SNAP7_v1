/** @file
  * 
  * @brief modbus tcp
  *
  */
	
/*********************************************************************************
 * INCLUDE
 */
/* system include */
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#include "lwip.h"
#include "api.h"
#include <string.h>

/* user include */
#include "modbus_tcp.h"
#include "modbus_tcp_func.h"
#include "modbus_constant.h"

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
extern uint16_t modbus_register_40000[MAX_REGISTES];
extern uint16_t modbus_register_30000[MAX_REGISTES];

extern osMessageQId modbusTcpRxQueueHandle;
extern struct netif gnetif;
extern void StartNetTask(void const * argument);
/*********************************************************************************
 * MACRO
 */

/*********************************************************************************
 * STATIC VARIABLE
 */
static uint8_t count_error = 0;
static uint8_t config_tcp_mode;
struct netconn * nc;
struct netconn * in_nc;
volatile err_t res;

/*********************************************************************************
 * GLOBAL VARIABLE
 */
osThreadId netTaskHandle;
osThreadId TcpTaskHandle;
struct netconn *nc_com;

/* byte get transaction id */
uint16_t byte_get_transaction_id;
/* byte get protocol id */
uint16_t byte_get_protocol_id;
/* byte get message length */
uint16_t byte_get_message_len;
/* byte get address slave */
uint8_t byte_get_add_slave_tcp;
/* byte get function */
uint8_t byte_get_function_tcp;
/* byte get add register start */
uint16_t add_register_start_tcp;
/* byte get number register */
uint16_t number_register_tcp;
/* byte get number byte folow */
uint8_t number_byte_follow_tcp;
uint16_t buf_register_tcp[MAX_REGISTES];

/* flag receive request or response */
uint8_t count_flag_receive_success_tcp = 0;
uint8_t flag_request_or_response_func1_tcp = RECEIVE_REQUEST;
uint8_t flag_request_or_response_func2_tcp = RECEIVE_REQUEST;
uint8_t flag_request_or_response_func3_tcp = RECEIVE_REQUEST;
uint8_t flag_request_or_response_func4_tcp = RECEIVE_REQUEST;
uint8_t flag_request_or_response_func15_tcp = RECEIVE_REQUEST;
uint8_t flag_request_or_response_func16_tcp = RECEIVE_REQUEST;

/*********************************************************************************
 * STATIC FUNCTION
 */

static void StartTcpTask(void const * argument);
static void StartnetTask(void const * argument);
static void function1_handle_tcp(void);
static void function2_handle_tcp(void);
static void function3_handle_tcp(void);
static void function4_handle_tcp(void);
static void function15_handle_tcp(void);
static void function16_handle_tcp(void);
/*********************************************************************************
 * GLOBAL FUNCTION
 */

/**
 * @brief        
 * 
 * @param
 */
void modbus_tcp_port_init(uint8_t tcp_modde)
{
	if(tcp_modde == TCP_CLIENT)
	{
		printf("SET TCP_MODE_CLIENT\r\n");
		config_tcp_mode =  TCP_CLIENT;
	}
	
	if(tcp_modde == TCP_SERVER)
	{
		printf("SET TCP_MODE_SERVER\r\n");
		config_tcp_mode =  TCP_SERVER;
	}

	/* Start Tcp task init */
	if(tcp_modde == TCP_SERVER || tcp_modde == TCP_CLIENT)
	{
		osThreadDef(TcpTask, StartTcpTask, osPriorityNormal, 0, 256);
		TcpTaskHandle = osThreadCreate(osThread(TcpTask), NULL);
	}
}

/**
 * @brief        
 * 
 * @param
 */
void modbus_tcp_check_input(void)
{
	if(count_flag_receive_success_tcp != 0)
	{
		/* byte get transaction id */
		byte_get_transaction_id = mb_tcp_read_transaction_id();
		/* byte get protocol id */
		byte_get_protocol_id = mb_tcp_read_protocol_id();
		/* byte get message length */
		byte_get_message_len = mb_tcp_read_message_length();
		/* byte get address slave */
		byte_get_add_slave_tcp = mb_tcp_read_addslave();
		/* check add slave */
		if (byte_get_add_slave_tcp == add_slave)
		{
			/* byte get function */
			byte_get_function_tcp = mb_tcp_read_function();
			
			switch(byte_get_function_tcp)
			{
				case FUNCTION_01:
					function1_handle_tcp();
					count_flag_receive_success_tcp-=1;
					break;
				case FUNCTION_02:
					function2_handle_tcp();
					count_flag_receive_success_tcp-=1;
					break;
				case FUNCTION_03:
					function3_handle_tcp();
				  count_flag_receive_success_tcp-=1;
					break;
				case FUNCTION_04:
					function4_handle_tcp();
					count_flag_receive_success_tcp-=1;
					break;
				case FUNCTION_15:
					function15_handle_tcp();
					count_flag_receive_success_tcp-=1;
					break;
				case FUNCTION_16:
					function16_handle_tcp();
				  count_flag_receive_success_tcp-=1;
					break;
			}
		}
		else
		{
			/* error add slave */
			printf("modbus tcp error add slave \r\n");
			/* clear data on queue */
		}
	}
}

/*********************************************************************************
 * STATIC FUNCTION
 */
static void function1_handle_tcp(void)
{
	if(flag_request_or_response_func1_tcp == RECEIVE_REQUEST)
	{
		printf("function1_handle RECEIVE_REQUEST\r\n");
		/* get data add register start */
		add_register_start_tcp = mb_tcp_read_add_register_start();
		/* get data number register */
		number_register_tcp = mb_tcp_read_number_register();
		/* handle */
		modbus_tcp_function1_response(byte_get_transaction_id, add_register_start_tcp, number_register_tcp);
	}
	else
	{
		printf("function1_handle RECEIVE_REPONSE\r\n");
		/* The number of data bytes to follow */
		number_byte_follow_tcp = mb_tcp_read_number_byte_follow();
		for(uint16_t count = 0; count < number_byte_follow_tcp; count++)
		{
			buf_register_tcp[count] = mb_tcp_read_byte_coil();
			printf("buf_register_tcp_func1[%d] is %d\r\n",count,buf_register_tcp[count]);
			/*printf and handle*/
		}
		flag_request_or_response_func1_tcp = RECEIVE_REQUEST;
	}
}

static void function2_handle_tcp(void)
{
	if(flag_request_or_response_func2_tcp == RECEIVE_REQUEST)
	{
		printf("function2_handle RECEIVE_REQUEST\r\n");
		/* get data add register start */
		add_register_start_tcp = mb_tcp_read_add_register_start();
		/* get data number register */
		number_register_tcp = mb_tcp_read_number_register();
		/* handle */
		modbus_tcp_function2_response(byte_get_transaction_id, add_register_start_tcp, number_register_tcp);
	}
	else
	{
		printf("function2_handle RECEIVE_REPONSE\r\n");
		/* The number of data bytes to follow */
		number_byte_follow_tcp = mb_tcp_read_number_byte_follow();
		for(uint16_t count = 0; count < number_byte_follow_tcp; count++)
		{
			buf_register_tcp[count] = mb_tcp_read_byte_coil();
			printf("buf_register_tcp_func2[%d] is %d\r\n",count,buf_register_tcp[count]);
		}
		flag_request_or_response_func2_tcp = RECEIVE_REQUEST;
	}
}

static void function3_handle_tcp(void)
{
	if(flag_request_or_response_func3_tcp == RECEIVE_REQUEST)
	{
		printf("function3_handle RECEIVE_REQUEST\r\n");
		/* get data add register start */
		add_register_start_tcp = mb_tcp_read_add_register_start();
		/* get data number register */
		number_register_tcp = mb_tcp_read_number_register();
		/* handle */
		modbus_tcp_function3_response(byte_get_transaction_id, add_register_start_tcp,number_register_tcp);
	}
	else
	{
		printf("function3_handle RECEIVE_REPONSE\r\n");
		/* The number of data bytes to follow */
		number_byte_follow_tcp = mb_tcp_read_number_byte_follow();
		for(uint16_t count = 0; count < (number_byte_follow_tcp / 2); count++)
		{
			buf_register_tcp[count] = mb_tcp_read_value_register();
			printf("buf_register_tcp_func3[%d] is %d\r\n",count,buf_register_tcp[count]);
		}
		flag_request_or_response_func3_tcp = RECEIVE_REQUEST;
	}
}

static void function4_handle_tcp(void)
{
	if(flag_request_or_response_func4_tcp == RECEIVE_REQUEST)
	{
		printf("function4_handle RECEIVE_REQUEST\r\n");
		/* get data add register start */
		add_register_start_tcp = mb_tcp_read_add_register_start();
		/* get data number register */
		number_register_tcp = mb_tcp_read_number_register();
		/* handle */
		modbus_tcp_function4_response(byte_get_transaction_id, add_register_start_tcp,number_register_tcp);
	}
	else
	{
		printf("function4_handle RECEIVE_REPONSE\r\n");
		/* The number of data bytes to follow */
		number_byte_follow_tcp = mb_tcp_read_number_byte_follow();
		for(uint16_t count = 0; count < (number_byte_follow_tcp / 2); count++)
		{
			buf_register_tcp[count] = mb_tcp_read_value_register();
			printf("buf_register_tcp_func4[%d] is %d\r\n",count,buf_register_tcp[count]);
		}
		flag_request_or_response_func4_tcp = RECEIVE_REQUEST;
	}
}

static void function15_handle_tcp(void)
{
	if(flag_request_or_response_func15_tcp == RECEIVE_REQUEST)
	{
		printf("function15_handle RECEIVE_REQUEST\r\n");
		/* get data add register start */
		add_register_start_tcp = mb_tcp_read_add_register_start();
		/* get data number register */
		number_register_tcp = mb_tcp_read_number_register();
		/* byte byte folow */
		number_byte_follow_tcp = mb_tcp_read_number_byte_follow();
		if(add_register_start_tcp > modbus_number_coil || number_register_tcp > modbus_number_coil \
			 || add_register_start_tcp +  number_register_tcp > modbus_number_coil)
		{
			/* error */
			printf("error function 15 receive request\r\n");
		}
		else
		{
			
			/*send Response */
			modbus_tcp_function15_response(byte_get_transaction_id, add_slave, add_register_start_tcp, number_register_tcp);
			/* update register */
			for(uint16_t count = add_register_start_tcp; count < number_register_tcp; count++)
			{
//				modbus_register_tcp_00000[count] = mb_tcp_read_byte_coil();
				uint8_t byte_buff = mb_tcp_read_byte_coil();
				modbus_register_00000[count++] = byte_buff & 0x01;
//				printf("modbus_register_00000[%d] is %d\r\n",count,modbus_register_00000[count]);
				modbus_register_00000[count++] = byte_buff & 0x02;
//				printf("modbus_register_00000[%d] is %d\r\n",count,modbus_register_00000[count]);
				modbus_register_00000[count++] = byte_buff & 0x04;
//				printf("modbus_register_00000[%d] is %d\r\n",count,modbus_register_00000[count]);
				modbus_register_00000[count++] = byte_buff & 0x08;
//				printf("modbus_register_00000[%d] is %d\r\n",count,modbus_register_00000[count]);
				modbus_register_00000[count++] = byte_buff & 0x10;
//				printf("modbus_register_00000[%d] is %d\r\n",count,modbus_register_00000[count]);
				modbus_register_00000[count++] = byte_buff & 0x20;
//				printf("modbus_register_00000[%d] is %d\r\n",count,modbus_register_00000[count]);
				modbus_register_00000[count++] = byte_buff & 0x40;
//				printf("modbus_register_00000[%d] is %d\r\n",count,modbus_register_00000[count]);
				modbus_register_00000[count++] = byte_buff & 0x80;
//				printf("modbus_register_00000[%d] is %d\r\n",count,modbus_register_00000[count]);
			}
		}
	}
	else //if(flag_request_or_response_func15_tcp == RECEIVE_REQUEST)
	{
		/* co the doc ve de xu ly */
		/* hien tai thi khong lam gi voi du lieu nay*/
		/* clead data */
		printf("function16_handle RECEIVE_RESPONSE\r\n");
		number_register_tcp = mb_tcp_read_add_register_start();
		number_register_tcp = mb_tcp_read_number_register();
		flag_request_or_response_func16_tcp = RECEIVE_REQUEST;
	}
}

static void function16_handle_tcp(void)
{
	if(flag_request_or_response_func16_tcp == RECEIVE_REQUEST)
	{
		printf("function16_handle RECEIVE_REQUEST\r\n");
		/* get data add register start */
		add_register_start_tcp = mb_tcp_read_add_register_start();
		/* get data number register */
		number_register_tcp = mb_tcp_read_number_register();
		/* byte byte folow */
		number_byte_follow_tcp = mb_tcp_read_number_byte_follow();
		if(add_register_start_tcp > modbus_number_register || number_register_tcp > modbus_number_register \
			 || add_register_start_tcp +  number_register_tcp > modbus_number_register)
		{
			/* error */
			printf("error function 16 receive request\r\n");
		}
		else
		{
			
			/*send Response */
			modbus_tcp_function16_response(byte_get_transaction_id, add_slave, add_register_start_tcp, number_register_tcp);
			/* update register */
			for(uint16_t count = add_register_start_tcp; count < number_register_tcp; count++)
			{
				modbus_register_40000[count] = mb_tcp_read_value_register();
				printf("modbus_register_rtu[%d] is %d\r\n",count,modbus_register_40000[count]);
			}
		}
	}
	else //if(flag_request_or_response_func16_tcp == RECEIVE_REQUEST)
	{
		/* co the doc ve de xu ly */
		/* hien tai thi khong lam gi voi du lieu nay*/
		/* clead data */
		printf("function16_handle RECEIVE_RESPONSE\r\n");
		number_register_tcp = mb_tcp_read_add_register_start();
		number_register_tcp = mb_tcp_read_number_register();
		flag_request_or_response_func16_tcp = RECEIVE_REQUEST;
	}
}








/**
 * @brief        
 * 
 * @param
 */
static void StartTcpTask(void const * argument)
{
	/* init code for LWIP */
	
	char * buffer = pvPortMalloc(1024);
	
	ip_addr_t local_ip;
	ip_addr_t remote_ip;
	/* LWIP */ 
	MX_LWIP_Init();
	printf("LWIP init complete\r\n");
	/* wait gnetif ip_addr */
	while(gnetif.ip_addr.addr == 0)
  {
		osDelay(10);
		printf("LWIP init failed\r\n");
	}
	printf("DHCP worked ip: %s\r\n",ip4addr_ntoa(&gnetif.ip_addr));
	/***************************************************************************************************/
	/* Set connect to TCP Server */
	if(config_tcp_mode == TCP_CLIENT) {
	printf("TCP_MODE_CLIENT\r\n");
	local_ip = gnetif.ip_addr;
	ip4addr_aton("192.168.1.101", &remote_ip);
	
	nc = netconn_new(NETCONN_TCP);
	if(nc == NULL)
	{
		printf("new error tcp client \r\n");
		while(1) osDelay(1);
	}
	
	res = netconn_bind(nc, &local_ip, 0);
	if(res != 0)
	{
		printf("bind error: %d\r\n",res);
		while(1) osDelay(1);
	}
	
	res = netconn_connect(nc, &remote_ip, 502);
	if(res != 0)
	{
		printf("connect error: %d\r\n",res);
		while(1) osDelay(1);
	}
	else
	{
		printf("connected\r\n");
	}
	
	osThreadDef(netTask, StartnetTask, osPriorityIdle, 0, 256);
	netTaskHandle = osThreadCreate(osThread(netTask), NULL );
}
	
	/* End set connect to TCP Server */
	/***************************************************************************************************/
	
	/***************************************************************************************************/
	/* Set connect to TCP Client */
	if(config_tcp_mode == TCP_SERVER){
	printf("TCP_MODE_SERVER\r\n");
	/* Set a new connect tcp*/
	nc = netconn_new(NETCONN_TCP);
	/* Bind a netconn to a specific local IP address and port 502*/
	res = netconn_bind(nc, IP_ADDR_ANY, 502);
	if(res != 0)
	{
		printf("bind net_conn error\r\n");
		while(1) osDelay(10);
	}
	else
	{
		printf("bind net_conn complete\r\n");
	}
	/* Set a TCP netconn into listen mode */
	res = netconn_listen(nc);
	if(res != 0)
	{
		printf("listen error\r\n");
		while(1) osDelay(10);
	}
	else
	{
		printf("listen net_conn complete\r\n");
	}
	}
	/* End set connect to TCP Client */
	/***************************************************************************************************/
	
	/* Loop netconn_accept */
	for(;;)
  {
		if( config_tcp_mode ==  TCP_CLIENT){
		osDelay(1);
		}
		
		if( config_tcp_mode ==  TCP_SERVER){
		res = netconn_accept(nc,&in_nc);
		printf("netconn_accept\r\n");
		if(res != 0)
		{
			printf("listen error\r\n");
//			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET);
//			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_RESET);
		}
		else
		{
			osThreadDef(netTask, StartnetTask, osPriorityIdle, 0, 256);
			netTaskHandle = osThreadCreate(osThread(netTask), (void*) in_nc);
//			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_SET);
//			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET);
			printf("incoming connection\r\n");
		}
	 }
  }
}

static void StartnetTask(void const * argument)
{
	/* USER CODE BEGIN StartNetTask */
	nc_com = (struct netconn*) argument;
	struct netbuf *nb;
	volatile err_t res_;

	
	uint16_t len;
	char * buffer = pvPortMalloc(1024);
	
	/* Infinite loop */
  for(;;)
  {
		if( config_tcp_mode ==  TCP_SERVER){
		/* get data eth */
    netconn_recv(nc_com, &nb);
		if(res_ != 0)
		{
		printf("recv error: %d\r\n",res_);
		while(1) osDelay(1);
		}
		else
		{
//			printf("recv successful\r\n");
//			printf("1 with len  = %d\r\n",len);
			len = netbuf_len(nb);
//			if(len == 0)
//			{
//				printf("2 with len = %d\r\n",len);
//			}
			netbuf_copy(nb, buffer, len);
			netbuf_delete(nb);
			/* put data to queue */
		for(uint8_t count = 0; count < len; count++)
		{
			osMessagePut(modbusTcpRxQueueHandle, buffer[count], 1);
		}
		count_flag_receive_success_tcp++;
		}
	}
		
		if( config_tcp_mode ==  TCP_CLIENT){
		res = netconn_recv(nc, &nb);
		if(res != 0)
		{
			printf("recv error: %d\r\n",res);
			while(1) osDelay(1);
		}
		else
		{
			len = netbuf_len(nb);
			netbuf_copy(nb, buffer, len);
			netbuf_delete(nb);
			/* put data to queue */
			for(uint8_t count = 0; count < len; count++)
			{
				osMessagePut(modbusTcpRxQueueHandle, buffer[count], 1);
			}
			count_flag_receive_success_tcp++;
		}
		}
		osDelay(1);
  }
}

void tcp_sendata(uint8_t *pData, uint16_t length)
{
	/*if connection by TCP SERVER */
	if( config_tcp_mode ==  TCP_CLIENT)
	{
		res = netconn_write(nc,pData,length, NETCONN_COPY);
	}
	
	/*if connection by TCP CLIENT */
	if( config_tcp_mode ==  TCP_SERVER)
	{
		res = netconn_write(nc_com,pData,length, NETCONN_COPY);
	}
	
	/* check res */
	if(res != ERR_OK)
	{
		printf("error connetion to client\r\n");
		count_error++;
		if(count_error == 254)
		{
			count_error = 0;
			netconn_close(nc);
			netconn_delete(nc);
			netconn_close(nc_com);
			netconn_delete(nc_com);
			/* Set a new connect tcp*/
		nc = netconn_new(NETCONN_TCP);
		/* Bind a netconn to a specific local IP address and port 502*/
		res = netconn_bind(nc, IP_ADDR_ANY, 502);
		if(res != 0)
		{
			printf("bind net_conn error\r\n");
			while(1) osDelay(10);
		}
		else
		{
			printf("bind net_conn complete\r\n");
		}
		/* Set a TCP netconn into listen mode */
		res = netconn_listen(nc);
		if(res != 0)
		{
			printf("listen error\r\n");
			while(1) osDelay(10);
		}
		else
		{
			printf("listen net_conn complete\r\n");
		}
			printf("netconn_delete to client ********************************\\r\n");
		}
	}
	else
	{
		count_error = 0;
	}
}
