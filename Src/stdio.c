/*
 CREAT BY:https://letanphuc.net/2015/09/stm32f0-uart-tutorial-5/
*/

#include <stdio.h>
#include "usart.h"
#include <stdbool.h>

UART_HandleTypeDef*  DEBUG = &huart3;

/* Private function prototypes -----------------------------------------------*/
#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE  
{
    /* Place your implementation of fputc here */
    /* e.g. write a character to the USART */
    HAL_UART_Transmit(DEBUG, (uint8_t *)&ch, 1, 100);
    return ch;
}
/* USER CODE END PFP */
