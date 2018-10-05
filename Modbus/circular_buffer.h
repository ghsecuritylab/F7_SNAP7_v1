/** @file
  * 
  * @brief CIRCULAR BUFFER 
  *
  */
#ifndef _CIRCULAR_BUFFER_H
#define _CIRCULAR_BUFFER_H

/*********************************************************************************
 * INCLUDE
 */
 
#include "stm32f7xx_hal.h"
#include <stdio.h>
#include <stdbool.h>

/*********************************************************************************
 * TYPEDEFS
 */
 
typedef struct{
	volatile uint8_t *buff;  // buffer data
	uint16_t head;           // head of circular buffer
	uint16_t tail;           // tail of circular buffer
	uint16_t size;           // size of circular buffer
	bool data_los;           // flag if loss data 
}	circular_buf_t;

/**
 * @brief        CIRCULAR BUFFER initialize circular buffer
 * 
 * @param[in]    *cbuf     address of circular buffer
 * @param[in]    *array    address array save data    
 * @param[in]    size      size of buffer
 */
void circular_buf_init(circular_buf_t *cbuf, volatile uint8_t *array, uint16_t size);

/**
 * @brief        CIRCULAR BUFFER reset circular buffer
 * 
 * @param[in]    *cbuf     address of circular buffer
 */
void circular_buf_reset(circular_buf_t *cbuf);

/**
 * @brief        check circular buffer full
 * 
 * @param[in]   *cbuf      address of circular buffer
 * 
 * @return 1    if circular buffer full
 */
bool circular_buf_full(circular_buf_t *cbuf);

/**
 * @brief        check circular buffer empty
 * 
 * @param[in]   *cbuf      address of circular buffer
 * 
 * @return 1    if circular buffer empty
 */
bool circular_buf_empty(circular_buf_t *cbuf);

/**
 * @brief        CIRCULAR BUFFER put data into circular buffer
 * 
 * @param[in]   *cbuf      address of circular buffer
 * @param[in]   data       data to put
 */
void circular_buf_put(circular_buf_t *cbuf, uint8_t data);

/**
 * @brief        CIRCULAR BUFFER get data of circular buffer
 * 
 * @param[in]   *cbuf      address of circular buffer
 * @param[in]   *data      address data to get
 */
void circular_buf_get(circular_buf_t *cbuf, uint8_t *data);

/**
 * @brief        CIRCULAR BUFFER get data of circular buffer
 * 
 * @param[in]   *cbuf      address of circular buffer
 * @param[in]   *data      address data to get
 */
uint8_t circular_buf_check(circular_buf_t *cbuf);


/**
 * @brief        CIRCULAR BUFFER count number data of circular buffer
 * 
 * @param[in]   *cbuf      address of circular buffer
 */
uint16_t circular_buf_count_data(circular_buf_t *cbuf);

/**
 * @brief        CIRCULAR BUFFER get data of circular buffer
 * 
 * @param[in]   *cbuf      address of circular buffer
 * @param[in]   *data      address data to get
 */
void circular_buf_get_array(circular_buf_t *cbuf, uint8_t *array, uint8_t size);

#endif /* _CIRCULAR_BUFFER_H */
