#ifndef __UAR_H
#define __UAR_H

#include "stdio.h"
#include "stm32f4xx_hal.h"

extern volatile uint8_t rx_len;
extern volatile uint8_t recv_end_flag;
extern volatile uint8_t rx_len_1;
extern volatile uint8_t recv_end_flag_1;
extern uint8_t  rx_buffer1[100];
extern uint8_t  rx_buffer[100];
extern char     BUFFER_SIZE;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart3;

void Data_Turn(void);
void Data_Turn1(void);


#endif

