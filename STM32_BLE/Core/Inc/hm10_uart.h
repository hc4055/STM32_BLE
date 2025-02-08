/*
 * hm10_uart.h
 *
 *  Created on: Feb 8, 2025
 *      Author: PC
 */

#ifndef INC_HM10_UART_H_
#define INC_HM10_UART_H_

#include "stdint.h"

void hm10_uart_init(uint32_t baud,uint32_t freq);

void hm10_write_char(unsigned char ch);

void hm10_write_at_command(unsigned char * ch);

void hm10_write_string(unsigned char * ch);

#endif /* INC_HM10_UART_H_ */
