/*
 * hm10.c
 *
 *  Created on: Feb 8, 2025
 *      Author: PC
 */
#include "hm10.h"
#include "hm10_uart.h"
#include "stdio.h"

static char AT_Array[20];
volatile char uart1_rec;

void setBuadRate(HM10_BuadRate_Typedef baud)
{

	sprintf(AT_Array,"AT+BAUD%d\r\n",baud);
	hm10_write_at_command(AT_Array);
}

void setRole(HM10_Role_Typedef role)
{
	sprintf(AT_Array,"AT+ROLE%d\r\n",role);
	hm10_write_at_command(AT_Array);
}

void setName(char *c)
{
	sprintf(AT_Array,"AT+NAME%s\r\n",c);
	hm10_write_at_command(AT_Array);
}

void FactoryReset()
{
	sprintf(AT_Array,"AT+RENEW\r\n");
	hm10_write_at_command(AT_Array);
}

void moduleReset()
{
	sprintf(AT_Array,"AT+RESET\r\n");
	hm10_write_at_command(AT_Array);
}

void USART1_IRQHandler (void)
{

	if(USART1->SR & USART_SR_RXNE){
	uart1_rec=USART1->DR;

	USART1->SR &= ~USART_SR_RXNE;}


}



