#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "sys.h" 
	
#define BUFLEN 256      //���黺���С
typedef struct _UART_BUF
{
    char buf [BUFLEN+1];               
    unsigned int index ;
}UART_BUF;
	
#define USART_DEBUG		USART1
	
void uart1_init(u32 bound);
void uart2_init(u32 bound);
void UART1_send_byte(char data);
void UART2_send_byte(char data);
void Uart1_SendStr(char*SendBuf);
void Uart2_SendStr(char*SendBuf);
void Usart_SendString(USART_TypeDef *USARTx, unsigned char *str, unsigned short len);
void UsartPrintf(USART_TypeDef *USARTx, char *fmt,...);
extern UART_BUF buf_uart1;     //CH340
extern UART_BUF buf_uart2;     //NBIOT


#endif


