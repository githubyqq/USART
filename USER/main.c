#include "sys.h"
#include "led.h"
#include "usart.h"
#include "delay.h"

int main(void)
{	  
	uart1_init(115200);	//串口初始化
	delay_init();				//延时函数初始化
	LED_Init();		  	//初始化与LED连接的硬件接口
	while(1)
	{
		printf("hello world\n");
		LED0=0;
		delay_ms(500);
		LED0=1;
	}
}

