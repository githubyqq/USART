#include "sys.h"
#include "led.h"
#include "usart.h"
#include "delay.h"

int main(void)
{	  
	uart1_init(115200);	//���ڳ�ʼ��
	delay_init();				//��ʱ������ʼ��
	LED_Init();		  	//��ʼ����LED���ӵ�Ӳ���ӿ�
	while(1)
	{
		printf("hello world\n");
		LED0=0;
		delay_ms(500);
		LED0=1;
	}
}

