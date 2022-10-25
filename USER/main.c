#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "hdl_aw86224.h"
#include "haptic_nv.h"
int dadaa = 0;

int main(void)
{
	HAL_Init();						//��ʼ��HAL��
	Stm32_Clock_Init(RCC_PLL_MUL9); //����ʱ��,72M
	delay_init(72);					//��ʼ����ʱ����
	uart_init(115200);				//��ʼ������

	AW86224_Init();			//��ʼ��IIC
	while (AT24CXX_Check()) //��ⲻ��
	{
		delay_ms(500);
		AT24CXX_Check();
	}
	printf(".\r\n"); //���ڲ���ʱ��
	haptic_nv_boot_init();
	printf(".\r\n");
	while (1)
	{
		HAL_GPIO_WritePin(AW_RST_GPIO_Port, AW_RST_Pin, GPIO_PIN_RESET); // PB5��0��Ĭ�ϳ�ʼ�������
		delay_ms(1000);
		HAL_GPIO_WritePin(AW_RST_GPIO_Port, AW_RST_Pin, GPIO_PIN_SET); // PB5��0��Ĭ�ϳ�ʼ�������
		delay_ms(1000);
	}
}
