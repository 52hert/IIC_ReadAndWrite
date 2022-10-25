#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "hdl_aw86224.h"
#include "haptic_nv.h"
int dadaa = 0;

int main(void)
{
	HAL_Init();						//初始化HAL库
	Stm32_Clock_Init(RCC_PLL_MUL9); //设置时钟,72M
	delay_init(72);					//初始化延时函数
	uart_init(115200);				//初始化串口

	AW86224_Init();			//初始化IIC
	while (AT24CXX_Check()) //检测不到
	{
		delay_ms(500);
		AT24CXX_Check();
	}
	printf(".\r\n"); //用于测试时间
	haptic_nv_boot_init();
	printf(".\r\n");
	while (1)
	{
		HAL_GPIO_WritePin(AW_RST_GPIO_Port, AW_RST_Pin, GPIO_PIN_RESET); // PB5清0，默认初始化后灯灭
		delay_ms(1000);
		HAL_GPIO_WritePin(AW_RST_GPIO_Port, AW_RST_Pin, GPIO_PIN_SET); // PB5清0，默认初始化后灯灭
		delay_ms(1000);
	}
}
