#include "myiic.h"
#include "delay.h"

#define AW86224_SDA_GPIO_Port GPIOD
#define AW86224_SDA_Pin GPIO_PIN_12
#define AW86224_SCL_GPIO_Port GPIOD
#define AW86224_SCL_Pin GPIO_PIN_13

// IIC��ʼ�����˶�
void IIC_Init(void)
{
	GPIO_InitTypeDef GPIO_Initure = {0}; //����

	__HAL_RCC_GPIOD_CLK_ENABLE(); //ʹ��GPIOBʱ��

	// PH4,5��ʼ������
	GPIO_Initure.Pin = AW86224_SDA_Pin | AW86224_SCL_Pin;
	GPIO_Initure.Mode = GPIO_MODE_OUTPUT_OD;   //�������
	GPIO_Initure.Pull = GPIO_NOPULL;		   //����
	GPIO_Initure.Speed = GPIO_SPEED_FREQ_HIGH; //����
	HAL_GPIO_Init(GPIOD, &GPIO_Initure);	   //��ʼ�������
}

void SDA_OUT(void)
{
	GPIO_InitTypeDef GPIO_Initure = {0}; //����

	// PH4,5��ʼ������
	GPIO_Initure.Pin = AW86224_SDA_Pin;
	GPIO_Initure.Mode = GPIO_MODE_OUTPUT_OD;   //�������
	GPIO_Initure.Pull = GPIO_NOPULL;		   //����
	GPIO_Initure.Speed = GPIO_SPEED_FREQ_HIGH; //����
	HAL_GPIO_Init(GPIOD, &GPIO_Initure);	   //��ʼ�������
}
void SDA_IN(void)
{
	GPIO_InitTypeDef GPIO_Initure = {0}; //����

	// PH4,5��ʼ������
	GPIO_Initure.Pin = AW86224_SDA_Pin;
	GPIO_Initure.Mode = GPIO_MODE_INPUT;	   //�������
	GPIO_Initure.Pull = GPIO_NOPULL;		   //����
	GPIO_Initure.Speed = GPIO_SPEED_FREQ_HIGH; //����
	HAL_GPIO_Init(GPIOD, &GPIO_Initure);	   //��ʼ�������
}

//����IIC��ʼ�ź� �˶�
void IIC_Start(void)
{
	SDA_OUT(); // sda�����
	IIC_SDA = 1;
	IIC_SCL = 1;
	delay_us(2);
	IIC_SDA = 0; // START:when CLK is high,DATA change form high to low
	delay_us(2);
	IIC_SCL = 0; //ǯסI2C���ߣ�׼�����ͻ��������
	delay_us(2);
}
//����IICֹͣ�ź� �˶�
void IIC_Stop(void)
{
	SDA_OUT(); // sda�����
	IIC_SCL = 0;
	IIC_SDA = 0; // STOP:when CLK is high DATA change form low to high
	delay_us(2);
	IIC_SCL = 1;
	IIC_SDA = 1; //����I2C���߽����ź�
	delay_us(2);
}
//�ȴ�Ӧ���źŵ��� �˶�
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
u8 IIC_Wait_Ack(void)
{
	u8 ucErrTime = 0;
	SDA_IN(); // SDA����Ϊ����
	IIC_SDA = 1;
	delay_us(1);
	IIC_SCL = 1;
	delay_us(1);
	while (READ_SDA)
	{
		ucErrTime++;
		if (ucErrTime > 250)
		{
			IIC_Stop();
			return 1;
		}
		delay_us(1);
	}
	IIC_SCL = 0; //ʱ�����0
	return 0;
}
//����ACKӦ�� �˶�
void IIC_Ack(void)
{
	IIC_SCL = 0;
	SDA_OUT();
	IIC_SDA = 0;
	delay_us(2);
	IIC_SCL = 1;
	delay_us(2);
	IIC_SCL = 0;
}
//������ACKӦ��		 �˶�
void IIC_NAck(void)
{
	IIC_SCL = 0;
	SDA_OUT();
	IIC_SDA = 1;
	delay_us(2);
	IIC_SCL = 1;
	delay_us(2);
	IIC_SCL = 0;
}
// IIC����һ���ֽ� �˶�
//���شӻ�����Ӧ��
// 1����Ӧ��
// 0����Ӧ��
void IIC_Send_Byte(u8 txd)
{
	u8 t;
	SDA_OUT();
	IIC_SCL = 0; //����ʱ�ӿ�ʼ���ݴ���
	delay_us(1);
	for (t = 0; t < 8; t++)
	{
		if ((txd & 0x80) >> 7)
		{
			IIC_SDA = 1;
		}
		else
			IIC_SDA = 0;

		txd <<= 1;
		delay_us(2);
		IIC_SCL = 1;
		delay_us(2);
		IIC_SCL = 0;
		delay_us(2);
	}
}
//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK
u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i, receive = 0;
	SDA_IN(); // SDA����Ϊ����
	for (i = 0; i < 8; i++)
	{
		IIC_SCL = 0;
		delay_us(2);
		IIC_SCL = 1;
		receive <<= 1;
		if (READ_SDA)
			receive++;
		delay_us(2);
	}
	if (!ack)
		IIC_NAck(); //����nACK
	else
		IIC_Ack(); //����ACK
	return receive;
}