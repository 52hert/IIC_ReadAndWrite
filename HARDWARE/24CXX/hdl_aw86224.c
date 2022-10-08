#include "hdl_aw86224.h"
#include "delay.h"


static void Pin_Init(void);

//��ʼ��aw86224�ӿ�
void AW86224_Init(void)
{
	IIC_Init(); // IIC��ʼ��
	Pin_Init();
}

static void Pin_Init(void)
{
	GPIO_InitTypeDef GPIO_Initure;
	__HAL_RCC_GPIOG_CLK_ENABLE();

	GPIO_Initure.Pin = AW86224_Trig_Pin;	   // PB5
	GPIO_Initure.Mode = GPIO_MODE_OUTPUT_PP;   //�������
	GPIO_Initure.Pull = GPIO_PULLUP;		   //����
	GPIO_Initure.Speed = GPIO_SPEED_FREQ_HIGH; //����
	HAL_GPIO_Init(AW86224_Trig_Port, &GPIO_Initure);
	HAL_GPIO_WritePin(AW86224_Trig_Port, AW86224_Trig_Pin, GPIO_PIN_RESET); // PB5��0��Ĭ�ϳ�ʼ�������
}

u8 AT24CXX_ReadOneByte(u16 ReadAddr)
{
	u8 temp = 0;
	IIC_Start();
	IIC_Send_Byte(0xB0 + ((ReadAddr / 256) << 1)); //����������ַ0XA0,д����
	IIC_Wait_Ack();
	IIC_Send_Byte(ReadAddr % 256); //���͵͵�ַ
	IIC_Wait_Ack();
	IIC_Start();
	IIC_Send_Byte(0xB1); //�������ģʽ
	IIC_Wait_Ack();
	temp = IIC_Read_Byte(0);
	IIC_Stop(); //����һ��ֹͣ����
	return temp;
}

void AT24CXX_WriteOneByte(u16 WriteAddr, u8 DataToWrite)
{
	IIC_Start();
	IIC_Send_Byte(0xB0 + ((WriteAddr / 256) << 1)); //����������ַ0XA0,д����
	IIC_Wait_Ack();
	IIC_Send_Byte(WriteAddr % 256); //���͵͵�ַ
	IIC_Wait_Ack();
	IIC_Send_Byte(DataToWrite); //�����ֽ�
	IIC_Wait_Ack();
	IIC_Stop(); //����һ��ֹͣ����
	delay_ms(10);
}

uint8_t AW86224_WriteLenByte(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data)
{
	uint8_t count = 0;

	// osSemaphoreAcquire(i2c2_semaphore, osWaitForever);	/**< ��ȡI2C��Դ */

	IIC_Start();
	IIC_Send_Byte(dev); //!< ����д����
	IIC_Wait_Ack();
	IIC_Send_Byte(reg); //!< ���͵�ַ
	IIC_Wait_Ack();
	for (count = 0; count < length; count++)
	{
		IIC_Send_Byte(data[count]);
		IIC_Wait_Ack();
	}
	IIC_Stop(); ///< ����һ��ֹͣ����

	// osSemaphoreRelease(i2c2_semaphore); 		/**< �ͷ�I2C��Դ */

	return 1; //!< status == 0;
}

u32 AW86224FCR_ReadLenByte(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data)
{
	uint8_t count = 0;

	// osSemaphoreAcquire(i2c2_semaphore, osWaitForever);	/**< ��ȡI2C��Դ */

	IIC_Start();
	IIC_Send_Byte(dev); //!< ����д����
	if (IIC_Wait_Ack() != 0)
	{
		// osSemaphoreRelease(i2c2_semaphore); 		/**< �ͷ�I2C��Դ */
		return 0; // iicͨѶʧ��
	}
	IIC_Send_Byte(reg); //!< ���͵�ַ
	if (IIC_Wait_Ack() != 0)
	{
		// osSemaphoreRelease(i2c2_semaphore); 		/**< �ͷ�I2C��Դ */
		return 0; // iicͨѶʧ��
	}
	IIC_Start();
	IIC_Send_Byte(dev + 1); //!< �������ģʽ
	if (IIC_Wait_Ack() != 0)
	{
		// osSemaphoreRelease(i2c2_semaphore); 		/**< �ͷ�I2C��Դ */
		return 0; // iicͨѶʧ��
	}

	for (count = 0; count < length; count++)
	{
		if (count != length - 1)
			data[count] = IIC_Read_Byte(1); //!< ��ACK�Ķ�����
		else
			data[count] = IIC_Read_Byte(0); //!< ���һ���ֽ�NACK
	}
	IIC_Stop();

	// osSemaphoreRelease(i2c2_semaphore); 		/**< �ͷ�I2C��Դ */

	return count;
}

//����1:���ʧ��
//����0:���ɹ�
u8 AT24CXX_Check(void)
{
	u8 temp;
	temp = AT24CXX_ReadOneByte(0x0A); //����ÿ�ο�����дAT24CXX
	if (temp == 0X55)
		return 0;
	else //�ų���һ�γ�ʼ�������
	{
		AT24CXX_WriteOneByte(0X0A, 0X55);
		temp = AT24CXX_ReadOneByte(0X0A);
		if (temp == 0X55)
			return 0;
	}
	return 1;
}


#if 0
void AT24CXX_Read(u16 ReadAddr, u8 *pBuffer, u16 NumToRead)
{

	while (NumToRead)
	{
		*pBuffer++ = AT24CXX_ReadOneByte(ReadAddr++);
		NumToRead--;
	}
}

void AT24CXX_Write(u16 WriteAddr, u8 *pBuffer, u16 NumToWrite)
{
	while (NumToWrite--)
	{
		AT24CXX_WriteOneByte(WriteAddr, *pBuffer);
		WriteAddr++;
		pBuffer++;
	}
}
#endif
