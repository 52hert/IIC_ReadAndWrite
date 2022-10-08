#include "hdl_aw86224.h"
#include "delay.h"


static void Pin_Init(void);

//初始化aw86224接口
void AW86224_Init(void)
{
	IIC_Init(); // IIC初始化
	Pin_Init();
}

static void Pin_Init(void)
{
	GPIO_InitTypeDef GPIO_Initure;
	__HAL_RCC_GPIOG_CLK_ENABLE();

	GPIO_Initure.Pin = AW86224_Trig_Pin;	   // PB5
	GPIO_Initure.Mode = GPIO_MODE_OUTPUT_PP;   //推挽输出
	GPIO_Initure.Pull = GPIO_PULLUP;		   //上拉
	GPIO_Initure.Speed = GPIO_SPEED_FREQ_HIGH; //高速
	HAL_GPIO_Init(AW86224_Trig_Port, &GPIO_Initure);
	HAL_GPIO_WritePin(AW86224_Trig_Port, AW86224_Trig_Pin, GPIO_PIN_RESET); // PB5清0，默认初始化后灯灭
}

u8 AT24CXX_ReadOneByte(u16 ReadAddr)
{
	u8 temp = 0;
	IIC_Start();
	IIC_Send_Byte(0xB0 + ((ReadAddr / 256) << 1)); //发送器件地址0XA0,写数据
	IIC_Wait_Ack();
	IIC_Send_Byte(ReadAddr % 256); //发送低地址
	IIC_Wait_Ack();
	IIC_Start();
	IIC_Send_Byte(0xB1); //进入接收模式
	IIC_Wait_Ack();
	temp = IIC_Read_Byte(0);
	IIC_Stop(); //产生一个停止条件
	return temp;
}

void AT24CXX_WriteOneByte(u16 WriteAddr, u8 DataToWrite)
{
	IIC_Start();
	IIC_Send_Byte(0xB0 + ((WriteAddr / 256) << 1)); //发送器件地址0XA0,写数据
	IIC_Wait_Ack();
	IIC_Send_Byte(WriteAddr % 256); //发送低地址
	IIC_Wait_Ack();
	IIC_Send_Byte(DataToWrite); //发送字节
	IIC_Wait_Ack();
	IIC_Stop(); //产生一个停止条件
	delay_ms(10);
}

uint8_t AW86224_WriteLenByte(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data)
{
	uint8_t count = 0;

	// osSemaphoreAcquire(i2c2_semaphore, osWaitForever);	/**< 获取I2C资源 */

	IIC_Start();
	IIC_Send_Byte(dev); //!< 发送写命令
	IIC_Wait_Ack();
	IIC_Send_Byte(reg); //!< 发送地址
	IIC_Wait_Ack();
	for (count = 0; count < length; count++)
	{
		IIC_Send_Byte(data[count]);
		IIC_Wait_Ack();
	}
	IIC_Stop(); ///< 产生一个停止条件

	// osSemaphoreRelease(i2c2_semaphore); 		/**< 释放I2C资源 */

	return 1; //!< status == 0;
}

u32 AW86224FCR_ReadLenByte(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data)
{
	uint8_t count = 0;

	// osSemaphoreAcquire(i2c2_semaphore, osWaitForever);	/**< 获取I2C资源 */

	IIC_Start();
	IIC_Send_Byte(dev); //!< 发送写命令
	if (IIC_Wait_Ack() != 0)
	{
		// osSemaphoreRelease(i2c2_semaphore); 		/**< 释放I2C资源 */
		return 0; // iic通讯失败
	}
	IIC_Send_Byte(reg); //!< 发送地址
	if (IIC_Wait_Ack() != 0)
	{
		// osSemaphoreRelease(i2c2_semaphore); 		/**< 释放I2C资源 */
		return 0; // iic通讯失败
	}
	IIC_Start();
	IIC_Send_Byte(dev + 1); //!< 进入接收模式
	if (IIC_Wait_Ack() != 0)
	{
		// osSemaphoreRelease(i2c2_semaphore); 		/**< 释放I2C资源 */
		return 0; // iic通讯失败
	}

	for (count = 0; count < length; count++)
	{
		if (count != length - 1)
			data[count] = IIC_Read_Byte(1); //!< 带ACK的读数据
		else
			data[count] = IIC_Read_Byte(0); //!< 最后一个字节NACK
	}
	IIC_Stop();

	// osSemaphoreRelease(i2c2_semaphore); 		/**< 释放I2C资源 */

	return count;
}

//返回1:检测失败
//返回0:检测成功
u8 AT24CXX_Check(void)
{
	u8 temp;
	temp = AT24CXX_ReadOneByte(0x0A); //避免每次开机都写AT24CXX
	if (temp == 0X55)
		return 0;
	else //排除第一次初始化的情况
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
