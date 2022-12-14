#ifndef _MYIIC_H
#define _MYIIC_H
#include "sys.h"
// IO操作
#define IIC_SCL PDout(13) // SCL
#define IIC_SDA PDout(12) // SDA
#define READ_SDA PDin(12) //输入SDA

// IIC所有操作函数
void IIC_Init(void);                 //初始化IIC的IO口
void IIC_Start(void);                //发送IIC开始信号
void IIC_Stop(void);                 //发送IIC停止信号
void IIC_Send_Byte(u8 txd);          // IIC发送一个字节
u8 IIC_Read_Byte(unsigned char ack); // IIC读取一个字节
u8 IIC_Wait_Ack(void);               // IIC等待ACK信号
void IIC_Ack(void);                  // IIC发送ACK信号
void IIC_NAck(void);                 // IIC不发送ACK信号

#endif
