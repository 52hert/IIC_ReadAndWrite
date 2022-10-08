#ifndef _24CXX_H
#define _24CXX_H
#include "sys.h"
#include "myiic.h"

#define AW86224_Trig_Port GPIOG
#define AW86224_Trig_Pin GPIO_PIN_6

u8 AT24CXX_ReadOneByte(u16 ReadAddr);                                                  //指定地址读取一个字节
void AT24CXX_WriteOneByte(u16 WriteAddr, u8 DataToWrite);                              //指定地址写入一个字节
void AT24CXX_WriteLenByte(u16 WriteAddr, u32 DataToWrite, u8 Len);                     //指定地址开始写入指定长度的数据
u32 AW86224FCR_ReadLenByte(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data);   //指定地址开始读取指定长度数据
void AT24CXX_Write(u16 WriteAddr, u8 *pBuffer, u16 NumToWrite);                        //从指定地址开始写入指定长度的数据
uint8_t AW86224_WriteLenByte(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data); //从指定地址开始读出指定长度的数据
// void AT24CXX_Read(u16 ReadAddr, u8 *pBuffer, u16 NumToRead);

u8 AT24CXX_Check(void);  //检查器件
void AW86224_Init(void); //初始化IIC

/* 实现以下函数，全文替换 */
void hdl_motor_init(void);   //马达初始化
void hdl_motor_uninit(void); //
void hdl_motor_run(uint32_t period, uint32_t time, uint32_t cnt);
void hdl_motor_stop(void); //在马达震动时强制停止

#endif
