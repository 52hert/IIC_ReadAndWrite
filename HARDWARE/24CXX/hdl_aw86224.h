#ifndef _24CXX_H
#define _24CXX_H
#include "sys.h"
#include "myiic.h"

#define AW86224_Trig_Port GPIOG
#define AW86224_Trig_Pin GPIO_PIN_6

u8 AT24CXX_ReadOneByte(u16 ReadAddr);                                                  //ָ����ַ��ȡһ���ֽ�
void AT24CXX_WriteOneByte(u16 WriteAddr, u8 DataToWrite);                              //ָ����ַд��һ���ֽ�
void AT24CXX_WriteLenByte(u16 WriteAddr, u32 DataToWrite, u8 Len);                     //ָ����ַ��ʼд��ָ�����ȵ�����
u32 AW86224FCR_ReadLenByte(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data);   //ָ����ַ��ʼ��ȡָ����������
void AT24CXX_Write(u16 WriteAddr, u8 *pBuffer, u16 NumToWrite);                        //��ָ����ַ��ʼд��ָ�����ȵ�����
uint8_t AW86224_WriteLenByte(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data); //��ָ����ַ��ʼ����ָ�����ȵ�����
// void AT24CXX_Read(u16 ReadAddr, u8 *pBuffer, u16 NumToRead);

u8 AT24CXX_Check(void);  //�������
void AW86224_Init(void); //��ʼ��IIC

/* ʵ�����º�����ȫ���滻 */
void hdl_motor_init(void);   //����ʼ��
void hdl_motor_uninit(void); //
void hdl_motor_run(uint32_t period, uint32_t time, uint32_t cnt);
void hdl_motor_stop(void); //�������ʱǿ��ֹͣ

#endif
