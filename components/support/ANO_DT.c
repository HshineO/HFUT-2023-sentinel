#include "ANO_DT.h"
//������λ��ͨ��Э��    ��pid�������õ�

#include "main.h"

#include "usart.h"


#define BYTE0(dwTemp) (*(char*)(&dwTemp))
#define BYTE1(dwTemp) (*((char*)(&dwTemp)+1))
#define BYTE2(dwTemp) (*((char*)(&dwTemp)+2))
#define BYTE3(dwTemp) (*((char*)(&dwTemp)+3))

uint8_t DataSendBuf[100];

/**
  * @brief          ��������������λ�������������ڲ��ε��ԡ�
  * @param[out]     chassis_move_transit:"chassis_move"����ָ��.
  * @retval         none
  */
void ANODT_SendF1(int32_t Data1,int32_t Data2,int32_t Data3,int32_t Data4,uint16_t Tot_len)//F1����ʽ֡
{
	uint8_t cnt=0;
	DataSendBuf[cnt++]=0xAA;  //֡ͷ
	DataSendBuf[cnt++]=0xFF;  //Ŀ���ַ
	DataSendBuf[cnt++]=0xF1;  //������
    DataSendBuf[cnt++]=Tot_len;  //���ݳ���
	
	DataSendBuf[cnt++]=BYTE0(Data1);
	DataSendBuf[cnt++]=BYTE1(Data1);
	DataSendBuf[cnt++]=BYTE2(Data1);
	DataSendBuf[cnt++]=BYTE3(Data1);

	DataSendBuf[cnt++]=BYTE0(Data2);
	DataSendBuf[cnt++]=BYTE1(Data2);
	DataSendBuf[cnt++]=BYTE2(Data2);
	DataSendBuf[cnt++]=BYTE3(Data2);
	
	DataSendBuf[cnt++]=BYTE0(Data3);
	DataSendBuf[cnt++]=BYTE1(Data3);
	DataSendBuf[cnt++]=BYTE2(Data3);
	DataSendBuf[cnt++]=BYTE3(Data3);
	
	DataSendBuf[cnt++]=BYTE0(Data4);
	DataSendBuf[cnt++]=BYTE1(Data4);
	DataSendBuf[cnt++]=BYTE2(Data4);
	DataSendBuf[cnt++]=BYTE3(Data4);
	
	uint8_t sc=0;  //��У��
	uint8_t ac=0;  //����У��
	for(uint8_t i=0;i<DataSendBuf[3]+4;i++)
	{
		sc+=DataSendBuf[i];
		ac+=sc;
	}
	DataSendBuf[cnt++]=sc;
	DataSendBuf[cnt++]=ac;
	
	for(uint8_t i=0;i<cnt;i++)
		HAL_UART_Transmit(&huart1,&DataSendBuf[i],1,100);
}
