#include "ANO_DT.h"
//匿名上位机通信协议    调pid看波形用的

#include "main.h"

#include "usart.h"


#define BYTE0(dwTemp) (*(char*)(&dwTemp))
#define BYTE1(dwTemp) (*((char*)(&dwTemp)+1))
#define BYTE2(dwTemp) (*((char*)(&dwTemp)+2))
#define BYTE3(dwTemp) (*((char*)(&dwTemp)+3))

uint8_t DataSendBuf[100];

/**
  * @brief          用于连接匿名上位机进行蓝牙串口波形调试。
  * @param[out]     chassis_move_transit:"chassis_move"变量指针.
  * @retval         none
  */
void ANODT_SendF1(int32_t Data1,int32_t Data2,int32_t Data3,int32_t Data4,uint16_t Tot_len)//F1灵活格式帧
{
	uint8_t cnt=0;
	DataSendBuf[cnt++]=0xAA;  //帧头
	DataSendBuf[cnt++]=0xFF;  //目标地址
	DataSendBuf[cnt++]=0xF1;  //功能码
    DataSendBuf[cnt++]=Tot_len;  //数据长度
	
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
	
	uint8_t sc=0;  //和校验
	uint8_t ac=0;  //附加校验
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
