#include "ANO_DT.h"
//匿名上位机通信协议    调pid看波形用的

#include "usb_device.h"
#include "usbd_cdc_if.h"

#include "main.h"

#include "usart.h"

#include "bsp_buzzer.h"

#define BYTE0(dwTemp) (*(char*)(&dwTemp))
#define BYTE1(dwTemp) (*((char*)(&dwTemp)+1))
#define BYTE2(dwTemp) (*((char*)(&dwTemp)+2))
#define BYTE3(dwTemp) (*((char*)(&dwTemp)+3))

uint8_t DataSendBuf[100];

void Buzz_Callback()
{
	buzzer_off();
}

void ANODT_SendF1(int32_t Angle,int32_t speed_rpm,int32_t Angle_target,int32_t speed_target)//F1灵活格式帧
{
	uint8_t cnt=0;
	DataSendBuf[cnt++]=0xAA;  //帧头
	DataSendBuf[cnt++]=0xFF;  //目标地址
	DataSendBuf[cnt++]=0xF1;  //功能码
	DataSendBuf[cnt++]=16;  //数据长度
	
	DataSendBuf[cnt++]=BYTE0(Angle);
	DataSendBuf[cnt++]=BYTE1(Angle);
	DataSendBuf[cnt++]=BYTE2(Angle);
	DataSendBuf[cnt++]=BYTE3(Angle);

	DataSendBuf[cnt++]=BYTE0(speed_rpm);
	DataSendBuf[cnt++]=BYTE1(speed_rpm);
	DataSendBuf[cnt++]=BYTE2(speed_rpm);
	DataSendBuf[cnt++]=BYTE3(speed_rpm);


	DataSendBuf[cnt++]=BYTE0(Angle_target);
	DataSendBuf[cnt++]=BYTE1(Angle_target);
	DataSendBuf[cnt++]=BYTE2(Angle_target);
	DataSendBuf[cnt++]=BYTE3(Angle_target);


	
	DataSendBuf[cnt++]=BYTE0(speed_target);
	DataSendBuf[cnt++]=BYTE1(speed_target);
	DataSendBuf[cnt++]=BYTE2(speed_target);
	DataSendBuf[cnt++]=BYTE3(speed_target);


	
	uint8_t sc=0;  //和校验
	uint8_t ac=0;  //附加校验
	for(uint8_t i=0;i<DataSendBuf[3]+4;i++)
	{
		sc+=DataSendBuf[i];
		ac+=sc;
	}
	DataSendBuf[cnt++]=sc;
	DataSendBuf[cnt++]=ac;
	
	for(uint8_t i=0;i<cnt;i++)HAL_UART_Transmit(&huart1,&DataSendBuf[i],1,100);
	
	
	
//	CDC_Transmit_FS(DataSendBuf, cnt);
}
