#ifndef __ANO_DT_H__
#define __ANO_DT_H__
#include "struct_typedef.h"

#define chassis_Debug 0
#define gimbal_Debug  0
#define shoot_Debug   0

//ע������ܵ����ݳ��ȣ���λ���ֽڣ�
void ANODT_SendF1(int32_t Data1,int32_t Data2,
				  int32_t Data3,int32_t Data4,
				  uint16_t Tot_len);
#endif

