#ifndef __LEIDA_HANDLE_H
#define __LEIDA_HANDLE_H
#include "sys.h" 
#include "usart.h"
#include "arm_math.h"



#define ANGLE_PER_FRAME 				12
#define HEADER 							0x54
#define POINT_PER_PACK 					12
#define LENGTH  						0x2C 	//低五位是一帧数据接收到的点数，目前固定是12，高三位预留


extern u8 one_frame_data_success_flag,one_lap_data_success_flag;
extern u32 lap_count,PointDataProcess_count,test_once_flag,Dividing_point;




typedef struct __attribute__((packed)) Point_Data
{
	u16 distance;//距离
	u8 confidence;//置信度
	
}LidarPointStructDef;//一帧数据中每个点包含的数据


typedef struct __attribute__((packed)) Pack_Data
{
	uint8_t header;
	uint8_t ver_len;
	uint16_t speed;
	uint16_t start_angle;
	LidarPointStructDef point[POINT_PER_PACK];
	uint16_t end_angle;
	uint16_t timestamp;
	uint8_t crc8;
}LiDARFrameTypeDef;//一帧数据结构体

typedef struct __attribute__((packed)) PointDataProcess_
{
	u16 distance;
	float angle;
}PointDataProcessDef;//经过处理后的数据

extern PointDataProcessDef PointDataProcess[800];//更新800个数据
extern PointDataProcessDef Dataprocess[800];//解析一圈后的数据
extern LiDARFrameTypeDef Pack_Data;
extern PointDataProcessDef TempData[12];  //超过了0度的下一圈数据临时存储
uint8_t LEIDA_HANDLE1(u8 arr[],u16 size);
void data_process(void);




#endif


