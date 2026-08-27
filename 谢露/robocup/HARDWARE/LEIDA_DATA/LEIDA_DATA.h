#ifndef __LEIDA_DATA_H
#define __LEIDA_DATA_H

#include "sys.h" 
#include "DMA.h"
#include "arm_math.h"
#include "delay.h"
#include "Servo.h"
#include "stdio.h"	
typedef struct{
	float angle;
	float distance;
	
}_LEIDA_DATA;									//极坐标


typedef struct{
	float _x;
	float _y;
}_LEIDA_DATA_plane;									//平面坐标


#define LEIDA_DATA_COUNTER 600		//雷达数据数目


extern _LEIDA_DATA LEIDA_DATA[];		//雷达极坐标
extern _LEIDA_DATA_plane LEIDA_DATA_plane[];//雷达平面坐标
uint16_t LEIDA_DATA_HANDLE1(_LEIDA_DATA data[],u8 arr[],u16 size);


extern _LEIDA_DATA DATA_360paixu[];
extern uint16_t DATA_360paixu_cnt;	
void LEIDA_DATA_360paixu(_LEIDA_DATA data[], _LEIDA_DATA arr[], uint16_t size) ;

extern _LEIDA_DATA DATA_houlun_paixu[];
extern u16 DATA_houlun;
void Houlun_paixu(_LEIDA_DATA data[] ,_LEIDA_DATA arr[],u16 size);

extern _LEIDA_DATA DATA_0_180[];
extern u16 Cnt_180;
void LEIDA_DATA_HANDLE3_2(_LEIDA_DATA arr[],u16 size);

extern u8 y_z_daoche_flag;
#define youdao 2
#define zuodao 1
#define budao 0

extern _LEIDA_DATA_plane left_duan_plane[];
extern _LEIDA_DATA_plane right_duan_plane[];
extern _LEIDA_DATA_plane left_135_85_plane[];
extern _LEIDA_DATA_plane right_45_110_plane[];

extern _LEIDA_DATA left_duan[];
extern _LEIDA_DATA right_duan[];
extern _LEIDA_DATA left_135_85[];
extern _LEIDA_DATA right_45_110[];


extern uint16_t left_135_85_cnt_J;
extern uint16_t right_45_110_cnt_J;
extern uint16_t LEFT_cnt_CL_J;		//小于Duandian_d					
extern uint16_t RIGHT_cnt_CL_J;
extern u16 left_duan_cnt_J;//一直储存到断点处
extern u16 right_duan_cnt_J;


extern u16 LEFT_cnt_CL;		//小于Duandian_d					
extern u16 RIGHT_cnt_CL;
extern u16 left_duan_cnt;     //一直储存到断点处
extern u16 right_duan_cnt;
extern uint16_t left_135_85_cnt;
extern uint16_t right_45_110_cnt;
uint16_t qianfang_0_180_handle(_LEIDA_DATA arr[],float *left_duan_y,float *right_duan_y);

#define zuo 1
#define you 2
#define wu 0
void Jizuobiao_lvbo(u8 flag ,_LEIDA_DATA_plane data_plane[],_LEIDA_DATA arr[],u16 size,u16 *genxin_size);
void Jizuobiao_lvbo2(_LEIDA_DATA_plane arr[],u16 size,u16 *genxin_size);
void ZW_ave(_LEIDA_DATA_plane arr[],u16 startline,u16 endline,float *ZW_JZB_ave);
void ZW_G1(u8 pid_slt,float ZW_JZB_ave,float *ZW_ave_G1);
void K_ave(_LEIDA_DATA_plane arr[],u16 startline,u16 endline,double *nihejiao,u8 duanshu);

float paodao_kuan(_LEIDA_DATA_plane arr[],u16 start_line,u16 end_line,float xiaxian,float shangxian);
float paodao_G1(u8 flag ,float left_d_ave,float right_d_ave);
void  nihe_G1(u8 pid_slt ,double nihejiao,double *nihejiao_G1);



#define    CL_jixianjiao 65.0  

#define    Start_dis 0.0f
#define    End_dis   2500.0f


#define    LR_bianjiejiao  60.0f
#define    Start_ang  60.0f


#define zhuan_a1_L  120.0
#define zhuan_a1_R  60.0  

#define R_CL 1
#define L_CL 2
#define YZ_j 3
#define ZZ_j 4

#define ZW 0
#define CL 1

#define chekuan 140.0f


typedef struct
{
  float k;
	float b;

}Midline_type;


extern float R_CL_a,   R_CL_d,    R_CL_kp,  R_CL_ki,  R_CL_kd;
extern float L_CL_a,   L_CL_d,    L_CL_kp,  L_CL_ki,  L_CL_kd;

//右转
extern float YZ_j_a,       YZ_j_d,  YZ_j_ZW,    YZ_j_kp,  YZ_j_ki,   YZ_j_kd;

//左转
extern float ZZ_j_a,       ZZ_j_d,   ZZ_j_ZW,   ZZ_j_kp,   ZZ_j_ki,  ZZ_j_kd;

extern float  Duandian_d ;
extern float zhuandian;





void Tidu(u8 flag ,double nihejiao_1[],double *nihejiao_x1);
void qianfang30_PD(float paodao_piancha_G1 ,float nihejiaodu,float ZW_ave_G1,uint16_t flag);
void Midline_fit_2(_LEIDA_DATA_plane *centerline,u16 startline,u16 endline,double *nihejiao);


#endif

