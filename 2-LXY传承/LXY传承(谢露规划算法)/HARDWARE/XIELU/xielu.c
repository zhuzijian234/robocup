/**
 * @file    xielu.c
 * @brief   谢露舵机控制算法移植(雷达点流水线 + 信号构造 + qianfang30_PD)
 *
 * 移植自 谢露/robocup/HARDWARE/LEIDA_DATA/LEIDA_DATA.c:74-730 与
 * 谢露/robocup/Test/Test.c:57-250, 算法与参数原值照搬, 仅做以下改动:
 *   1. LEIDA_DATA_HANDLE3_2 改名 XIELU_HANDLE3_2(与本工程同名函数签名冲突)
 *   2. 删全部调试 printf(收尾留 qianfang30_PD 1 行)
 *   3. 删 IWDG_Feed / 分叉分支死代码(骨架无看门狗/无 LED)
 *   4. 全部 continue 改 return
 *   5. 输出钳位: 骨架 Servo.c 无钳位, 谢露原在 Servo.c 钳 → 移到本模块
 *   6. 数组按本工程 LEIDA_DATA_COUNTER(800) 与 /2(400) 开大
 *   7. 防御守卫: 原版多处 u16 下溢/数组写满越界隐患(点数=0 或帧特大时)
 *
 * 电机控制流程、雷达数据接收(TIM5 速度环、USART2+DMA1_Stream5、HANDLE1 0x54
 * 解析)不在本文件, 全部不动。
 */

#include "xielu.h"
#include "Servo.h"
#include "stdio.h"

/* ============ 模块级数组与状态(仅本文件可见) ============ */

static _LEIDA_DATA DATA_360paixu[LEIDA_DATA_COUNTER];
static uint16_t DATA_360paixu_cnt=0;

static _LEIDA_DATA DATA_houlun_paixu[LEIDA_DATA_COUNTER];
static u16 DATA_houlun=0;

static _LEIDA_DATA DATA_0_180[LEIDA_DATA_COUNTER/3*2];
static u16 Cnt_180=0;

static _LEIDA_DATA_plane left_duan_plane[LEIDA_DATA_COUNTER/2];
static _LEIDA_DATA_plane right_duan_plane[LEIDA_DATA_COUNTER/2];
static _LEIDA_DATA_plane left_135_85_plane[LEIDA_DATA_COUNTER/2];
static _LEIDA_DATA_plane right_45_110_plane[LEIDA_DATA_COUNTER/2];

static _LEIDA_DATA left_duan[LEIDA_DATA_COUNTER/2];
static _LEIDA_DATA right_duan[LEIDA_DATA_COUNTER/2];
static _LEIDA_DATA left_135_85[LEIDA_DATA_COUNTER/2];
static _LEIDA_DATA right_45_110[LEIDA_DATA_COUNTER/2];

static uint16_t left_135_85_cnt_J;
static uint16_t right_45_110_cnt_J;
static u16 left_duan_cnt_J;      /* 一直存到断点处 */
static u16 right_duan_cnt_J;

static uint16_t left_135_85_cnt;
static uint16_t right_45_110_cnt;
static uint16_t LEFT_cnt_CL;     /* 小于Duandian_d */
static uint16_t RIGHT_cnt_CL;
static u16 left_duan_cnt;        /* 一直存到断点处 */
static u16 right_duan_cnt;

/* ============ PD 参数(谢露原值, 蓝牙 kp/kp2/kd/kd2 现场可调) ============ */

float R_CL_a=0.6,    R_CL_d=0.1,    R_CL_kp=0.5,  R_CL_ki=0 ,  R_CL_kd=0.3;
float L_CL_a=0.6,    L_CL_d=0.1,    L_CL_kp=0.5,  L_CL_ki=0 ,  L_CL_kd=0.3;

float YZ_j_a=0.6,    YZ_j_d=0.6,    YZ_j_ZW=0.12, YZ_j_kp=0.9,  YZ_j_ki=0,   YZ_j_kd=0.13;
float ZZ_j_a=0.6,    ZZ_j_d=0.6,    ZZ_j_ZW=0.12, ZZ_j_kp=0.9,  ZZ_j_ki=0,   ZZ_j_kd=0.13;

float Duandian_d = 700.0f;
float zhuandian = 200.0f;

/* ============ 最近一帧输出(蓝牙遥测) ============ */
static float XIELU_err = 0;
static float XIELU_servo_pwm = XIELU_SERVO_MID;
/* 当前模式 1/2/3/4, 0=从未决策。跨帧粘滞保持, 绝不能清零 */
static uint16_t XIELU_pid_slt = 0;

/* ============ 流水线: 按角度排列 ============ */

static void LEIDA_DATA_360paixu(_LEIDA_DATA data[], _LEIDA_DATA arr[], uint16_t size)
{
	uint16_t zero_p = 0;
	uint16_t first_angle_p=0;
	uint16_t i = 0;
	uint16_t j = 0;
	float first_angle=arr[0].angle;
	u8 first_angle_flag=0;
	u8 zero_flag=0;

	DATA_360paixu_cnt=0;

	/* 筛选断点 */
	for (i = 0; i < size; i++) {
		if((fabs(arr[i].angle-360)<=2.0f)) {
			zero_p=i;
			zero_flag=1;
		}
		if(fabs(arr[i].angle-first_angle)<=1.5f) {
			first_angle_p=i;
			first_angle_flag=1;
		}
		if(zero_flag==1&&first_angle_flag==1&&first_angle_p>10) {
			break;
		}
	}
	for (i = 0,j=zero_p; i < (first_angle_p+3)&&i<size; i++,j++) {
		data[i]=arr[j];
		DATA_360paixu_cnt++;
		if(j==first_angle_p) j=0;
	}
}

/* ============ 流水线: 后轮区滤波(只留 0-250 与 290-360 两段) ============ */

static void Houlun_paixu(_LEIDA_DATA data[] ,_LEIDA_DATA arr[],u16 size)
{
	u16 i=0;
	DATA_houlun=0;
	for(i=0;i<size;i++)
	{
		if(arr[i].angle<250.0f&&arr[i].angle>0)
		{
			data[DATA_houlun].angle=arr[i].angle;
			data[DATA_houlun].distance=arr[i].distance;
			DATA_houlun++;
		}
	}
	for(i=0;i<size;i++)
	{
		if(arr[i].angle<360.0f&&arr[i].angle>290.0f){
			data[DATA_houlun].angle=arr[i].angle;
			data[DATA_houlun].distance=arr[i].distance;
			DATA_houlun++;
		}
	}
}

/* ============ 流水线: 尖刺滤波 + 前方有效点(谢露原 LEIDA_DATA_HANDLE3_2) ============ */

static void XIELU_HANDLE3_2(_LEIDA_DATA arr[],u16 size)
{
	u16 i,k;

	Cnt_180=0;

	if(size<7) return;      /* 防御: 原版 size-6 在 u16 下溢, 点数不足直接返回 */

	for(i=3;i<size-6;i++){
		k=0;
		if ((fabs(arr[i].distance-arr[i+1].distance)>=150) )
			k++;
		if( (fabs(arr[i].distance-arr[i+2].distance)>=200)  )
			k++;
		if(  (fabs(arr[i].distance-arr[i-1].distance)>=150)   )
			k++;
		if(   (fabs(arr[i].distance-arr[i-2].distance)>=200)  )
			k++;
		if(k>=3) continue;//滤波

		if((arr[i].distance>=100.0f)&&(arr[i].distance*arm_sin_f32(arr[i].angle*PI/180))>-200.0f){
			DATA_0_180[Cnt_180].angle = arr[i].angle;
			DATA_0_180[Cnt_180].distance = arr[i].distance;
			Cnt_180++;
		}
	}
}

/* ============ 前方 0~180 扫描: 得两侧墙断点距离与墙点数组 ============ */

static uint16_t qianfang_0_180_handle(_LEIDA_DATA arr[],float *left_duan_y,float *right_duan_y){
	u16 i,j;
	u8 left_duan_flag=0;
	u8 right_duan_flag=0;
	float x_temp=0,y_temp=0;
	float j_y_temp=0;

	u8 L_End_dis_flag=0;
	u8 R_End_dis_flag=0;

	*left_duan_y=6000;
	*right_duan_y=6000;

	left_duan_cnt_J=0;
	right_duan_cnt_J=0;

	left_135_85_cnt_J=0;
	right_45_110_cnt_J=0;
	L_End_dis_flag=0;
	R_End_dis_flag=0;

	if(Cnt_180<8) return 1;     /* 防御: 原版 Cnt_180-4 在 u16 下溢越界 */

	for(i=3 ,j=Cnt_180-4;i<(Cnt_180-4)&&j>3;i++,j--){
		x_temp=arr[i].distance*arm_cos_f32(arr[i].angle*PI/180);
		y_temp=arr[i].distance*arm_sin_f32(arr[i].angle*PI/180);

		j_y_temp=arr[j].distance*arm_sin_f32(arr[j].angle*PI/180);

		/* 左断点 */
		if(y_temp>End_dis&&arr[i+1].distance>End_dis) L_End_dis_flag=1;
		if(L_End_dis_flag!=1&&left_duan_flag==0&&arr[i].angle<=250.0f&&arr[i].angle>(90.0f-LR_bianjiejiao)&&y_temp>-200.0f){
			if(fabs(arr[i].distance-arr[i+1].distance)>=350){
				if(fabs(arr[i-1].distance-arr[i+2].distance)>=350)
					if(fabs(arr[i-2].distance-arr[i+3].distance)>=350)
						if(fabs(arr[i-3].distance-arr[i+4].distance)>=350)
						{
							*left_duan_y = arr[i-3].distance*arm_sin_f32(arr[i-3].angle*PI/180);
							if(*left_duan_y<5000)
							{
								left_duan_flag=1;
							}
						}
			}
			/* 防御: 数组写满即止, 防止越界 */
			if(left_duan_flag==0&&y_temp>Start_dis&&L_End_dis_flag!=1&&left_duan_cnt_J<(LEIDA_DATA_COUNTER/2-1))
			{
				if(arr[i].angle<(180.0f-Start_ang)&&arr[i].angle>(90.0f-LR_bianjiejiao))
				{
					left_135_85[left_135_85_cnt_J].angle=arr[i].angle;
					left_135_85[left_135_85_cnt_J].distance=arr[i].distance;

					left_135_85_cnt_J++;
				}

				left_duan[left_duan_cnt_J].distance=arr[i].distance;
				left_duan[left_duan_cnt_J].angle=arr[i].angle;

				left_duan_cnt_J++;
			}
		}

		/* 右断点 */
		if(j_y_temp>End_dis&&arr[j-1].distance>End_dis) R_End_dis_flag=1;
		if(right_duan_flag==0&&R_End_dis_flag!=1&&((arr[j].angle<(90.0f+LR_bianjiejiao)&&arr[j].angle>=0.0f)||(arr[j].angle<=360.0f&&arr[j].angle>290.0f))&&j_y_temp>-200.0f){
			if(fabs(arr[j].distance-arr[j-1].distance)>=350){
				if(fabs(arr[j+1].distance-arr[j-2].distance)>=350)
					if(fabs(arr[j+2].distance-arr[j-3].distance)>=350)
						if(fabs(arr[j+3].distance-arr[j-4].distance)>=350)
						{
							*right_duan_y = arr[j+3].distance*arm_sin_f32(arr[j+3].angle*PI/180);
							if(*right_duan_y<5000)
							{
								right_duan_flag=1;
							}
						}
			}
			/* 防御: 数组写满即止, 防止越界 */
			if(right_duan_flag==0&&j_y_temp>Start_dis&&R_End_dis_flag!=1&&right_duan_cnt_J<(LEIDA_DATA_COUNTER/2-1)){
				if(arr[j].angle>Start_ang&&arr[j].angle<(90.0f+LR_bianjiejiao))//50-110°
				{
					right_45_110[right_45_110_cnt_J].angle=arr[j].angle;
					right_45_110[right_45_110_cnt_J].distance=arr[j].distance;
					right_45_110_cnt_J++;
				}

				right_duan[right_duan_cnt_J].distance=arr[j].distance;
				right_duan[right_duan_cnt_J].angle=arr[j].angle;

				right_duan_cnt_J++;
			}
		}
	}

	return 1;
}

/* ============ 极坐标 -> 平面坐标(带滤波, 按左右/全部分类) ============ */

static void Jizuobiao_lvbo(u8 flag ,_LEIDA_DATA_plane data_plane[],_LEIDA_DATA arr[],u16 size,u16 *genxin_size)
{
	u16 i=0,j=0;
	u8 k=0;
	u16 t=0;
	*genxin_size=0;
	t=0;

	if(size<7) return;      /* 防御: 原版 size-4-t 在 u16 下溢, 点数不足直接返回 */

	for(i=3;i<size-4-t;i++)
	{
		k=0;
		if ((fabs(arr[i].distance-arr[i+1].distance)>=150) )
			k++;
		if( (fabs(arr[i].distance-arr[i+2].distance)>=200)  )
			k++;
		if(  (fabs(arr[i].distance-arr[i-1].distance)>=150)   )
			k++;
		if(   (fabs(arr[i].distance-arr[i-2].distance)>=200)  )
			k++;
		if(k>=2) continue;//滤波

		if(j >= LEIDA_DATA_COUNTER/2-1) break;      /* 防御: 数组写满即止, 防止越界 */

		if(flag==zuo){
			if(arr[i].angle>90.0f&&(arr[i].distance*arm_sin_f32(arr[i].angle*PI/180))<Duandian_d){
				data_plane[j]._x = arr[i].distance*arm_cos_f32(arr[i].angle*PI/180);
				data_plane[j]._y = arr[i].distance*arm_sin_f32(arr[i].angle*PI/180);
				if(data_plane[j]._y<Duandian_d) LEFT_cnt_CL=j;//记录最大行数
				j++;
			}
		}else if(flag==you){
			if(arr[i].angle<90.0f&&(arr[i].distance*arm_sin_f32(arr[i].angle*PI/180))<Duandian_d){
				data_plane[j]._x = arr[i].distance*arm_cos_f32(arr[i].angle*PI/180);
				data_plane[j]._y = arr[i].distance*arm_sin_f32(arr[i].angle*PI/180);
				if(data_plane[j]._y<Duandian_d) RIGHT_cnt_CL=j;//记录最大行数
				j++;
			}
		}else{
			data_plane[j]._x = arr[i].distance*arm_cos_f32(arr[i].angle*PI/180);
			data_plane[j]._y = arr[i].distance*arm_sin_f32(arr[i].angle*PI/180);
			j++;
		}
	}
	*genxin_size=j;
}

/* ============ x 跳变截断(找连续墙段的终点) ============ */

static void Jizuobiao_lvbo2(_LEIDA_DATA_plane arr[],u16 size,u16 *genxin_size)
{
	u16 i=0;

	if(size<5){*genxin_size=0;return;}      /* 防御: 原版 size-3 在 u16 下溢 */

	for(i=2;i<size-3;i++)
	{
		if(fabs(arr[i]._x-arr[i+1]._x)>=150){
			if(fabs(arr[i-1]._x-arr[i+2]._x)>=150)
				if(fabs(arr[i-2]._x-arr[i+3]._x)>=150)
				{
					break;
				}
		}
	}
	*genxin_size=i;
}

/* ============ S弯特征: 墙点极径平均 ============ */

static void ZW_ave(_LEIDA_DATA_plane arr[],u16 startline,u16 endline,float *ZW_JZB_ave)
{
	u16 i=0;
	float sum=0;
	int Cnt=endline-startline;//0-4 5点
	if(Cnt<1) {*ZW_JZB_ave=1000; return;}
	for(i=startline;i<endline;i++)
	{
		sum+=sqrt(arr[i]._x*arr[i]._x+arr[i]._y*arr[i]._y);
	}
	*ZW_JZB_ave=sum/(Cnt*1.0f);
}

static void ZW_G1(u8 pid_slt,float ZW_JZB_ave,float *ZW_ave_G1){
	if(pid_slt==ZZ_j){
		*ZW_ave_G1=(800.0f-ZW_JZB_ave)/(800.0f-180.0f);
	}else if(pid_slt==YZ_j){
		*ZW_ave_G1=(ZW_JZB_ave-800.0f)/(800.0f-180.0f);
	}
}

/* ============ 分段斜率拟合墙角度(4段加权 0.17/0.22/0.295/0.315) ============ */

static void K_ave(_LEIDA_DATA_plane arr[],u16 startline,u16 endline,double *nihejiao,u8 duanshu)
{
	_LEIDA_DATA_plane K_duanshu[10];

	double angle_sum=0;
	u16 i=0;
	float K;
	double K_angle;
	double K_angle_shuzu[10];
	int cnt=0;
	cnt=(int)(endline-startline);


	if(cnt>22)
	{
		for(i=0;i<duanshu;i++)
		{
			K_duanshu[i]._y=(arr[startline+cnt/duanshu*i]._y+arr[startline+cnt/duanshu*i+1]._y+arr[startline+cnt/duanshu*i+2]._y+arr[startline+cnt/duanshu*i+3]._y)/4.0f;
			K_duanshu[i]._x=(arr[startline+cnt/duanshu*i]._x+arr[startline+cnt/duanshu*i+1]._x+arr[startline+cnt/duanshu*i+2]._x+arr[startline+cnt/duanshu*i+3]._x)/4.0f;
		}
		K_duanshu[i]._y=(arr[endline-1]._y+arr[endline-2]._y+arr[endline-3]._y+arr[endline-4]._y)/4.0f;
		K_duanshu[i]._x=(arr[endline-1]._x+arr[endline-2]._x+arr[endline-3]._x+arr[endline-4]._x)/4.0f;

		for(i=0;i<duanshu;i++)
		{
			if((K_duanshu[i+1]._x-K_duanshu[i]._x)!=0)
			{
				K=(K_duanshu[i+1]._y-K_duanshu[i]._y)/(K_duanshu[i+1]._x-K_duanshu[i]._x);
				K_angle= atan((double)(K)) * (180.0f/ PI);
				if (K_angle < 0) K_angle += 180.0;

			}else
				K_angle=90.0;
			K_angle_shuzu[i]=K_angle;
		}
		angle_sum=0.17*K_angle_shuzu[0]+0.22*K_angle_shuzu[1]+0.295*K_angle_shuzu[2]+0.315*K_angle_shuzu[3];
		*nihejiao=angle_sum;
	}else if(cnt>9)
	{
		for(i=0;i<2;i++)
		{
			K_duanshu[i]._y=(arr[startline+cnt/2*i]._y+arr[startline+cnt/2*i+1]._y+arr[startline+cnt/2*i+2]._y)/3.0f;
			K_duanshu[i]._x=(arr[startline+cnt/2*i]._x+arr[startline+cnt/2*i+1]._x+arr[startline+cnt/2*i+2]._x)/3.0f;
		}
		K_duanshu[i]._y=(arr[endline-1]._y+arr[endline-2]._y+arr[endline-3]._y)/3.0f;
		K_duanshu[i]._x=(arr[endline-1]._x+arr[endline-2]._x+arr[endline-3]._x)/3.0f;
		for(i=0;i<2;i++)
		{
			if((K_duanshu[i+1]._x-K_duanshu[i]._x)!=0)
			{
				K=(K_duanshu[i+1]._y-K_duanshu[i]._y)/(K_duanshu[i+1]._x-K_duanshu[i]._x);
				K_angle= atan((double)(K)) * (180.0f/ PI);
				if (K_angle < 0) K_angle += 180.0;
			}else
				K_angle=90.0;

			angle_sum+=K_angle;
		}
		*nihejiao=angle_sum/2.0;
	}else{
		*nihejiao=90.0;
		return;
	}
}

/* ============ 跑道宽度(某侧墙点 x 均值) ============ */

static float paodao_kuan(_LEIDA_DATA_plane arr[],u16 start_line,u16 end_line,float xiaxian,float shangxian ){
	float ave=0;
	float sum=0;
	u16 cnt=0;
	u16 i=0;

	if(end_line<=1) return 0;       /* 防御: 原版 end_line-1 在 u16 下溢 */

	for(i=start_line;i<end_line-1;i++)
	{
		if(arr[i]._y<shangxian&&arr[i]._y>xiaxian)
		{
			sum+=arr[i]._x;
			cnt++;
		}
	}
	if(cnt==0) return 0;
	ave=sum/(cnt*1.0f);
	return ave;
}

/* ============ 跑道偏差归一化(一侧无墙 -> ±2 哨兵) ============ */

static float paodao_G1(u8 flag ,float left_d_ave,float right_d_ave)
{
	float G1=0;
	float R_L=0;
	float paodao_piancha=0;//跑道偏差：R_L-2*chekuan

	if(right_d_ave==0) return -2.0f;
	if(left_d_ave==0) return 2.0f;
	R_L=right_d_ave-left_d_ave-2*chekuan;
	paodao_piancha=right_d_ave+left_d_ave;
	if(R_L==0) return 0;
	G1=-paodao_piancha/R_L;//paodao_piancha_G1在-1到1，1代表靠右侧需要左转，pwm加，
	if(flag==ZW)
	{
	}else if(flag==CL)
	{
	}

	return G1;
}

/* ============ 拟合角度归一化 ============ */

static void nihe_G1(u8 pid_slt ,double nihejiao,double *nihejiao_G1)
{
	if(pid_slt==ZZ_j||pid_slt==YZ_j)
	{
		if(pid_slt==ZZ_j)   *nihejiao_G1=(nihejiao-90.0)/(zhuan_a1_L-90.0);
		else 				*nihejiao_G1=(nihejiao-90.0)/(90.0-zhuan_a1_R);
	}
	else if(pid_slt==R_CL||pid_slt==L_CL){
		*nihejiao_G1=(nihejiao-90.0)/(90.0-CL_jixianjiao);
	}
}

/* ============ 最小二乘直线拟合(den=0 -> k=40000) ============ */

static void Midline_fit_2(_LEIDA_DATA_plane *centerline, u16 startline, u16 endline, double *nihejiao) {
	u16 i=0;
	u16 n;
	float sumX = 0, sumY = 0, sumXX = 0, sumXY = 0;
	float num;
	float den;
	float k=0;
	n = endline - startline;

	for ( i = startline; i < endline; i++) {
		sumX += centerline[i]._x;
		sumY += centerline[i]._y;
		sumXX += centerline[i]._x * centerline[i]._x;
		sumXY += centerline[i]._x * centerline[i]._y;
	}
	num = n * sumXY - sumX * sumY;
	den = n * sumXX - sumX * sumX;

	/* 竖直线 */
	if (den == 0) {
		k = 40000;
	} else {
		k = num / den;
	}

	*nihejiao = atan((double)(k)) * (180.0f/ PI);
	if (*nihejiao < 0) *nihejiao += 180.0;
}

/* ============ 三段角度梯度加权(原版只打印用, 保留调用) ============ */

static void Tidu(u8 flag ,double nihejiao_1[],double *nihejiao_x1)
{
	double Tidu1=0,Tidu2=0;

	Tidu1=nihejiao_1[1]-nihejiao_1[0];
	Tidu2=nihejiao_1[2]-nihejiao_1[1];
	if(flag==YZ_j)
	{
		if(Tidu1<=0&&Tidu2<=0)//越走越弯
			*nihejiao_x1=(0.25*nihejiao_1[0]+0.3*nihejiao_1[1]+0.45*nihejiao_1[2]);
		else if(Tidu1<=0&&Tidu2>=0)
			*nihejiao_x1=(0.25*nihejiao_1[0]+0.45*nihejiao_1[1]+0.3*nihejiao_1[2]);
		else
			*nihejiao_x1=(0.45*nihejiao_1[0]+0.3*nihejiao_1[1]+0.25*nihejiao_1[2]);
	}else if(flag==ZZ_j)
	{
		if(Tidu1>=0&&Tidu2>=0)//越走越弯
			*nihejiao_x1=(0.25*nihejiao_1[0]+0.3*nihejiao_1[1]+0.45*nihejiao_1[2]);
		else if(Tidu1>=0&&Tidu2<=0)
			*nihejiao_x1=(0.25*nihejiao_1[0]+0.45*nihejiao_1[1]+0.3*nihejiao_1[2]);
		else
			*nihejiao_x1=(0.45*nihejiao_1[0]+0.3*nihejiao_1[1]+0.25*nihejiao_1[2]);
	}
}

/* ============ 前方PD(谢露原 qianfang30_PD, :659-730) ============ */

static void qianfang30_PD(float paodao_piancha_G1 ,float nihejiaodu_G1,float ZW_ave_G1,uint16_t flag)
{
	static float servo_leftTMid,servo_rigthTMid;

	float servo_pwm = XIELU_SERVO_MID;

	/* 函数内静态历史状态, 原样保留, 模式切换不清零 */
	static float err=0,err_1=0,err_r=0,err_r_r=0,err_r_r_r=0;
	static float err_sum=0;
	servo_leftTMid=(float)(XIELU_SERVO_LEFT-XIELU_SERVO_MID);
	servo_rigthTMid=(float)(XIELU_SERVO_MID-XIELU_SERVO_RIGHT);

	/* 右曲线 */
	if(flag==R_CL){
		err=(R_CL_a*nihejiaodu_G1+R_CL_d*paodao_piancha_G1)*servo_rigthTMid;
		if(err>servo_leftTMid) err=servo_leftTMid;
		else if(err<-servo_rigthTMid) err=-servo_rigthTMid;
		err_sum=err+err_r+err_r_r+err_r_r_r;
		servo_pwm=(float)(XIELU_SERVO_MID)+R_CL_kp*err+R_CL_kd*(err-err_1)+R_CL_ki*err_sum;
	}else if(flag==L_CL){
		/* 左曲线 */
		err=(L_CL_a*nihejiaodu_G1+L_CL_d*paodao_piancha_G1)*servo_leftTMid;
		if(err>servo_leftTMid) err=servo_leftTMid;
		else if(err<-servo_rigthTMid) err=-servo_rigthTMid;
		err_sum=err+err_r+err_r_r+err_r_r_r;
		servo_pwm=(float)(XIELU_SERVO_MID)+L_CL_kp*err+L_CL_kd*(err-err_1)+L_CL_ki*err_sum;
	}else if(flag==YZ_j) {
		/* 右直角 */
		err=(YZ_j_a*nihejiaodu_G1+YZ_j_d*paodao_piancha_G1+YZ_j_ZW*ZW_ave_G1)*servo_rigthTMid;
		if(err>servo_leftTMid) err=servo_leftTMid;
		else if(err<-servo_rigthTMid) err=-servo_rigthTMid;
		err_sum=err+err_r+err_r_r+err_r_r_r;
		servo_pwm=(float)(XIELU_SERVO_MID)+YZ_j_kp*err+YZ_j_kd*(err-err_1)+YZ_j_ki*err_sum;
	}else if(flag==ZZ_j) {
		/* 左直角 */
		err=(ZZ_j_a*nihejiaodu_G1+ZZ_j_d*paodao_piancha_G1+ZZ_j_ZW*ZW_ave_G1)*servo_leftTMid;
		if(err>servo_leftTMid) err=servo_leftTMid;
		else if(err<-servo_rigthTMid) err=-servo_rigthTMid;
		err_sum=err+err_r+err_r_r+err_r_r_r;
		servo_pwm=(float)(XIELU_SERVO_MID)+ZZ_j_kp*err+ZZ_j_kd*(err-err_1)+ZZ_j_ki*err_sum;
	}

	/* 输出钳位(谢露原在 Servo.c 钳, 本工程 Servo.c 无钳位, 移到这里) */
	if(servo_pwm>XIELU_SERVO_LEFT)  servo_pwm=XIELU_SERVO_LEFT;
	else if(servo_pwm<XIELU_SERVO_RIGHT) servo_pwm=XIELU_SERVO_RIGHT;

	printf("pid_slt=%d,err=%.2f,servo_pwm=%.2f\r\n",flag,err,servo_pwm);

	err_r_r_r=err_r_r;
	err_r_r=err_r;
	err_r=err;
	err_1=0.2*err_r_r_r+0.3f*err_r_r+0.6f*err_r;

	XIELU_err=err;
	XIELU_servo_pwm=servo_pwm;
	Servo_ChangePwm((uint16_t)servo_pwm);
}

/* ============ 一帧处理(谢露 Test.c Handle3_TEST 循环体三分支) ============ */

void XIELU_ProcessFrame(uint16_t cnt)
{
	u16 qianfang_0_180=0;

	float left_duan_y=6000;
	float right_duan_y=6000;

	float left_d_ave=0;
	float right_d_ave=0;

	double nihejiao_L_x1=90;
	double nihejiao_R_x1=90;
	double nihejiao_M_x1=90;

	double nihejiao_M_x1_G1=0;//右转nih
	float paodao_piancha_G1=0;//paodao_piancha_G1在-1到1，1代表靠右侧需要左转，pwm加，

	double nihejiao=90;
	double nihejiao_1[3]={90,90,90};
	double nihejiao_x1=90;
	double nihejiao_x1_G1=0;
	double nihejiao_G1=0;

	float ZW_JZB_ave=0;
	float ZW_ave_G1=0;

	LEIDA_DATA_360paixu(DATA_360paixu,LEIDA_DATA,cnt);//原始数据转成按角度排列

	Houlun_paixu(DATA_houlun_paixu ,DATA_360paixu,DATA_360paixu_cnt);
	XIELU_HANDLE3_2(DATA_houlun_paixu,DATA_houlun);//滤波

	qianfang_0_180=qianfang_0_180_handle(DATA_0_180,&left_duan_y,&right_duan_y);
	if(qianfang_0_180==0) return;

	left_135_85_cnt=0;
	right_45_110_cnt=0;
	LEFT_cnt_CL=0;
	RIGHT_cnt_CL=0;

	Jizuobiao_lvbo(zuo,left_duan_plane,left_duan,left_duan_cnt_J,&left_duan_cnt);
	Jizuobiao_lvbo(you,right_duan_plane,right_duan,right_duan_cnt_J,&right_duan_cnt);
	Jizuobiao_lvbo(wu,left_135_85_plane,left_135_85,left_135_85_cnt_J,&left_135_85_cnt);
	Jizuobiao_lvbo(wu,right_45_110_plane,right_45_110,right_45_110_cnt_J,&right_45_110_cnt);

	Jizuobiao_lvbo2(left_duan_plane,LEFT_cnt_CL,&LEFT_cnt_CL);
	Jizuobiao_lvbo2(right_duan_plane,RIGHT_cnt_CL,&RIGHT_cnt_CL);

	nihejiao_M_x1_G1=0;

	right_d_ave=0;
	left_d_ave=0;
	paodao_piancha_G1=0;

	ZW_ave_G1=0;
	nihejiao_G1=0;
	nihejiao_x1_G1=0;

	if(left_duan_y<zhuandian&&right_duan_y<zhuandian){
		/* 分支1: 双侧断点都近(分叉/十字): 保持舵机, 跳过本帧(原版亮灯+continue, 死代码不搬) */
		return;
	}else if(left_duan_y>zhuandian&&right_duan_y>zhuandian){
		/* 分支2: 双侧断点都远(无断点): 两侧墙拟合角度 -> R_CL/L_CL 或直接打满 */

		K_ave(left_duan_plane,(u16)(LEFT_cnt_CL*0.63),(u16)(LEFT_cnt_CL*0.99),&nihejiao_L_x1,4);
		K_ave(right_duan_plane,(u16)(RIGHT_cnt_CL*0.63),(u16)(RIGHT_cnt_CL*0.99),&nihejiao_R_x1,4);

		nihejiao_M_x1=(nihejiao_L_x1+nihejiao_R_x1)/2.0;

		left_d_ave=paodao_kuan(left_duan_plane,1,(u16)(LEFT_cnt_CL*0.95),0.0f,400.0f);
		right_d_ave=paodao_kuan(right_duan_plane,1,(u16)(RIGHT_cnt_CL*0.95),0.0f,400.0f);

		if((right_d_ave-left_d_ave)<100||right_d_ave<0||left_d_ave>0)
		{
			return;
		}

		if(nihejiao_M_x1<=0||nihejiao_M_x1>=180.0) {
			return;
		}
		else if(nihejiao_M_x1<90.0&&(nihejiao_M_x1>=CL_jixianjiao))          {XIELU_pid_slt=R_CL;}
		else if(nihejiao_M_x1<CL_jixianjiao&&nihejiao_M_x1>0)					{Servo_ChangePwm(XIELU_SERVO_RIGHT);XIELU_err=-275.0f;XIELU_servo_pwm=XIELU_SERVO_RIGHT;return;}
		else if((nihejiao_M_x1<=(180.0-CL_jixianjiao))&&nihejiao_M_x1>90.0)  {XIELU_pid_slt=L_CL;}
		else if(nihejiao_M_x1>(180.0-CL_jixianjiao)&&nihejiao_M_x1<180.0)	{Servo_ChangePwm(XIELU_SERVO_LEFT);XIELU_err=275.0f;XIELU_servo_pwm=XIELU_SERVO_LEFT;return;}

		paodao_piancha_G1=paodao_G1(CL,left_d_ave, right_d_ave);

		nihe_G1((u8)XIELU_pid_slt ,nihejiao_M_x1,&nihejiao_M_x1_G1);

		qianfang30_PD(paodao_piancha_G1,(float)(nihejiao_M_x1_G1), ZW_ave_G1,XIELU_pid_slt);

	}else if(left_duan_y>zhuandian&&right_duan_y<=zhuandian||left_duan_y<zhuandian&&right_duan_y>=zhuandian){
		/* 分支3: 单侧断点(弯道入口): 只拟合远侧墙 -> YZ_j/ZZ_j 直角 */

		if(left_duan_y>zhuandian&&right_duan_y<=zhuandian){//右转
			ZW_ave(left_135_85_plane,((u16)(left_135_85_cnt*0.05)),((u16)(left_135_85_cnt*0.95)),&ZW_JZB_ave);
			Midline_fit_2(left_135_85_plane,((u16)(left_135_85_cnt*0.05)),((u16)(left_135_85_cnt*0.95)), &nihejiao);
			Midline_fit_2(left_135_85_plane,((u16)(left_135_85_cnt*0.05)),((u16)(left_135_85_cnt*0.45)), &nihejiao_1[0]);
			Midline_fit_2(left_135_85_plane,((u16)(left_135_85_cnt*0.35)),((u16)(left_135_85_cnt*0.65)), &nihejiao_1[1]);
			Midline_fit_2(left_135_85_plane,((u16)(left_135_85_cnt*0.65)),((u16)(left_135_85_cnt*0.95)), &nihejiao_1[2]);

			XIELU_pid_slt=YZ_j;

		}else if(left_duan_y<zhuandian&&right_duan_y>=zhuandian){//左转
			ZW_ave(right_45_110_plane,((u16)(right_45_110_cnt*0.05)),((u16)(right_45_110_cnt*0.95)),&ZW_JZB_ave);
			Midline_fit_2(right_45_110_plane,((u16)(right_45_110_cnt*0.05)),((u16)(right_45_110_cnt*0.95)), &nihejiao);
			Midline_fit_2(right_45_110_plane,((u16)(right_45_110_cnt*0.05)),((u16)(right_45_110_cnt*0.45)), &nihejiao_1[0]);
			Midline_fit_2(right_45_110_plane,((u16)(right_45_110_cnt*0.35)),((u16)(right_45_110_cnt*0.65)), &nihejiao_1[1]);
			Midline_fit_2(right_45_110_plane,((u16)(right_45_110_cnt*0.65)),((u16)(right_45_110_cnt*0.95)), &nihejiao_1[2]);

			XIELU_pid_slt=ZZ_j;
		}

		/* 只测未断一侧的跑道宽, 另一侧=0 -> paodao_G1 出 ±2 哨兵 */
		if(LEFT_cnt_CL<4&&RIGHT_cnt_CL>4)
		{
			left_d_ave=0;
			right_d_ave=paodao_kuan(right_duan_plane,0,RIGHT_cnt_CL,0.0f,200.0f);
		}else if(RIGHT_cnt_CL<4&&LEFT_cnt_CL>4)
		{
			right_d_ave=0;
			left_d_ave=paodao_kuan(left_duan_plane,0,LEFT_cnt_CL,0.0f,200.0f);
		}

		Tidu((u8)XIELU_pid_slt ,nihejiao_1,&nihejiao_x1);

		nihe_G1((u8)XIELU_pid_slt ,nihejiao_x1,&nihejiao_x1_G1);
		nihe_G1((u8)XIELU_pid_slt ,nihejiao,&nihejiao_G1);
		paodao_piancha_G1=paodao_G1(ZW,left_d_ave, right_d_ave);
		ZW_G1((u8)XIELU_pid_slt,ZW_JZB_ave,&ZW_ave_G1);

		qianfang30_PD(paodao_piancha_G1,(float)(nihejiao_G1),ZW_ave_G1,XIELU_pid_slt);
	}
}

/* ============ 遥测接口 ============ */

float XIELU_GetErr(void){ return XIELU_err; }

float XIELU_GetServoPwm(void){ return XIELU_servo_pwm; }

uint16_t XIELU_GetPidSlt(void){ return XIELU_pid_slt; }

void XIELU_ActiveGains(float *kp, float *kd)
{
	switch(XIELU_pid_slt){
		case L_CL: *kp=L_CL_kp; *kd=L_CL_kd; break;
		case YZ_j: *kp=YZ_j_kp; *kd=YZ_j_kd; break;
		case ZZ_j: *kp=ZZ_j_kp; *kd=ZZ_j_kd; break;
		case R_CL:
		default:   *kp=R_CL_kp; *kd=R_CL_kd; break;   /* 0=未决策: 回曲线参数 */
	}
}
