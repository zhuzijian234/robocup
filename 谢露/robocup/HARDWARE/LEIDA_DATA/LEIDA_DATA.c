#include "LEIDA_DATA.h" 






_LEIDA_DATA LEIDA_DATA[LEIDA_DATA_COUNTER];
_LEIDA_DATA_plane LEIDA_DATA_plane[LEIDA_DATA_COUNTER];

#define zhenchangdu 58
#define head1 ((uint8_t)0xA5)
#define head2 ((uint8_t)0x5A)
#define dianshu ((uint8_t)16)
uint16_t LEIDA_DATA_HANDLE1(_LEIDA_DATA data[],u8 arr[],u16 size){			//500个点，500/12*47=2052个数据每一个包（共47个字节）的数据处理，好像默认了只要找到三个数据包就可以认为转一圈的数据（大概720个）就是正确的？所以得改
	u16 i=0,j=0,k=0;
	float start_angle=0;
	float end_angle=0;
	u16 cnt=0;

	
	for(i=0;i<size-zhenchangdu-zhenchangdu-1;i++){																							//3个起始报文确定起始应答
		if(arr[i]==head1&&arr[i+1]==head2)
		{
			if(arr[i+zhenchangdu]==head1&&arr[i+zhenchangdu+1]==head2)
			{
				if(arr[i+zhenchangdu*2]==head1&&arr[i+zhenchangdu*2+1]==head2)
				{
					break;
				}
			}
		}
	}
	if(i==size-zhenchangdu-zhenchangdu-1)return 0;
		
	for( j = 0;(i<size-zhenchangdu-1)&&(j<LEIDA_DATA_COUNTER-dianshu) ;i+=zhenchangdu,j+=dianshu){		//根据数据包填充800个数据
		if( arr[i]==head1&&arr[i+1]==head2 )		
		{											
														
			start_angle  =  ( ((u16)arr[i+5]<<8)+(u16)arr[i+6] )/100.0f;	
			end_angle = ( ((u16)arr[i+55]<<8)+(u16)arr[i+56] )/100.0f;
			if(start_angle>end_angle)																//结束角度和开始角度被0度分割的情况
			end_angle +=360;
		
		
			for(k = 0;k<dianshu;k++){
//				if(1.0f*(  ((u16)arr[i+7+3*k]<<8)+(u16)arr[i+6+3*k]  )>100){
					data[j+k].distance = 1.0f*(  ((u16)arr[i+7+3*k]<<8)+(u16)arr[i+8+3*k]  );							//求距离
					data[j+k].angle =  start_angle+(end_angle-start_angle)/dianshu*k;													//求角度					
					
					if(data[j+k].angle>360.0f)data[j+k].angle-=360.0f;																	//将角度限制在360以内										
					if(data[j+k].angle<0.0f)data[j+k].angle+=360.0f;	
					cnt++;
					data[j+k].angle = -1.0f*data[j+k].angle+360.0f+85.0f;							//excel绘图做角度变换
				
					if(data[j+k].angle>360.0f)data[j+k].angle-=360.0f;																	//将角度限制在360以内										
					if(data[j+k].angle<0.0f)data[j+k].angle+=360.0f;
	//								printf("\ni=%d,j+k=%d\n",i,j+k);
//				}
			}
				
				

		}
	}
	
	return cnt;		//表明数据解析成功
}


_LEIDA_DATA DATA_360paixu[LEIDA_DATA_COUNTER]; 
uint16_t DATA_360paixu_cnt=0;
// 筛选并排序函数  
void LEIDA_DATA_360paixu(_LEIDA_DATA data[], _LEIDA_DATA arr[], uint16_t size) {  
    uint16_t zero_p = 0;
	uint16_t first_angle_p=0;	
	uint16_t i = 0;
	uint16_t j = 0;
	float first_angle=arr[0].angle;
	u8 first_angle_flag=0;
	u8 zero_flag=0;
	
	DATA_360paixu_cnt=0;
	
    // 筛选数据  
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

_LEIDA_DATA DATA_houlun_paixu[LEIDA_DATA_COUNTER];
u16 DATA_houlun=0;
void Houlun_paixu(_LEIDA_DATA data[] ,_LEIDA_DATA arr[],u16 size)
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

_LEIDA_DATA DATA_0_180[LEIDA_DATA_COUNTER/3*2];
//_LEIDA_DATA_plane temp[LEIDA_DATA_COUNTER/3*2];
u16 Cnt_180=0;
//data2
void LEIDA_DATA_HANDLE3_2(_LEIDA_DATA arr[],u16 size){						//找出距离不为0的有效点,返回有效点个数
	u16 i,k;
	
	
    Cnt_180=0;
	
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


u8 y_z_daoche_flag=budao;

_LEIDA_DATA_plane left_duan_plane[LEIDA_DATA_COUNTER/2];
_LEIDA_DATA_plane right_duan_plane[LEIDA_DATA_COUNTER/2];
_LEIDA_DATA_plane left_135_85_plane[LEIDA_DATA_COUNTER/2];
_LEIDA_DATA_plane right_45_110_plane[LEIDA_DATA_COUNTER/2];
_LEIDA_DATA left_duan[LEIDA_DATA_COUNTER/2];
_LEIDA_DATA right_duan[LEIDA_DATA_COUNTER/2];
_LEIDA_DATA left_135_85[LEIDA_DATA_COUNTER/2];
_LEIDA_DATA right_45_110[LEIDA_DATA_COUNTER/2];


uint16_t left_135_85_cnt_J;
uint16_t right_45_110_cnt_J;

u16 left_duan_cnt_J;//一直储存到断点处
u16 right_duan_cnt_J;

uint16_t left_135_85_cnt;
uint16_t right_45_110_cnt;
uint16_t LEFT_cnt_CL;		//小于Duandian_d					
uint16_t RIGHT_cnt_CL;
u16 left_duan_cnt;//一直储存到断点处
u16 right_duan_cnt;

//得到正负15cm数据
//传入Cnt_180到最大的数据
uint16_t qianfang_0_180_handle(_LEIDA_DATA arr[],float *left_duan_y,float *right_duan_y){
	u16 i,j;
	u8 left_duan_flag=0;
	u8 right_duan_flag=0;
	float x_temp=0,y_temp=0;
	float j_y_temp=0;
	
	u8 L_End_dis_flag=0;
	u8 R_End_dis_flag=0;
	u8 zuodao_cnt=0,youdao_cnt=0;
	
	*left_duan_y=6000;
	*right_duan_y=6000;

	left_duan_cnt_J=0;
	right_duan_cnt_J=0;
    
	
	left_135_85_cnt_J=0;
	right_45_110_cnt_J=0;
	L_End_dis_flag=0;
	R_End_dis_flag=0;
	for(i=3 ,j=Cnt_180-4;i<(Cnt_180-4)&&j>3;i++,j--){			
		x_temp=arr[i].distance*arm_cos_f32(arr[i].angle*PI/180);
		y_temp=arr[i].distance*arm_sin_f32(arr[i].angle*PI/180);
		
		
		
		j_y_temp=arr[j].distance*arm_sin_f32(arr[j].angle*PI/180);
		
				//左断点
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
			if(left_duan_flag==0&&y_temp>Start_dis&&L_End_dis_flag!=1)
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
		
		//右断点
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
								
//								printf("\nAngle:%.2f  distance:%.1f\r\n",arr[j-1].angle,arr[j-1].distance);
//								printf("Angle:%.2f  distance:%.1f\r\n",arr[j+2].angle,arr[j+2].distance);
								right_duan_flag=1;
								
							}
						}
			}
			if(right_duan_flag==0&&j_y_temp>Start_dis&&R_End_dis_flag!=1){
				
				
				if(arr[j].angle>Start_ang&&arr[j].angle<(90.0f+LR_bianjiejiao))//50-110度
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
	printf("*left_duan_y:%.1f\r\n",*left_duan_y);
	printf("*right_duan_y:%.1f\r\n",*right_duan_y);

	return 1;
	
}





void Jizuobiao_lvbo(u8 flag ,_LEIDA_DATA_plane data_plane[],_LEIDA_DATA arr[],u16 size,u16 *genxin_size)
{
	u16 i=0,j=0;
	u8 k=0;
	u16 t=0;
	*genxin_size=0;
	t=0;
//	
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
		
		
		
		if(flag==zuo){
			if(arr[i].angle>90.0f&&(arr[i].distance*arm_sin_f32(arr[i].angle*PI/180))<Duandian_d){

				data_plane[j]._x = arr[i].distance*arm_cos_f32(arr[i].angle*PI/180);
				data_plane[j]._y = arr[i].distance*arm_sin_f32(arr[i].angle*PI/180);	
				if(data_plane[j]._y<Duandian_d) LEFT_cnt_CL=j;//记录的是索引
				j++;
			}
			
		}else if(flag==you){
			if(arr[i].angle<90.0f&&(arr[i].distance*arm_sin_f32(arr[i].angle*PI/180))<Duandian_d){

				data_plane[j]._x = arr[i].distance*arm_cos_f32(arr[i].angle*PI/180);
				data_plane[j]._y = arr[i].distance*arm_sin_f32(arr[i].angle*PI/180);	
				if(data_plane[j]._y<Duandian_d) RIGHT_cnt_CL=j;//记录的是索引
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

void Jizuobiao_lvbo2(_LEIDA_DATA_plane arr[],u16 size,u16 *genxin_size)
{
	u16 i=0;
	
	
	
	
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

void ZW_ave(_LEIDA_DATA_plane arr[],u16 startline,u16 endline,float *ZW_JZB_ave)
{
	u16 i=0;
	float sum=0;
	int Cnt=endline-startline;//0-4 5个
	if(Cnt<1) {*ZW_JZB_ave=1000; return;}
	for(i=startline;i<endline;i++)
	{
		sum+=sqrt(arr[i]._x*arr[i]._x+arr[i]._y*arr[i]._y);
	}
	*ZW_JZB_ave=sum/(Cnt*1.0f);
}
void ZW_G1(u8 pid_slt,float ZW_JZB_ave,float *ZW_ave_G1){
	printf("ZW_JZB_ave=%.2f\r\n",ZW_JZB_ave);
	if(pid_slt==ZZ_j){
		*ZW_ave_G1=(800.0f-ZW_JZB_ave)/(800.0f-180.0f);
	}else if(pid_slt==YZ_j){
		*ZW_ave_G1=(ZW_JZB_ave-800.0f)/(800.0f-180.0f);
	}
	printf("ZW_ave_G1=%.2f\r\n",*ZW_ave_G1);
}
void K_ave(_LEIDA_DATA_plane arr[],u16 startline,u16 endline,double *nihejiao,u8 duanshu)
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
//			angle_sum+=K_angle;
		}
		angle_sum=0.17*K_angle_shuzu[0]+0.22*K_angle_shuzu[1]+0.295*K_angle_shuzu[2]+0.315*K_angle_shuzu[3];
//		*nihejiao=angle_sum/((double)duanshu);
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
		printf("K_ave  Error cnt=%d\r\n",cnt);
		return;
		
	}
	printf("K_ave  startline arr[%d]._x=%.2f   arr[%d]._y=%.2f\n",startline,arr[startline]._x,startline,arr[startline]._y);
	printf("K_ave  endline arr[%d]._x=%.2f   arr[%d]._y=%.2f\n",endline,arr[endline]._x,endline,arr[endline]._y);

}

float paodao_kuan(_LEIDA_DATA_plane arr[],u16 start_line,u16 end_line,float xiaxian,float shangxian ){
	float ave=0;
	float sum=0;
	u16 cnt=0;
	u16 i=0;
	
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



float paodao_G1(u8 flag ,float left_d_ave,float right_d_ave)
{
	float G1=0;
	float R_L=0;
	float paodao_piancha=0;//最大偏差：R_L-2*chekuan
	
	if(right_d_ave==0) return -2.0f;
	if(left_d_ave==0) return 2.0f;
	R_L=right_d_ave-left_d_ave-2*chekuan;
	paodao_piancha=right_d_ave+left_d_ave;//最大偏差：R_L-2*chekuan 如果左边为0，30cm-26=4
	if(R_L==0) return 0;
	G1=-paodao_piancha/R_L;//paodao_piancha_G1从-1到1，1靠进右道，要左转，pwm加，
	if(flag==ZW)
	{
//		if(G1<0)   G1=-G1*G1;
//		else G1=G1*G1;	
	}else if(flag==CL)
	{
//		if(G1<0)   G1=-G1*G1;
//		else G1=G1*G1;
	}
	printf("R_L =%.2f, paodao_piancha =%.2f，paodao_piancha_G1 =%.2f\n",R_L,paodao_piancha,G1);
	
	return G1;
}

void  nihe_G1(u8 pid_slt ,double nihejiao,double *nihejiao_G1)
{
	if(pid_slt==ZZ_j||pid_slt==YZ_j)
	{
		if(pid_slt==ZZ_j)   *nihejiao_G1=(nihejiao-90.0)/(zhuan_a1_L-90.0);
		else 				*nihejiao_G1=(nihejiao-90.0)/(90.0-zhuan_a1_R);
	
		//if(*nihejiao_G1<0)
//		*nihejiao_G1=-(*nihejiao_G1)*(*nihejiao_G1);
//	else
//		*nihejiao_G1=(*nihejiao_G1)*(*nihejiao_G1);
	}
		
	

	else if(pid_slt==R_CL||pid_slt==L_CL){
		*nihejiao_G1=(nihejiao-90.0)/(90.0-CL_jixianjiao);
		
//		if(*nihejiao_G1<0)
//		*nihejiao_G1=-(*nihejiao_G1)*(*nihejiao_G1);
//	else
//		*nihejiao_G1=(*nihejiao_G1)*(*nihejiao_G1);
		
	}

	
}





// 假设_LEIDA_DATA_plane和其他类型已经正确定义  
void Midline_fit_2(_LEIDA_DATA_plane *centerline, u16 startline, u16 endline, double *nihejiao) {  
    u16 i=0;
	u16 n;  
    float sumX = 0, sumY = 0, sumXX = 0, sumXY = 0;  
	float num;  
    float den;  
	float k=0;
    n = endline - startline;
	//printf("Midline_fit_2  startline centerline[%d]._x=%.2f   centerline[%d]._y=%.2f\n",startline,centerline[startline]._x,startline,centerline[startline]._y);
	//printf("Midline_fit_2  endline centerline[%d]._x=%.2f   centerline[%d]._y=%.2f\n",endline-1,centerline[endline-1]._x,endline,centerline[endline-1]._y);

	//Plane_Print("centerline", centerline, startline,endline);
	
    for ( i = startline; i < endline; i++) {  
        sumX += centerline[i]._x;  
        sumY += centerline[i]._y;  
        sumXX += centerline[i]._x * centerline[i]._x;  
        sumXY += centerline[i]._x * centerline[i]._y;  
    }  
	num = n * sumXY - sumX * sumY;
    den = n * sumXX - sumX * sumX;
  
    // 避免除以零  
    if (den == 0) {  
        k = 40000;  
         
    } else {  
        k = num / den;  
          
    }  
  
    *nihejiao = atan((double)(k)) * (180.0f/ PI);  
    if (*nihejiao < 0) *nihejiao += 180.0;  
    
   // printf("k=%.2f,  nihejiao=%.2lf\n", k,  *nihejiao);  
}



void Tidu(u8 flag ,double nihejiao_1[],double *nihejiao_x1)
{
	
	double Tidu1=0,Tidu2=0;
	
	Tidu1=nihejiao_1[1]-nihejiao_1[0];
	Tidu2=nihejiao_1[2]-nihejiao_1[1];
	if(flag==YZ_j)
	{
		if(Tidu1<=0&&Tidu2<=0)//越来越急
			*nihejiao_x1=(0.25*nihejiao_1[0]+0.3*nihejiao_1[1]+0.45*nihejiao_1[2]);
		else if(Tidu1<=0&&Tidu2>=0)
			*nihejiao_x1=(0.25*nihejiao_1[0]+0.45*nihejiao_1[1]+0.3*nihejiao_1[2]);
		else 
			*nihejiao_x1=(0.45*nihejiao_1[0]+0.3*nihejiao_1[1]+0.25*nihejiao_1[2]);
		
	}else if(flag==ZZ_j)
	{
		if(Tidu1>=0&&Tidu2>=0)//越累越急
			*nihejiao_x1=(0.25*nihejiao_1[0]+0.3*nihejiao_1[1]+0.45*nihejiao_1[2]);
		else if(Tidu1>=0&&Tidu2<=0)
			*nihejiao_x1=(0.25*nihejiao_1[0]+0.45*nihejiao_1[1]+0.3*nihejiao_1[2]);
		else 
			*nihejiao_x1=(0.45*nihejiao_1[0]+0.3*nihejiao_1[1]+0.25*nihejiao_1[2]);
	}
	
	
}






float R_CL_a=0.6,    R_CL_d=0.1,    R_CL_kp=0.5,  R_CL_ki=0 ,  R_CL_kd=0.3;
float L_CL_a=0.6,    L_CL_d=0.1,    L_CL_kp=0.5,  L_CL_ki=0 ,  L_CL_kd=0.3;


float YZ_j_a=0.6,       YZ_j_d=0.6,    YZ_j_ZW=0.12,   YZ_j_kp=0.9,  YZ_j_ki=0,   YZ_j_kd=0.13;


float ZZ_j_a=0.6,       ZZ_j_d=0.6,    ZZ_j_ZW=0.12,   ZZ_j_kp=0.9,  ZZ_j_ki=0,   ZZ_j_kd=0.13;

float  Duandian_d = 700.0f;

float  zhuandian = 200.0f;



void qianfang30_PD(float paodao_piancha_G1 ,float nihejiaodu_G1,float ZW_ave_G1,uint16_t flag)
{ 
	static float servo_leftTMid,servo_rigthTMid;
	
	//中线拟合
	
	float servo_pwm = servo_Midpwm1;
	
	static float err=0,err_1=0,err_r=0,err_r_r=0,err_r_r_r=0; 
	static float err_sum=0;
	servo_leftTMid=(float)(servo_left-servo_Midpwm1);
	servo_rigthTMid=(float)(servo_Midpwm1-servo_right);
	
	
	
	
	//中线拟合
	
	if(flag==R_CL){
		
		err=(R_CL_a*nihejiaodu_G1+R_CL_d*paodao_piancha_G1)*servo_rigthTMid;
		
		if(err>servo_leftTMid) err=servo_leftTMid;
		else if(err<-servo_rigthTMid) err=-servo_rigthTMid;
		err_sum=err+err_r+err_r_r+err_r_r_r;
//		
		servo_pwm=(float)(servo_Midpwm1)+R_CL_kp*err+R_CL_kd*(err-err_1)+R_CL_ki*err_sum;//向左偏的太严重就向右转，右转取决于err，本次err绝对值越大，偏的越严重，越需要矫正，假如本次的err是-13，上一次是-15，err-err1=2，kd可以减小超调，假如本次err是-13，上一次是-10，说明越偏越严重，err-err1=-3，kd加大矫正作用
		
	}else if(flag==L_CL){
		
		err=(L_CL_a*nihejiaodu_G1+L_CL_d*paodao_piancha_G1)*servo_leftTMid;
		if(err>servo_leftTMid) err=servo_leftTMid;
		else if(err<-servo_rigthTMid) err=-servo_rigthTMid;
		err_sum=err+err_r+err_r_r+err_r_r_r;
//		
		servo_pwm=(float)(servo_Midpwm1)+L_CL_kp*err+L_CL_kd*(err-err_1)+L_CL_ki*err_sum;//向右偏的太严重就向左转，左转转取决于err，本次err绝对值越大，偏的越严重，越需要矫正，假如本次err是13，上一次是15，说明矫正有效果，err-err1=-2，kd可以减小超调，假如本次是13，上次是10，err-err1=3，说明越偏越严重，kd可加大矫正作用
		
		
	}
	
	
	//右转
	else if(flag==YZ_j) {
														
		err=(YZ_j_a*nihejiaodu_G1+YZ_j_d*paodao_piancha_G1+YZ_j_ZW*ZW_ave_G1)*servo_rigthTMid;
		if(err>servo_leftTMid) err=servo_leftTMid;
		else if(err<-servo_rigthTMid) err=-servo_rigthTMid;
		err_sum=err+err_r+err_r_r+err_r_r_r;
//		
		servo_pwm=(float)(servo_Midpwm1)+YZ_j_kp*err+YZ_j_kd*(err-err_1)+YZ_j_ki*err_sum;
	
	}
	
	//左转
	else if(flag==ZZ_j) {
															
		err=(ZZ_j_a*nihejiaodu_G1+ZZ_j_d*paodao_piancha_G1+ZZ_j_ZW*ZW_ave_G1)*servo_leftTMid;
		if(err>servo_leftTMid) err=servo_leftTMid;
		else if(err<-servo_rigthTMid) err=-servo_rigthTMid;
		err_sum=err+err_r+err_r_r+err_r_r_r;
//		
		servo_pwm=(float)(servo_Midpwm1)+ZZ_j_kp*err+ZZ_j_kd*(err-err_1)+ZZ_j_ki*err_sum;
	
	}
	
	printf("flag=%d,err=%.2f,err_1=%.2f,servo_pwm=%.2f\n",flag,err,err_1,servo_pwm);
	err_r_r_r=err_r_r;
	err_r_r=err_r;
	err_r=err;
	err_1=0.2*err_r_r_r+0.3f*err_r_r+0.6f*err_r;
	Servo_ChangePwm((uint16_t)servo_pwm);
}













