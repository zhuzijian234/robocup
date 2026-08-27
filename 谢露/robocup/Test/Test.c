#include "sys.h"
#include "Test.h"	




void Handle3_TEST(void){
	u16 chalu_cnt=0;
	u16 cnt=0;
	u8 DMA_USART2_RX_BUF_r_r[DMA_USART2_RX_BUF_LEN];
	
	u16 qianfang_0_180=0;
	
	
	float left_duan_y=6000;
	float right_duan_y=6000;
	
	float left_d_ave=0;
	float right_d_ave=0;
	
	double nihejiao_L_x1=90;
	double nihejiao_R_x1=90;
	double nihejiao_M_x1=90;
	
	double nihejiao_M_x1_G1=0;//向右转nih
	float paodao_piancha_G1=0;//paodao_piancha_G1从-1到1，1靠进右道，要左转，pwm加，
	
	
	u8 pid_slt=0;
	
	
	double nihejiao=90;
	
	
	
	double nihejiao_1[3]={90,90,90};
	
	
	
	
	
	double nihejiao_x1=90;
	
	double nihejiao_x1_G1=0;
	
	
	double nihejiao_G1=0;
	
	
	
	float ZW_JZB_ave=0;
	float ZW_ave_G1=0;
	
	
	
					
	while(1){
		
		//printf("1\n");
		if(DMA_RX_DONE)//有没有可能出现dma标志刚等于0，又转运了数据到DMA_USART2_RX_BUF_r，但是没关系dma可以转运成功只会改变DMA_USART2_RX_BUF_r，而且雷达的数据没这么快
		{			
			DMA_RX_DONE = 0;	
			memcpy(DMA_USART2_RX_BUF_r_r,DMA_USART2_RX_BUF_r,	DMA_USART2_RX_BUF_LEN);	
			cnt=LEIDA_DATA_HANDLE1(LEIDA_DATA,DMA_USART2_RX_BUF_r_r,DMA_USART2_RX_BUF_LEN	);	
			printf("\r\n\r\nstart\r\n\r\n");
			printf("cnt=%d\n",cnt);
																																								//验证报文后做数据处理
#if 1			
			if(cnt>200)					
			{
				
				printf("Speed_now=%.2f,Moto_pwm=%d\n",Speed_now,Moto_pwm);
				LEIDA_DATA_360paixu(DATA_360paixu,LEIDA_DATA,cnt);//原始数据转成按降序排列
				
				Houlun_paixu(DATA_houlun_paixu ,DATA_360paixu,DATA_360paixu_cnt);
				LEIDA_DATA_HANDLE3_2(DATA_houlun_paixu,DATA_houlun);//滤波			
				
				printf("Cnt_180=%d\r\n",Cnt_180);
				
				
				
				qianfang_0_180=qianfang_0_180_handle(DATA_0_180,&left_duan_y,&right_duan_y);
				if(qianfang_0_180==0) { printf("yidaoche\r\n");	continue;}
//				printf("qianfang40_cnt=%d\r\n",qianfang40_cnt);

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
				
				
//				

//					
////				
//				
				
				
				
				nihejiao_M_x1_G1=0;
				
				right_d_ave=0;
				left_d_ave=0;
				paodao_piancha_G1=0;
				
				ZW_ave_G1=0;
				nihejiao_G1=0;
				nihejiao_x1_G1=0;
				
				
				if(left_duan_y<zhuandian&&right_duan_y<zhuandian){
					GPIO_SetBits(GPIOF,GPIO_Pin_9);//灭灯
					GPIO_SetBits(GPIOF,GPIO_Pin_10);//灭灯
				
					continue;
					BLUE_change_sign=1;
//					
					if(left_duan_y>200.0f&&right_duan_y>200.0f){
						K_ave(left_duan_plane,(u16)(left_duan_cnt*0.05),(u16)(left_duan_cnt*0.95),&nihejiao_L_x1,3);
						K_ave(right_duan_plane,(u16)(right_duan_cnt*0.05),(u16)(right_duan_cnt*0.95),&nihejiao_R_x1,3);
						left_d_ave=paodao_kuan(left_duan_plane,0,LEFT_cnt_CL,0.0f,500.0f);
						right_d_ave=paodao_kuan(right_duan_plane,0,RIGHT_cnt_CL,0.0f,500.0f);
						nihejiao_M_x1=(nihejiao_L_x1+nihejiao_R_x1)/2.0;
						

						
						
						
						if(nihejiao_M_x1<90.0&&(nihejiao_M_x1>=CL_jixianjiao))          {pid_slt=R_CL;printf("nihejiao_M_x1=%.2lf\r\n",nihejiao_M_x1);}
						else if(nihejiao_M_x1<CL_jixianjiao&&nihejiao_M_x1>0)					{Servo_ChangePwm(servo_right);continue;}
						
						else if((nihejiao_M_x1<=(180.0-CL_jixianjiao))&&nihejiao_M_x1>90.0)  	{pid_slt=L_CL;printf("nihejiao_M_x1=%.2lf\r\n",nihejiao_M_x1);}
						else if(nihejiao_M_x1>(180.0-CL_jixianjiao)&&nihejiao_M_x1<180.0)		{Servo_ChangePwm(servo_left);continue;}
						

						paodao_piancha_G1=paodao_G1(CL,left_d_ave, right_d_ave);
						
						nihe_G1(pid_slt ,nihejiao_M_x1,&nihejiao_M_x1_G1);
					

						qianfang30_PD(paodao_piancha_G1,(float)(nihejiao_M_x1_G1), ZW_ave_G1,pid_slt);
						
						continue;
					}
					else
					{
						continue;
						chalu_cnt++;
						chalu(chalu_cnt);
						
					}
				}else if(left_duan_y>zhuandian&&right_duan_y>zhuandian){
					
					
					
					K_ave(left_duan_plane,(u16)(LEFT_cnt_CL*0.63),(u16)(LEFT_cnt_CL*0.99),&nihejiao_L_x1,4);
					K_ave(right_duan_plane,(u16)(RIGHT_cnt_CL*0.63),(u16)(RIGHT_cnt_CL*0.99),&nihejiao_R_x1,4);
					
					
					nihejiao_M_x1=(nihejiao_L_x1+nihejiao_R_x1)/2.0;
					
					
					left_d_ave=paodao_kuan(left_duan_plane,1,(u16)(LEFT_cnt_CL*0.95),0.0f,400.0f);
					right_d_ave=paodao_kuan(right_duan_plane,1,(u16)(RIGHT_cnt_CL*0.95),0.0f,400.0f);
				
				
					
					if((right_d_ave-left_d_ave)<100||right_d_ave<0||left_d_ave>0) 
					{
							
						continue;
					}
					
					
					
					if(nihejiao_M_x1<=0||nihejiao_M_x1>=180.0) {
						
						continue;
					}
					else if(nihejiao_M_x1<90.0&&(nihejiao_M_x1>=CL_jixianjiao))          {pid_slt=R_CL;printf("nihejiao_M_x1=%.2lf\r\n",nihejiao_M_x1);}
					else if(nihejiao_M_x1<CL_jixianjiao&&nihejiao_M_x1>0)					{Servo_ChangePwm(servo_right);continue;}
					
					else if((nihejiao_M_x1<=(180.0-CL_jixianjiao))&&nihejiao_M_x1>90.0)  	{pid_slt=L_CL;printf("nihejiao_M_x1=%.2lf\r\n",nihejiao_M_x1);}
					else if(nihejiao_M_x1>(180.0-CL_jixianjiao)&&nihejiao_M_x1<180.0)		{Servo_ChangePwm(servo_left);continue;}
					
					paodao_piancha_G1=paodao_G1(CL,left_d_ave, right_d_ave);
					
					nihe_G1(pid_slt ,nihejiao_M_x1,&nihejiao_M_x1_G1);
				

					qianfang30_PD(paodao_piancha_G1,(float)(nihejiao_M_x1_G1), ZW_ave_G1,pid_slt);
				
					
				}else if(left_duan_y>zhuandian&&right_duan_y<=zhuandian||left_duan_y<zhuandian&&right_duan_y>=zhuandian){
					
					
					if(left_duan_y>zhuandian&&right_duan_y<=zhuandian){//右转
					
						ZW_ave(left_135_85_plane,((u16)(left_135_85_cnt*0.05)),((u16)(left_135_85_cnt*0.95)),&ZW_JZB_ave);
						Midline_fit_2(left_135_85_plane,((u16)(left_135_85_cnt*0.05)),((u16)(left_135_85_cnt*0.95)), &nihejiao);
						Midline_fit_2(left_135_85_plane,((u16)(left_135_85_cnt*0.05)),((u16)(left_135_85_cnt*0.45)), &nihejiao_1[0]);
						Midline_fit_2(left_135_85_plane,((u16)(left_135_85_cnt*0.35)),((u16)(left_135_85_cnt*0.65)), &nihejiao_1[1]);
						Midline_fit_2(left_135_85_plane,((u16)(left_135_85_cnt*0.65)),((u16)(left_135_85_cnt*0.95)), &nihejiao_1[2]);
						
						
						pid_slt=YZ_j;
						
					}else if(left_duan_y<zhuandian&&right_duan_y>=zhuandian){//左转
						
						ZW_ave(right_45_110_plane,((u16)(right_45_110_cnt*0.05)),((u16)(right_45_110_cnt*0.95)),&ZW_JZB_ave);
						Midline_fit_2(right_45_110_plane,((u16)(right_45_110_cnt*0.05)),((u16)(right_45_110_cnt*0.95)), &nihejiao);
						Midline_fit_2(right_45_110_plane,((u16)(right_45_110_cnt*0.05)),((u16)(right_45_110_cnt*0.45)), &nihejiao_1[0]);
						Midline_fit_2(right_45_110_plane,((u16)(right_45_110_cnt*0.35)),((u16)(right_45_110_cnt*0.65)), &nihejiao_1[1]);
						Midline_fit_2(right_45_110_plane,((u16)(right_45_110_cnt*0.65)),((u16)(right_45_110_cnt*0.95)), &nihejiao_1[2]);
						
						
						pid_slt=ZZ_j;
					}
					
					
					if(LEFT_cnt_CL<4&&RIGHT_cnt_CL>4)
					{
						left_d_ave=0;
						right_d_ave=paodao_kuan(right_duan_plane,0,RIGHT_cnt_CL,0.0f,200.0f);
					}else if(RIGHT_cnt_CL<4&&LEFT_cnt_CL>4)
					{
						right_d_ave=0;
						left_d_ave=paodao_kuan(left_duan_plane,0,LEFT_cnt_CL,0.0f,200.0f);
					}
					
					
						
					Tidu(pid_slt ,nihejiao_1,&nihejiao_x1);
					
					nihe_G1(pid_slt ,nihejiao_x1,&nihejiao_x1_G1);
					nihe_G1(pid_slt ,nihejiao,&nihejiao_G1);
					paodao_piancha_G1=paodao_G1(ZW,left_d_ave, right_d_ave);
					ZW_G1(pid_slt,ZW_JZB_ave,&ZW_ave_G1);
					printf("nihejiao_x1_G1=%.2lf,nihejiao_G1=%.2lf,paodao_piancha_G1=%.2lf,ZW_ave_G1=%.2f\r\n",nihejiao_x1_G1,nihejiao_G1,paodao_piancha_G1,ZW_ave_G1);
					qianfang30_PD(paodao_piancha_G1,(float)(nihejiao_G1),ZW_ave_G1,pid_slt);	 
					IWDG_Feed();//喂狗
				}
				
				
					
				
			}//if（cnt）
			
#endif      //if（cnt）
		}//if(DMA_RX_DONE)
	}//while

}
	

void chalu(u16 chalu_cnt){
	switch(chalu_cnt)
	{
		case 1:
		{
			Servo_ChangePwm(servo_left+30);
			delay_ms(500);
			break;
		}
		case 2:
		{
			Servo_ChangePwm(servo_right+30);
			delay_ms(500);
			break;
		}
		default:  
		{  
			break;  
		}  
	}
}


