void USART_SendFloat(USART_TypeDef *USARTx, float data) {  
    // 将float的地址转换为uint8_t指针，以便按字节访问  
    uint8_t *floatBytes = (uint8_t *)&data;  
	uint8_t i = 0;
    // 创建一个数组来存储大端格式的字节  
    uint8_t sendFrame[sizeof(float)+4];  
	sendFrame[0]=0x55;
	sendFrame[1]=0xd1;
	sendFrame[6]=0x33;
	sendFrame[7]=0x1d;
    // 交换字节顺序  
    for ( i = 0; i < sizeof(float); i++) {  
        sendFrame[sizeof(float)  - i+1] = floatBytes[i];  
    }  
  
    // 发送大端格式的字节  
    for ( i = 0; i < sizeof(float)+4; i++) {  
        USART_SendData(USARTx, sendFrame[i]);  
        // 等待发送完成  
        while (USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET);  
    }  
}



u8 Serial_RxData;
u8 Serial_RxFlag;

void Serial_SendByte(uint8_t Byte)
{
	USART_SendData(USART6, Byte);
	while (USART_GetFlagStatus(USART6, USART_FLAG_TXE) == RESET);
}

void USART1_IRQHandler(void)
{
	if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET)
	{
		Serial_RxData = USART_ReceiveData(USART1);
		Serial_RxFlag = 1;
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
	}
}
uint8_t Serial_GetRxFlag(void)
{
	if (Serial_RxFlag == 1)
	{
		Serial_RxFlag = 0;
		return 1;
	}
	return 0;
}

uint8_t Serial_GetRxData(void)
{
	return Serial_RxData;
}



void Handle1_Plane(void){//3000个原始数据转为极坐标数据，打印到串口
	u16 i=0;
//	u16 cnt=0;
	float angle=354.32;
//	u8 s = 0x01;
//	USART_SendData(USART1, s);


	
	USART_SendFloat(USART1, angle);
//	printf("%d",sizeof(float));
	

	
	while(1){
		
//		for(i=0;i<10;i++)
//		{
//			printf("%f\n",LEIDA_DATA[i].angle);
//		}
//		if(DMA_RX_DONE){			
//			DMA_RX_DONE = 0;	
//			cnt=LEIDA_DATA_HANDLE1(LEIDA_DATA,DMA_USART2_RX_BUF_r,DMA_USART2_RX_BUF_LEN	);																																																//验证报文后做数据处理
//			if(cnt!=0)					//将数据转为极坐标
//			{
//				
//				for ( i = 0; i < cnt; i++) {  
//					USART_SendFloat(USART1, LEIDA_DATA[i].angle); // 发送angle
//					USART_SendFloat(USART1, LEIDA_DATA[i].distance); // 发送distance  
//				} 
//					
//			}
//		}
		
	}
}



//#include <stdio.h>

int daduanxiaoduan() {//是小端
    unsigned int i = 1;
    char *c = (char*)&i;
    if (*c)
        printf("Little endian\n");
    else
        printf("Big endian\n");
    return 0;
}


void Debug_TEST(void){

//	u8 t;
//	u8 len;	
//	u16 times=0;  
//	while(1)
//	{
//		if(USART_RX_STA&0x8000)
//		{					   
//			len=USART_RX_STA&0x3fff;//得到此次接收到的数据长度
//			printf("\r\n您发送的消息为:\r\n");
//			printf("\r\n您发送的cd为:%d\r\n",len);
//			for(t=0;t<len;t++)
//			{
//				USART_SendData(USART1, USART_RX_BUF[t]);         //向串口1发送数据
//				while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//等待发送结束
//			}
//			printf("\r\n\r\n");//插入换行
//			USART_RX_STA=0;
//		}else
//		{
//			times++;
//			if(times%200==0)printf("请输入数据,以回车键结束\r\n");  
//			delay_ms(10);   
//		}
//	}
}







else{
								
								
								printf("左转\n");
								if(min_y<zhuan_d1){
									 if(nihejiao>(90.0f+zhuan_a1)){//大于90+20
										printf("前方被拦截，左打死\n");
										Servo_ChangePwm(servo_left);
									}else{
										printf("前方被拦截，右转\n");
										pid_slt=ZZ_j;
	//									qianfang30_PD(min_y,(float)(nihejiao),&F_line,pid_slt);
									}
								}else if(min_y<zhuan_d2){
									if(nihejiao>(90.0f+zhuan_a2)){//大于90+40
										printf("前较远方被拦截，左打死\n");
										Servo_ChangePwm(servo_left);
									}else{
										printf("前方，左转\n");
										pid_slt=ZZ_h;
	//									qianfang30_PD(min_y,(float)(nihejiao),&F_line,pid_slt);
									}
								}else if(min_y<zhuan_d3){
									if(nihejiao>(90.0f+zhuan_a3)){//大于90+70，找到不能右打死的范围
										printf("远方被拦截，左打死\n");
										Servo_ChangePwm(servo_left);
									}else{
										printf("前方，左转\n");
										pid_slt=ZZ_y;
	//									qianfang30_PD(min_y,(float)(nihejiao),&F_line,pid_slt);
									}
									
								}else {
									printf("极远，不左转\n");
									Servo_ChangePwm(servo_Midpwm1);
								}
							}




















float tie_daoche_a=65,tie_daoche_d=220;
float tie_a1=30, tie_d1=250;
float tie_a2=50, tie_d2=500;
float tie_a3=80, tie_d3=800;

extern float tie_daoche_a,tie_daoche_d;
extern float tie_a1, tie_d1;
extern float tie_a2, tie_d2;
extern float tie_a3, tie_d3;

//	//右贴，左转 a:d 角度距离权值比   kp调整幅度 kd限制超调
//	static float YT_j_a=1,    YT_j_d=0.3,    YT_j_kp=0.9,    YT_j_kd=0.05;
//	static float YT_h_a=1,    YT_h_d=0.3,    YT_h_kp=0.9,    YT_h_kd=0.05;
//	static float YT_y_a=1,   YT_y_d=0.3,    YT_y_kp=0.9,    YT_y_kd=0.05;
//	
//	//左贴，右转
//	static float ZT_j_a=1,    ZT_j_d=0.3,    ZT_j_kp=0.9,    ZT_j_kd=0.05;
//	static float ZT_h_a=1,    ZT_h_d=0.3,    ZT_h_kp=0.9,    ZT_h_kd=0.05;
//	static float ZT_y_a=1,   ZT_y_d=0.3,    ZT_y_kp=0.9,    ZT_y_kd=0.05;
u16 left_duan_cnt=0,right_duan_cnt=0;
float left_ave=4000,right_ave=4000,left_min_y=4000,left_min_yx=4000,left_min_x=4000,left_min_xy=4000,right_min_y=4000,right_min_yx=4000,right_min_x=4000,right_min_xy=4000;
	
					{//有断点
							left_duan_cnt=Duan_p+1;//左侧断点数
							printf("left_duan_cnt=%d\n",left_duan_cnt);
							right_duan_cnt= qianfang30_cnt-Duan_p-1;//右侧断点数
							printf("right_duan_cnt=%d\n",right_duan_cnt);
							
							left_ave = Qianfang_ave(qianfang30_plane,0,Duan_p,&left_min_y,&left_min_yx,&left_min_x,&left_min_xy);
							printf("left_ave=%.2f\n",left_ave);
							right_ave = Qianfang_ave(qianfang30_plane,Duan_p+1,Duan_p+right_duan_cnt,&right_min_y,&right_min_yx,&right_min_x,&right_min_xy);
							printf("right_ave=%.2f\n",right_ave);
							
							//检查是否有侧贴 
							if(right_ave<left_ave){
								printf("必先向左转\n");
								if(right_duan_cnt>nihe_mcnt){//nihe_mcnt=8
									Midline_fit_2(qianfang30_plane,(qianfang30_cnt-right_duan_cnt/3*2),qianfang30_cnt,&right_duan_line, &nihejiao);
									printf("必先向左转\n");
									
									if(right_min_y<tie_d1){
										
										if((nihejiao>(90.0f+tie_daoche_a))&&nihejiao<=180.0f&&right_min_y<tie_daoche_d)//大于90+55就打死
										{
											printf("右急贴，左daoche\n");
											daoche_flag=1;
											y_z_daoche_flag=zuodao;
											
										}else if(nihejiao>(90.0f+tie_a1)&&(nihejiao<=(90.0f+tie_daoche_a))){  //大于90+30
											printf("右贴\n");
											Servo_ChangePwm(servo_left);
										}else {//90<nihejiao<(90.0f+tie_a1)
											printf("右贴缓\n");//>90 <105
											pid_slt=YT_j;
	//										qianfang30_PD(right_min_y,(float)(nihejiao),&right_duan_line,pid_slt);
										}
										
									}else if(right_min_y<=tie_d2){
										
										if((nihejiao>90.0f+tie_a2)&&nihejiao<=180.0f){
											printf("1右贴\n");
											Servo_ChangePwm(servo_left);
										}else {//90<nihejiao<(90.0f+tie_a2)
											printf("1右贴缓\n");
											pid_slt=YT_h;
	//										qianfang30_PD(right_min_y,(float)(nihejiao),&right_duan_line,pid_slt);
										}
										
									}else if(right_min_y<tie_d3){
										
											printf("右方较远\n");
											pid_slt=YT_y;
	//										qianfang30_PD(right_min_y,(float)(nihejiao),&right_duan_line,pid_slt);
									}else {
										printf("右方很远\n");
										Servo_ChangePwm(servo_Midpwm1);
									}
									
								}else{//right_duan_cnt<8
									Servo_ChangePwm(servo_Midpwm1+2);
									printf("right_duan_cnt<8\n");
								}
								
							}else{
								
								if(left_duan_cnt>nihe_mcnt){
									Midline_fit_2(qianfang30_plane,0,Duan_p/3*2,&left_duan_line, &nihejiao);
									printf("必先向右转\n");
									if(left_min_y<=tie_d1){
										
										if((nihejiao<(90.0f-tie_daoche_a))&&nihejiao>=0&&left_min_y<tie_daoche_d)//小于于90-35就打死
										{
											printf("左急贴，右daoche\n");
											daoche_flag=1;
											y_z_daoche_flag=youdao;
											
										}else if(nihejiao<(90.0f-tie_a1)&&(nihejiao>=(90.0f-tie_daoche_a))){  //90-20
											printf("左贴\n");
											Servo_ChangePwm(servo_right);
										}else {
											printf("左贴缓\n");
											pid_slt=ZT_j;
	//										qianfang30_PD(left_min_y,(float)(nihejiao),&left_duan_line,pid_slt);
										}
										
									}else if(left_min_y<=tie_d2){
										
										if(nihejiao<(90.0f-tie_a1)&&nihejiao>0){
											printf("1左贴\n");
											Servo_ChangePwm(servo_right);
										}else {//90>nihejiao>(90.0f-tie_a2)
											printf("1左贴缓\n");
											pid_slt=ZT_h;
	//										qianfang30_PD(left_min_y,(float)(nihejiao),&left_duan_line,pid_slt);
										}
										
									}else if(left_min_y<tie_d3){//0-90度
										
											printf("左方较远\n");
											pid_slt=ZT_y;
	//										qianfang30_PD(left_min_y,(float)(nihejiao),&left_duan_line,pid_slt);
									}else {
										printf("左方很远\n");
										Servo_ChangePwm(servo_Midpwm1);
									}
									
									
								}else{//left_duan_cnt<8
									Servo_ChangePwm(servo_Midpwm1-2);
									printf("letf_duan_cnt<8\n");
								}
							}
						}















else if(flag==YT_j) {
															//200-250
//		err=(YT_j_a*(nihejiaodu-90.0f)/tie_a1+YT_j_d*(tie_d1-miny)/(tie_d1-DD))*servo_leftTMid;
//		servo_pwm=(float)(servo_Midpwm1)+YT_j_kp*err+YT_j_kd*(err-err_1);
	
	}else if(flag==YT_h) {
															//250-500
//		err=(YT_h_a*(nihejiaodu-90.0f)/tie_a2+YT_h_d*(tie_d2-miny)/(tie_d2-tie_d1))*servo_leftTMid;
//		servo_pwm=(float)(servo_Midpwm1)+YT_h_kp*err+YT_h_kd*(err-err_1);
//	
	}else if(flag==YT_y) {
															//500-800
//		err=(YT_y_a*(nihejiaodu-90.0f)/tie_a3+YT_y_d*(tie_d3-miny)/(tie_d3-tie_d2))*servo_leftTMid;
//		servo_pwm=(float)(servo_Midpwm1)+YT_y_kp*err+YT_y_kd*(err-err_1);
	}
	
	//左贴，右转
	else if(flag==ZT_j) {
															//200-250
//		err=(ZT_j_a*(nihejiaodu-90.0f)/tie_a1+ZT_j_d*(tie_d1-miny)/(tie_d1-DD))*servo_rigthTMid;
//		servo_pwm=(float)(servo_Midpwm1)+ZT_j_kp*err+ZT_j_kd*(err-err_1);
	
	}else if(flag==ZT_h) {
															//250-500
//		err=(ZT_h_a*(nihejiaodu-90.0f)/tie_a2+ZT_h_d*(tie_d2-miny)/(tie_d2-tie_d1))*servo_rigthTMid;
//		servo_pwm=(float)(servo_Midpwm1)+ZT_h_kp*err+ZT_h_kd*(err-err_1);
	
	}else if(flag==ZT_y) {
															//500-800
//		err=(ZT_y_a*(nihejiaodu-90.0f)/tie_a3+ZT_y_d*(tie_d3-miny)/(tie_d3-tie_d2))*servo_rigthTMid;
//		servo_pwm=(float)(servo_Midpwm1)+ZT_y_kp*err+ZT_y_kd*(err-err_1);
	}