/***********************************************
��˾����Ȥ�Ƽ�(��ݸ)���޹�˾
Ʒ�ƣ�WHEELTEC
������wheeltec.net
�Ա����̣�shop114407458.taobao.com 
����ͨ: https://minibalance.aliexpress.com/store/4455017
�汾��V1.0
�޸�ʱ�䣺2022-09-01

Brand: WHEELTEC
Website: wheeltec.net
Taobao shop: shop114407458.taobao.com 
Aliexpress: https://minibalance.aliexpress.com/store/4455017
Version: V1.0
Update��2022-09-01

All rights reserved
***********************************************/
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */
//�������´���,֧��printf����,������Ҫѡ��use MicroLIB	  
#if 1               
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
void _sys_exit(int x) 
{ 
	x = x; 
} 
////�ض���fputc���� 
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0); //ʹ�ô���1   
	USART1->DR = (u8) ch;      
	return ch;
}
#endif


/**************************************************************************
Function: Serial port 1 sends data
Input   : The data to send
Output  : none
�������ܣ�����1��������
��ڲ�����Ҫ���͵�����
����  ֵ����
**************************************************************************/
void usart1_send(u8 data)
{
	USART1->DR = data;
	while((USART1->SR&0x40)==0);	
}

u8 Usart1_Receive_buf[1];          //����1�����ж����ݴ�ŵĻ�����
u8 Uart5_Receive_buf[1];          //����5�����ж����ݴ�ŵĻ�����
PointDataProcessDef PointDataProcess[70] ;//����70������
LiDARFrameTypeDef Pack_Data;
/* USER CODE END 0 */

UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;

/* UART5 init function */
void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 512000;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */
   HAL_UART_Receive_IT(&huart5,Uart5_Receive_buf,sizeof(Uart5_Receive_buf)); //�򿪴���5�����ж�
  /* USER CODE END UART5_Init 2 */

}
/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
  /* USER CODE END USART1_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==UART5)
  {
  /* USER CODE BEGIN UART5_MspInit 0 */

  /* USER CODE END UART5_MspInit 0 */
    /* UART5 clock enable */
    __HAL_RCC_UART5_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**UART5 GPIO Configuration
    PC12     ------> UART5_TX
    PD2     ------> UART5_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART5;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART5;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* UART5 interrupt Init */
    HAL_NVIC_SetPriority(UART5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(UART5_IRQn);
  /* USER CODE BEGIN UART5_MspInit 1 */

  /* USER CODE END UART5_MspInit 1 */
  }
  else if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==UART5)
  {
  /* USER CODE BEGIN UART5_MspDeInit 0 */

  /* USER CODE END UART5_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_UART5_CLK_DISABLE();

    /**UART5 GPIO Configuration
    PC12     ------> UART5_TX
    PD2     ------> UART5_RX
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_12);

    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_2);

    /* UART5 interrupt Deinit */
    HAL_NVIC_DisableIRQ(UART5_IRQn);
  /* USER CODE BEGIN UART5_MspDeInit 1 */

  /* USER CODE END UART5_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

u8 Len = 255;  //�����ݳ���
extern u8 flag ;

void data_process(void)//һ֡���ݵĴ���
{
	static u8 data_cnt = 0;
	u8 i;
	u32 distance_sum1 = 0;//�����
	u16 average_distance1 = 0;//ƽ������
	u32 distance_sum2 = 0;//�����
	u16 average_distance2 = 0;//ƽ������
	float Angle;
	float angle1,angle2;
	Len = (Len-20)/2;
	Angle = ((Pack_Data.angle_H<<8)+Pack_Data.angle_L)/100;
	angle1 = Angle - 10;
	angle2 = Angle - 5;
	
	if(angle1<0)
		angle1 += 360;
	if(angle2<0)
		angle2 += 360;
	for(i = 0;i<Len;i++)//��ƽ���ֳ�������
	{
		if(i<(Len/2))
			distance_sum1 += Pack_Data.point[i];
		else
			distance_sum2 +=  Pack_Data.point[i];
	}
	average_distance1 = distance_sum1/(Len/2);//��ƽ������
	average_distance2 = distance_sum2/(Len-(Len/2));//��ƽ������
	flag = 1;
	PointDataProcess[data_cnt].angle = angle1;
	PointDataProcess[data_cnt].distance = average_distance1;//50�������ж�
	PointDataProcess[data_cnt+1].angle = angle2;
	PointDataProcess[data_cnt+1].distance = average_distance2;//50�������ж�
	data_cnt += 2;
	if(data_cnt >= 70)
		flag = 1,
		data_cnt = 0;

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef*huart) //���ջص�����
{
	if(huart -> Instance == UART5)
	{
		static u8 state = 0;//״̬λ	
		static u8 cnt = 0;//����һ֡70����ļ���
		u8 temp_data;
	  temp_data=Uart5_Receive_buf[0];	
			if (state > 7)
			{
				if(state < ( Len - 12))
				{
					if(state%2 == 0)//ż����8,10......�Ǿ���ĸ�λ
					{
						Pack_Data.point[cnt] = temp_data;
						state++;
					}
					else//��λ����ʱ��������
					{
						Pack_Data.point[cnt] = (Pack_Data.point[cnt]<<8)+temp_data;
						if(Pack_Data.point[cnt]==0XFFFF)   Pack_Data.point[cnt] = 0;
						else
							Pack_Data.point[cnt] = (Pack_Data.point[cnt] & 0X7FFF);
						cnt++;	
						state++;
					}
				}
				else if((state>(Len-13))&&(state<(Len-2)))//GPSʱ����ϢԤ��λ
				{
					state++;
				}
				else if(state == (Len-2))
				{
					if(temp_data == END_0)//֡β
					{
						Pack_Data.end_0 = temp_data;
						state++;
					} 
					else
					{
						state = 0;
						cnt = 0;
						memset(&Pack_Data,0,sizeof(Pack_Data));
					}
			
				}
				else if (state == (Len-1))
				{
					if(temp_data == END_1)//֡β
					{
						Pack_Data.end_1 = temp_data;
						data_process();
						state = 0;//��λ
						cnt = 0;//��λ
						Len = 255; //��λ
					} 
					else
					{
						state = 0;
						cnt = 0;
						memset(&Pack_Data,0,sizeof(Pack_Data));//����
					}
				}
			}
			else 
			{
				switch(state)
				{
					case 0:
						if(temp_data == HEADER_0)//ͷ�̶�
						{
							Pack_Data.header_0 = temp_data;
							state++;
						} else state = 0;
						break;
					case 1:
						if(temp_data == HEADER_1)//ͷ�̶�
						{
							Pack_Data.header_1 = temp_data;
							state++;
             
						} else state = 0;
						break;
					case 2:
							Pack_Data.Len_H = temp_data;//�����Ƕ�
							state++;
						break;
					case 3:
							Pack_Data.Len_L = (Pack_Data.Len_H<<8)+temp_data;
							Len = Pack_Data.Len_L;
							state++;
						break;
					case 4:
						Pack_Data.angle_H = temp_data;//�Ƕȸ�λ
						state++;
						break;
					case 5:
						Pack_Data.angle_L = temp_data;
						state++;
						break;
					case 6:
						Pack_Data.speed_H = temp_data;
						state++;
						break;
					case 7:
						Pack_Data.speed_L = temp_data;
						state++;
						break;
					default: break;

				}
			}
	  HAL_UART_Receive_IT(&huart5,Uart5_Receive_buf,sizeof(Uart5_Receive_buf));//����5�ص�����ִ�����֮����Ҫ�ٴο��������жϵȴ���һ�ν����жϵķ���	
	}		
}
/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
