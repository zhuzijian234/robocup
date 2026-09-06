/**
 * @file    bsp_bluetooth.c
 * @brief   HC-05蓝牙模块驱动 — USART6
 *
 * 硬件: USART6 (PC6=TX, PC7=RX), 波特率9600
 * 连接状态: PC2 (HC-05 STATE引脚, 高=已连接, 低=未连接)
 *
 * 功能:
 *   - AT指令透传
 *   - IDLE线路检测实现变长帧接收
 *   - STATE引脚监控连接状态
 *   - 向手机APP发送/接收ASCII字符串
 *
 * 对应谢露版引脚复用: 蓝牙从USART3(PB10/PB11)移至USART6(PC6/PC7)
 *
 * 原始出处: 立创开发板开源BSP (www.lckfb.com)
 */

#include "bsp_bluetooth.h"
#include "stdio.h"

unsigned char Bluetooth_ConnectFlag = 0;  /* 蓝牙连接状态: 0=未连接, 1=已连接 */
unsigned char BLERX_BUFF[BLERX_LEN_MAX]; /*接收缓冲区*/
unsigned char BLERX_FLAG = 0;            /*一帧数据接收完成标志*/
unsigned char BLERX_LEN  = 0;            /*当前已收到的字节数  */

/**
 * @brief  初始化USART6的GPIO引脚 (TX=PC6, RX=PC7)
 * @param  bund: 波特率 (默认9600)
 */
void Bluetooth_GPIO_Init(unsigned int bund)
{
    GPIO_InitTypeDef GPIO_InitStructure;
/*RCC:时钟系统 AHB1:GPIO挂载的总线 Periph:外设 Clock:时钟 Cmd:使能还是关闭*/
    RCC_AHB1PeriphClockCmd(BSP_BLUETOOTH_TX_RCC, ENABLE);
    RCC_AHB1PeriphClockCmd(BSP_BLUETOOTH_RX_RCC, ENABLE);

/*GPIO:外设 Pin:引脚 AF:复用功能 Config:配置
/*参数1: BSP_BLUETOOTH_TX_PORT
       → GPIOC
       含义: TX引脚在哪个GPIO端口上 → PC6在C口*/

/*参数2: BSP_BLUETOOTH_TX_SOURCE
       → GPIO_PinSource6
       含义: 这个端口里的第几号引脚 → 第6号*/

/*参数3: BSP_BLUETOOTH_AF
       → GPIO_AF_USART6
       含义: 复用给哪个外设 → USART6*/

    GPIO_PinAFConfig(BSP_BLUETOOTH_TX_PORT, BSP_BLUETOOTH_TX_SOURCE, BSP_BLUETOOTH_AF);
    GPIO_PinAFConfig(BSP_BLUETOOTH_RX_PORT, BSP_BLUETOOTH_RX_SOURCE, BSP_BLUETOOTH_AF);

    /* TX引脚 (PC6) */
    GPIO_StructInit(&GPIO_InitStructure); /*软件层面写初始化结构体的函数,防止忘记初始化*/
    GPIO_InitStructure.GPIO_Pin   = BSP_BLUETOOTH_TX_PIN; /*选哪几个脚*/
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF; /*复用模式*/
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;/*反转斜率*/
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; /*推挽输出:驱动能力强,信号干净*/
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP; /*空闲时上拉*/
    GPIO_Init(BSP_BLUETOOTH_TX_PORT, &GPIO_InitStructure); /*写入硬件*/

    /* RX引脚 (PC7) */
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin   = BSP_BLUETOOTH_RX_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(BSP_BLUETOOTH_RX_PORT, &GPIO_InitStructure);

    /* USART6配置 */
    USART_InitTypeDef USART_InitStructure;
    RCC_APB2PeriphClockCmd(BSP_BLUETOOTH_RCC, ENABLE); /*开启USART6时钟挂载的APB2总线的时钟*/
    USART_DeInit(BSP_BLUETOOTH);/*把USART6的寄存器配置恢复到默认状态*/

    USART_StructInit(&USART_InitStructure);
    USART_InitStructure.USART_BaudRate            = bund; /* 波特率 */
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;/*数据位数*/
    USART_InitStructure.USART_StopBits            = USART_StopBits_1; /*1位停止位*/
    USART_InitStructure.USART_Parity              = USART_Parity_No; /*无效验*/
    USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;/*同时收发*/
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;/*无硬件流控*/
    USART_Init(BSP_BLUETOOTH, &USART_InitStructure); /*写入硬件*/

    /* 使能RXNE(字节接收)和IDLE(帧结束)中断 */
    USART_ITConfig(BSP_BLUETOOTH, USART_IT_RXNE, ENABLE); /*每收到一个字节,触发USART6中断*/
    USART_ITConfig(BSP_BLUETOOTH, USART_IT_IDLE, ENABLE); /*硬件自动检测(看高电平),总线空闲,触发USART6中断*/
    /*清理残留中断标志*/
    USART_ClearFlag(BSP_BLUETOOTH, USART_FLAG_RXNE);
    USART_ClearFlag(BSP_BLUETOOTH, USART_IT_IDLE);
    USART_Cmd(BSP_BLUETOOTH, ENABLE);
    /*把 USART6->CR1 的 UE（USART Enable）bit 置 1。在此之前 USART6 配置完毕但不工作，这行一调，正式开始收发。*/
    /* NVIC中断优先级配置 */
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel                   = BSP_BLUETOOTH_IRQ;/*中断通道号*/
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; /*抢占优先级为1,雷达数据最高,必须优先响应*/
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 1; /*子优先级为1,抢占优先级相同且同时到达子优先级小的先跑*/
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE; /*启用中断*/
    NVIC_Init(&NVIC_InitStructure);
}

/**
 * @brief  通过蓝牙发送单个字符
 */
void BLE_Send_Bit(unsigned char ch)
{
    USART_SendData(BSP_BLUETOOTH, (uint8_t)ch);
    while (RESET == USART_GetFlagStatus(BSP_BLUETOOTH, USART_FLAG_TXE));
}

/**
 * @brief  通过蓝牙发送字符串
 */
void BLE_send_String(unsigned char *str)
{   /*判断逻辑:指针本身非空并且不指向\0*/
    while (str && *str) {
        BLE_Send_Bit(*str++); /*先取*str当前字符的值再指针后移*/
    }
}

/**
 * @brief  清空蓝牙接收缓冲区和标志
 */
void Clear_BLERX_BUFF(void)
{
    BLERX_LEN  = 0; /*把接受指针拨回原点*/
    BLERX_FLAG = 0; /*清除一帧完成的标志,准备接受下一帧*/
}

/**
 * @brief  初始化HC-05 STATE引脚 (PC2) 为输入,这是连接状态脚,判断是否连上
 *         STATE = 高电平时已连接, 低电平时未连接
 */
void Bluetooth_Link_Gpio_Init(void)
{
    RCC_AHB1PeriphClockCmd(BLUETOOTH_LINK_RCC, ENABLE);/*开启GPIOC时钟*/

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin   = BLUETOOTH_LINK_GPIO;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN; /*普通输入*/
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; /*推挽*/
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP; /*默认上拉*/
    GPIO_Init(BLUETOOTH_LINK_PORT, &GPIO_InitStructure);

    GPIO_ResetBits(BLUETOOTH_LINK_PORT, BLUETOOTH_LINK_GPIO); /*先拉低*/
}

/**
 * @brief  蓝牙完整初始化: GPIO + USART6 + STATE引脚
 */
void Bluetooth_Init(void)
{
    Bluetooth_GPIO_Init(9600);
    Bluetooth_Link_Gpio_Init();
#if DEBUG
    printf("Bluetooth_Init succeed!\r\n");
#endif
}

/**
 * @brief  获取当前蓝牙连接状态
 * @return 1=已连接, 0=未连接
 */
unsigned char Get_Bluetooth_ConnectFlag(void)
{
    return Bluetooth_ConnectFlag;
}

/**
 * @brief  读取HC-05 STATE引脚，更新连接状态
 *
 * STATE引脚: 低电平=未连接, 高电平=已连接
 */
void Bluetooth_Mode(void)
{
    static char flag = 0; /*静态变量flag,用于记录连接状态的变化*/

    if (DISCONNECT == BLUETOOTH_LINK) {
        Bluetooth_ConnectFlag = 0;
        if (flag == 1) {
            flag = 0;
        }
        return;
    }

    if (CONNECT == BLUETOOTH_LINK) {
        Bluetooth_ConnectFlag = 1;
        if (flag == 0) {
            flag = 1;
        }
    }
}

/**
 * @brief  处理接收到的蓝牙数据（打印到调试串口）
 */
void Receive_Bluetooth_Data(void)
{
    if (BLERX_FLAG == 1) {
        printf("data = %s\r\n", BLERX_BUFF);
        Clear_BLERX_BUFF();
    }
}

/**
 * @brief  向已连接的手机发送数据
 * @param  dat: 待发送的字符串
 */
void Send_Bluetooth_Data(char *dat)
{
    Bluetooth_Mode();
    if (Bluetooth_ConnectFlag == 1) {
        BLE_send_String((unsigned char *)dat);
    }
}

/**
 * @brief  USART6中断服务函数(ISR) — 蓝牙数据接收
 *重写默认的USART6_IRQHandler()函数,实现蓝牙数据接收
 * 双中断策略:RXNE看每个字节,IDLE看一帧数据结束
 *   - RXNE: 逐字节存入BLERX_BUFF[]
 *   - IDLE: 检测线路空闲（帧结束）-> 置位BLERX_FLAG通知主循环
 */
void BSP_BLUETOOTH_IRQHandler(void)
{
    /* RXNE: 收到一个字节 */
    if (USART_GetITStatus(BSP_BLUETOOTH, USART_IT_RXNE) == SET) { /*检查RXNE硬件标志是否置位,RXNE中断是否使能*/
        if (BLERX_LEN < BLERX_LEN_MAX - 1) {
            BLERX_BUFF[BLERX_LEN++] = USART_ReceiveData(BSP_BLUETOOTH);/*留一字节给\0*/
        } else {
            USART_ReceiveData(BSP_BLUETOOTH);  /* 读DR清除中断标志，丢弃数据(缓冲区满了)*/
        }
        USART_ClearITPendingBit(BSP_BLUETOOTH, USART_IT_RXNE);
    }

    /* IDLE: 一帧数据接收完毕 */
    if (USART_GetITStatus(BSP_BLUETOOTH, USART_IT_IDLE) == SET) {
        volatile uint32_t temp; /*防止删除temp变量*/
        temp = BSP_BLUETOOTH->SR;  /* 读SR清除IDLE标志 */
        temp = BSP_BLUETOOTH->DR;  /* 读DR清除IDLE标志 */

        BLERX_BUFF[BLERX_LEN] = '\0';  /* 确保字符串正确结束 */
        BLERX_FLAG            = 1;     /* 通知主循环: 有新数据 */
    }
}
