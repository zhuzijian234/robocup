//                         _ooOoo_
//                        o8888888o
//                        88" . "88
//                        (| ^_^ |)
//                        O\  =  /O
//                     ____/`---'\____
//                   .'  \\|     |//  `.
//                  /  \\|||  :  |||//  \
//                 /  _||||| -:- |||||-  \
//                 |   | \\\  -  /// |   |
//                 | \_|  ''\---/''  |   |
//                 \  .-\__  `-`  ___/-. /
//               ___`. .'  /--.--\  `. . ___
//             ."" '<  `.___\_<|>_/___.'  >'"".
//           | | :  `- \`.;`\ _ /`;.`/ - ` : | |
//           \  \ `-.   \_ __\ /__ _/   .-` /  /
//     ========`-.____`-.___\_____/___.-`____.-'========
//                          `=---='
//     ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//        ���汣��       ��Ȧ����     �����޸�

#include "stm32f4xx.h"
#include "usart.h"
#include "delay.h"
#include "DMA.h"
#include "leida_pwm.h"
#include "LEIDA_DATA.h"
#include "BLUE.h"
#include "centre_line.h"
#include "timer.h"
#include "moto.h"
#include "Servo.h"
#include "PWM.h"

void Start_Scan(void)
{
    while ((USART6->SR & 0X40) == 0);
    USART6->DR = 0X54;
    while ((USART6->SR & 0X40) == 0);
    USART6->DR = 0XA2;

    while ((USART6->SR & 0X40) == 0);
    USART6->DR = 0X04;
    while ((USART6->SR & 0X40) == 0);
    USART6->DR = 0X40;

    while ((USART6->SR & 0X40) == 0);
    USART6->DR = 0X0B;
    while ((USART6->SR & 0X40) == 0);
    USART6->DR = 0X00;

    while ((USART6->SR & 0X40) == 0);
    USART6->DR = 0X00;
    while ((USART6->SR & 0X40) == 0);
    USART6->DR = 0Xc8;
}
/***************************��������************************
�����PE4,PE5M+-;
���PWM��TIME2
10.10�������˵���й���������
�޸����ٶȼ��㷽ʽ���ٶȶ���
Speed_PID������ʽ�޸������pwm�޷�
�ϳ���һ�µ������-->���ݴ�ӡ��һ�²Σ�112�У�-->����ģ����
***********************************************************/

uint16_t RIGHT_duandian;
uint16_t LEFT_duandian;
float paodao_distance       = 700;
float paodao_distance_r     = 700;
float paodao_distance_r_r   = 0;
float paodao_distance_r_r_r = 0;
uint16_t state_left_cnt     = 0;
uint16_t state_right_cnt    = 0;
uint16_t state_left_cnt_2   = 0;
uint16_t state_right_cnt_2  = 0;

extern float Speed_now;

#define duandian_distance 600
uint16_t state_sta = 1;

float duandian_DIStance = 600;

int main(void)
{

    u32 t                 = 0;
    uint16_t ceshi_cnt    = 0;
    uint16_t break_flag   = 0;
    uint16_t danbian_flag = 0;
    float jiaodu_piancha;
    float servo_pwm;
    float servo_midpwm = 156.5;
    float qulu_forward;
    float qulu_jinduan;
    uint16_t pid_select           = 0;
    uint16_t pid_select_last      = 0;
    uint16_t pid_select_last_last = 0;
    uint16_t tubian               = 0;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    delay_init(84);
    PWM_Init_leida();
    BLUE_init(115200); // ��������3��ʼ��
    PWM_SetCompare_leida(97);
    uart6_init(230400); // �״�ͨ��USART6��ʼ��
    DMA_Initializes();
    // Start_Scan();														//8Hz

    Servo_Init(84, 20000, servo_midpwm); // �����ʼ��//180  156  138				TIM3

    Midline_PD_Init(&Servo_pd, 0.0575, 0.14, 0.0597, 0.12, 0.02, 0.105); // ���PID��ʼ��   0.0575,0.14,0.0575,0.12,0.02,0.1  500��0.07,0.14,0.1,0.008   550��0.058,0.14,0.14,0.011    600:0.06,0.14,0.25,0.011
    Speed_PID_Init(&Speed_pid, 8.5, 0.505, 0);                           // �ٶȻ�PID��ʼ��  10.5, 0.425 ,0   0.6,0,5.5��PD����

    moto_pwm = (uint16_t)(90 * 1);
    Moto_Init(42, 100, moto_pwm);     // �����ʼ�� 20k 0.5ռ�ձ�		TIM2			370 740
    Encoder_Init();                   // ��������ʼ��			TIM4
    TIM5_Int_Init(100 - 1, 8400 - 1); // ��ϱ���������		TIM5		��ʱ��ʱ��84M����Ƶϵ��8400������84M/8400=10Khz�ļ���Ƶ�ʣ�����100��Ϊ10ms

    // ���22.5
    BLUE_BUFFER_STA = 0;
    Speed_mubiao    = 17;                                                  // 0.053,0.040,0.0368,0.089,0.022,0.068  ��������
    Midline_PD_Init(&Servo_pd, 0.035, 0.040, 0.0395, 0.075, 0.022, 0.020); // 0.031,0.040,0.0353,0.088,0.022,0.040
    BLUE_ANGLE_LEFT_RIGHT = 90;
    BLUE_DIS_RIGHT        = 27; // 27
    BLUE_DIS_LEFT         = 50; // 50
    BLUE_Y_RIGHT          = 1200;
    BLUE_Y_LEFT           = 1350;
    BLUE_Y_STRA_SEL       = 0;
    duandian_DIStance     = 550;
    printf("Start\r\n");
    while (1) {

        //  Speed_mubiao = ((fabs(Midline_forward.k) > 1.5) || (fabs(Midline.k) > 1.5) ) ? 18 : 15;
        // printf("speed_now:%f,speed_mubiao:%f\r\r\n",Speed_now,Speed_mubiao);

        if (DMA_RX_DONE) {

            DMA_RX_DONE = 0;
            //							if (Speed_now > 19)
            //							{
            //								BLUE_Y_RIGHT = 1350;
            //								BLUE_Y_LEFT = 1450;
            //
            //							}
            //							else
            //							{
            //								BLUE_Y_RIGHT = 1250;
            //								BLUE_Y_LEFT = 1350;
            //
            //							}
#if 1
            printf("1");
            delay_ms(300);
#endif
#if 0 // ��֤���ĺ������ݴ���
            if (LEIDA_DATA_HANDLE1(LEIDA_DATA, DMA_USART6_RX_BUF_r, DMA_USART6_RX_BUF_LEN)) // ������תΪ������
            {
                //// printf("\r\n\r\n\r\n�����ɹ�\r\n\r\n\r\n");
                // ѡ�����벻Ϊ��(����100)����Ч��
                valid_couter = LEIDA_DATA_HANDLE3_2(LEIDA_DATA2, LEIDA_DATA, LEIDA_DATA_COUNTER); // ȥ����Ч����
                // printf("valid:%d\r\n",valid_couter);
                if (valid_couter <= 20) {
                    break_flag = 1;
                    break;
                }
                //! valid_couter������Ч����
                /* HANDLE3_2内部已掐头去尾各10个点，此处不再重复减去 */

                //// LEIDA_DATA_HANDLE2(LEIDA_DATA_plane,LEIDA_DATA2,valid_couter);												//תΪƽ������
                //! ȷ����������mm,����600-900֮���򲻸���
                paodao_distance_r = LEIDA_Distance(LEIDA_DATA2, valid_couter); // ȷ����������
                paodao_distance   = ((paodao_distance_r > 600) && (paodao_distance_r < 900)) ? paodao_distance_r : paodao_distance;

                ////	printf("paodao_distance:%f\r\n",paodao_distance);
                // 90~180����С��4000�ĵ�
                LEFT_cnt = LEIDA_DATA_HANDLE6(LEIDA_DATA_LEFT, LEIDA_DATA2, valid_couter); // �ҵ����ұ߽�
                // 0~90����С��4000�ĵ�

                //	LEIDA_DATA_HANDLE2(LEIDA_DATA_LEFT_Plane,LEIDA_DATA_LEFT,LEFT_cnt);

                RIGHT_cnt = LEIDA_DATA_HANDLE7(LEIDA_DATA_RIGHT, LEIDA_DATA2, valid_couter);

                //	LEIDA_DATA_HANDLE2(LEIDA_DATA_RIGHT_Plane,LEIDA_DATA_RIGHT,RIGHT_cnt);

                //

                // ����ǰ��������70��-110�㣩
                Forward_cnt = LEIDA_DATA_HANDLE5(LEIDA_DATA_Forward, LEIDA_DATA2, valid_couter); // ��70��110��				//�ҵ�ǰ������
                // 70~90�����[200,2000]mm
                Forward_cnt_2 = LEIDA_DATA_HANDLE5_2(LEIDA_DATA_Forward_2, LEIDA_DATA2, valid_couter, 70, 90);
                // 90~110�����[200,2000]mm
                Forward_cnt_3 = LEIDA_DATA_HANDLE5_2(LEIDA_DATA_Forward_3, LEIDA_DATA2, valid_couter, 90, 110);
                // ��С�䣿��
                if (Forward_cnt) {
                    Midline_fit(LEIDA_DATA_Forward, (uint16_t)(Forward_cnt * 1.0f / 20 * 2), (uint16_t)(Forward_cnt * 1.0f / 20 * 18), &Midline_forward); // ǰ���������
                    danbian_flag = 0;
                    if ((Forward_cnt_2 > 5) && (Forward_cnt_3 > 5)) {
                        Midline_fit(LEIDA_DATA_Forward_2, 1, (uint16_t)(Forward_cnt_2 - 1), &Midline_forward_2);
                        Midline_fit(LEIDA_DATA_Forward_3, 1, (uint16_t)(Forward_cnt_3 - 1), &Midline_forward_3);
                        danbian_flag = 1; // ����
                    }

                    //										printf("%qianfnag:%f\r\n",Midline_forward.k);
                    //										printf("%qianfnag_��:%f\r\n",Midline_forward_2.k);
                    //										printf("%qianfnag_��:%f\r\n",Midline_forward_3.k);
                }
                // �Ҳ���Сy���루ͻ��㣩
                RIGHT_duandian = LEIDA_DATA_HANDLE9(LEIDA_DATA_RIGHT, RIGHT_cnt); // �Ҷϵ���
                // �����Сy����
                LEFT_duandian = LEIDA_DATA_HANDLE8(LEIDA_DATA_LEFT, LEFT_cnt); // ��ϵ���
                // ����С�ľ���
                if ((LEFT_duandian > 100) && (LEFT_duandian < duandian_DIStance) && (RIGHT_duandian > 100) && (RIGHT_duandian < duandian_DIStance)) { // ����һ���ϵ�
                    if (LEFT_duandian > RIGHT_duandian)
                        LEFT_duandian = 0;
                    else
                        RIGHT_duandian = 0;
                }

                // ��߽�Ҫ��ת
                //									if((LEFT_duandian>100)&&(LEFT_duandian<duandian_DIStance))
                //									{		//���ݶϵ��ж���һ�ߴ�ֱ������ж�ֱ��
                //											//0~80��ĵ��м�����x����<100mm����ǣ�1����0
                //										state_sta = LEIDA_DATA_HANDLE13(LEIDA_DATA_RIGHT,RIGHT_cnt,0,80);

                //									}
                //									//�ұ߽�Ҫ��ת
                //									else if((RIGHT_duandian>100)&&(RIGHT_duandian<duandian_DIStance)){
                //
                //											state_sta = LEIDA_DATA_HANDLE13(LEIDA_DATA_LEFT,LEFT_cnt,100,180);
                //
                //									}
                //									else
                //											state_sta = 1;
                //
                // printf("RIGHT_duandian:%d\r\n",RIGHT_duandian);
                //	printf("LEFT_duandian:%d\r\n",LEFT_duandian);

                pid_select_last_last = pid_select_last; // PIDѡ��
                pid_select_last      = pid_select;

                // if((RIGHT_duandian>

                if ((RIGHT_duandian > 0) && (RIGHT_duandian < duandian_DIStance)) {
                    // printf("%d �ұ߽�ͻ��\r\n",RIGHT_duandian);

                    if ((fabs(Midline_forward.k) < 0.35) && (Forward_cnt) && (RIGHT_duandian < duandian_distance)) {
                        // printf("ʹ��ǰ������\r\n");
                        if (Midline_forward.k < 0) Midline_forward.k = -Midline_forward.k;
                        printf("k: %f,b: %f\r\n", Midline_forward.k, Midline_forward.b);
                        pid_select = 3;
                        servo_pwm  = Midline_PD(LEIDA_DATA_Forward, &Servo_pd, &Midline_forward, servo_midpwm, (uint16_t)(Forward_cnt / 20.0 * 2), (uint16_t)(Forward_cnt / 20.0 * 18), pid_select);

                        state_left_cnt_2 = 0;

                    } else if ((fabs(Midline_forward.k) >= 0.35) && (fabs(Midline_forward.k) < 0.7) && (Forward_cnt) && (RIGHT_duandian < duandian_distance) && ((fabs(Midline_forward_2.k) < 0.25) || (fabs(Midline_forward_3.k) < 0.25)) && (danbian_flag == 1)) {
                        //	printf("ʹ��ǰ������,б�ʲ���\r\n");
                        if (Midline_forward.k < 0) Midline_forward.k = -Midline_forward.k;
                        printf("k: %f,b: %f\r\n", Midline_forward.k, Midline_forward.b);
                        pid_select = 8;
                        servo_pwm  = Midline_PD(LEIDA_DATA_Forward, &Servo_pd, &Midline_forward, servo_midpwm, (uint16_t)(Forward_cnt / 20.0 * 2), (uint16_t)(Forward_cnt / 20.0 * 18), pid_select);
                    } else {
                        if ((pid_select_last == 3) && (pid_select_last_last == 3)) state_left_cnt_2 = 1;
                        if (state_left_cnt_2 > 0) {
                            // Servo_ChangePwm(1360);
                            // printf("����");

                            pid_select = 1;
                            state_left_cnt_2 -= 1;
                        } else {
                            LEIDA_DATA_HANDLE2(LEIDA_DATA_LEFT_Plane, LEIDA_DATA_LEFT, LEFT_cnt); // ��߽�תΪƽ������
                            LEFT_cnt = LEIDA_DATA_HANDLE10(LEIDA_DATA_LEFT_Plane, LEFT_cnt);      // ��߽�ȥ��
                            Midline_fit(LEIDA_DATA_LEFT_Plane, (uint16_t)(LEFT_cnt / 20.0 * 15), (uint16_t)(LEFT_cnt / 20.0 * 19), &Midline);
                            // printf("Lk: %f,Lb: %f\r\n",Midline.k,Midline.b);
                            pid_select = 1; // ����
                            servo_pwm  = Midline_PD(LEIDA_DATA_LEFT_Plane, &Servo_pd, &Midline, servo_midpwm, (uint16_t)(LEFT_cnt / 20.0 * 15), (uint16_t)(LEFT_cnt / 20.0 * 19), pid_select);
                        }
                    }
                }
                // else if((LEFT_duandian>0)&&(LEFT_duandian<duandian_DIStance)&&(state_sta == 0)){
                else if ((LEFT_duandian > 0) && (LEFT_duandian < duandian_DIStance)) {
                    //	printf("%d ��߽�ͻ��\r\n",LEFT_duandian);

                    if ((fabs(Midline_forward.k) < 0.35) && (Forward_cnt) && (LEFT_duandian < duandian_distance)) {
                        //	printf("ʹ��ǰ������\r\n");
                        printf("k: %f,b: %f\r\n", Midline_forward.k, Midline_forward.b);
                        if (Midline_forward.k > 0) Midline_forward.k = -Midline_forward.k;
                        pid_select = 4;
                        servo_pwm  = Midline_PD(LEIDA_DATA_Forward, &Servo_pd, &Midline_forward, servo_midpwm, (uint16_t)(Forward_cnt / 20.0 * 2), (uint16_t)(Forward_cnt / 20.0 * 18), pid_select);

                        state_right_cnt_2 = 0;
                    } else if ((fabs(Midline_forward.k) >= 0.35) && (fabs(Midline_forward.k) < 0.7) && (Forward_cnt) && (LEFT_duandian < duandian_distance) && ((fabs(Midline_forward_2.k) < 0.25) || (fabs(Midline_forward_3.k) < 0.25)) && (danbian_flag == 1)) {
                        //	printf("ʹ��ǰ������,б�ʲ���\r\n");
                        printf("k: %f,b: %f\r\n", Midline_forward.k, Midline_forward.b);
                        if (Midline_forward.k > 0) Midline_forward.k = -Midline_forward.k;
                        pid_select = 9;
                        servo_pwm  = Midline_PD(LEIDA_DATA_Forward, &Servo_pd, &Midline_forward, servo_midpwm, (uint16_t)(Forward_cnt / 20.0 * 2), (uint16_t)(Forward_cnt / 20.0 * 18), pid_select);
                    }

                    else {
                        if ((pid_select_last == 4) && (pid_select_last_last == 4)) state_right_cnt_2 = 1;
                        if (state_right_cnt_2 > 0) {
                            // Servo_ChangePwm(1800);
                            //	printf("����");

                            pid_select = 2;
                            state_right_cnt_2 -= 1;
                        } else {
                            LEIDA_DATA_HANDLE2(LEIDA_DATA_RIGHT_Plane, LEIDA_DATA_RIGHT, RIGHT_cnt); // �ұ߽�תΪƽ������
                            RIGHT_cnt = LEIDA_DATA_HANDLE10(LEIDA_DATA_RIGHT_Plane, RIGHT_cnt);      // �ұ߽�ȥ��
                            Midline_fit(LEIDA_DATA_RIGHT_Plane, (uint16_t)RIGHT_cnt / 20.0 * 15, (uint16_t)RIGHT_cnt / 20.0 * 19, &Midline);
                            // printf("Rk: %f,Rb: %f\r\n",Midline.k,Midline.b);
                            pid_select = 2; // ����
                            servo_pwm  = Midline_PD(LEIDA_DATA_RIGHT_Plane, &Servo_pd, &Midline, servo_midpwm, (uint16_t)(RIGHT_cnt / 20.0 * 15), (uint16_t)(RIGHT_cnt / 20.0 * 19), pid_select);
                        }
                    }

                } else {
                    CENTER_cnt = LEIDA_DATA_HANDLE4(LEIDA_DATA_CENTER, LEIDA_DATA2, valid_couter); // �ҵ�����

                    if ((pid_select_last == 3) && (pid_select_last_last == 3)) state_left_cnt = 2;
                    if ((pid_select_last == 4) && (pid_select_last_last == 4)) state_right_cnt = 2;

                    pid_select = 0;

                    if (state_left_cnt > 0) {
                        // printf("%d �ұ߽�ͻ��:ʹ�ý�����������\r\n",RIGHT_duandian);
                        Servo_ChangePwm(1360);
                        //	printf("����");
                        //																	Midline_fit(LEIDA_DATA_CENTER,(uint16_t)(CENTER_cnt/20.0*2),(uint16_t)(CENTER_cnt/20.0*7),&Midline);																	//�о���������
                        //	printf("k: %f,b: %f\r\n",Midline.k,Midline.b);

                        //																	servo_pwm = Midline_PD(LEIDA_DATA_CENTER,&Servo_pd,&Midline,servo_midpwm,(uint16_t)(CENTER_cnt/20.0*2),(uint16_t)(CENTER_cnt/20.0*7),pid_select);
                        state_left_cnt--;
                    }

                    if (state_right_cnt > 0) {
                        //	printf("%d ��߽�ͻ��:ʹ�ý�����������\r\n",LEFT_duandian);
                        Servo_ChangePwm(1800);
                        // printf("����");
                        //																	Midline_fit(LEIDA_DATA_CENTER,(uint16_t)(CENTER_cnt/20.0*2),(uint16_t)(CENTER_cnt/20.0*7),&Midline);																	//�о���������
                        //	printf("k: %f,b: %f\r\n",Midline.k,Midline.b);

                        //																	servo_pwm = Midline_PD(LEIDA_DATA_CENTER,&Servo_pd,&Midline,servo_midpwm,(uint16_t)(CENTER_cnt/20.0*2),(uint16_t)(CENTER_cnt/20.0*7),pid_select);
                        state_right_cnt--;
                    }

                    if ((state_left_cnt == 0) && (state_right_cnt == 0)) {
                        zhongxian_chuizhi = LEIDA_DATA_HANDLE11(LEIDA_DATA_CENTER, (uint16_t)(CENTER_cnt / 20.0 * 8), (uint16_t)(CENTER_cnt / 20.0 * 18)); // �ж������Ƿ�ֱ

                        // printf("%d :ʹ��Զ����������\r\n");
                        if (zhongxian_chuizhi == 0) {
                            Midline_fit(LEIDA_DATA_CENTER, (uint16_t)(CENTER_cnt / 20.0 * 8), (uint16_t)(CENTER_cnt / 20.0 * 18), &Midline); // �о���������
                            // printf("Ck: %f,Cb: %f\r\n",Midline.k,Midline.b);
                            if (fabs(Midline.k) <= 0.1)
                                Servo_ChangePwm((uint16_t)servo_pwm);
                            // servo_pwm = Midline_PD(LEIDA_DATA_CENTER,&Servo_pd,&Midline,servo_midpwm,(uint16_t)(CENTER_cnt/20.0*8),(uint16_t)(CENTER_cnt/20.0*18),0);
                            else if (CENTER_cnt < 8) {
                                servo_pwm = Midline_PD(LEIDA_DATA_CENTER, &Servo_pd, &Midline, servo_midpwm, (uint16_t)(CENTER_cnt / 20.0 * 8), (uint16_t)(CENTER_cnt / 20.0 * 18), 7);
                                ceshi_cnt++;
                                //	printf("%d\r\n\r\n\r\n",ceshi_cnt);
                            } else
                                servo_pwm = Midline_PD(LEIDA_DATA_CENTER, &Servo_pd, &Midline, servo_midpwm, (uint16_t)(CENTER_cnt / 20.0 * 8), (uint16_t)(CENTER_cnt / 20.0 * 18), pid_select);

                        } else {
                            // printf("���ߴ�ֱ\r\n");
                            servo_pwm = Midline_PD(LEIDA_DATA_CENTER, &Servo_pd, &Midline, servo_midpwm, (uint16_t)(CENTER_cnt / 20.0 * 8), (uint16_t)(CENTER_cnt / 20.0 * 18), 5);
                        }
                    }
                }
                //
                //
            } else {
                // printf("\r\n\r\n\r\n����ʧ��\r\n\r\n\r\n");
                ;
            }
            //
            if (break_flag == 1) {
                break_flag = 0;
            }

#endif
        }
    }
}
