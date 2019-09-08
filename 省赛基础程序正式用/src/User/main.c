/****************************************************************************************************
��ƽ    ̨������K66FX���ܳ�VDĸ��
����    д��CHIUSIR
��E-mail  ��chiusir@163.com
������汾��V1.0
�������¡�2018��4��28��
�������Ϣ�ο����е�ַ��
����    վ��http://www.lqist.cn
���Ա����̡�http://shop36265907.taobao.com
------------------------------------------------
��dev.env.��IAR8.2������
��Target  ��K66FX1M0VLQ18
��Crystal �� 50.000Mhz
��busclock��110.000MHz
��pllclock��220.000MHz
******************************************************************************************************/
#include "include.h" 
#include "myiic.h"


extern int16  AD[5];
extern int16  AD_max[5];
extern int16  AD_min[5];
extern float  AD_TURE[5];
extern int bianmaqi[2]; 
extern int    angle;            //����Ƕ�
extern int Target_Velocity;                                    //Ŀ���ٶ�(������ֵ)
extern int speed_left;
extern int speed_right;
extern int yuanhuan;
extern int steer_out;
extern int steer_out;
extern float Bias1;
extern float Encoder;
extern int Encoder_Integral;
extern int    element_event;                                   //����Ԫ�ؼ�¼    
extern float  ad_error; 
extern float  ad_error_ago; 
extern int yuanhuanfangxiang;
extern int yuanhuan_times;
extern int ru_yuanhuan;
extern int time_num;
extern float angle_X;
extern float  bianmaqi_distence;
extern int     yuan_distence;
extern float pinghua_angle;
extern short start_luzhang;
extern float Gyro_Y_ago;
extern float  Gyro_Y;
extern float kaerman_angle;
extern int  chongchusaidao;
extern short duanlu;                                             
extern short  luzhang_ff_1;                                 //ײ·�ϱ�־λ 
extern int   chaoshengbo_distence[2];                       
extern short duanlu_send_flag;
extern short qipao_send_flag;
extern short shifou_luzhang;
extern short duanl_stop_flag;
extern short duanl_stop_once;
extern short zhili_frist;
extern int    summmmm; 
extern float angle_X;                                     //X��Ƕ�
extern float angle_X_ago ;                             //��һ��X��Ƕ�
float    angle_change;                                      //�Ƕȱ仯���Ļ���
short huiche_flag=0;                                    //�ᳵ����ѡ��


short   jijij=0;
short  distance_flag=0;                                        //���������ݶ�ȡ
int     data[2];                                                 //����������
u8     shepin_data;                                            //��Ƶ��������
int   chaoshengbo_count=0;
int   Car_Count=0;
float nummmmm;
short Speed_Smooth_Count=0;
short Stand_Smooth_Count=0;

float   duanli_stop_distance=0;
float Angle_Balance,Gyro_Balance,Gyro_Turn=0;                  //ƽ����� ƽ����ٶ� С��ת����ٶ�
int Balance_Pwm,Turn_Pwm=0;
float Angle;
int MotorLeft,MotorRight=0;                              //���ҵ����pwmֵ
int Encoder_Left,Encoder_Right;                         //���ұ��������������
uint16_t VL53L0X_distence;                               //������
short zhili_flag=0;
short RoadPianCha=0;
short duanlu_stand=0;
int tim=1;                                               //pit��ʱ�ж�����
int   jjjjj=0;

double dis=0;

float pitch=0.0;
float pitch_g=-36.0f;                                       //�����ǻ��ֽǶ�,��ʼֵ

float QingJiaoZhongZhiTarget=-36.0f;                        //��е���Ƕ�
float Q_angle = 0.001;                                      //Ԥ�⣨���̣���������,�Ƕ����Ŷ�0.001f
float Q_gyro  = 0.005f;                                     //Ԥ�⣨���̣���������,���ٶ����Ŷ�0.005f
float R_angle = 0.3f;                                       //�������۲⣩��������0.5

unsigned short send_data[9];                               //��λ������
float Speed_angle;
float ex_output;
extern int Target_Velocity;
uint8_t text1[]={"BP"};
uint8_t text2[]={"VP"};
uint8_t text3[]={"Mo"};
uint8_t text4[]={"EI"};
uint8_t text5[]={"ET"};
uint8_t text6[]={"Ag"};
float t=1.0f;
int TEST=1;





/*******************
��ʼ������ͨ��PWM
********************/
void Motor_Init(void)
{
    
    FTM_PWM_Init(FTM3,FTM_CH0,5000,0);//Mot0-PTC1
    FTM_PWM_Init(FTM0,FTM_CH2,5000,0);//Mot1-PTC2 
    FTM_PWM_Init(FTM0,FTM_CH3,5000,0);//Mot0-PTC3
    FTM_PWM_Init(FTM0,FTM_CH4,5000,0);//Mot1-PTC4 
}





/*************************
����GPIO��ʼ��
************************/
void  bomakaiguan()
{
    GPIO_Init(PTA,8,GPI,1);               //1
    GPIO_Init(PTA,14,GPI,1);              //2
    GPIO_Init(PTA,15,GPI,1);              //3
    GPIO_Init(PTA,17,GPI,1);             //4
    GPIO_Init(PTA,19,GPI,1);             //6
    GPIO_Init(PTA,9,GPO,0);             //������
    GPIO_Init(PTE,27,GPI,1);             //�ɻɹ��������б�
    
}





/******��������ʼ��*********/
void bianmaqi_init()
{
    FTM_AB_Init(FTM2);                                                       //�ұ�������ʼ��  PB18 PB19
    FTM_AB_Init(FTM1);                                                      //���������ʼ��  PA12 PA13
}


/**********************************
  �����������ж�
*******************************/
void UART3_IRQHandler(void)                                      
{ 
  
    static  short di=0;
    distance_flag=1;
    data[di]=UART_Get_Char(UART_3);
    di++;
    if(di==2)
    {
        distance_flag=0;
        di=0;
    }
}


/*********************************************
    nrf2401 ��·���ص������ݷ��������
*********************************************/  
void nrf2401_tongxin()
{
     static short jjj=1;
       if(duanlu_send_flag)
        {
          sendState('A',1);
          duanlu_send_flag=0;
        }
          
       else if(qipao_send_flag)
       {
         sendState('B',1);
         
       }
       else
       {
          shepin_data=rxcheck();                                        //��Ƶ���ݶ�ȡ
       }
       if(shepin_data=='A')
       {
         duanl_stop_flag=0;
         duanl_stop_once=1;
         duanli_stop_distance=0;
         duanlu_stand=0;
       }
       if(duanlu==1&&jjj==1)
       {
         sendState('C',1);
         jjj=0;
       }
}

/***************************
���뿪�ع��ܺ���
***************************/
void  bomakaiguan_function()
{
    short  num1,num2,num3,num4,num5;
    short  sum_bomakaiguan=0;    
       if(GPIO_Get(PTA19)==1)                                     //5
         num5=10000;
       else
         num5=0;
       if(GPIO_Get(PTA17)==1)                                    //4 
         num4=1000;   
       else
         num4=0;
       if(GPIO_Get(PTA15)==1)                                     //3             
         num3=100;
       else 
         num3=0;
       if(GPIO_Get(PTA14)==1)                                     //2
         num2=10;
       else
         num2=0;
       if (GPIO_Get(PTA8)==1)                                    //1
         num1=1;
       else
         num1=0;
       
    sum_bomakaiguan=num1+num2+num3+num4+num5;
       
    if(sum_bomakaiguan==1)                                      //���ֵУ׼  ���뿪��1
    {
      adc_refer_get() ;                                        
    } 
    else if(sum_bomakaiguan==10)                               //д������     ���뿪��2
    {
      static  short flag_wirte=1; 
      if(flag_wirte)
      {
    //    FLASH_EraseSector(254);
    //    FLASH_WriteBuf(254,(uint8_t *)AD_max, sizeof(AD_max), 0);
        flag_wirte=0;
    //    FLASH_ReadBuff(254, 0, sizeof(AD_max), (char *)AD_max);
      }
    }
    else if(sum_bomakaiguan==100)                             //�ᳵ������     ���뿪��3
    { 
      huiche_flag=0;
    }
    else if(sum_bomakaiguan==1000)                            //�ᳵ��׷��     ���뿪��4
    {
      huiche_flag=1;
    }    
    else if(sum_bomakaiguan==11000)                           //����ģʽ     ���뿪��64       �ر�ʱ�ȹ�4�ٹ�6                                               
    {
      Target_Velocity=72;
    }
    else if(sum_bomakaiguan==10100)                            //ֱ����ģʽ     ���뿪��63      �ر�ʱ�ȹ�3�ٹ�6
    {
     zhili_frist=0 ;
    }
    else if(sum_bomakaiguan==10001)                            //��������    ���뿪��61        �ر�ʱ�ȹ�1�ٹ�6
    {
      shifou_luzhang=1;
    }
    else if(sum_bomakaiguan==10010)                            //�رձ���    ���뿪��62        �ر�ʱ�ȹ�2�ٹ�6
    {
      shifou_luzhang=0;
    }
}



/******************************
��ʱ���ж�
*****************************/
void PIT0_Interrupt()
{

    static short  stop_count=0;
    Car_Count+=1;
    if(Car_Count%4==0)
    {
      //ֱ����        
        Get_Dip_Angle();                                         //�ǶȻ�ȡ��4ms���ڣ�
        IN_Control(ex_output);
        dianji_go();
        
        if(zhili_flag==0)
        {
            speed_output();
            if(duanlu_stand==1)
            {
              pinghua_angle=0;
            }
        }
        else
        {
          pinghua_angle=0;
        }
        
        EX_Control(QingJiaoZhongZhiTarget+pinghua_angle);
        if(luzhang_ff_1==1)
        {
          speed_left=0;
          speed_right=0;
        }
        MotorLeft=Balance_Pwm +speed_left;               //===�������ֵ������PWM   
        MotorRight=Balance_Pwm +speed_right;              //===�������ֵ������PWM  
     //       if(start_luzhang==0&&duanlu==0&&luzhang_ff_1==0&&(ad_data_ading(1,3)<40|| Angle_Balance<-60||Angle_Balance>0) )                      
     //           Set_Pwm(0,0);
            if(start_luzhang==1&&duanlu==0&&(Angle_Balance<-60||Angle_Balance>0))
                Set_Pwm(0,0);
            else if(duanl_stop_flag==1)
            {
                stop_count++;
                duanli_stop_distance+=(-(bianmaqi[0]*0.00452)+(bianmaqi[1]*0.00452))*0.5;
                if(stop_count>800)
                {
                  stop_count=0;
                  duanl_stop_flag=0;
                  duanlu_stand=1;
                  angle_change=0;
                }
                else 
                {
                 if(huiche_flag)
                 {
                   if(Angle_Balance>-50)
                   {
                     Set_Pwm(1200,1200);
                   }
                   else
                   {
                    if(stop_count<200)
                      Set_Pwm(0,0);
                    else
                    {
                     angle_change+=(angle_X-angle_X_ago);                              //���ֵõ��Ƕȱ仯
                    if(angle_change<160)
                      Set_Pwm(1500,-1500);
                    else
                       Set_Pwm(0,0);
                    }
                   }
                }
                else
                {
                   if(Angle_Balance>-55)
                   {
                     Set_Pwm(1200,1200);
                   }
                   else
                   {
                    if(stop_count<200)
                      Set_Pwm(0,0);
                   }
                }
               }
            }
            else if(qipao_send_flag==1)
            {
               Set_Pwm(0,0);
               GPIO_Ctrl(PTA,9,1);
               
            }
            else
            {   
              if((Encoder_Left+Encoder_Right)<10&&Encoder_Left>0&&Encoder_Right>0)
                Set_Pwm(MotorLeft,MotorRight);
              else
                Set_Pwm(MotorLeft,MotorRight);
            }

       LED_zhishi();
     //  VL53L0X_distence_get() ;                                      //�����ȡ
        
    }
    if(Car_Count%40==0)
    {
      //�ٶȻ�
        Velocity_Ctrl_Angle(Encoder_Left,Encoder_Right);              //�ٶ�PI����
        Car_Count = 0; 
    }
   if(distance_flag==0)
     {     chaoshengbo_distence[1]=chaoshengbo_distence[0];
           chaoshengbo_distence[0] =(256*data[0]+data[1])*0.1;
           UART_Put_Char(UART_3,0x55);
      }
    PIT_Flag_Clear(PIT0);                                             //���жϱ�־λ 

} 





void main(void)
{    
    
    PLL_Init(PLL220);             //��ʼ��PLLΪ?M,�� ��Ϊ��M
    DisableInterrupts;            //�ر��ж�
    LED_Init();
    //  LCD_Init();
    // LCD_CLS();
    //TFTSPI_Init(); 
    //TFTSPI_CLS(u16BLACK);          //����Ļ
    PIT_Init(PIT0, tim);          //�ж϶�ʱ tim ms
    Motor_Init();                //�����ʼ�� 
    NRF24l01_Init();              //��Ƶ��ʼ��
    adc_ready();                  //��вɼ���ʼ��
    bomakaiguan();                //���뿪�س�ʼ��
    UART_Init(UART_4,115200);      //�������ڳ�ʼ��
    UART_Init(UART_3,9600);        //���������ڳ�ʼ��
    UART_Irq_En(UART_4);           //���������ж�
    UART_Irq_En(UART_3);           //�����������жϳ�ʼ��
  //  VL53L0X_init();                 //VL53L0X��ʼ��
    bianmaqi_init();               //��������ʼ��
    icm20602_init();                //�����Ǽ��ٶȼƳ�ʼ��
    //   LQMT9V034_Init();              //����ͷ��ʼ��
    GPIO_Ctrl(PTA,9,1);
    time_delay_ms(100);
    GPIO_Ctrl(PTA,9,0);
    EnableInterrupts;             //�����ж�

    
    while(1)  
    { 
        
       nrf2401_tongxin();                                              //2401���ݷ��ͽ���
       bomakaiguan_function();                                         //���뿪�ع���
    //   Data_Send(UART_4,send_data);//����ʾ������ʾ��������λ����
      jijij= GPIO_Get(PTE27);

    }
}






























