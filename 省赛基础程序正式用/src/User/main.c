/****************************************************************************************************
【平    台】龙邱K66FX智能车VD母板
【编    写】CHIUSIR
【E-mail  】chiusir@163.com
【软件版本】V1.0
【最后更新】2018年4月28日
【相关信息参考下列地址】
【网    站】http://www.lqist.cn
【淘宝店铺】http://shop36265907.taobao.com
------------------------------------------------
【dev.env.】IAR8.2及以上
【Target  】K66FX1M0VLQ18
【Crystal 】 50.000Mhz
【busclock】110.000MHz
【pllclock】220.000MHz
******************************************************************************************************/
#include "include.h" 
#include "myiic.h"


extern int16  AD[5];
extern int16  AD_max[5];
extern int16  AD_min[5];
extern float  AD_TURE[5];
extern int bianmaqi[2]; 
extern int    angle;            //舵机角度
extern int Target_Velocity;                                    //目标速度(编码器值)
extern int speed_left;
extern int speed_right;
extern int yuanhuan;
extern int steer_out;
extern int steer_out;
extern float Bias1;
extern float Encoder;
extern int Encoder_Integral;
extern int    element_event;                                   //赛道元素记录    
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
extern short  luzhang_ff_1;                                 //撞路障标志位 
extern int   chaoshengbo_distence[2];                       
extern short duanlu_send_flag;
extern short qipao_send_flag;
extern short shifou_luzhang;
extern short duanl_stop_flag;
extern short duanl_stop_once;
extern short zhili_frist;
extern int    summmmm; 
extern float angle_X;                                     //X轴角度
extern float angle_X_ago ;                             //上一次X轴角度
float    angle_change;                                      //角度变化量的积分
short huiche_flag=0;                                    //会车方案选择


short   jijij=0;
short  distance_flag=0;                                        //超声波数据读取
int     data[2];                                                 //超声波数据
u8     shepin_data;                                            //射频接受数据
int   chaoshengbo_count=0;
int   Car_Count=0;
float nummmmm;
short Speed_Smooth_Count=0;
short Stand_Smooth_Count=0;

float   duanli_stop_distance=0;
float Angle_Balance,Gyro_Balance,Gyro_Turn=0;                  //平衡倾角 平衡角速度 小车转向角速度
int Balance_Pwm,Turn_Pwm=0;
float Angle;
int MotorLeft,MotorRight=0;                              //左右电机的pwm值
int Encoder_Left,Encoder_Right;                         //左右编码器的脉冲计数
uint16_t VL53L0X_distence;                               //激光测距
short zhili_flag=0;
short RoadPianCha=0;
short duanlu_stand=0;
int tim=1;                                               //pit定时中断周期
int   jjjjj=0;

double dis=0;

float pitch=0.0;
float pitch_g=-36.0f;                                       //陀螺仪积分角度,初始值

float QingJiaoZhongZhiTarget=-36.0f;                        //机械零点角度
float Q_angle = 0.001;                                      //预测（过程）噪声方差,角度置信度0.001f
float Q_gyro  = 0.005f;                                     //预测（过程）噪声方差,角速度置信度0.005f
float R_angle = 0.3f;                                       //测量（观测）噪声方差0.5

unsigned short send_data[9];                               //上位机数据
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
初始化各个通道PWM
********************/
void Motor_Init(void)
{
    
    FTM_PWM_Init(FTM3,FTM_CH0,5000,0);//Mot0-PTC1
    FTM_PWM_Init(FTM0,FTM_CH2,5000,0);//Mot1-PTC2 
    FTM_PWM_Init(FTM0,FTM_CH3,5000,0);//Mot0-PTC3
    FTM_PWM_Init(FTM0,FTM_CH4,5000,0);//Mot1-PTC4 
}





/*************************
各类GPIO初始化
************************/
void  bomakaiguan()
{
    GPIO_Init(PTA,8,GPI,1);               //1
    GPIO_Init(PTA,14,GPI,1);              //2
    GPIO_Init(PTA,15,GPI,1);              //3
    GPIO_Init(PTA,17,GPI,1);             //4
    GPIO_Init(PTA,19,GPI,1);             //6
    GPIO_Init(PTA,9,GPO,0);             //蜂鸣器
    GPIO_Init(PTE,27,GPI,1);             //干簧管起跑线判别
    
}





/******编码器初始化*********/
void bianmaqi_init()
{
    FTM_AB_Init(FTM2);                                                       //右编码器初始化  PB18 PB19
    FTM_AB_Init(FTM1);                                                      //左编码器初始化  PA12 PA13
}


/**********************************
  超声波串口中断
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
    nrf2401 断路，重点线数据发送与接受
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
          shepin_data=rxcheck();                                        //射频数据读取
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
拨码开关功能函数
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
       
    if(sum_bomakaiguan==1)                                      //电感值校准  拨码开关1
    {
      adc_refer_get() ;                                        
    } 
    else if(sum_bomakaiguan==10)                               //写入扇区     拨码开关2
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
    else if(sum_bomakaiguan==100)                             //会车：交错     拨码开关3
    { 
      huiche_flag=0;
    }
    else if(sum_bomakaiguan==1000)                            //会车：追逐     拨码开关4
    {
      huiche_flag=1;
    }    
    else if(sum_bomakaiguan==11000)                           //低速模式     拨码开关64       关闭时先关4再关6                                               
    {
      Target_Velocity=72;
    }
    else if(sum_bomakaiguan==10100)                            //直立后到模式     拨码开关63      关闭时先关3再关6
    {
     zhili_frist=0 ;
    }
    else if(sum_bomakaiguan==10001)                            //开启避障    拨码开关61        关闭时先关1再关6
    {
      shifou_luzhang=1;
    }
    else if(sum_bomakaiguan==10010)                            //关闭避障    拨码开关62        关闭时先关2再关6
    {
      shifou_luzhang=0;
    }
}



/******************************
定时器中断
*****************************/
void PIT0_Interrupt()
{

    static short  stop_count=0;
    Car_Count+=1;
    if(Car_Count%4==0)
    {
      //直立环        
        Get_Dip_Angle();                                         //角度获取用4ms周期，
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
        MotorLeft=Balance_Pwm +speed_left;               //===计算左轮电机最终PWM   
        MotorRight=Balance_Pwm +speed_right;              //===计算右轮电机最终PWM  
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
                     angle_change+=(angle_X-angle_X_ago);                              //积分得到角度变化
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
     //  VL53L0X_distence_get() ;                                      //距离读取
        
    }
    if(Car_Count%40==0)
    {
      //速度环
        Velocity_Ctrl_Angle(Encoder_Left,Encoder_Right);              //速度PI控制
        Car_Count = 0; 
    }
   if(distance_flag==0)
     {     chaoshengbo_distence[1]=chaoshengbo_distence[0];
           chaoshengbo_distence[0] =(256*data[0]+data[1])*0.1;
           UART_Put_Char(UART_3,0x55);
      }
    PIT_Flag_Clear(PIT0);                                             //清中断标志位 

} 





void main(void)
{    
    
    PLL_Init(PLL220);             //初始化PLL为?M,总 线为？M
    DisableInterrupts;            //关闭中断
    LED_Init();
    //  LCD_Init();
    // LCD_CLS();
    //TFTSPI_Init(); 
    //TFTSPI_CLS(u16BLACK);          //清屏幕
    PIT_Init(PIT0, tim);          //中断定时 tim ms
    Motor_Init();                //电机初始化 
    NRF24l01_Init();              //射频初始化
    adc_ready();                  //电感采集初始化
    bomakaiguan();                //拨码开关初始化
    UART_Init(UART_4,115200);      //蓝牙串口初始化
    UART_Init(UART_3,9600);        //超声波串口初始化
    UART_Irq_En(UART_4);           //蓝牙串口中断
    UART_Irq_En(UART_3);           //超声波串口中断初始化
  //  VL53L0X_init();                 //VL53L0X初始化
    bianmaqi_init();               //编码器初始化
    icm20602_init();                //陀螺仪加速度计初始化
    //   LQMT9V034_Init();              //摄像头初始化
    GPIO_Ctrl(PTA,9,1);
    time_delay_ms(100);
    GPIO_Ctrl(PTA,9,0);
    EnableInterrupts;             //开启中断

    
    while(1)  
    { 
        
       nrf2401_tongxin();                                              //2401数据发送接受
       bomakaiguan_function();                                         //拨码开关功能
    //   Data_Send(UART_4,send_data);//虚拟示波器显示（匿名上位机）
      jijij= GPIO_Get(PTE27);

    }
}






























