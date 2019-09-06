/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
【平    台】北京龙邱智能科技MK66FX1M0VLQ18开发板
【编    写】CHIUSIR
【备    注】
【软件版本】V1.0
【最后更新】2017年01月20日
【相关信息参考下列地址】
【网    站】http://www.lqist.cn
【淘宝店铺】http://shop36265907.taobao.com
【交流邮箱】chiusir@163.com
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
#include "include.h"

extern float Angle;
extern float K_Gyro_y;
extern short RoadPianCha;
extern float QingJiaoZhongZhiTarget;

extern float Angle_Balance,Gyro_Balance,Gyro_Turn;              //平衡倾角 平衡陀螺仪 转向陀螺仪
extern int Balance_Pwm,Velocity_Pwm,Turn_Pwm;
extern float pitch;
extern unsigned short send_data[9];
extern int Encoder_Left;                                   //左右编码器的脉冲计数
extern int Encoder_Right;                            
extern int element_event;
extern int location_event;


/*************路障备用方案****************/
short     luzhang_ff_1=0;                                     //撞路障标志位 
short     luzhang_ff_2=0;                                     //防重复执行标志位
#define   bizhang_speed_angle  4                              //避障备用方案倒退速度角
extern int chaoshengbo_distence[2];
extern short start_luzhang;                                   //路障标志位
extern float  bianmaqi_distence;


int Acceleration_Z;                                         //Z轴加速度计
int bianmaqi[2] ;                                           //存储编码器
int I_xianfu;
float angle_X;                                                //X轴角度
float    angle_X_ago ;                                       //上一次X轴角度
float   icm_gyro_y_ago;
float   pitch_g_ago;
float    kaerman_angle;
int     Target_Velocity=78;                                    //目标速度(编码器值)   //72
/********************************* //
速度环pid
*********************************/
float SPEED_CONTROL_P=(30.0f/100);                       //30
float SPEED_CONTROL_I=(0.1f/100);                      //0.007        //0.03
float SPEED_CONTROL_D=0.0f;
/*********************************
直立内外环pid
*********************************/
float EX_BALANCE_P  =28.0f;                     //25       30    28
float EX_BALANCE_I  =0.2;                       //0.1f;    0.2   0.2
float EX_BALANCE_D  =0.0f;                    //0.0f;      0     0
float IN_BALANCE_P  =10.0f;                   //13         13     11.5
float IN_BALANCE_I  =0.0;                    //1.2;       0       0
float IN_BALANCE_D  =0.0f;                  //5           0      8

extern float Speed_angle;
float  Speed_angle_old;
int Speed_Delta,Encoder_Integral;
float Encoder;
float pinghua_angle;
float ex_err_last = 0, ex_err = 0;
extern float ex_output;
float ex_integral;

float in_err [4];
float in_output = 0;
float in_integral;
extern float  AD_TURE[5];
float IN_D_temp;
float EX_D_temp;
extern float t;
extern int TEST;

void UART4_IRQHandler(void)
{
    LED_Ctrl(LED4,RVS);
    s8 data[20];
    data[0]=UART_Get_Char (UART_4);
    if(data[0]=='A')
        QingJiaoZhongZhiTarget+=0.1;
    if(data[0]=='B')
        QingJiaoZhongZhiTarget-=0.1;
    if(data[0]=='C')
        IN_BALANCE_P+=1;
    if(data[0]=='D')
        IN_BALANCE_P-=1;
    if(data[0]=='E')
        IN_BALANCE_D+=1;
    if(data[0]=='F')
        IN_BALANCE_D-=1;
    if(data[0]=='G')
        EX_BALANCE_P+=1;
    if(data[0]=='H')
        EX_BALANCE_P-=1;
    if(data[0]=='I')
        EX_BALANCE_D+=1;
    if(data[0]=='J')
        EX_BALANCE_D-=1;
    if(data[0]=='K')
        TEST*=-1;
    if(data[0]=='L')
        EX_BALANCE_P-=1;
    //sprintf(data,"你发送的数据为：%c\n",UART_Get_Char (UART_4));
    
    //UART_Put_Buff(UART_4,data,1);      //就发送出去 
}










void Get_Dip_Angle(void)
{
    float k=0.0;                                  //kalman滤波暂存
    extern float pitch_g;
    static  int  bianmaqi_ago[2];
    angle_X_ago=angle_X;
    icm_gyro_y_ago=icm_gyro_y;
    pitch_g_ago=pitch_g;

    get_icm20602_accdata(); //获取加速度数据
    get_icm20602_gyro();    //获取陀螺仪数据

    
    pitch=atan2((float)icm_acc_x,(float)icm_acc_z)*180/PI;
    
    pitch_g+=(K_Gyro_y)*0.24*0.001*4;
    angle_X-=(icm_gyro_x-11)*0.12*0.001*4;
    Gyro_Balance=K_Gyro_y;                                                      //更新平衡角速度  //陀螺仪量程幅度转换，也可以通过调整卡尔曼的参数实现
    
    k=Kalman_Filter(pitch,-1*((icm_gyro_y-4)/16.38));                        //卡尔曼滤波，用倾角
    kaerman_angle=k;
    
    bianmaqi_ago[0]=bianmaqi[0];
    bianmaqi_ago[1]=bianmaqi[1];
    
    bianmaqi[0]=FTM_AB_Get(FTM2);
    bianmaqi[1]=FTM_AB_Get(FTM1);
 
    
    Encoder_Left=-bianmaqi[0];                                //脉冲为前进+，后退为- ，每?ms采集一次    
    Encoder_Right=bianmaqi[1];                                 //脉冲为前进+，后退为-  
    
    send_data[0]=(int)(Angle_Balance*10.0);//ex_output;//Speed_angle;
    send_data[1]=(int)in_output;//EX_BALANCE_P * ex_err;//Encoder*SPEED_CONTROL_P;
    send_data[2]=(int)IN_BALANCE_P * in_err[0];//EX_BALANCE_I * ex_integral;//Encoder_Integral*SPEED_CONTROL_I;
    send_data[3]=(int)IN_BALANCE_D * (in_err[1]-in_err[0]);//EX_D_temp;
    send_data[4]=(int)(ex_output*10);//in_output;
    send_data[5]=(int)(Encoder_Integral);
    send_data[6]=(int)(Speed_angle*100);
    send_data[7]=(int)(K_Gyro_y*10);
    //send_data[8]=(int)(pitch*10);
 //(int)pitch;  
    //Data_Send(UART_4,send_data);                             //虚拟示波器显示（匿名上位机）
    Angle_Balance=k;                                    //更新平衡倾角
    Gyro_Turn=icm_gyro_x;                                       //更新转向角速度    
    
    //Acceleration_Z=icm_acc_z;                                //更新Z轴加速度计		
}




/**************************************************************************
函数功能：速度PI控制 修改前进后退速度，请修Target_Velocity，比如，改成20就比较慢了
入口参数：左轮编码器、右轮编码器
返回  值：速度控制PWM
**************************************************************************/
void Velocity_Ctrl_Angle(int Encoder_Left,int Encoder_Right)
{  
   static int flag[2];
   static short cm_10_flag=0;
   
    Speed_Delta =(Encoder_Left+Encoder_Right)/2-Target_Velocity;

    Encoder *= 0.8;		                                      //一阶低通滤波器       
    Encoder +=Speed_Delta*0.2;	                              //一阶低通滤波器    
    
  //  if(Speed_Delta<-15||Speed_Delta>15)
       Encoder_Integral +=Speed_Delta;                                //距离积分             
    
    if(Encoder_Integral>1999)   Encoder_Integral=1999;              //积分限幅
    if(Encoder_Integral<-1999)   Encoder_Integral=-1999;             //积分限幅
    
    flag[1]=flag[0];
    flag[0]=(Encoder_Left+Encoder_Right)/2-Target_Velocity;
    
   // if(flag[0]*flag[1]<0)
   //    Encoder_Integral*=0.5;

    Speed_angle_old=Speed_angle;
    Speed_angle=(Encoder*SPEED_CONTROL_P+Encoder_Integral*SPEED_CONTROL_I+SPEED_CONTROL_D*(Speed_angle-Speed_angle_old));     //速度控制,输出角度，单位度	
    if(Speed_angle>6)     Speed_angle=6;
    if(Speed_angle<-6)     Speed_angle=-6;
    
/*************避障备用方案*********************/    
  /*  if(luzhang_ff_2==0&&luzhang_ff_1==0&&start_luzhang==0&&(chaoshengbo_distence[0]<20&&chaoshengbo_distence[0]>10))
    {
      luzhang_ff_1=1;
    }
    if(luzhang_ff_1==1)
    {
      start_luzhang=0;                                                   //路障标志位置0
      Speed_angle=bizhang_speed_angle;
      luzhang_ff_2=1;
      if(chaoshengbo_distence[0]>80)
      {
          bianmaqi_distence=0;                                          //积分清0
          luzhang_ff_1=0;                                                   //撞路障标志位清0
          cm_10_flag=1;

      }
    }
    if(cm_10_flag==1)
    {
      start_luzhang=0;
      bianmaqi_distence+=(-(bianmaqi[0]*0.00452)+(bianmaqi[1]*0.00452))*0.5;
      if(bianmaqi_distence>10)
      {
        bianmaqi_distence=0;
        cm_10_flag=0;
        start_luzhang=1;w
      }
    }*/
    
}

//串级控制////※※※方案2
void EX_Control(float set_angle)
{ 
    static float flag[2];
    
    ex_err_last = ex_err;
    ex_err = Angle_Balance -(set_angle);//偏差////※※※
    
   // if(ex_err<-2||ex_err>2)
       ex_integral += ex_err;             //积分
    
    if(ex_integral>600)   ex_integral=600;              //积分限幅
    if(ex_integral<-600)   ex_integral=-600;             //积分限幅
     flag[1]=flag[0];
     flag[0]=Angle_Balance -(set_angle);
    
  //  if(flag[0]*flag[1]<0)
 //      ex_integral*=0;
    
    ex_output = (EX_BALANCE_P * ex_err + 
                 EX_BALANCE_I * ex_integral + 
                     EX_BALANCE_D * (ex_err - ex_err_last));             //外环输出角速度，单位度每秒
    EX_D_temp=EX_BALANCE_D * (ex_err - ex_err_last);
    
    //    static float in_err_last = 0, in_err = 0;
    //    static float in_output = 0;
    //    static float in_integral;
    
}
void IN_Control(float ex_output)
{
  //  IN_BALANCE_P  =320/EX_BALANCE_P;
    //内环(角速度)
    in_err[1] = in_err[0];
    in_err[0] = (ex_output+K_Gyro_y);
    in_integral += in_err[0];
    if(in_integral>700)   in_integral=700;              //积分限幅
    if(in_integral<-700)   in_integral=-700;             //积分限幅
    
    in_output = IN_BALANCE_P * in_err[0] +  IN_BALANCE_I * in_integral +   IN_BALANCE_D * (in_err[0]-in_err[1]);              //内环输出pwm
    
    IN_D_temp=IN_BALANCE_D * ( in_err[0]-in_err[1] );
    Balance_Pwm = in_output;


}



void speed_output()
{
  float value;
  static int speed_count=0;
  if(speed_count>=10)
    speed_count=0;
  value=Speed_angle-Speed_angle_old;
  pinghua_angle=value*(speed_count+1)/10+Speed_angle_old;
  speed_count++;

}
/**************************************************************************
函数功能：求绝对值函数
入口参数：int
返回  值：unsigned int
**************************************************************************/
int My_Abs(int v)
{ 		   
    if(v<0)  v=-v;  	  
    return  v;
}

void LED_zhishi()
{
    if((Encoder_Left+Encoder_Right)/2>0.9*Target_Velocity&&(Encoder_Left+Encoder_Right)/2<1.1*Target_Velocity)
    {
        //GPIO_Ctrl (PTC, 16,1);
        LED_Ctrl(LED5,ON);
    }
    else
    {
        //GPIO_Ctrl (PTC, 16,0);
        LED_Ctrl(LED5,OFF);
    }
    if((Encoder_Left+Encoder_Right)/2<0.9*Target_Velocity)
        LED_Ctrl(LED6,ON);
    else
        LED_Ctrl(LED6,OFF);
    if((Encoder_Left+Encoder_Right)/2>1.1*Target_Velocity)
        LED_Ctrl(LED4,ON);
    else
        LED_Ctrl(LED4,OFF);
    
}











