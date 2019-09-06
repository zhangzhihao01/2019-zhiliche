#include "include.h"


extern u8 Pixle[LCDH][LCDW];                             //二值化后用于OLED显示的数据

/*******路障参数********/
extern uint16_t count[3];
extern int bianmaqi[2];                                 //编码器数值 一个脉冲0.00452cm
extern float angle_X;                                     //X轴角度
extern float angle_X_ago ;                             //上一次X轴角度
extern float kaerman_angle;
extern float QingJiaoZhongZhiTarget;
extern int chaoshengbo_count;
extern int Target_Velocity;
extern short luzhang_ff_2;                                //避障备用方案，防重复判断标志位
extern short luzhang_ff_1;                                //撞路障标志位
float  bianmaqi_distence=0;                               //编码器积分出距离
int    chaoshengbo_distence[2];                             //超声波测距，单位cm 
short start_luzhang=0;                                   //路障标志位
short  judge_luzhang=0;                                  //maybe 路障
short  shifou_luzhang=0;                                 //是否避障
#define  steer_out_turn      210                           //路障开环
#define  steer_out_straight  -200                           //避开路障后走一段直道
#define  steer_out_back      -180                            //回到赛道
#define  distence_turn       20                           //拐出路障距离                                    
#define  distence_straight    20                            //避开路障后走一段直道的距离
#define  distence_back        13                          //回赛道的距离




/*****通用参数*******/
int16  AD_valu[5][3];                                     //这几个为全局变量，用的时候放在主函数
int16  AD_sum[5];
int16  AD[5];                                           //归一化之前的值
int16  AD_max[5]={1024,1024,1024,1024,1024};            //5个电感最大值记录               ***1
int16  AD_min[5]={0,0,0,0,0};                          //5个电感最小值记录
int16  refer[3] ={390,390,390};                                       //固定电磁参考值 
float  refer_TURE[3]={38,38,38};                                 //归一化的电磁参考值               ***2         
float  AD_TURE[5] ;                                //真正归一化的值
int     max;                                            //最大电感标记
int    event_ago;                                       
int    location_event;                                  //电磁线位置记录
int    element_event;                                   //赛道元素记录
short  P_D_bili=26;
float  ad_error;                                        //电磁偏移量
float  ad_error_ago; 
int16   ad_valu[8];
int    summmmm;                                        //  横电感总和
int    sum_all;                                        //  所有电感总和


#define     ad_error_max    100                                //舵机误差最大值                 ***3
#define     shizi_min       100                                   //识别十字的最小值(0,4)          ***4  
#define     wandaoshucha_min    5		                 //左右转弯时竖电感的差值(0,4)     ***5
#define     wandaoshuhe_min    10                              // 左右转弯时竖电感的和值         ***6
#define     zhidaohen_min      150 		         //直道时横电感总值的最小值                ***7
#define     zhidaoshu_max       3    		        //直道时竖电感总值的最大值                ***8


/***********圆环参数*************/
#define     ruyuanhuanhenghe_min  260                        //入圆环时横电感和最小值              ***9
#define     ruyuanhuanhenghe_max  280                           //入圆环时横电感和最大值          ***10
#define      yuanhuanhenghe_min   140                      //圆环横电感最小值                       ***11
#define      yuanhuanhenghe_max   280                      //圆环横电感最大值                       ***12
#define      yuanhuanshuhe_min    8                         //圆环竖电感和最小值                     ***150
#define      yuanhuanshucha_min   6                       //圆环竖电感差最小值                     ***16 
#define     chuhuan_dianganzhi    280                       //出环电感值                          ***18
#define     yuanhuan_longth       500                          //圆环操作总长度
#define    goin_yuan              35                          //进入圆环所走距离
#define     goout_yuan            5                        //出环所走距离
int     guochengcanshu[2];                               
int      yuanhuan_times=0;                             //进入圆环的次数
int     yuanhuanfangxiang;                             //记录圆环方向，1为右边，0为左边
int     time_num=0;
float     yuan_distence=0.0;                          //圆环操作距离记录
int     istimetoyuanhuan=0;                          //开始进入圆环                             ***13
int      ruhuan_diangan_jilu=260;

/*********断路参数*************/
short duanlu=0;                                              //断路标志位
short duanlu_send_flag=0;                                    //2401发送断路信息标志位
short qipao_send_flag=0;                                     //2401发送终点线信息标志位
float duan_lu_distance;                                      //断路距离
float duan_lu_adc_sum[2];                                    //断路AD和记录
float out_duan_lu_adc[2];                                  //判断出路障所需变量 
short duanl_stop_flag=0;                                    //断路停车标志位
short duanl_stop_once=0;                                     //停止一次
short zhili_frist=1;                                         //直立车先到断路
#define duanli_AD_max          145                         //断路光敏电阻AD和最大值
#define duanli_AD_min          120                         //断路光敏电阻AD和最小值
#define tiao_bian_AD             20                          //出断路跳变AD


float     steer_contorl_p;                               //舵机PD
float     steer_contorl_d;
float       steer_out;                                      //PD之后的转向值                            
int       ru_yuanhuan=0;                                    //入圆标志位
int       yuanhuan=0;                                     //圆环标志位 
int       chuhuan=0;                                        //出环标志位
int     angle;                                            //传给舵机的数
int     speed_left;                                            //传给电机的数
int     speed_right;


extern int Encoder_Left,Encoder_Right;
extern int MotorLeft,MotorRight;
extern unsigned short send_data[9];
extern float Angle_Balance;
/************************
ADC端口初始化
adc_chose：ADC0/ADC1
adc_digit：精度
************************/



void adc_ready()                                               //ADC采集初始化
{
    ADC_Init(adc_chose);
    ADC_Start(adc_chose,adc_num1,adc_digit); 
    ADC_Start(adc_chose,adc_num2,adc_digit);
    ADC_Start(adc_chose,adc_num3,adc_digit);
    ADC_Start(adc_chose,adc_num4,adc_digit);
    ADC_Start(adc_chose,adc_num5,adc_digit);
    ADC_Start(adc_chose,adc_num7,adc_digit);
    ADC_Start(adc_chose,adc_num8,adc_digit);
    ADC_Init(ADC_0);
    ADC_Start(ADC_0,adc_num6,adc_digit);


    
}

/********************
ADC采集与平滑滤波
ad_valu[5]：原始数据
AD[5]：滤波[3]之后的数据
********************/



void Read_adc()                                               //ADC采集原始数据
{
    int16  i=0,j=0;
    
    
    ad_valu[0]=ADC_Ave(adc_chose,adc_num1,adc_digit,10);     
    ad_valu[1]=ADC_Ave(adc_chose,adc_num2,adc_digit,10);     
    ad_valu[2]=ADC_Ave(adc_chose,adc_num3,adc_digit,10);    
    ad_valu[3]=ADC_Ave(adc_chose,adc_num4,adc_digit,10);    
    ad_valu[4]=ADC_Ave(adc_chose,adc_num5,adc_digit,10); 
 //   ad_valu[5]=ADC_Ave(ADC_0,adc_num6,adc_digit,10);                     //超声波
    ad_valu[6]=ADC_Ave(adc_chose,adc_num7,adc_digit,10)/4096.0*100;        //光电对管1
    ad_valu[7]=ADC_Ave(adc_chose,adc_num8,adc_digit,10)/4096.0*100;        //光电对管2
    

    
    
    for(j=0;j<5;j++)
        for(i=0;i<huanum-1;i++)                                                    //滑动滤波
        {
            AD_valu[j][i]=AD_valu[j][i+1];
            
        }
    for(i=0;i<5;i++) 
    {
        AD_valu[i][huanum-1]=ad_valu[i];
    }
    for(j=0;j<5;j++)
        for(i=0;i<huanum;i++)
        {
            AD_sum[j]+=AD_valu[j][i];
        }
    for(i=0;i<5;i++)
    {
        AD[i]=AD_sum[i]/huanum;                                 //较准确的AD值
        AD_sum[i]=0;
    }
    
    
    
}

/**********************
断路检测
**********************/
void  duan_lu()
{
   static short judge_duan_or_qi=0;
   static short stop_once=0; 
   static float distance=0;
   static float  distance_2=0;
   static short   jijiji=1;
    duan_lu_adc_sum[1]=duan_lu_adc_sum[0];
    duan_lu_adc_sum[0]=ad_valu[6]+ad_valu[7];
    distance-=(bianmaqi[0]*0.00452);
    if(distance>100)
      distance=100;
    
    if((duanlu==0)&&(distance==100)&&start_luzhang==0&&(Angle_Balance>-56&&Angle_Balance<0)&&((Encoder_Left+Encoder_Right)>20)&&((duan_lu_adc_sum[0]<duanli_AD_max)&&(duan_lu_adc_sum[0]>duanli_AD_min)))
    {
       distance_2-=(bianmaqi[0]*0.00452);
       if(distance_2>10)
       {
         if((duan_lu_adc_sum[0]<duanli_AD_max)&&(duan_lu_adc_sum[0]>duanli_AD_min)&&(jijiji)==1)
         {
            duanlu=1;                                                                         //断路标志置1
            GPIO_Ctrl(PTA,9,1);
            duanlu_send_flag=0;
            out_duan_lu_adc[0]=duan_lu_adc_sum[0];
            distance_2=0;
            jijiji=0;                                                                         //断路只判别一次
         }
         else
         {
           duanlu=0;
           distance_2=0;
         }
       }
    }
    if(duanlu==1)
    {
   /*   duan_lu_distance-=(bianmaqi[0]*0.00452);
      if(duan_lu_distance>10)
      {     if((judge_duan_or_qi==0)&&(!GPIO_Get(PTE27)))
              {
                judge_duan_or_qi=1; 
              }
            if(judge_duan_or_qi)
             {
              if(duan_lu_distance>20)
              {
                duanlu=0;
                duan_lu_distance=0;
                judge_duan_or_qi=0;
                GPIO_Ctrl(PTA,9,0);
               
              }
             }*/
        //    else
        //    {           
              if(duanl_stop_once==0&&zhili_frist==1&&stop_once==0)
              {
                duanl_stop_flag=1;
                stop_once=1;
              }
              out_duan_lu_adc[1]=duan_lu_adc_sum[0];
              if(out_duan_lu_adc[1]-out_duan_lu_adc[0]>tiao_bian_AD)
              {
                duan_lu_distance=0;
                duanl_stop_flag=0;
                duanlu=0;
                duanlu_send_flag=1;
                GPIO_Ctrl(PTA,9,0);
                out_duan_lu_adc[0]=0.0;
                out_duan_lu_adc[1]=0.0;
                duanl_stop_once=0;
                stop_once=0;
              
              }
   //         }
   //   }
   }
  
}


/*******************
起跑线检测
*******************/
void start_line_judge()
{
  static short distance=0;
  distance-=(bianmaqi[0]*0.00452);
  if(distance>100)
     distance=100;
  if(distance==100)
  {
    if(!GPIO_Get(PTE27))
    {
      qipao_send_flag=1;
    }

  }
}


/****************************
adc参考最大值采集
*****************************/
void adc_refer_get()                                    //采集最大值和最小值
{
    int i;
     
    for(i=0;i<5;i++)
    {
        Read_adc();
        if(AD[i]>AD_max[i])
        {
            AD_max[i]=AD[i];                                //五个电感采集                             
            if(i==1)                                       //当得到1的最强电感时，保存2号电感的值
                refer[0]=AD[2];
            else if(i==2)                                       //当得到2的最强电感时，保存1号电感的值
                refer[1]=AD[1];
            else if(i==3)
                refer[2]=AD[2];                              //当得到3的最强电感时，保存2号电感的值
            
        }
        
    }
    
    
}



/******************************
数值归一化，范围0~100
AD_TURE：归一化之后的值
******************************/



void adc_guiyi()                                                            
{ 
    int i;
    for(i=0;i<5;i++)
    {
        AD_TURE[i]=(float)(AD[i]-AD_min[i])/(AD_max[i]-AD_min[i])*100;
        
        if(AD_TURE[i]<1)
            AD_TURE[i]=1;
        else if(AD_TURE[i]>100)
            AD_TURE[i]=100;
    }
    for(i=1;i<4;i++)
        refer_TURE[i-1]=(float)abs(refer[i-1]-AD_min[i])/(float)(AD_max[i]-AD_min[i])*100;         //相关值归一化
    
    summmmm=ad_data_ading(1,3);
    sum_all=ad_data_ading(0,4);
}





/***************
ad_data_ading(int i,int j)
得到从i到j的电感的和
******************/
int16 ad_data_ading(int i,int j)
{
    int16 sum = 0;
    for(;i<j+1 ;i++){
        sum += AD_TURE[i];
    }
    return sum;
}
/********************************
电磁线位置判断，
ad_erro：   中心电感与最大值的差值
location_event： 中心线位置
k :需要试验的固定值
********************************/



void  location_judge()                                     //电磁线位置解算
{ 
    int i;
    float temp=0;
    for(i=1;i<4;i++)                                           //寻找电动势最强的电感（中间3个中）
    {
        if(AD_TURE[i]>temp)
        {
            temp=AD_TURE[i];
            max=i;
        }
    } 
    
    if((max==1)&&(AD_TURE[2]<refer_TURE[0]))                   //电磁线在1号电感的左边
    {
        location_event=0;                                       //左偏大
        event_ago=location_event;                              //记录上一次位置
    }
    else if ((max==1)&&(AD_TURE[2]>refer_TURE[0]))             //电磁线在编号1~2电感之间
    {
        location_event=1;                                         //左偏小
        event_ago=location_event;
    }
    else if ((max==2)&&(AD_TURE[1]>refer_TURE[1]))             //电磁线在编号1~2电感之间
    {                                                          //左偏小
        location_event=2;                                         
        event_ago=location_event;
    }
    else if ((max==2)&&(AD_TURE[1]<refer_TURE[1]))             //电磁线在编号2~3电感之间
    {
        location_event=3;                                        //右偏小
        event_ago=location_event;
    }
    else if ((max==3)&&(AD_TURE[2]>refer_TURE[2]))             //电磁线在编号2~3电感之间
    {
        location_event=4;                                        
        
        event_ago=location_event;
        
    }
    else if ((max==3)&&(AD_TURE[2]<refer_TURE[2]))             //电磁线在编号3电感右边
    {
        location_event=5;                                        //右偏大
        
        event_ago=location_event;
    }
    
    
    else                                                     //若检测失败则按原来的跑
    {
        location_event=event_ago;
    }
    
    
} 

void  steer_calculate()
{
    
  
    if(AD_TURE[1] +AD_TURE[2] + AD_TURE[3] > zhidaohen_min&& AD_TURE[0] +  AD_TURE[4] < zhidaoshu_max && max == 2)
    {
        element_event = 0;  		//0为直道
    }
    else if(AD_TURE[0] +AD_TURE[4] > shizi_min)
    {
        element_event = 3;			   //3为十字
    }
    else  if(( (AD_TURE[0] - AD_TURE[4]) > wandaoshucha_min) && ((AD_TURE[0] + AD_TURE[4]) > wandaoshuhe_min))
    {
        element_event = 1;			//1为左转弯
    }
    else if( AD_TURE[4] - AD_TURE[0] > wandaoshucha_min && AD_TURE[0] + AD_TURE[4] > wandaoshuhe_min)
    {
        element_event = 2;			//2为右转弯
    }
 
    /*************比例系数确定********************/
    
    if(yuanhuan==1)
       P_D_bili=30;
   /* else if (chuhuan==1)
       P_D_bili=25;
    else
    {
      if(element_event==1||element_event==3)
         P_D_bili=25;
      else if(element_event==1||element_event==2)
         P_D_bili=25;
    }
    
    */
    if((Encoder_Left+Encoder_Right)/2>Target_Velocity*1.2)
        P_D_bili=30;
    
    
    
}    







/***************************
电磁偏移量计算
***************************/



void  diancipianyiliang()   

{
   float  AD1=AD_TURE[1];
   float  AD3=AD_TURE[3];
    if((element_event==3||element_event==0)&&yuanhuan==0)
    {
      AD1+=50;
      AD3+=50;
    }
   
    if(  (element_event == 0 || element_event == 3) &&(location_event == 3 || location_event== 2))          // 
    {
	ad_error = 100 * (AD1 - AD3) / (AD1 + AD3);
        
    }

  
   else  if( location_event == 1|| location_event == 2 )           
    {
        ad_error =( 100 * (AD_TURE[1] - AD_TURE[3])/(AD_TURE[1] + AD_TURE[3]) + 
                   (100 * (AD_TURE[1] - AD_TURE[2])/(AD_TURE[1] + AD_TURE[2]) +10 ))/2 ;         //43
        
    }      
   else  if( location_event == 0  )           
    {
        ad_error =( 100 * (AD_TURE[1] - AD_TURE[3])/(AD_TURE[1] + AD_TURE[3]) + 
                   (100 * (AD_TURE[1] - AD_TURE[2])/(AD_TURE[1] + AD_TURE[2]) +30 ))/2 ;         //43
        
    }  
    else if(location_event == 3 || location_event == 4  )              
    {			                                                                         
	ad_error =( 100 * (AD_TURE[1] - AD_TURE[3])/(AD_TURE[1] + AD_TURE[3]) + 
                   (100 * (AD_TURE[2] -AD_TURE[3])/(AD_TURE[2] +AD_TURE[3]) -10 ))/2 ;
        
        
    }
     else if(  location_event == 5 )              
    {			                                                                         
	ad_error =( 100 * (AD_TURE[1] - AD_TURE[3])/(AD_TURE[1] + AD_TURE[3]) + 
                   (100 * (AD_TURE[2] -AD_TURE[3])/(AD_TURE[2] +AD_TURE[3]) -30 ))/2 ;
        
        
    }
    
    if(ad_error > ad_error_max)
    {
  	ad_error = ad_error_max;
    }
    else if(ad_error < -ad_error_max)
    {
  	ad_error = -ad_error_max;
    }        
}



/*******************
圆环操作
*******************/



void  yuanhuan_judge()
{
    static short num=0;
    static int   start=0;
    static short i=1;
    
    if(chuhuan==0&&yuanhuan==0&&element_event!=3&&ru_yuanhuan==0&&ad_data_ading(1,3)>ruyuanhuanhenghe_min)    //检测到圆环
    {
        ru_yuanhuan=1;             
        yuan_distence=0.0;                                                        
        
    }
    if(yuanhuan==0&&ru_yuanhuan==1)
    {

        guochengcanshu[1]=guochengcanshu[0];                                       //更新过程量                 
        guochengcanshu[0]=AD[2];
        if(num<0)
            num=0;
        if(guochengcanshu[0]<guochengcanshu[1])                                    //入圆点判定，若递减则+1
            num++;
        else
            num--;
        if(((AD_TURE[0])<AD_TURE[4])&&(i==1))                                                 //方向判定，左》右+1，else -1
          {
            yuanhuanfangxiang=0;
            i=0;
          }
        else if((AD_TURE[0]>(AD_TURE[4]))&&(i==1))
        {
          yuanhuanfangxiang=1;
          i=0;
        }
        if(num>4)                                                                  //递减3次判断入环点                  
        { 
            GPIO_Ctrl(PTA,9,1);                                                   //蜂鸣器
            num=0;
            yuanhuan=1;
            ru_yuanhuan=0;
            i=1;
            guochengcanshu[0]=0;                                                      //过程量清0
            guochengcanshu[1]=0;
            ruhuan_diangan_jilu=ad_data_ading(1,3);                                   //记录入环电感值
        }
        
    }
    if(yuanhuan==1)                                                                  //入圆执行        
    {
        start=1;                                                                 //计时标志位置1
        
        if(yuanhuanfangxiang==1)
        {
            AD_TURE[3]*=0.35;                                                      // 削弱一侧电感
        }
        else if(yuanhuanfangxiang==0)
        {
            AD_TURE[1]*=0.35;
        }
        if(yuan_distence>goin_yuan)                                             //一段距离后圆环标志位清0
        {
            GPIO_Ctrl(PTA,9,0);
            yuanhuan=0;
            ru_yuanhuan=0;
            chuhuan=1;                                                              //置1，防止重复入环                                                           
            
        }
        
    }
    
    
    if(start)                                                                
    {
        yuan_distence+=(-(bianmaqi[0]*0.00452)+(bianmaqi[1]*0.00452))*0.5;                                          //距离积分
        
        if(yuan_distence>yuanhuan_longth)                                              //一段距离内不重复执行入环程序
        {
            chuhuan=0;
            yuan_distence=0.0;
            yuanhuan_times++;
            start=0;
        }
    }
    
}

void chuhuan_judge()
{
  static short chuhuan_flag=0; 
  static short Forbid=0;
  static short distence=0;
  static short start=0;
    if(Forbid==0&&chuhuan==1&&ad_data_ading(1,3)>chuhuan_dianganzhi)      //出环检测
      {
        chuhuan_flag=1;
      }
    if(chuhuan_flag)
     {
        Forbid=1;
        start=1;    
        if(yuanhuanfangxiang==0)
        {
          AD_TURE[3]*=0.5;
        }
        else if (yuanhuanfangxiang==1)
        {
          AD_TURE[1]*=0.5;
        }
     }
    if(start==1)
    {
      distence-=(bianmaqi[0]*0.00452);
      if(distence>goout_yuan)
      {
        chuhuan_flag=0;
      }
      if(distence>100)
      {
        start=0;
        Forbid=0;
      }
    }
}


/***********************
PD
************************/
void  PD_set()
{

    
    steer_contorl_p=4;
    steer_contorl_d=230;                          
    
    
    
    
}


void  duoji()
{
    float  duty;
    
    if(yuanhuan==1)                                                  //检测到入环
    {   
        if(yuanhuanfangxiang==1)
            angle=Step_Right;                                         //左拐
        else
            angle=Step_Left;                                          //右拐
    }
    
    
    else 
    {
        steer_out =steer_contorl_p *ad_error + steer_contorl_d * (ad_error - ad_error_ago);
        
        ad_error_ago=ad_error;                     //记录上一次的数值
        duty=steer_out*0.356;
        
        angle=(int)(Step_Middle+duty);
    }
    
    if(angle > Step_Right)                                             
        angle = Step_Right;
    if(angle< Step_Left)
        angle = Step_Left;
    
    FTM_PWM_Duty(FTM3,FTM_CH7,angle);
    
}




void Set_Pwm(int MotorLeft,int MotorRight)                           
{
    
    int Limit=3800;                             //pwm限幅//※重要,可改※
    int dead=0;                                         //<30?
    MotorLeft*=1;
    MotorRight*=1;
    if(MotorLeft>Limit) MotorLeft=Limit;
    if(MotorLeft<-Limit)  MotorLeft=-Limit;	
    if(MotorRight>Limit)   MotorRight=Limit;
    if(MotorRight<-Limit)  MotorRight=-Limit;	
    
    
    if(MotorLeft<=0)
    {
        FTM_PWM_Duty(FTM3, FTM_CH0, 0);      //PTA4
        FTM_PWM_Duty(FTM0, FTM_CH2,-(MotorLeft+dead));             //PTA5
    }
    else
    {
        //        FTM_PWM_Duty(FTM0, FTM_CH0, 0);
        //        FTM_PWM_Duty(FTM0, FTM_CH1, 0);     
        //        DELAY_us(100);                 //抄来的,反转可以保护电机
        FTM_PWM_Duty(FTM3, FTM_CH0,MotorLeft+dead);             //PTA4      
        FTM_PWM_Duty(FTM0, FTM_CH2, 0 );     //PTA5
    }
    
    if(MotorRight<=0)
    {
        FTM_PWM_Duty(FTM0, FTM_CH3, -(MotorRight+dead));           //PTA6
        FTM_PWM_Duty(FTM0, FTM_CH4, 0);  //PTA7
    }
    else
    {
        
        //        FTM_PWM_Duty(FTM0, FTM_CH0, 0);
        //        FTM_PWM_Duty(FTM0, FTM_CH1, 0);     
        //        DELAY_us(100);                 //抄来的,反转可以保护电机
        //        
        FTM_PWM_Duty(FTM0, FTM_CH3,0);
        FTM_PWM_Duty(FTM0, FTM_CH4, MotorRight+dead );
    }
}








void  dianji()
{
    float  duty;
    float  speed_middle;
    float  k;
    static short  i=0;
    
    speed_middle= (Encoder_Left+Encoder_Right)/2 ;                       //小车直道速度 
    k=speed_middle/P_D_bili;                                                  //待测参数 ,速度相关系数
    
    if(luzhang_ff_1==0&&duanl_stop_flag==0&&start_luzhang==0&&judge_luzhang==0&&chuhuan==0&&(chaoshengbo_distence[0]<90&&chaoshengbo_distence[0]>60))                  // ||((0.1*count[2])<70&&(0.1*count[2])>60)         
    { 
         judge_luzhang=1;
    }
   if(judge_luzhang)
   {
       bianmaqi_distence+=(-(bianmaqi[0]*0.00452)+(bianmaqi[1]*0.00452))*0.5;
       if(bianmaqi_distence>10&&bianmaqi_distence<20)
       {
         if(shifou_luzhang==1&&chaoshengbo_distence[0]<80&&chaoshengbo_distence[0]>40&&i==0)
         {
           start_luzhang=1;
           i=1;                                                               //路障程序只执行一次
         }       
         judge_luzhang=0;
         bianmaqi_distence=0;
       }
   }
   if(start_luzhang==1)
   {
       bianmaqi_distence+=(-(bianmaqi[0]*0.00452)+(bianmaqi[1]*0.00452))*0.5;
       
    if(bianmaqi_distence<distence_turn)                             //拐
    {
        GPIO_Ctrl(PTA,9,1);
        steer_out=steer_out_turn;

    }
    else if(bianmaqi_distence<(distence_turn+distence_straight))                    //直
      {
        GPIO_Ctrl(PTA,9,0);
        steer_out=steer_out_straight;

      }
    else                        //回
      {
        if(sum_all>150)
        {
           start_luzhang=0;                                              //路障标志位置0
           bianmaqi_distence=0;                                          //积分清0
           chaoshengbo_distence[0]=0;
           chaoshengbo_distence[1]=0;
           luzhang_ff_2=0;
           
        }
        if(bianmaqi_distence>(distence_turn+distence_straight+distence_back))
           steer_out=0;
        else
           steer_out=steer_out_back;

      }

  }
    
    
 else
    
  { 
      steer_out =steer_contorl_p *ad_error + steer_contorl_d * (ad_error - ad_error_ago); 
     // send_data[8]=(int)(steer_contorl_d * (ad_error - ad_error_ago));
      ad_error_ago=ad_error;                     //记录上一次的数值
     // send_data[7]=(int)(steer_contorl_p *ad_error);
     // send_data[6]=(int)steer_out;
      
      
  }
    duty=steer_out*k;   
    speed_left=(int)duty;
    speed_right=(int)-duty;    
}







void  duoji_go()                              //舵机
{
    Read_adc(); 
    adc_guiyi();
    location_judge();
    steer_calculate();
    diancipianyiliang() ;
    duoji();
}

void  dianji_go()                            //电机
{
    Read_adc(); 
    adc_guiyi();
    location_judge();
    steer_calculate();
    PD_set();
    yuanhuan_judge();
    chuhuan_judge() ;
    diancipianyiliang();
    start_line_judge();
    duan_lu();
    dianji();
}
