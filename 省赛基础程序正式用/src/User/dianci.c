#include "include.h"


extern u8 Pixle[LCDH][LCDW];                             //��ֵ��������OLED��ʾ������

/*******·�ϲ���********/
extern uint16_t count[3];
extern int bianmaqi[2];                                 //��������ֵ һ������0.00452cm
extern float angle_X;                                     //X��Ƕ�
extern float angle_X_ago ;                             //��һ��X��Ƕ�
extern float kaerman_angle;
extern float QingJiaoZhongZhiTarget;
extern int chaoshengbo_count;
extern int Target_Velocity;
extern short luzhang_ff_2;                                //���ϱ��÷��������ظ��жϱ�־λ
extern short luzhang_ff_1;                                //ײ·�ϱ�־λ
float  bianmaqi_distence=0;                               //���������ֳ�����
int    chaoshengbo_distence[2];                             //��������࣬��λcm 
short start_luzhang=0;                                   //·�ϱ�־λ
short  judge_luzhang=0;                                  //maybe ·��
short  shifou_luzhang=0;                                 //�Ƿ����
#define  steer_out_turn      210                           //·�Ͽ���
#define  steer_out_straight  -200                           //�ܿ�·�Ϻ���һ��ֱ��
#define  steer_out_back      -180                            //�ص�����
#define  distence_turn       20                           //�ճ�·�Ͼ���                                    
#define  distence_straight    20                            //�ܿ�·�Ϻ���һ��ֱ���ľ���
#define  distence_back        13                          //�������ľ���




/*****ͨ�ò���*******/
int16  AD_valu[5][3];                                     //�⼸��Ϊȫ�ֱ������õ�ʱ�����������
int16  AD_sum[5];
int16  AD[5];                                           //��һ��֮ǰ��ֵ
int16  AD_max[5]={1024,1024,1024,1024,1024};            //5��������ֵ��¼               ***1
int16  AD_min[5]={0,0,0,0,0};                          //5�������Сֵ��¼
int16  refer[3] ={390,390,390};                                       //�̶���Ųο�ֵ 
float  refer_TURE[3]={38,38,38};                                 //��һ���ĵ�Ųο�ֵ               ***2         
float  AD_TURE[5] ;                                //������һ����ֵ
int     max;                                            //����б��
int    event_ago;                                       
int    location_event;                                  //�����λ�ü�¼
int    element_event;                                   //����Ԫ�ؼ�¼
short  P_D_bili=26;
float  ad_error;                                        //���ƫ����
float  ad_error_ago; 
int16   ad_valu[8];
int    summmmm;                                        //  �����ܺ�
int    sum_all;                                        //  ���е���ܺ�


#define     ad_error_max    100                                //���������ֵ                 ***3
#define     shizi_min       100                                   //ʶ��ʮ�ֵ���Сֵ(0,4)          ***4  
#define     wandaoshucha_min    5		                 //����ת��ʱ����еĲ�ֵ(0,4)     ***5
#define     wandaoshuhe_min    10                              // ����ת��ʱ����еĺ�ֵ         ***6
#define     zhidaohen_min      150 		         //ֱ��ʱ������ֵ����Сֵ                ***7
#define     zhidaoshu_max       3    		        //ֱ��ʱ�������ֵ�����ֵ                ***8


/***********Բ������*************/
#define     ruyuanhuanhenghe_min  260                        //��Բ��ʱ���к���Сֵ              ***9
#define     ruyuanhuanhenghe_max  280                           //��Բ��ʱ���к����ֵ          ***10
#define      yuanhuanhenghe_min   140                      //Բ��������Сֵ                       ***11
#define      yuanhuanhenghe_max   280                      //Բ���������ֵ                       ***12
#define      yuanhuanshuhe_min    8                         //Բ������к���Сֵ                     ***150
#define      yuanhuanshucha_min   6                       //Բ������в���Сֵ                     ***16 
#define     chuhuan_dianganzhi    280                       //�������ֵ                          ***18
#define     yuanhuan_longth       500                          //Բ�������ܳ���
#define    goin_yuan              35                          //����Բ�����߾���
#define     goout_yuan            5                        //�������߾���
int     guochengcanshu[2];                               
int      yuanhuan_times=0;                             //����Բ���Ĵ���
int     yuanhuanfangxiang;                             //��¼Բ������1Ϊ�ұߣ�0Ϊ���
int     time_num=0;
float     yuan_distence=0.0;                          //Բ�����������¼
int     istimetoyuanhuan=0;                          //��ʼ����Բ��                             ***13
int      ruhuan_diangan_jilu=260;

/*********��·����*************/
short duanlu=0;                                              //��·��־λ
short duanlu_send_flag=0;                                    //2401���Ͷ�·��Ϣ��־λ
short qipao_send_flag=0;                                     //2401�����յ�����Ϣ��־λ
float duan_lu_distance;                                      //��·����
float duan_lu_adc_sum[2];                                    //��·AD�ͼ�¼
float out_duan_lu_adc[2];                                  //�жϳ�·��������� 
short duanl_stop_flag=0;                                    //��·ͣ����־λ
short duanl_stop_once=0;                                     //ֹͣһ��
short zhili_frist=1;                                         //ֱ�����ȵ���·
#define duanli_AD_max          145                         //��·��������AD�����ֵ
#define duanli_AD_min          120                         //��·��������AD����Сֵ
#define tiao_bian_AD             20                          //����·����AD


float     steer_contorl_p;                               //���PD
float     steer_contorl_d;
float       steer_out;                                      //PD֮���ת��ֵ                            
int       ru_yuanhuan=0;                                    //��Բ��־λ
int       yuanhuan=0;                                     //Բ����־λ 
int       chuhuan=0;                                        //������־λ
int     angle;                                            //�����������
int     speed_left;                                            //�����������
int     speed_right;


extern int Encoder_Left,Encoder_Right;
extern int MotorLeft,MotorRight;
extern unsigned short send_data[9];
extern float Angle_Balance;
/************************
ADC�˿ڳ�ʼ��
adc_chose��ADC0/ADC1
adc_digit������
************************/



void adc_ready()                                               //ADC�ɼ���ʼ��
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
ADC�ɼ���ƽ���˲�
ad_valu[5]��ԭʼ����
AD[5]���˲�[3]֮�������
********************/



void Read_adc()                                               //ADC�ɼ�ԭʼ����
{
    int16  i=0,j=0;
    
    
    ad_valu[0]=ADC_Ave(adc_chose,adc_num1,adc_digit,10);     
    ad_valu[1]=ADC_Ave(adc_chose,adc_num2,adc_digit,10);     
    ad_valu[2]=ADC_Ave(adc_chose,adc_num3,adc_digit,10);    
    ad_valu[3]=ADC_Ave(adc_chose,adc_num4,adc_digit,10);    
    ad_valu[4]=ADC_Ave(adc_chose,adc_num5,adc_digit,10); 
 //   ad_valu[5]=ADC_Ave(ADC_0,adc_num6,adc_digit,10);                     //������
    ad_valu[6]=ADC_Ave(adc_chose,adc_num7,adc_digit,10)/4096.0*100;        //���Թ�1
    ad_valu[7]=ADC_Ave(adc_chose,adc_num8,adc_digit,10)/4096.0*100;        //���Թ�2
    

    
    
    for(j=0;j<5;j++)
        for(i=0;i<huanum-1;i++)                                                    //�����˲�
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
        AD[i]=AD_sum[i]/huanum;                                 //��׼ȷ��ADֵ
        AD_sum[i]=0;
    }
    
    
    
}

/**********************
��·���
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
            duanlu=1;                                                                         //��·��־��1
            GPIO_Ctrl(PTA,9,1);
            duanlu_send_flag=0;
            out_duan_lu_adc[0]=duan_lu_adc_sum[0];
            distance_2=0;
            jijiji=0;                                                                         //��·ֻ�б�һ��
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
�����߼��
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
adc�ο����ֵ�ɼ�
*****************************/
void adc_refer_get()                                    //�ɼ����ֵ����Сֵ
{
    int i;
     
    for(i=0;i<5;i++)
    {
        Read_adc();
        if(AD[i]>AD_max[i])
        {
            AD_max[i]=AD[i];                                //�����вɼ�                             
            if(i==1)                                       //���õ�1����ǿ���ʱ������2�ŵ�е�ֵ
                refer[0]=AD[2];
            else if(i==2)                                       //���õ�2����ǿ���ʱ������1�ŵ�е�ֵ
                refer[1]=AD[1];
            else if(i==3)
                refer[2]=AD[2];                              //���õ�3����ǿ���ʱ������2�ŵ�е�ֵ
            
        }
        
    }
    
    
}



/******************************
��ֵ��һ������Χ0~100
AD_TURE����һ��֮���ֵ
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
        refer_TURE[i-1]=(float)abs(refer[i-1]-AD_min[i])/(float)(AD_max[i]-AD_min[i])*100;         //���ֵ��һ��
    
    summmmm=ad_data_ading(1,3);
    sum_all=ad_data_ading(0,4);
}





/***************
ad_data_ading(int i,int j)
�õ���i��j�ĵ�еĺ�
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
�����λ���жϣ�
ad_erro��   ���ĵ�������ֵ�Ĳ�ֵ
location_event�� ������λ��
k :��Ҫ����Ĺ̶�ֵ
********************************/



void  location_judge()                                     //�����λ�ý���
{ 
    int i;
    float temp=0;
    for(i=1;i<4;i++)                                           //Ѱ�ҵ綯����ǿ�ĵ�У��м�3���У�
    {
        if(AD_TURE[i]>temp)
        {
            temp=AD_TURE[i];
            max=i;
        }
    } 
    
    if((max==1)&&(AD_TURE[2]<refer_TURE[0]))                   //�������1�ŵ�е����
    {
        location_event=0;                                       //��ƫ��
        event_ago=location_event;                              //��¼��һ��λ��
    }
    else if ((max==1)&&(AD_TURE[2]>refer_TURE[0]))             //������ڱ��1~2���֮��
    {
        location_event=1;                                         //��ƫС
        event_ago=location_event;
    }
    else if ((max==2)&&(AD_TURE[1]>refer_TURE[1]))             //������ڱ��1~2���֮��
    {                                                          //��ƫС
        location_event=2;                                         
        event_ago=location_event;
    }
    else if ((max==2)&&(AD_TURE[1]<refer_TURE[1]))             //������ڱ��2~3���֮��
    {
        location_event=3;                                        //��ƫС
        event_ago=location_event;
    }
    else if ((max==3)&&(AD_TURE[2]>refer_TURE[2]))             //������ڱ��2~3���֮��
    {
        location_event=4;                                        
        
        event_ago=location_event;
        
    }
    else if ((max==3)&&(AD_TURE[2]<refer_TURE[2]))             //������ڱ��3����ұ�
    {
        location_event=5;                                        //��ƫ��
        
        event_ago=location_event;
    }
    
    
    else                                                     //�����ʧ����ԭ������
    {
        location_event=event_ago;
    }
    
    
} 

void  steer_calculate()
{
    
  
    if(AD_TURE[1] +AD_TURE[2] + AD_TURE[3] > zhidaohen_min&& AD_TURE[0] +  AD_TURE[4] < zhidaoshu_max && max == 2)
    {
        element_event = 0;  		//0Ϊֱ��
    }
    else if(AD_TURE[0] +AD_TURE[4] > shizi_min)
    {
        element_event = 3;			   //3Ϊʮ��
    }
    else  if(( (AD_TURE[0] - AD_TURE[4]) > wandaoshucha_min) && ((AD_TURE[0] + AD_TURE[4]) > wandaoshuhe_min))
    {
        element_event = 1;			//1Ϊ��ת��
    }
    else if( AD_TURE[4] - AD_TURE[0] > wandaoshucha_min && AD_TURE[0] + AD_TURE[4] > wandaoshuhe_min)
    {
        element_event = 2;			//2Ϊ��ת��
    }
 
    /*************����ϵ��ȷ��********************/
    
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
���ƫ��������
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
Բ������
*******************/



void  yuanhuan_judge()
{
    static short num=0;
    static int   start=0;
    static short i=1;
    
    if(chuhuan==0&&yuanhuan==0&&element_event!=3&&ru_yuanhuan==0&&ad_data_ading(1,3)>ruyuanhuanhenghe_min)    //��⵽Բ��
    {
        ru_yuanhuan=1;             
        yuan_distence=0.0;                                                        
        
    }
    if(yuanhuan==0&&ru_yuanhuan==1)
    {

        guochengcanshu[1]=guochengcanshu[0];                                       //���¹�����                 
        guochengcanshu[0]=AD[2];
        if(num<0)
            num=0;
        if(guochengcanshu[0]<guochengcanshu[1])                                    //��Բ���ж������ݼ���+1
            num++;
        else
            num--;
        if(((AD_TURE[0])<AD_TURE[4])&&(i==1))                                                 //�����ж�������+1��else -1
          {
            yuanhuanfangxiang=0;
            i=0;
          }
        else if((AD_TURE[0]>(AD_TURE[4]))&&(i==1))
        {
          yuanhuanfangxiang=1;
          i=0;
        }
        if(num>4)                                                                  //�ݼ�3���ж��뻷��                  
        { 
            GPIO_Ctrl(PTA,9,1);                                                   //������
            num=0;
            yuanhuan=1;
            ru_yuanhuan=0;
            i=1;
            guochengcanshu[0]=0;                                                      //��������0
            guochengcanshu[1]=0;
            ruhuan_diangan_jilu=ad_data_ading(1,3);                                   //��¼�뻷���ֵ
        }
        
    }
    if(yuanhuan==1)                                                                  //��Բִ��        
    {
        start=1;                                                                 //��ʱ��־λ��1
        
        if(yuanhuanfangxiang==1)
        {
            AD_TURE[3]*=0.35;                                                      // ����һ����
        }
        else if(yuanhuanfangxiang==0)
        {
            AD_TURE[1]*=0.35;
        }
        if(yuan_distence>goin_yuan)                                             //һ�ξ����Բ����־λ��0
        {
            GPIO_Ctrl(PTA,9,0);
            yuanhuan=0;
            ru_yuanhuan=0;
            chuhuan=1;                                                              //��1����ֹ�ظ��뻷                                                           
            
        }
        
    }
    
    
    if(start)                                                                
    {
        yuan_distence+=(-(bianmaqi[0]*0.00452)+(bianmaqi[1]*0.00452))*0.5;                                          //�������
        
        if(yuan_distence>yuanhuan_longth)                                              //һ�ξ����ڲ��ظ�ִ���뻷����
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
    if(Forbid==0&&chuhuan==1&&ad_data_ading(1,3)>chuhuan_dianganzhi)      //�������
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
    
    if(yuanhuan==1)                                                  //��⵽�뻷
    {   
        if(yuanhuanfangxiang==1)
            angle=Step_Right;                                         //���
        else
            angle=Step_Left;                                          //�ҹ�
    }
    
    
    else 
    {
        steer_out =steer_contorl_p *ad_error + steer_contorl_d * (ad_error - ad_error_ago);
        
        ad_error_ago=ad_error;                     //��¼��һ�ε���ֵ
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
    
    int Limit=3800;                             //pwm�޷�//����Ҫ,�ɸġ�
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
        //        DELAY_us(100);                 //������,��ת���Ա������
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
        //        DELAY_us(100);                 //������,��ת���Ա������
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
    
    speed_middle= (Encoder_Left+Encoder_Right)/2 ;                       //С��ֱ���ٶ� 
    k=speed_middle/P_D_bili;                                                  //������� ,�ٶ����ϵ��
    
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
           i=1;                                                               //·�ϳ���ִֻ��һ��
         }       
         judge_luzhang=0;
         bianmaqi_distence=0;
       }
   }
   if(start_luzhang==1)
   {
       bianmaqi_distence+=(-(bianmaqi[0]*0.00452)+(bianmaqi[1]*0.00452))*0.5;
       
    if(bianmaqi_distence<distence_turn)                             //��
    {
        GPIO_Ctrl(PTA,9,1);
        steer_out=steer_out_turn;

    }
    else if(bianmaqi_distence<(distence_turn+distence_straight))                    //ֱ
      {
        GPIO_Ctrl(PTA,9,0);
        steer_out=steer_out_straight;

      }
    else                        //��
      {
        if(sum_all>150)
        {
           start_luzhang=0;                                              //·�ϱ�־λ��0
           bianmaqi_distence=0;                                          //������0
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
      ad_error_ago=ad_error;                     //��¼��һ�ε���ֵ
     // send_data[7]=(int)(steer_contorl_p *ad_error);
     // send_data[6]=(int)steer_out;
      
      
  }
    duty=steer_out*k;   
    speed_left=(int)duty;
    speed_right=(int)-duty;    
}







void  duoji_go()                              //���
{
    Read_adc(); 
    adc_guiyi();
    location_judge();
    steer_calculate();
    diancipianyiliang() ;
    duoji();
}

void  dianji_go()                            //���
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
