/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
��ƽ    ̨�������������ܿƼ�MK66FX1M0VLQ18���İ�
����    д��CHIUSIR
����    ע��
������汾��V1.0
�������¡�2017��01��20��
�������Ϣ�ο����е�ַ��
����    վ��http://www.lqist.cn
���Ա����̡�http://shop36265907.taobao.com
���������䡿chiusir@163.com
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
#ifndef __LQ_CTRL_H_
#define __LQ_CTRL_H_
    
    #define DIFFERENCE 30    

      
    int My_Abs(int a);
    void Get_Dip_Angle();
    int Balance(float Angle,float Gyro);
    int Balance_Ctrl_Pwm(float Angle,float Gyro);   
    void Velocity_Ctrl_Angle(int encoder_left,int encoder_right);
    void speed_output(void);
    void Motor_Pwm_Out(s16  Speed_L ,s16  Speed_R ,s16 MaxPwm );    
    int Turn_Ctrl_Pwm(int encoder_left,int encoder_right,float gyro);//ת�����  
    void EX_Control(float set_angle);
    void IN_Control(float ex_output);
    void LED_zhishi(void);
    
#endif