/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
【平    台】北京龙邱智能科技MK66FX1M0VLQ18核心板
【编    写】CHIUSIR
【备    注】
【软件版本】V1.0
【最后更新】2017年01月20日
【相关信息参考下列地址】
【网    站】http://www.lqist.cn
【淘宝店铺】http://shop36265907.taobao.com
【交流邮箱】chiusir@163.com
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
    int Turn_Ctrl_Pwm(int encoder_left,int encoder_right,float gyro);//转向控制  
    void EX_Control(float set_angle);
    void IN_Control(float ex_output);
    void LED_zhishi(void);
    
#endif