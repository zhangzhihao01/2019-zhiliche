#define  adc_num1   ADC1_SE12                //5路adc
#define  adc_num2   ADC1_SE10               
#define  adc_num3   ADC1_SE11               
#define  adc_num4   ADC1_SE8               
#define  adc_num5   ADC1_SE9
#define  adc_num6   ADC0_SE16
#define  adc_num7   ADC1_SE5a
#define  adc_num8   ADC1_SE7a 
#define  adc_chose      ADC_1              //ADC_1/2
#define  adc_digit  ADC_12bit              //ADC精度
#define   huanum       3                   //滑动平均滤波的次数
#define     Step_Right       2800          //可再稍微大一点 
#define     Step_Middle      2540 
#define     Step_Left        2280 

void adc_ready(void);
void adc_refer_get(void);
void Read_adc(void);
void adc_refer_get(void);
void adc_guiyi(void);
void  location_judge(void);
void  steer_calculate(void);
void  duoji(void);
void  dianji(void);
void Set_Pwm(int MotorLeft,int MotorRight);
void  PD_set(void);
void  yuanhuan_judge(void);
void chuhuan_judge(void)  ;  
void  duoji_go(void);
void  dianji_go(void);
void  diancipianyiliang(void); 
void  duan_lu(void);
int16 ad_data_ading(int i,int j);
