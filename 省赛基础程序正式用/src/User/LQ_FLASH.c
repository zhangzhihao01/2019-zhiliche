/*-----------------------------------------------------------------------------------------------------
【平    台】龙邱K60核心板-智能车板
【编    写】LQ-005
【E-mail  】chiusir@163.com
【软件版本】V1.0，龙邱开源代码，仅供参考，后果自负
【最后更新】2019年04月02日
【网    站】http://www.lqist.cn
【淘宝店铺】http://shop36265907.taobao.com
【编译平台】IAR 8.2
【功    能】FLASH 读写例子
【注意事项】不要写靠前的扇区  会覆盖程序
-------------------------------------------------------------------------------------------------------*/
#include "include.h"



/*------------------------------------------------------------------------------------------------------
【函    数】Test_Flash
【功    能】测试flash 读写
【参    数】无
【返 回 值】无
【实    例】Test_Flash(); 
【注意事项】
--------------------------------------------------------------------------------------------------------*/
void Test_Flash(void)
{   
    char  txt[30];
	UART_Init(UART4, 115200);     //串口4初始化
    OLED_Init();                  //LCD初始化
    OLED_CLS();                   //LCD清屏
	KEY_Init();
	FLASH_Init();                 //Flash初始化
    OLED_P8x16Str(15,0,"LQ FLASH Test"); 
    printf("\r\nLQ FLASH Test");
	printf("\r\n不要写靠前的扇区  会覆盖程序 \n");
	
    uint8_t  write_data[10] = {1,2,3,4,5,6,7,8,9,10};                   //写入的数据
    uint8_t  read_data[10]  = {0};
    
    float  write_float[10] = {1.0, 1.1, 1.2, 1.3, 100.156, 3.1415926, 1000.35, 20.7, 9.9, 10};   //写入的数据
    float  read_float[10] = {0};
    
	FLASH_EraseSector(1);
    FLASH_WriteBuf(1,write_data, 10, 0);//写入扇区
//    printf("\r\n有 %d 位 \n", sizeof(write_data));
    
    FLASH_EraseSector(2);
    FLASH_WriteBuf(2,(uint8_t *)write_float, sizeof(write_float), 0);//写入扇区
//    printf("\r\n有 %d 位 \n", sizeof(write_float));
    
    
	/* 从倒数第一个扇区 0偏移位置开始 读出数据 */
	read_data[0] = FLASH_Read(1, 0, uint8_t);
    read_data[1] = FLASH_Read(1, 1, uint8_t);
    read_data[2] = FLASH_Read(1, 2, uint8_t);
    read_data[3] = FLASH_Read(1, 3, uint8_t);
    read_data[4] = FLASH_Read(1, 4, uint8_t);
    read_data[5] = FLASH_Read(1, 5, uint8_t);
    read_data[6] = FLASH_Read(1, 6, uint8_t);
    read_data[7] = FLASH_Read(1, 7, uint8_t);
    read_data[8] = FLASH_Read(1, 8, uint8_t);
    read_data[9] = FLASH_Read(1, 9, uint8_t);
    
    /* 从倒数第2个扇区 0偏移位置开始 读出数据 */
	FLASH_ReadBuff(2, 0, sizeof(write_float), (char *)read_float);
   

	for(int i = 0; i < 10; i++)
    {
        if(read_data[i] != write_data[i])
        {
            printf("第 %d 个数据出错 \n", i);
            printf("Flash is fail \n");
        }
    }

	for(int i = 0; i < 10; i++)
    {
        if(write_float[i] != read_float[i])
        {
            printf("第 %d 个数据出错 \n", i);
            printf("Flash is fail \n");
        }
    }
    
    sprintf((char*)txt,"uint8_t:%2d",read_data[9]);
    OLED_P6x8Str(0,2,(u8*)txt);                     //OLED显示写入后再读出的第10个整型数据
    
    sprintf((char*)txt,"float:%6.3f",read_float[4]);
    OLED_P6x8Str(0,3,(u8*)txt);                     //OLED显示写入后再读出的第5个浮点数据
    
    printf("\r\nFlash 读写成功 \n");    
        
	while(1);

}
