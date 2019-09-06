/*-----------------------------------------------------------------------------------------------------
��ƽ    ̨������K60���İ�-���ܳ���
����    д��LQ-005
��E-mail  ��chiusir@163.com
������汾��V1.0������Դ���룬�����ο�������Ը�
�������¡�2019��04��02��
����    վ��http://www.lqist.cn
���Ա����̡�http://shop36265907.taobao.com
������ƽ̨��IAR 8.2
����    �ܡ�FLASH ��д����
��ע�������Ҫд��ǰ������  �Ḳ�ǳ���
-------------------------------------------------------------------------------------------------------*/
#include "include.h"



/*------------------------------------------------------------------------------------------------------
����    ����Test_Flash
����    �ܡ�����flash ��д
����    ������
���� �� ֵ����
��ʵ    ����Test_Flash(); 
��ע�����
--------------------------------------------------------------------------------------------------------*/
void Test_Flash(void)
{   
    char  txt[30];
	UART_Init(UART4, 115200);     //����4��ʼ��
    OLED_Init();                  //LCD��ʼ��
    OLED_CLS();                   //LCD����
	KEY_Init();
	FLASH_Init();                 //Flash��ʼ��
    OLED_P8x16Str(15,0,"LQ FLASH Test"); 
    printf("\r\nLQ FLASH Test");
	printf("\r\n��Ҫд��ǰ������  �Ḳ�ǳ��� \n");
	
    uint8_t  write_data[10] = {1,2,3,4,5,6,7,8,9,10};                   //д�������
    uint8_t  read_data[10]  = {0};
    
    float  write_float[10] = {1.0, 1.1, 1.2, 1.3, 100.156, 3.1415926, 1000.35, 20.7, 9.9, 10};   //д�������
    float  read_float[10] = {0};
    
	FLASH_EraseSector(1);
    FLASH_WriteBuf(1,write_data, 10, 0);//д������
//    printf("\r\n�� %d λ \n", sizeof(write_data));
    
    FLASH_EraseSector(2);
    FLASH_WriteBuf(2,(uint8_t *)write_float, sizeof(write_float), 0);//д������
//    printf("\r\n�� %d λ \n", sizeof(write_float));
    
    
	/* �ӵ�����һ������ 0ƫ��λ�ÿ�ʼ �������� */
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
    
    /* �ӵ�����2������ 0ƫ��λ�ÿ�ʼ �������� */
	FLASH_ReadBuff(2, 0, sizeof(write_float), (char *)read_float);
   

	for(int i = 0; i < 10; i++)
    {
        if(read_data[i] != write_data[i])
        {
            printf("�� %d �����ݳ��� \n", i);
            printf("Flash is fail \n");
        }
    }

	for(int i = 0; i < 10; i++)
    {
        if(write_float[i] != read_float[i])
        {
            printf("�� %d �����ݳ��� \n", i);
            printf("Flash is fail \n");
        }
    }
    
    sprintf((char*)txt,"uint8_t:%2d",read_data[9]);
    OLED_P6x8Str(0,2,(u8*)txt);                     //OLED��ʾд����ٶ����ĵ�10����������
    
    sprintf((char*)txt,"float:%6.3f",read_float[4]);
    OLED_P6x8Str(0,3,(u8*)txt);                     //OLED��ʾд����ٶ����ĵ�5����������
    
    printf("\r\nFlash ��д�ɹ� \n");    
        
	while(1);

}
