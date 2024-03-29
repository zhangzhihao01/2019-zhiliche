/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
【平    台】北京龙邱智能科技MK66FX1M0VLQ18核心板
【编    写】CHIUSIR
【备    注】
【软件版本】V1.0
【最后更新】2016年08月20日
【相关信息参考下列地址】
【网    站】http://www.lqist.cn
【淘宝店铺】http://shop36265907.taobao.com
【交流邮箱】chiusir@163.com
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/

#ifndef INCLUDE_H_
#define INCLUDE_H_

//通用头文件
    #include    <stdio.h>                       //printf
    #include    <string.h>                      //memcpy
    #include    <stdlib.h>                      //malloc

//Cortex-M内核MCU寄存器头文件
    #include "MK66F18.h"   //寄存器映像头文件
    #include "arm_math.h "
    #include "Systick.h"

//MCU内部模块驱动的头文件
    #include "GPIO.h"
    #include "GPIO_Cfg.h"
    #include "PLL.h"
    #include "FTM.h"    
    #include "UART.h"
    #include "ADC.h"
    #include "PLL.h"    
    #include "PIT.h"
    #include "I2C.h"
    //#include "SPI.h"
    #include "DMA.h"
    #include "Lptmr.h"    
    #include "RTC.h"
    #include "LQ_SGP18T.h"
    #include "MK60_FLASH.h"
//中断向量表及中断函数声明
    #include "vectors.h"

//数据类型及端口名称重定义
    #include "common.h"

//外部设备及自定义功能驱动的头文件

    #include "LQLED.h"
    #include "LQKEY.h"
    #include "TSL1401.h"
    #include "LQ12864.h"
    #include "LQIIC.h"
    #include "dianci.h"
    #include "LQ12864.h"

    #include "LQcontrol.h"
    #include "LQMPU6050.h"
    #include "SPI.h"

    #include "LQMT9V034.h"
    #include "VL53L0.h"
    #include "NRF2401.h"
    #include "SEEKFREE_ICM20602.h"


#endif