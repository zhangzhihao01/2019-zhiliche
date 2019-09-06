#include "include.h"
#include "myiic.h"

  	u8 val = 0;
        u8 gbuf[16];
	u8 DeviceRangeStatusInternal;
	uint32_t cnt = 0;
	uint16_t count[3];


        

uint16_t bswap(u8 b[])
{
	uint16_t val = ((b[0]<< 8) & b[1]);
	return val;
}

uint16_t VL53L0X_decode_vcsel_period(short vcsel_period_reg)
{
	uint16_t vcsel_period_pclks = (vcsel_period_reg + 1) <<1;
	return vcsel_period_pclks;
}

uint16_t makeuint16(int lsb, int msb)
{
    return ((msb & 0xFF) << 8) | (lsb & 0xFF);
}

//IIC连续写
//addr:器件地址 
//reg:寄存器地址
//len:写入长度
//buf:数据区
//返回值:0,正常
//    其他,错误代码
u8 VL53L0X_Write_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{
	u8 i; 
    m_IIC_Start(); 
	m_IIC_SendByte((addr<<1)|0);//发送器件地址+写命令	
	if(m_IIC_WaitAck())	//等待应答
	{
		m_IIC_Stop();		 
		return 1;		
	}
   m_IIC_SendByte(reg);	//写寄存器地址
    m_IIC_WaitAck();		//等待应答
	for(i=0;i<len;i++)
	{
		m_IIC_SendByte(buf[i]);	//发送数据
		if(m_IIC_WaitAck())		//等待ACK
		{
			m_IIC_Stop();	 
			return 1;		 
		}		
	}    
    m_IIC_Stop();	 
	return 0;	
} 
//IIC连续读
//addr:器件地址
//reg:要读取的寄存器地址
//len:要读取的长度
//buf:读取到的数据存储区
//返回值:0,正常
//    其他,错误代码
u8 VL53L0X_Read_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{ 
 	m_IIC_Start(); 
	m_IIC_SendByte((addr<<1)|0);//发送器件地址+写命令	
	if(m_IIC_WaitAck())	//等待应答
	{
		m_IIC_Stop();		 
		return 1;		
	}
    m_IIC_SendByte(reg);	//写寄存器地址
    m_IIC_WaitAck();		//等待应答
    m_IIC_Start();
	m_IIC_SendByte((addr<<1)|1);//发送器件地址+读命令	
    m_IIC_WaitAck();		//等待应答 
	while(len)
	{
		if(len==1)*buf=m_IIC_ReadByte(0);//读数据,发送nACK 
		else *buf=m_IIC_ReadByte(1);		//读数据,发送ACK  
		len--;
		buf++; 
	}    
    m_IIC_Stop();	//产生一个停止条件 
	return 0;	
}
//IIC写一个字节 
//reg:寄存器地址
//data:数据
//返回值:0,正常
//    其他,错误代码
u8 VL53L0X_Write_Byte(u8 reg,u8 data) 				 
{ 
  m_IIC_Start(); 
	m_IIC_SendByte((VL53L0X_Add<<1)|0);//发送器件地址+写命令	
	if(m_IIC_WaitAck())	//等待应答
	{
		m_IIC_Stop();		 
		return 1;		
	}
    m_IIC_SendByte(reg);	//写寄存器地址
    m_IIC_WaitAck();		//等待应答 
	m_IIC_SendByte(data);//发送数据
	if(m_IIC_WaitAck())	//等待ACK
	{
		m_IIC_Stop();	 
		return 1;		 
	}		 
    m_IIC_Stop();	 
	return 0;
}
//IIC读一个字节 
//reg:寄存器地址 
//返回值:读到的数据
u8 VL53L0X_Read_Byte(u8 reg)
{
	u8 res;
    m_IIC_Start(); 
	//IIC_Send_Byte((VL53L0X_Add<<1)|0);//发送器件地址+写命令	
	m_IIC_SendByte(0x52);//发送器件地址+写命令	
	m_IIC_WaitAck();		//等待应答 
    m_IIC_SendByte(reg);	//写寄存器地址
    m_IIC_WaitAck();		//等待应答
    m_IIC_Start();
	m_IIC_SendByte(0x53 );//发送器件地址+读命令	
    m_IIC_WaitAck();		//等待应答 
	res=m_IIC_ReadByte(0);//读取数据,发送nACK 
    m_IIC_Stop();			//产生一个停止条件 
	return res;		
}

        
void VL53L0X_init()
{
    m_IIC_Init();
    VL53L0X_Write_Byte(VL53L0X_REG_SYSRANGE_START, 0x01);
    while(cnt < 100)
       {
	   time_delay_ms(10);
	   val = VL53L0X_Read_Byte(VL53L0X_REG_RESULT_RANGE_STATUS);
	   if( val & 0x01) break;
                cnt++;
        }
    cnt=0;
}
        
void   VL53L0X_distence_get()
{
  
       VL53L0X_Write_Byte(VL53L0X_REG_SYSRANGE_START, 0x01);
       VL53L0X_Read_Len(VL53L0X_Add, 0x14 , 12, gbuf);
       count[2] = makeuint16(gbuf[11], gbuf[10]);

}


