/**
  ******************************************************************************
  * @file    bsp_lcd_tp_i2c.c
  * @author  embedfire
  * @brief   
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */

#include "bsp_lcd_tp_i2c.h"
#include "bsp_dwt.h"

void TP_I2C_Config(void)
{
     /*     引脚使用模拟I2C
        PD7   ------>   I2C_SCL        
        PD3   ------>   I2C_SDA
    */        
    
    GPIO_InitTypeDef GPIO_InitStruct ={0};        
    
    __HAL_RCC_GPIOD_CLK_ENABLE();
    
    GPIO_InitStruct.Pin = TP_I2C_SCL_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(TP_I2C_SCL_GPIO_PORT, &GPIO_InitStruct);   
 
    GPIO_InitStruct.Pin = TP_I2C_SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(TP_I2C_SDA_GPIO_PORT, &GPIO_InitStruct);            
}


/*
*********************************************************************************************************
*	函 数 名: TP_I2C_Start
*	功能说明: CPU发起I2C总线启动信号
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void TP_I2C_Start(void)
{
	/* 当SCL高电平时，SDA出现一个下跳沿表示I2C总线启动信号 */
	TP_I2C_SDA_1();
	TP_I2C_SCL_1();
	TP_I2C_DELAY;
	TP_I2C_SDA_0();
	TP_I2C_DELAY;
	TP_I2C_SCL_0();
	TP_I2C_DELAY;
}

/*
*********************************************************************************************************
*	函 数 名: TP_SW_I2C_Stop
*	功能说明: CPU发起I2C总线停止信号
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void TP_I2C_Stop(void)
{
	/* 当SCL高电平时，SDA出现一个上跳沿表示I2C总线停止信号 */
	TP_I2C_SDA_0();
	TP_I2C_SCL_1();
	TP_I2C_DELAY;
	TP_I2C_SDA_1();
    TP_I2C_DELAY;
}

/*
*********************************************************************************************************
*	函 数 名: TP_I2C_SendByte
*	功能说明: CPU向I2C总线设备发送8bit数据
*	形    参：byte ： 等待发送的字节
*	返 回 值: 无
*********************************************************************************************************
*/
void TP_I2C_SendByte(uint8_t byte)
{	
	for (uint8_t i = 0; i < 8; i++)/* 先发送字节的高位bit7 */
	{		
		if (byte & 0x80)
		{
			TP_I2C_SDA_1();
		}
		else
		{
			TP_I2C_SDA_0();
		}
		TP_I2C_DELAY;
		TP_I2C_SCL_1();
		TP_I2C_DELAY;	
		TP_I2C_SCL_0();
		if (i == 7)
		{          
			 TP_I2C_SDA_1(); /* 释放总线 */
		}
		byte <<= 1;	/* 左移一个bit */
		TP_I2C_DELAY;
	}
}

/*
*********************************************************************************************************
*	函 数 名: TP_I2C_ReadByte
*	功能说明: CPU从I2C总线设备读取8bit数据
*	形    参：无
*	返 回 值: 读到的数据
*********************************************************************************************************
*/
uint8_t TP_I2C_ReadByte(void)
{
	uint8_t value = 0;

	/* 读到第1个bit为数据的bit7 */
	for (uint8_t i = 0; i < 8; i++)
	{
		value <<= 1;
		TP_I2C_SCL_1();
		TP_I2C_DELAY;
		if (TP_I2C_SDA_READ())
		{
			value++;
		}
		TP_I2C_SCL_0();
		TP_I2C_DELAY;
	}
	return value;
}

/*
*********************************************************************************************************
*	函 数 名: TP_I2C_WaitAck
*	功能说明: CPU产生一个时钟，并读取器件的ACK应答信号
*	形    参：无
*	返 回 值: 返回0表示正确应答，1表示无器件响应
*********************************************************************************************************
*/
uint8_t TP_I2C_WaitAck(void)
{
    uint16_t   timeout = 0;
    
	TP_I2C_SDA_1();	/* CPU释放SDA总线 */
	TP_I2C_DELAY;
	TP_I2C_SCL_1();	/* CPU驱动SCL = 1, 此时器件会返回ACK应答 */
	TP_I2C_DELAY;
    
    while( TP_I2C_SDA_READ() == 1)
    {  
        timeout++;
        if(timeout == 600)
        {
            return 1;
        }      
    }
  
	TP_I2C_SCL_0();   
	TP_I2C_DELAY;
	return 0;
}

/*
*********************************************************************************************************
*	函 数 名: TP_I2C_Ack
*	功能说明: CPU产生一个ACK信号
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void TP_I2C_Ack(void)
{
	TP_I2C_SDA_0();	/* CPU驱动SDA = 0 */
	TP_I2C_DELAY;
	TP_I2C_SCL_1();	/* CPU产生1个时钟 */
	TP_I2C_DELAY;
	TP_I2C_SCL_0();
	TP_I2C_DELAY;
	TP_I2C_SDA_1();	/* CPU释放SDA总线 */
    TP_I2C_DELAY;
}

/*
*********************************************************************************************************
*	函 数 名: TP_I2C_NAck
*	功能说明: CPU产生1个NACK信号
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void TP_I2C_NAck(void)
{
	TP_I2C_SDA_1();	/* CPU驱动SDA = 1 */
	TP_I2C_DELAY;
	TP_I2C_SCL_1();	/* CPU产生1个时钟 */
	TP_I2C_DELAY;
	TP_I2C_SCL_0();
	TP_I2C_DELAY;	
}

/*******************************************************************************************************/


void TP_Rest(void)
{
    /*     
        PD6  ------>    TP_RST 
    */      

    GPIO_InitTypeDef GPIO_InitStruct ={0};    

    __HAL_RCC_GPIOD_CLK_ENABLE();
       
    /* 触摸芯片复位低电平有效，可将其上拉 */
    GPIO_InitStruct.Pin = TP_RST_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(TP_RST_GPIO_PORT, &GPIO_InitStruct);
    
    
    HAL_GPIO_WritePin(TP_RST_GPIO_PORT, TP_RST_GPIO_PIN ,GPIO_PIN_RESET);
	HAL_Delay ( 50 ); 					   

	HAL_GPIO_WritePin(TP_RST_GPIO_PORT, TP_RST_GPIO_PIN ,GPIO_PIN_SET);		 
	HAL_Delay ( 50 ); 	    
}

void TP_INT_ENABLE(uint8_t state)
{ 
    /* 
        PG8 ------>   TP_INT 
          
        当使用触摸信号中断，会尝试高/低电平，电平由芯片具体当前配置而定
        例程与触摸芯片默认配置中，低电平有效，可将其上拉，
     
        该函数可以表示为启用或者屏蔽触摸中断响应
    */    
    if(state == 1)
    {     
        GPIO_InitTypeDef GPIO_InitStruct ={0};        

        __HAL_RCC_GPIOG_CLK_ENABLE();
                                    
        GPIO_InitStruct.Pin = TP_INT_GPIO_PIN;     
        GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING; 
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        HAL_GPIO_Init(TP_INT_GPIO_PORT, &GPIO_InitStruct);
                   
        HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);   
                  
    }
    else
    {       
        HAL_GPIO_DeInit(TP_INT_GPIO_PORT,TP_INT_GPIO_PIN);         
    }   
}


/*********************************************END OF FILE**********************/
