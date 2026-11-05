#ifndef __BSP_LCD_TP_I2C_H
#define	__BSP_LCD_TP_I2C_H

#include "stm32f4xx.h"

/*模拟I2C时钟引脚*/
#define TP_I2C_SCL_PIN            GPIO_PIN_7                 
#define TP_I2C_SCL_GPIO_PORT      GPIOD   
/*模拟I2C数据引脚*/
#define TP_I2C_SDA_PIN            GPIO_PIN_3                 
#define TP_I2C_SDA_GPIO_PORT      GPIOD 

/* SCL = 1 */
#define TP_I2C_SCL_1()            HAL_GPIO_WritePin(TP_I2C_SCL_GPIO_PORT, TP_I2C_SCL_PIN,GPIO_PIN_SET)
/* SCL = 0 */
#define TP_I2C_SCL_0()            HAL_GPIO_WritePin(TP_I2C_SCL_GPIO_PORT, TP_I2C_SCL_PIN,GPIO_PIN_RESET)	
/* SDA = 1 */
#define TP_I2C_SDA_1()            HAL_GPIO_WritePin(TP_I2C_SDA_GPIO_PORT, TP_I2C_SDA_PIN,GPIO_PIN_SET)	
/* SDA = 0 */
#define TP_I2C_SDA_0()            HAL_GPIO_WritePin(TP_I2C_SDA_GPIO_PORT, TP_I2C_SDA_PIN,GPIO_PIN_RESET)	
/* 读SDA口线状态 */
#define TP_I2C_SDA_READ()         HAL_GPIO_ReadPin(TP_I2C_SDA_GPIO_PORT, TP_I2C_SDA_PIN)	
/* 模拟I2C延时方式 */
#define TP_I2C_DELAY              DWT_DelayUs(2)

void TP_I2C_Start(void);
void TP_I2C_Stop(void);
void TP_I2C_Ack(void);
void TP_I2C_NAck(void);
void TP_I2C_SendByte(uint8_t byte);
uint8_t TP_I2C_ReadByte(void);
uint8_t TP_I2C_WaitAck(void);

void TP_I2C_Config(void);




/*触摸芯片复位引脚*/
#define TP_RST_GPIO_PIN             GPIO_PIN_6
#define TP_RST_GPIO_PORT            GPIOD
/*触摸芯片信号引脚*/
#define TP_INT_GPIO_PIN             GPIO_PIN_8
#define TP_INT_GPIO_PORT            GPIOG

/*使用电容触摸屏时该脚未使用*/
/* PG9  Not used */
              
void TP_Rest(void);
void TP_INT_ENABLE(uint8_t state);

#endif /* __BSP_LCD_TP_I2C_H */
