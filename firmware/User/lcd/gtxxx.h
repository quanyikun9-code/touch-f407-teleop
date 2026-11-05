#ifndef __GTXXX_H
#define	__GTXXX_H

#include <stdint.h>

/* 例程中使用到的部分通用寄存器定义  */
#define GT_REG_PRODUCT_ID               0X8140      /* GT Product ID寄存器，起始地址 */
#define GT_REG_STATUS                   0X814E      /* GT 坐标信息的触摸状态、点数等 */
#define GT_REG_PONINTS                  0X814F      /* GT 第一个坐标信息，起始地址 */

#define GT_REG_STATUS_POINTS_NUM(x)      ((x) & 0xFU)          /* 取出GT_REG_STATUS寄存器值的触摸点数 */
#define GT_REG_STATUS_BUFFER(x)         (((x) & 0x80U) >> 7)   /* 取出GT_REG_STATUS寄存器值的坐标就绪标志 */

#define GT_ADDR_WRITE(ADDR)             (ADDR | 0)  /* GT 设备地址定义最后一位0为写 */
#define GT_ADDR_READ(ADDR)              (ADDR | 1)  /* GT 设备地址定义最后一位1为读 */

#define GT_PRODUCT_ID_SIZE              4           /* GT Product ID一般为4个字节 */
#define GT_PRODUCT_MAX_POINTS           10          /* GT 常见支持5点或者最大10点 */


typedef struct {
    uint8_t track_id;
    uint8_t x_l;
    uint8_t x_h;
    uint8_t y_l;
    uint8_t y_h;
    uint8_t size_l;
    uint8_t size_h;
    uint8_t reserved;
} GT_Reg_Point_t;




uint8_t Gt_Init(void);
void    Gt_Rest(void);
void    Gt_INT_ENABLE(uint8_t state);

uint8_t Gt_Reg_Write(uint8_t addr,uint16_t reg,uint8_t *buf,uint8_t size);
uint8_t Gt_Reg_Read(uint8_t addr,uint16_t reg,uint8_t *buf,uint8_t size);

uint8_t Gt_Get_Max_Points(void);
void    Gt_Get_ID_Temp(uint8_t* buf);

uint8_t Gt_Get_Points_Data(GT_Reg_Point_t* gt_reg_points,int8_t* p_num);


#endif /* __GTXXX_H */
