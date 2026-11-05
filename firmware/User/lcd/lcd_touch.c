/**
  ******************************************************************************
  * @file    lcd_touch.c
  * @author  embedfire
  * @brief   
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */
  
#include <stdio.h>
#include "gtxxx.h"
#include "bsp_lcd_tp_i2c.h"
#include "lcd_touch.h"

enum TouchIC   touch_ic;
uint8_t        touch_max_ponts;
uint8_t        touch_id_temp[5]; 
int8_t         touch_points_num;
Touch_Point_t  touch_points[5];

GT_Reg_Point_t gt_reg_points[GT_PRODUCT_MAX_POINTS];

uint8_t LCD_Touch_Config(void)
{        
    uint8_t ret = 0;
        
    ret = Gt_Init();
      
    if(ret == 0)
    {      
        touch_ic = GTXXX;       
        touch_max_ponts = Gt_Get_Max_Points();
        Gt_Get_ID_Temp(touch_id_temp);
 
        printf("GT ID : %s \r\n",touch_id_temp);
        printf("Max ponts num : %d\r\n",touch_max_ponts);     
    }
    else
    {  
       printf("LCD_Touch_Init Error ret %d \r\n",ret);        
    }  

    return ret;
}



void LCD_Touch_Point_Debug(Touch_Point_t* p_data,int8_t points_num)
{
    #if EANBLE_LCD_TOUCH_DEBUG
    
    if(points_num > 0)
    {
        printf("Points : %d \r\n",points_num);  
        
        for(int i = 0; i < points_num; i++)
        {                  
                printf("P %d: ",i+1);
                printf("X %d  ",p_data[i].x);
                printf("Y %d  ",p_data[i].y); 
                printf("\r\n");               
        } 
        printf("\r\n\r\n"); 
    }
    
    #endif    
}



uint8_t LCD_Touch_Get_Points_Data(Touch_Point_t* p_data,int8_t* p_num)
{
   uint8_t ret = 0; 
       
   switch(touch_ic)
   {     
       case  GTXXX:
       {       
           ret = Gt_Get_Points_Data(gt_reg_points,p_num);
           
           if(ret != 0)
           {
               return ret;
           }          
            
           if (*p_num > 5)
           {
               *p_num = 5; /* touch_points has room for five coordinates. */
           }

           if(*p_num > 0)
           {
                for(int i = 0; i < *p_num; i++)
                {                  
                        p_data[i].x = (gt_reg_points[i].x_h & 0xF) << 8 | gt_reg_points[i].x_l;
                        p_data[i].y = (gt_reg_points[i].y_h & 0xF) << 8 | gt_reg_points[i].y_l;                 
                }                         
           } 
           
           LCD_Touch_Point_Debug(p_data,*p_num);
           
           break;               
       }
           
       default:
           break;      
   }   

   return ret;
}


void EXTI9_5_IRQHandler(void)
{   
    if(__HAL_GPIO_EXTI_GET_IT(TP_INT_GPIO_PIN) != RESET) //确保是否产生了EXTI Line中断
    {	
        LCD_Touch_Get_Points_Data(touch_points,&touch_points_num);
           
        __HAL_GPIO_EXTI_CLEAR_IT(TP_INT_GPIO_PIN);   //清除中断标志位  
    }             
}
/*********************************************END OF FILE**********************/
