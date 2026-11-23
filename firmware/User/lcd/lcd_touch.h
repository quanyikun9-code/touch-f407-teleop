#ifndef __LCD_TOUCH_H
#define	__LCD_TOUCH_H

#include <stdint.h>

enum TouchIC
{
   GTXXX  
};

typedef struct {
    uint16_t x;
    uint16_t y;
    uint8_t track_id;
} Touch_Point_t;


extern enum TouchIC   touch_ic;
extern uint8_t        touch_max_ponts;
extern uint8_t        touch_id_temp[5]; 
extern int8_t         touch_points_num;
extern Touch_Point_t  touch_points[5];


uint8_t LCD_Touch_Config(void);
uint8_t LCD_Touch_Get_Points_Data(Touch_Point_t* p_data,int8_t* p_num);    


#define EANBLE_LCD_TOUCH_DEBUG 0


#endif /* __LCD_TOUCH_H */
