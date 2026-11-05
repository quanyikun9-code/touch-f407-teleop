#ifndef __BSP_SRAM_H
#define	__BSP_SRAM_H

#include "stm32f4xx.h"

#define Bank1_SRAM4_ADDR   ((uint32_t)(0x6C000000))	
#define IS62WV51216_SIZE 0x100000  

void SRAM_Config(void);
void SRAM_Pin_Init(void);
uint8_t SRAM_Test(void);

#endif /* __BSP_SRAM_H */
