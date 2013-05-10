#ifndef __MAIN_H
#define __MAIN_H

#include "stm32f4xx.h"

/* IRQ identification */
typedef enum {
	USART1_IRQ = 0,
	USART2_IRQ,
	USART3_IRQ,
	UART4_IRQ,
	UART5_IRQ,
	USART6_IRQ
}IRQ_PPP_Type;

extern uint32_t SystemTicks;

void Delay_ms(volatile uint32_t nTime);
void TimingDelay_Decrement(void);

#endif
