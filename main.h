#ifndef __MAIN_H
#define __MAIN_H

#include "stm32f4xx.h"

/* IRQ identification */
typedef enum {
	USART1_IRQ = 0, USART2_IRQ, USART3_IRQ, UART4_IRQ, UART5_IRQ, USART6_IRQ
} IRQ_PPP_Type;

typedef struct {
	uint8_t sec0;
	uint8_t sec1;
	uint8_t min0;
	uint8_t min1;
	uint8_t hour0;
	uint8_t hour1;
	uint8_t day0;
	uint8_t day1;
	uint8_t month0;
	uint8_t month1;
	uint8_t year0;
	uint8_t year1;
} DATE_TYPE;

extern uint32_t SystemTicks;

void Delay_ms(volatile uint32_t nTime);
void TimingDelay_Decrement(void);
void DispatchIRQ(IRQ_PPP_Type irq);

#endif
