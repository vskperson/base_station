#ifndef PERIF_H
#define PERIF_H

#include "stm32f0xx.h"
#include "stm32f0xx_gpio.h"
#include "stm32f0xx_rcc.h"
#include "stm32f0xx_usart.h"
#include "stm32f0xx_tim.h"
#include "stm32f0xx_spi.h"
#include "stm32f0xx_misc.h"
#include "stm32f0xx_adc.h"
#include "stm32f0xx_dma.h"
#include "stm32f0xx_exti.h"
#include "stm32f0xx_syscfg.h"
#include "nrf24_hal.h"
#include "nrf24.h"

#define UART_TXDelay while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET){}

#define ADC1_DR_Address                0x40012440

void initIOAnalog(void);
void initIO(void);
void initTimer14(void);
void initTimer15(void);
void initTimer16(void);
void initTimer17(void);
void initUSART(void);
void initSPI2(void);
void initADC(uint16_t* buffer, uint32_t size);
void initEXTI();

#endif
