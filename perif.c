#include "perif.h"

void initIOAnalog(void)
{
	  GPIO_InitTypeDef GPIO_InitStructure;

	  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	  GPIO_InitStructure.GPIO_Pin = 0xffff;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_3;
	  GPIO_Init(GPIOA, &GPIO_InitStructure);

	  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	  GPIO_Init(GPIOB, &GPIO_InitStructure);

	  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
	  GPIO_Init(GPIOC, &GPIO_InitStructure);

	  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOF, ENABLE);
	  GPIO_Init(GPIOF, &GPIO_InitStructure);
}

void initIO(void)
{
	  GPIO_InitTypeDef GPIO_InitStructure;

	  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_11;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_3;
	  GPIO_Init(GPIOA, &GPIO_InitStructure);

	  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_3;
	  GPIO_Init(GPIOB, &GPIO_InitStructure);

	  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOF, ENABLE);
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	  GPIO_Init(GPIOF, &GPIO_InitStructure);
/*
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_3;
	  GPIO_Init(GPIOF, &GPIO_InitStructure);
*/
}

void initUSART(void)
{
		//-----NVIC for USART
		NVIC_InitTypeDef NVIC_InitStructure;
		NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPriority = 2;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);

		GPIO_InitTypeDef GPIO_InitStructure;
		USART_InitTypeDef USART_InitStructure;

		USART_InitStructure.USART_BaudRate = 115200;
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;
		USART_InitStructure.USART_StopBits = USART_StopBits_1;
		USART_InitStructure.USART_Parity = USART_Parity_No;
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

		/* Enable GPIO clock */
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

		/* Enable USART clock */
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

		/* Connect PXx to USARTx_Tx */
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_1);

		/* Connect PXx to USARTx_Rx */
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_1);

		/* Configure USART Tx, Rx as alternate function push-pull */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;			//GPIO_Speed_Level_3
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; 			//GPIO_PuPd_UP
		GPIO_Init(GPIOA, &GPIO_InitStructure);

		/* USART configuration */
		USART_Init(USART1, &USART_InitStructure);

		/* Enable USART */
		USART_Cmd(USART1, ENABLE);

		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
}

void initTimer14(void)
{
		TIM_TimeBaseInitTypeDef TIMER_InitStructure;

	    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14, ENABLE);

	    TIM_TimeBaseStructInit(&TIMER_InitStructure);
	    TIMER_InitStructure.TIM_CounterMode = TIM_CounterMode_Up; 	// counting mode
	    TIMER_InitStructure.TIM_Prescaler =800; 					// keep in mind BUS prescalers
	    TIMER_InitStructure.TIM_Period = 10000;
	    TIM_TimeBaseInit(TIM14, &TIMER_InitStructure);

	    TIM_ClearITPendingBit(TIM14, TIM_IT_Update);				// clear Compare Interrupt flag
	    TIM_ITConfig(TIM14, TIM_IT_Update, ENABLE); 				// overflow interrupt

	    NVIC_InitTypeDef NVIC_InitStructure;
	    NVIC_InitStructure.NVIC_IRQChannel = TIM14_IRQn;
	    NVIC_InitStructure.NVIC_IRQChannelPriority = 2;				// 0(max)..3(min)
	    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	    NVIC_Init(&NVIC_InitStructure);

//	    TIM_Cmd(TIM14, ENABLE);
}

void initTimer15(void)
{
		TIM_TimeBaseInitTypeDef TIMER_InitStructure;

	    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM15, ENABLE);

	    TIM_TimeBaseStructInit(&TIMER_InitStructure);
	    TIMER_InitStructure.TIM_CounterMode = TIM_CounterMode_Up; 	// counting mode
	    TIMER_InitStructure.TIM_Prescaler =80; 					// keep in mind BUS prescalers
	    TIMER_InitStructure.TIM_Period = 100;
	    TIM_TimeBaseInit(TIM15, &TIMER_InitStructure);

	    TIM_ClearITPendingBit(TIM14, TIM_IT_Update);				// clear Compare Interrupt flag
	    TIM_ITConfig(TIM15, TIM_IT_Update, ENABLE); 				// overflow interrupt

	    NVIC_InitTypeDef NVIC_InitStructure;
	    NVIC_InitStructure.NVIC_IRQChannel = TIM15_IRQn;
	    NVIC_InitStructure.NVIC_IRQChannelPriority = 2;				// 0(max)..3(min)
	    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	    NVIC_Init(&NVIC_InitStructure);

//	    TIM_Cmd(TIM15, ENABLE);
}

void initTimer16(void)
{
	TIM_TimeBaseInitTypeDef TIMER_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM16, ENABLE);

    TIM_TimeBaseStructInit(&TIMER_InitStructure);
    TIMER_InitStructure.TIM_CounterMode = TIM_CounterMode_Up; 	// counting mode

    TIMER_InitStructure.TIM_Prescaler = 25; 					// keep in mind BUS prescalers
    TIMER_InitStructure.TIM_Period = 45;						//
    TIM_TimeBaseInit(TIM16, &TIMER_InitStructure);
    TIM_ITConfig(TIM16, TIM_IT_Update, ENABLE); 				// overflow interrupt

    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = TIM16_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority = 3;				// 0(max)..3(min)
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    //	    TIM_Cmd(TIM16, ENABLE);
}

void initTimer17(void)
{
	TIM_TimeBaseInitTypeDef TIMER_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM17, ENABLE);

    TIM_TimeBaseStructInit(&TIMER_InitStructure);
    TIMER_InitStructure.TIM_CounterMode = TIM_CounterMode_Up; 	// counting mode
    TIMER_InitStructure.TIM_Prescaler = 100; 					// keep in mind BUS prescalers
    TIMER_InitStructure.TIM_Period = 10;						//
    TIM_TimeBaseInit(TIM17, &TIMER_InitStructure);
    TIM_ITConfig(TIM17, TIM_IT_Update, ENABLE); 				// overflow interrupt

    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = TIM17_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority = 2;				// 0(max)..3(min)
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    //	    TIM_Cmd(TIM17, ENABLE);
}

void initSPI2(void)
{
	SPI_InitTypeDef SPI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

	/* SPI GPIO Configuration --------------------------------------------------*/

	/* Configure I/O for Flash Chip select */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* Connect SPI pins to AF0 */
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource13,GPIO_AF_0);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource15,GPIO_AF_0);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource14,GPIO_AF_0);

	/* SPI configuration */
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;

	SPI_Init(SPI2, &SPI_InitStructure);

	SPI_CalculateCRC(SPI2, DISABLE);

	SPI_NSSInternalSoftwareConfig(SPI2, SPI_NSSInternalSoft_Set);

	SPI_RxFIFOThresholdConfig(SPI2, SPI_RxFIFOThreshold_QF);

	SPI_Cmd(SPI2, ENABLE);

	/* drain SPI TX buffer,just in case*/
//	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET) {}
//	data=SPI_ReceiveData8(SPI2);
//	SPI_SendData8(SPI2, data);
}

void initADC(uint16_t* buffer, uint32_t size)
{
	ADC_InitTypeDef     ADC_InitStructure;
	DMA_InitTypeDef     DMA_InitStructure;
//	NVIC_InitTypeDef 	NVIC_InitStructure;

	ADC_DeInit(ADC1);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1 , ENABLE);

	/* DMA1 Channel1 Config */
	DMA_DeInit(DMA1_Channel1);

	ADC_GetCalibrationFactor(ADC1);

	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC1_DR_Address;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)buffer;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = size;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
//	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
//	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
//	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);

	/* DMA1 Channel1 enable */
	DMA_Cmd(DMA1_Channel1, ENABLE);

	/* Enable DMA1 Channel1 Transfer Complete interrupt */
//	DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);
	/* Enable DMA1 channel1 IRQ Channel */
//	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPriority = 1;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);

	ADC_DMARequestModeConfig(ADC1, ADC_DMAMode_OneShot);

	ADC_DMACmd(ADC1, ENABLE);

	ADC_StructInit(&ADC_InitStructure);

	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_ScanDirection = ADC_ScanDirection_Upward;
	ADC_Init(ADC1, &ADC_InitStructure);

//	ADC_VrefintCmd(ENABLE);

	ADC_ChannelConfig(ADC1, ADC_Channel_4, ADC_SampleTime_7_5Cycles);		//Button
	ADC_ChannelConfig(ADC1, ADC_Channel_5, ADC_SampleTime_7_5Cycles);		//Battery
	ADC_ChannelConfig(ADC1, ADC_Channel_6, ADC_SampleTime_7_5Cycles);		//Feedback
//	ADC_ChannelConfig(ADC1, ADC_Channel_17, ADC_SampleTime_7_5Cycles);		//Vref

//	ADC_AutoPowerOffCmd(ADC1, ENABLE);

	ADC_Cmd(ADC1, ENABLE);

	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_ADEN));
}


void initEXTI()
{
	  EXTI_InitTypeDef   EXTI_InitStructure;
	  NVIC_InitTypeDef   NVIC_InitStructure;

	  /* Enable SYSCFG clock */
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOF, EXTI_PinSource6);

	  /* Configure EXTI line */
	  EXTI_InitStructure.EXTI_Line = EXTI_Line6;
	  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	  EXTI_Init(&EXTI_InitStructure);

	  /* Enable and set EXTI Interrupt */
	  NVIC_InitStructure.NVIC_IRQChannel = EXTI4_15_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelPriority = 2;
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	  NVIC_Init(&NVIC_InitStructure);
}




