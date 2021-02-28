#include "stm32f0xx.h"
#include "stm32f0xx_tim.h"
#include "stm32f0xx_adc.h"
#include "stm32f0xx_exti.h"
#include "perif.h"
#include "nrf24.h"
#include "delay.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"

#define F_CPU 		8000000UL
#define TimerTick	F_CPU/1000-1
#define	ERROR_ACTION(CODE,POS)		do{}while(0)

QueueHandle_t xQueueFromADC, xQueueFromUART, xQueueFromRadio, xQueueFromRadioHndl;
TimerHandle_t xTimer[3];
SemaphoreHandle_t xSemaphore[3], xCNTSemaphore;
//uint8_t debugBuffer[400];

uint16_t ADCbuf[3]={1000, 1000, 1000};
struct Volts {
				 uint16_t Button;
				 uint16_t Battery;
				 uint16_t DCDC_out;
       	     };
struct Volts ADC_volts, Ready_volts;

uint8_t payload_length;									// Length of received payload
uint8_t nRF24_payload[32];								// Buffer to store a payload of maximum width
nRF24_RXResult pipe;									// Pipe number
nRF24_TXResult tx_res;
uint8_t nRF24_STATE=nRF24_STATE_RX;
volatile uint8_t nRF24_ADDR[] = { 0xE7, 0x1C, 0xE3 };	// Configure TX PIPE
uint8_t tmpDev[3][4]={									//addr0, addr1, addr2, lockFLAG. Total three devices (three lines of the massive)
						{0,0,0,0},
						{0,0,0,0},
						{0,0,0,0}
					 };
typedef struct {
                 uint8_t Addr0;
                 uint8_t Addr1;
                 uint8_t Addr2;
                 uint8_t dev_state;
                 uint8_t serv_byte;
               } BOBBER;
BOBBER bobFromRadio, bobFromQueue;

uint8_t devOffCNT=1, flg=0;

//----------------------TASKS---------------------
void vBuzzer (void *pvParameters)
{
	while(1)
	{
		xSemaphoreTake(xCNTSemaphore, portMAX_DELAY);

		TIM16->CNT = 0;
		TIM16->ARR=1000;
		TIM16->CCR1=500;
		TIM16->CCER |= (uint16_t)(5);					//turn on buzzer

		vTaskDelay(300);

		TIM16->CNT = 0;
		TIM16->ARR=500;
		TIM16->CCR1=250;

		vTaskDelay(200);

		TIM16->CCER &=~ (uint16_t)(5);					//turn off buzzer
	}
}

void vBlinker (void *pvParameters)
{
	uint8_t color=0;

	while(1)
	{
		 xQueueReceive(xQueueFromRadioHndl, &color, portMAX_DELAY);
		 xSemaphoreGive(xCNTSemaphore);

		 GPIO_SetBits(GPIOB, GPIO_Pin_7);
		 vTaskDelay(150);
		 GPIO_ResetBits(GPIOB, GPIO_Pin_7);
		 vTaskDelay(150);
		 GPIO_SetBits(GPIOB, GPIO_Pin_7);
		 vTaskDelay(150);
		 GPIO_ResetBits(GPIOB, GPIO_Pin_7);
		 vTaskDelay(150);
		 GPIO_SetBits(GPIOB, GPIO_Pin_7);
		 vTaskDelay(150);
		 GPIO_ResetBits(GPIOB, GPIO_Pin_7);
	}
}

void vDisabler (void *pvParameters)
{
	while(1)
	{
		if(devOffCNT){
						devOffCNT=0;
						vTaskDelay(50);
					}
		else{
						devOffCNT=1;
						vTaskDelay(50);
			}
	}
}

void vADCHandler (void *pvParameters)
{
	while(1)
	{
		if(xQueueReceive(xQueueFromADC, &Ready_volts, (TickType_t) 10) == pdPASS)
	       {

	       }
		else{		//receive timeout

	 	 	}
	}
}

void vRADIOHandler (void *pvParameters)
{
	uint8_t LEDcolor;
	uint8_t cnt=0, devFound=0;

	while(1)
	{
		xQueueReceive(xQueueFromRadio, &bobFromQueue, portMAX_DELAY);

		if(bobFromQueue.dev_state & 0x04)
		{
			devFound=0;
			for(cnt=0;cnt<3;cnt++){ 					//device from the list?
					if((tmpDev[cnt][0]==bobFromQueue.Addr0) && (tmpDev[cnt][1]==bobFromQueue.Addr1) && (tmpDev[cnt][2]==bobFromQueue.Addr2))
									{
										xSemaphoreTake(xSemaphore[cnt], portMAX_DELAY);
										if(!tmpDev[cnt][3]){	//if device found but the lock is 0
																LEDcolor=bobFromQueue.dev_state & 0b00011000;
																if(uxQueueSpacesAvailable(xQueueFromRadioHndl))
																{
																	xQueueSendToBack(xQueueFromRadioHndl, &LEDcolor, NULL);
																}
																tmpDev[cnt][3]=1;
																xTimerStart(xTimer[cnt], 0);
														   }
										devFound=1;		//device found
										xSemaphoreGive(xSemaphore[cnt]);
										break;
									}
							  	  }
			if(!devFound){	//if device was not found, then write the address
							for(cnt=0;cnt<3;cnt++){ 					//check for empty slot
													if((tmpDev[cnt][0]==0) && (tmpDev[cnt][1]==0) && (tmpDev[cnt][2]==0))
													{
														tmpDev[cnt][0]=bobFromQueue.Addr0;
														tmpDev[cnt][1]=bobFromQueue.Addr1;
														tmpDev[cnt][2]=bobFromQueue.Addr2;
														LEDcolor=bobFromQueue.dev_state & 0b00011000;
														if(uxQueueSpacesAvailable(xQueueFromRadioHndl))
															{
																xQueueSendToBack(xQueueFromRadioHndl, &LEDcolor, NULL);
															}
														tmpDev[cnt][3]=1;
														xTimerStart(xTimer[cnt], 0);
														break;
													}
											  	  }
						 }

	 	}
	}
}

void vUARTHandler (void *pvParameters)
{
	uint8_t rx_byte=0;

	while(1)
	{
		xQueueReceive(xQueueFromUART, &rx_byte, portMAX_DELAY);
		switch(rx_byte){
		 						case '1':	USART_SendData(USART1,'A');
		 									UART_TXDelay;
		 									break;
		 						case '2':	USART_SendData(USART1,Ready_volts.Battery>>8);
		 									UART_TXDelay;
		 									USART_SendData(USART1,Ready_volts.Battery);
		 									UART_TXDelay;
		 									USART_SendData(USART1,Ready_volts.Button>>8);
		 									UART_TXDelay;
		 									USART_SendData(USART1,Ready_volts.Button);
		 									UART_TXDelay;
		 									USART_SendData(USART1,Ready_volts.DCDC_out>>8);
		 									UART_TXDelay;
		 									USART_SendData(USART1,Ready_volts.DCDC_out);
		 									UART_TXDelay;
		 									break;
		 						case '3':	USART_SendData(USART1,tmpDev[0][0]);
		 								 	UART_TXDelay;
		 								 	USART_SendData(USART1,tmpDev[0][1]);
		 								 	UART_TXDelay;
		 								 	USART_SendData(USART1,tmpDev[0][2]);
		 								 	UART_TXDelay;
		 								 	USART_SendData(USART1,tmpDev[1][0]);
		 								 	UART_TXDelay;
		 								 	USART_SendData(USART1,tmpDev[1][1]);
		 								 	UART_TXDelay;
		 								 	USART_SendData(USART1,tmpDev[1][2]);
		 								 	UART_TXDelay;
		 								 	break;
		 						case '4':
		 									break;
		 						default :   break;
		           }
	}
}

void vTOneCallbackFunction(TimerHandle_t xTmr)
{
	xSemaphoreTake(xSemaphore[0], (TickType_t) 50 );
	tmpDev[0][3]=0;
	xSemaphoreGive(xSemaphore[0]);
}

void vTTwoCallbackFunction(TimerHandle_t xTmr)
{
	xSemaphoreTake(xSemaphore[1], (TickType_t) 50 );
	tmpDev[1][3]=0;
	xSemaphoreGive(xSemaphore[1]);
}

void vTThreeCallbackFunction(TimerHandle_t xTmr)
{
	xSemaphoreTake(xSemaphore[2], (TickType_t) 50 );
	tmpDev[2][3]=0;
	xSemaphoreGive(xSemaphore[2]);
}
//------------------INTERRUPTS---------------------
void TIM15_IRQHandler(void)
{
	BaseType_t xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;

	if (TIM_GetITStatus(TIM15, TIM_IT_Update) != RESET)			//Compare Interrupt
	{
		if(DMA_GetFlagStatus(DMA1_FLAG_TC1)){					//check transfer complete flag
												ADC_volts.Button  =ADCbuf[0];
												ADC_volts.Battery =ADCbuf[1];
												ADC_volts.DCDC_out=ADCbuf[2];
												ADC_StartOfConversion(ADC1);		//Start new AD conversion
												xQueueOverwriteFromISR(xQueueFromADC, &ADC_volts, &xHigherPriorityTaskWoken); // can send pointers to save RAM!
												if(xHigherPriorityTaskWoken){
																				portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
																			}
							 	 	 	 	}

		TIM_ClearITPendingBit(TIM15, TIM_IT_Update);			// clear Compare Interrupt flag
	}
}

void EXTI4_15_IRQHandler(void)
{
  if(EXTI_GetITStatus(EXTI_Line6) != RESET)
  {
//	  pipe = nRF24_ReadPayload(nRF24_payload, &payload_length);	// Get a payload from the transceiver
	  pipe = nRF24_ReadPayload( &bobFromRadio.Addr0, &payload_length);
	  nRF24_ClearIRQFlags();
	  xQueueSendToBackFromISR(xQueueFromRadio, &bobFromRadio, NULL);
/*	  USART_SendData(USART1,bobFromRadio.Addr0);
	  UART_TXDelay;
	  USART_SendData(USART1,bobFromRadio.Addr1);
	  UART_TXDelay;
	  USART_SendData(USART1,bobFromRadio.Addr2);
	  UART_TXDelay;
	  USART_SendData(USART1,bobFromRadio.dev_state);
	  UART_TXDelay;
	  USART_SendData(USART1,bobFromRadio.serv_byte);
	  UART_TXDelay;
*/
	  EXTI_ClearITPendingBit(EXTI_Line6);	/* Clear the EXTI line pending bit */
  }
}

void USART1_IRQHandler(void)									//one vector with many USART interrupt sources (if they are selected)
{
	uint8_t RXc=0;

    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
    	RXc = USART_ReceiveData(USART1);
        xQueueSendToBackFromISR(xQueueFromUART, &RXc, NULL);
    }
}
//---------------------MAIN--------------------
int main(void)
{
	initIOAnalog();
	initIO();
	initSPI2();
	initUSART();
//	initTimer14();
	initTimer15();
	initTimer16();
	initADC(ADCbuf, 3);
	initEXTI();

	nRF24_CE_L();  							// RX/TX disabled
	nRF24_CSN_H();							// Chip select disable (SPI interface)

	Delay_Init();							// Initialize delay
	Delay_ms(5);

	if(nRF24_Check()){
						USART_SendData(USART1, 0xAA);	//OK
						UART_TXDelay;
	    			 }
	else{
	    	USART_SendData(USART1, 0xBB);  //FAIL
	    	UART_TXDelay;
	    }

	nRF24_Init();  								// Initialize the nRF24L01 to its default state

	nRF24_DisableAA(0xFF);						// Disable ShockBurst for all RX pipes
	nRF24_SetRFChannel(115);					// Set RF channel
	nRF24_SetDataRate(nRF24_DR_250kbps);		// Set data rate
	nRF24_IntDisable();							// Enable only RX_DR interrupt

	nRF24_SetCRCScheme(nRF24_CRC_2byte);		// Set CRC scheme
	nRF24_SetAddrWidth(3);						// Set address width, its common for all pipes (RX and TX)

	nRF24_SetAddr(nRF24_PIPETX, nRF24_ADDR); 	// program TX address
	nRF24_SetAddr(nRF24_PIPE0, nRF24_ADDR);		// program address for pipe#0, must be same as TX (for Auto-ACK)

	nRF24_SetTXPower(nRF24_TXPWR_0dBm);			// Set TX power (maximum)
	nRF24_SetAutoRetr(nRF24_ARD_2500us, 0);		//
//	nRF24_EnableAA(nRF24_PIPE0);				// Enable Auto-ACK for pipe#0 (for ACK packets)
	nRF24_SetOperationalMode(nRF24_MODE_TX);	// Set operational mode (PTX == transmitter)
	nRF24_ClearIRQFlags();						// Clear any pending IRQ flags
	nRF24_SetPowerMode(nRF24_PWR_UP);			// Wake the transceiver
	nRF24_STATE=nRF24_STATE_TX;

	nRF24_CE_L();
	nRF24_ReConfig_RX();

	xQueueFromADC = xQueueCreate( 1, sizeof(ADC_volts) );
	xQueueFromUART= xQueueCreate( 8, 1 );
	xQueueFromRadio = xQueueCreate( 6, sizeof(bobFromRadio) );
	xQueueFromRadioHndl = xQueueCreate( 3, 1 );

	if(pdTRUE != xTaskCreate(vBuzzer,"Buzzer", configMINIMAL_STACK_SIZE, NULL, 1, NULL)) ERROR_ACTION(TASK_NOT_CREATE,0);
	if(pdTRUE != xTaskCreate(vBlinker,"Blinker", configMINIMAL_STACK_SIZE, NULL, 1, NULL)) ERROR_ACTION(TASK_NOT_CREATE,0);
	if(pdTRUE != xTaskCreate(vDisabler,"Disabler", configMINIMAL_STACK_SIZE, NULL, 1, NULL)) ERROR_ACTION(TASK_NOT_CREATE,0);
	if(pdTRUE != xTaskCreate(vADCHandler,"ADC_hndl", 128, NULL, 3, NULL)) ERROR_ACTION(TASK_NOT_CREATE,0);
	if(pdTRUE != xTaskCreate(vUARTHandler,"UART_hndl", 128, NULL, 1, NULL)) ERROR_ACTION(TASK_NOT_CREATE,0);
	if(pdTRUE != xTaskCreate(vRADIOHandler,"RADIO_hndl", 128, NULL, 2, NULL)) ERROR_ACTION(TASK_NOT_CREATE,0);

	xTimer[0] = xTimerCreate("Tmr1", 102, pdFALSE, ( void * ) 0, vTOneCallbackFunction);
	xTimer[1] = xTimerCreate("Tmr2", 102, pdFALSE, ( void * ) 0, vTTwoCallbackFunction);
	xTimer[2] = xTimerCreate("Tmr3", 102, pdFALSE, ( void * ) 0, vTThreeCallbackFunction);

	xSemaphore[0] = xSemaphoreCreateMutex();
	xSemaphore[1] = xSemaphoreCreateMutex();
	xSemaphore[2] = xSemaphoreCreateMutex();

	xCNTSemaphore = xSemaphoreCreateCounting(3, 0);

//	TIM_Cmd(TIM14, ENABLE);
	TIM_Cmd(TIM15, ENABLE);
	TIM_Cmd(TIM16, ENABLE);
	ADC_StartOfConversion(ADC1);

//	vTaskList(debugBuffer);

	SysTick_Config(TimerTick);			//1000Hz, 1ms update
	vTaskStartScheduler();

	while(1){
			}
}
