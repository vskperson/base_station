#ifndef __NRF24_HAL_H
#define __NRF24_HAL_H


// Hardware abstraction layer for NRF24L01+ transceiver (hardware depended functions)
// GPIO pins definition
// GPIO pins initialization and control functions
// SPI transmit functions


// Peripheral libraries
#include <stm32f0xx_gpio.h>
#include <stm32f0xx_spi.h>

// CE (chip enable) pin
#define nRF24_CE_PORT              GPIOA
#define nRF24_CE_PIN               GPIO_Pin_8
#define nRF24_CE_L()               GPIO_ResetBits(GPIOA, nRF24_CE_PIN)
#define nRF24_CE_H()               GPIO_SetBits(GPIOA, nRF24_CE_PIN)

// CSN (chip select negative) pin
#define nRF24_CSN_PORT             GPIOA
#define nRF24_CSN_PIN              GPIO_Pin_11
#define nRF24_CSN_L()              GPIO_ResetBits(GPIOA, nRF24_CSN_PIN)
#define nRF24_CSN_H()              GPIO_SetBits(GPIOA, nRF24_CSN_PIN)

// IRQ pin (PB10)
#define nRF24_IRQ_PORT             GPIOF
#define nRF24_IRQ_PIN              GPIO_Pin_6

// SPI port peripheral
#define nRF24_SPI_PORT             SPI2


// Function prototypes
void nRF24_GPIO_Init(void);
uint8_t nRF24_LL_RW(uint8_t data);

#endif // __NRF24_HAL_H
