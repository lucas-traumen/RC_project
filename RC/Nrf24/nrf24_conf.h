#ifndef NRF24_CONF_H
#define NRF24_CONF_H

#include "stm32f1xx_hal.h"

// Định nghĩa chân GPIO cho NRF24
#define NRF_CSN_PORT    GPIOA
#define NRF_CSN_PIN     GPIO_PIN_8

#define NRF_CE_PORT     GPIOA
#define NRF_CE_PIN      GPIO_PIN_9

// Timeout mặc định cho SPI DMA (ms)
#define NRF_DMA_TIMEOUT_MS  50

// Kích thước buffer tối đa
#define NRF_MAX_PAYLOAD_SIZE  32

#endif // NRF24_CONF_H

