#ifndef NRF24_CONF_H
#define NRF24_CONF_H

#include "stm32f1xx_hal.h"

// ========================
// GPIO mapping cho nRF24
// ========================
// Bạn đang dùng PA8 = CSN, PA9 = CE (giống cả TX/RX)
#define NRF_CSN_PORT    GPIOA
#define NRF_CSN_PIN     GPIO_PIN_8

#define NRF_CE_PORT     GPIOA
#define NRF_CE_PIN      GPIO_PIN_9

// ========================
// Tham số chung
// ========================

// Timeout mặc định cho 1 lần SPI DMA (ms)
#define NRF_DMA_TIMEOUT_MS      50U

// Payload tối đa cho nRF24
#define NRF_MAX_PAYLOAD_SIZE    32U

#endif // NRF24_CONF_H
