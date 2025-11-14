#ifndef NRF24_SPI_DMA_H
#define NRF24_SPI_DMA_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

// Khởi tạo SPI DMA
void NRF24_SPI_DMA_Init(SPI_HandleTypeDef *hspi);

// Transfer data qua SPI DMA (full-duplex)
HAL_StatusTypeDef nrf24_spi_transfer_dma(uint8_t *tx_buf, uint8_t *rx_buf, 
                                          uint16_t len, uint32_t timeout_ms);

// Kiểm tra SPI có đang bận không
uint8_t nrf24_spi_dma_busy(void);

// Callback khi DMA hoàn thành (gọi từ HAL_SPI_TxRxCpltCallback)
void nrf24_spi_dma_complete_cb(void);

#endif // NRF24_SPI_DMA_H


