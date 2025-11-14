#ifndef NRF24_SPI_DMA_H
#define NRF24_SPI_DMA_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

// Khởi tạo SPI cho driver nRF24 (gắn con trỏ SPI dùng chung)
void NRF24_SPI_DMA_Init(SPI_HandleTypeDef *hspi);

// Truyền SPI full-duplex qua DMA (len byte)
HAL_StatusTypeDef nrf24_spi_transfer_dma(uint8_t *tx_buf,
                                         uint8_t *rx_buf,
                                         uint16_t len,
                                         uint32_t timeout_ms);

// Kiểm tra driver có đang bận 1 transaction trước không
uint8_t nrf24_spi_dma_busy(void);

// Callback: phải được gọi khi DMA SPI hoàn thành
// -> Thường được gọi từ HAL_SPI_TxRxCpltCallback
void nrf24_spi_dma_complete_cb(void);

#endif // NRF24_SPI_DMA_H
