#ifndef NRF24_SPI_DMA_H
#define NRF24_SPI_DMA_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

void NRF24_SPI_DMA_Init(SPI_HandleTypeDef *hspi);

/* HÀM MỚI: Dùng SPI Polling (blocking) - AN TOÀN */
HAL_StatusTypeDef nrf24_spi_transfer_polling(uint8_t *tx_buf,
                                             uint8_t *rx_buf,
                                             uint16_t len,
                                             uint32_t timeout_ms);

/* HÀM CŨ: Dùng SPI DMA (Chúng ta sẽ không gọi hàm này nữa) */
HAL_StatusTypeDef nrf24_spi_transfer_dma(uint8_t *tx_buf,
                                         uint8_t *rx_buf,
                                         uint16_t len,
                                         uint32_t timeout_ms);

uint8_t nrf24_spi_dma_busy(void);
void nrf24_spi_dma_complete_cb(void);

/* HÀM MỚI: Hủy transaction DMA đang chạy */
void NRF24_SPI_DMA_Abort(void);

#endif // NRF24_SPI_DMA_H
