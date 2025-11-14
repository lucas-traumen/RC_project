#include "nrf24_spi_dma.h"
#include <string.h>

static SPI_HandleTypeDef *nrf_hspi = NULL;
static volatile uint8_t spi_dma_done_flag = 0;

// Buffer tĩnh cho dummy data
static uint8_t tx_dummy[256];
static uint8_t rx_dummy[256];

void NRF24_SPI_DMA_Init(SPI_HandleTypeDef *hspi)
{
    if (hspi == NULL) return;
    nrf_hspi = hspi;
    spi_dma_done_flag = 1; // sẵn sàng
}

uint8_t nrf24_spi_dma_busy(void)
{
    return (spi_dma_done_flag == 0);
}

void nrf24_spi_dma_complete_cb(void)
{
    spi_dma_done_flag = 1;
}

HAL_StatusTypeDef nrf24_spi_transfer_dma(uint8_t *tx_buf, uint8_t *rx_buf,
                                         uint16_t len, uint32_t timeout_ms)
{
    if (nrf_hspi == NULL || len == 0) return HAL_ERROR;
    if (len > 256) return HAL_ERROR;

    // Chuẩn bị buffer
    uint8_t *use_tx = tx_buf;
    uint8_t *use_rx = rx_buf;

    if (tx_buf == NULL) { memset(tx_dummy, 0xFF, len); use_tx = tx_dummy; }
    if (rx_buf == NULL) { use_rx = rx_dummy; }

    // Đợi SPI sẵn sàng
    uint32_t start = HAL_GetTick();
    while (nrf24_spi_dma_busy()) {
        if ((HAL_GetTick() - start) > timeout_ms) return HAL_TIMEOUT;
    }

    // Bắt đầu truyền
    spi_dma_done_flag = 0;
    HAL_StatusTypeDef st = HAL_SPI_TransmitReceive_DMA(nrf_hspi, use_tx, use_rx, len);
    if (st != HAL_OK) { spi_dma_done_flag = 1; return st; }

    // Chờ hoàn tất hoặc timeout
    start = HAL_GetTick();
    while (!spi_dma_done_flag) {
        if ((HAL_GetTick() - start) > timeout_ms) {
            HAL_SPI_Abort(nrf_hspi);
            spi_dma_done_flag = 1;
            return HAL_TIMEOUT;
        }
    }

    // Copy nếu cần
    if (rx_buf && use_rx == rx_dummy) memcpy(rx_buf, rx_dummy, len);
    return HAL_OK;
}





