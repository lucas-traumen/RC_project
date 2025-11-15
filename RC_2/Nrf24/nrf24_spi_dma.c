#include "nrf24_spi_dma.h"
#include <string.h>

static SPI_HandleTypeDef *nrf_hspi = NULL;
static volatile uint8_t spi_dma_done_flag = 1;

static uint8_t tx_dummy[64] __attribute__((aligned(4)));
static uint8_t rx_dummy[64] __attribute__((aligned(4)));

void NRF24_SPI_DMA_Init(SPI_HandleTypeDef *hspi)
{
    if (hspi == NULL) return;
    nrf_hspi = hspi;
    spi_dma_done_flag = 1;
    HAL_SPI_Abort(nrf_hspi);
}

uint8_t nrf24_spi_dma_busy(void)
{
    return (spi_dma_done_flag == 0);
}

void nrf24_spi_dma_complete_cb(void)
{
    spi_dma_done_flag = 1;
}

void NRF24_SPI_DMA_Abort(void)
{
    if (nrf_hspi != NULL) {
        HAL_SPI_Abort(nrf_hspi);
    }
    spi_dma_done_flag = 1;
}

/*
 * HÀM MỚI: Triển khai SPI Polling (blocking) - AN TOÀN
 */
HAL_StatusTypeDef nrf24_spi_transfer_polling(uint8_t *tx_buf,
                                             uint8_t *rx_buf,
                                             uint16_t len,
                                             uint32_t timeout_ms)
{
    if (nrf_hspi == NULL) return HAL_ERROR;
    
    // Đảm bảo không có DMA nào đang chạy (dù không thể)
    uint32_t start = HAL_GetTick();
    while (nrf24_spi_dma_busy()) {
        if ((HAL_GetTick() - start) > timeout_ms) {
            NRF24_SPI_DMA_Abort();
            return HAL_TIMEOUT;
        }
    }
    // Dùng hàm SPI thường (blocking)
    return HAL_SPI_TransmitReceive(nrf_hspi, tx_buf, rx_buf, len, timeout_ms);
}

/*
 * HÀM CŨ: Triển khai SPI DMA (ĐÃ SỬA LỖI ĐỆ QUY)
 * (Vẫn để đây, nhưng chúng ta sẽ không dùng nó nữa)
 */
HAL_StatusTypeDef nrf24_spi_transfer_dma(uint8_t *tx_buf,
                                         uint8_t *rx_buf,
                                         uint16_t len,
                                         uint32_t timeout_ms)
{
    if (nrf_hspi == NULL) return HAL_ERROR;

    uint8_t *use_tx = tx_buf;
    uint8_t *use_rx = rx_buf;

    if (tx_buf == NULL && rx_buf == NULL) return HAL_ERROR;
    if (len > 64) len = 64; 

    if (tx_buf == NULL) {
        memset(tx_dummy, 0xFF, len); 
        use_tx = tx_dummy;
    }
    if (rx_buf == NULL) {
        use_rx = rx_dummy;
    }

    uint32_t start = HAL_GetTick();
    while (nrf24_spi_dma_busy()) {
        if ((HAL_GetTick() - start) > timeout_ms) {
            NRF24_SPI_DMA_Abort();
            return HAL_TIMEOUT;
        }
    }

    HAL_SPI_StateTypeDef state = HAL_SPI_GetState(nrf_hspi);
    if (state != HAL_SPI_STATE_READY) {
        NRF24_SPI_DMA_Abort();
        HAL_Delay(1);
        state = HAL_SPI_GetState(nrf_hspi);
        if (state != HAL_SPI_STATE_READY) {
            return HAL_BUSY;
        }
    }

    spi_dma_done_flag = 0;
    
    /* SỬA LỖI ĐỆ QUY: Gọi hàm HAL_SPI... */
    HAL_StatusTypeDef st = HAL_SPI_TransmitReceive_DMA(nrf_hspi, use_tx, use_rx, len);

    if (st != HAL_OK) {
        spi_dma_done_flag = 1; 
    }
    return st;
}
