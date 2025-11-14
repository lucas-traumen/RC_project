#include "nrf24_spi_dma.h"
#include <string.h>

// SPI handle dùng chung cho nRF24
static SPI_HandleTypeDef *nrf_hspi = NULL;

// Cờ báo DMA hoàn thành
static volatile uint8_t spi_dma_done_flag = 0;

// Buffer tĩnh dùng khi tx_buf hoặc rx_buf = NULL
//static uint8_t tx_dummy[256];
//static uint8_t rx_dummy[256];

void NRF24_SPI_DMA_Init(SPI_HandleTypeDef *hspi)
{
    if (hspi == NULL) return;
    nrf_hspi = hspi;
    spi_dma_done_flag = 1; // ban đầu ở trạng thái "rảnh"
}

uint8_t nrf24_spi_dma_busy(void)
{
    return (spi_dma_done_flag == 0);
}

void nrf24_spi_dma_complete_cb(void)
{
    spi_dma_done_flag = 1;
}

//HAL_StatusTypeDef nrf24_spi_transfer_dma(uint8_t *tx_buf,
//                                         uint8_t *rx_buf,
//                                         uint16_t len,
//                                         uint32_t timeout_ms)
//{
//    if (nrf_hspi == NULL || len == 0) return HAL_ERROR;
//    if (len > sizeof(tx_dummy)) return HAL_ERROR;

//    // Chuẩn bị buffer
//    uint8_t *use_tx = tx_buf;
//    uint8_t *use_rx = rx_buf;

//    if (tx_buf == NULL) {
//        memset(tx_dummy, 0xFF, len);
//        use_tx = tx_dummy;
//    }
//    if (rx_buf == NULL) {
//        use_rx = rx_dummy;
//    }

//    // Đợi transaction trước kết thúc
//    uint32_t start = HAL_GetTick();
//    while (nrf24_spi_dma_busy()) {
//        if ((HAL_GetTick() - start) > timeout_ms)
//            return HAL_TIMEOUT;
//    }

//    // Bắt đầu truyền
//    spi_dma_done_flag = 0;
//    HAL_StatusTypeDef st =
//        HAL_SPI_TransmitReceive_DMA(nrf_hspi, use_tx, use_rx, len);

//    if (st != HAL_OK) {
//        spi_dma_done_flag = 1;   // nhả cờ để không kẹt
//        return st;
//    }

//    // Chờ DMA hoàn thành (được báo qua callback)
//    start = HAL_GetTick();
//    while (!spi_dma_done_flag) {
//        if ((HAL_GetTick() - start) > timeout_ms) {
//            HAL_SPI_Abort(nrf_hspi);
//            spi_dma_done_flag = 1;
//            return HAL_TIMEOUT;
//        }
//    }

//    // Nếu user muốn nhận dữ liệu và đang dùng rx_dummy -> copy lại
//    if (rx_buf && (use_rx == rx_dummy)) {
//        memcpy(rx_buf, rx_dummy, len);
//    }

//    return HAL_OK;
//}
HAL_StatusTypeDef nrf24_spi_transfer_dma(uint8_t *tx_buf,
                                         uint8_t *rx_buf,
                                         uint16_t len,
                                         uint32_t timeout_ms)
{
    if (nrf_hspi == NULL || len == 0) {
        return HAL_ERROR;
    }
    if (len > 32) {   // lệnh + payload nRF24 không vượt 32–33 byte
        return HAL_ERROR;
    }

    uint8_t tx_dummy[32];
    uint8_t rx_dummy[32];

    uint8_t *use_tx = tx_buf ? tx_buf : tx_dummy;
    uint8_t *use_rx = rx_buf ? rx_buf : rx_dummy;

    if (tx_buf == NULL) {
        memset(tx_dummy, 0xFF, len);
    }

    // Nếu SPI đang BUSY, abort cho sạch rồi transmit lại
    if (HAL_SPI_GetState(nrf_hspi) != HAL_SPI_STATE_READY) {
        HAL_SPI_Abort(nrf_hspi);
    }

    HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(
                                nrf_hspi,
                                use_tx,
                                use_rx,
                                len,
                                timeout_ms
                            );

    if (st != HAL_OK) {
        return st;
    }

    if (rx_buf && (use_rx == rx_dummy)) {
        memcpy(rx_buf, rx_dummy, len);
    }

    return HAL_OK;
}
/*
 * Callback HAL: nếu bạn KHÔNG muốn tự viết trong main.c,
 * thì KHÔNG define NRF24_SPI_DMA_USER_SUPPLIES_HAL_CALLBACKS,
 * và 2 hàm dưới sẽ được dùng mặc định.
 *
 * Nếu bạn muốn tự viết (như bên TX đang làm), hãy:
 *  - Định nghĩa NRF24_SPI_DMA_USER_SUPPLIES_HAL_CALLBACKS trong preprocessor
 *  - Viết HAL_SPI_TxRxCpltCallback / HAL_SPI_ErrorCallback trong main.c
 */
#ifndef NRF24_SPI_DMA_USER_SUPPLIES_HAL_CALLBACKS
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == nrf_hspi) {
        nrf24_spi_dma_complete_cb();
    }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == nrf_hspi) {
        // Vẫn nhả cờ để không kẹt
        nrf24_spi_dma_complete_cb();
    }
}
#endif
