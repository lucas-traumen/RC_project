#include "nrf24_dma_driver.h"
#include "nrf24_conf.h"       
#include "rc_radio_cfg.h"     
#include <string.h>
#include "core_cm3.h"         

static uint8_t spi_tx_buf[NRF_MAX_PAYLOAD_SIZE + 2]; // +2 cho an toàn
static uint8_t spi_rx_buf[NRF_MAX_PAYLOAD_SIZE + 2];

// =================== GPIO control ===================
void nrf24_ce_low(void) {
    HAL_GPIO_WritePin(NRF_CE_PORT, NRF_CE_PIN, GPIO_PIN_RESET);
}
void nrf24_ce_high(void) {
    HAL_GPIO_WritePin(NRF_CE_PORT, NRF_CE_PIN, GPIO_PIN_SET);
}
static inline void csn_low(void) {
    HAL_GPIO_WritePin(NRF_CSN_PORT, NRF_CSN_PIN, GPIO_PIN_RESET);
}
static inline void csn_high(void) {
    HAL_GPIO_WritePin(NRF_CSN_PORT, NRF_CSN_PIN, GPIO_PIN_SET);
}

// =================== DWT-based delay ===================
static inline void dwt_delay_init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    DWT->CYCCNT = 0;
}
//static inline void delay_us(uint32_t us) {
//    uint32_t cycles = us * (HAL_RCC_GetHCLKFreq() / 1000000);
//    uint32_t start = DWT->CYCCNT;
//    while ((DWT->CYCCNT - start) < cycles);
//}

// ==========================================================
// ===== ĐỌC/GHI THANH GHI (DÙNG POLLING) =====
// ==========================================================

/*
 * Dùng POLLING. An toàn cho lệnh 1-byte. Sẽ KHÔNG gây lỗi OVR.
 */
HAL_StatusTypeDef nrf24_read_register(uint8_t reg, uint8_t *buf, uint8_t len)
{
    spi_tx_buf[0] = NRF_CMD_R_REGISTER | reg;
    memset(spi_tx_buf + 1, NRF_CMD_NOP, len); 
    csn_low();
    HAL_StatusTypeDef st = nrf24_spi_transfer_polling(spi_tx_buf, spi_rx_buf, len + 1, 50);
    csn_high();
    if (st == HAL_OK && buf != NULL) {
        memcpy(buf, spi_rx_buf + 1, len);
    }
    return st;
}

/*
 * Dùng POLLING. An toàn.
 */
HAL_StatusTypeDef nrf24_write_register(uint8_t reg, const uint8_t *buf, uint8_t len)
{
    spi_tx_buf[0] = NRF_CMD_W_REGISTER | reg;
    memcpy(spi_tx_buf + 1, buf, len);
    csn_low();
    HAL_StatusTypeDef st = nrf24_spi_transfer_polling(spi_tx_buf, spi_rx_buf, len + 1, 50);
    csn_high();
    return st;
}

HAL_StatusTypeDef nrf24_read_reg_byte(uint8_t reg, uint8_t *value)
{
    return nrf24_read_register(reg, value, 1);
}

HAL_StatusTypeDef nrf24_write_reg_byte(uint8_t reg, uint8_t value)
{
    return nrf24_write_register(reg, &value, 1);
}

/*
 * Đọc STATUS (dùng POLLING). Đây là hàm QUAN TRỌNG NHẤT
 * (File rc_receiver.c của bạn gọi hàm này)
 */
HAL_StatusTypeDef nrf24_get_status(uint8_t *status)
{
    spi_tx_buf[0] = NRF_CMD_NOP;
    csn_low();
    HAL_StatusTypeDef st = nrf24_spi_transfer_polling(spi_tx_buf, spi_rx_buf, 1, 50);
    csn_high();
    if (st == HAL_OK && status != NULL) {
        *status = spi_rx_buf[0];
    }
    return st;
}

// ==========================================================
// ===== LỆNH (DÙNG POLLING) =====
// ==========================================================

HAL_StatusTypeDef nrf24_flush_tx(void)
{
    spi_tx_buf[0] = NRF_CMD_FLUSH_TX;
    csn_low();
    HAL_StatusTypeDef st = nrf24_spi_transfer_polling(spi_tx_buf, spi_rx_buf, 1, 50);
    csn_high();
    return st;
}

HAL_StatusTypeDef nrf24_flush_rx(void)
{
    spi_tx_buf[0] = NRF_CMD_FLUSH_RX;
    csn_low();
    HAL_StatusTypeDef st = nrf24_spi_transfer_polling(spi_tx_buf, spi_rx_buf, 1, 50);
    csn_high();
    return st;
}

HAL_StatusTypeDef nrf24_read_rx_payload_width(uint8_t *payload_len)
{
    spi_tx_buf[0] = NRF_CMD_R_RX_PL_WID;
    spi_tx_buf[1] = NRF_CMD_NOP;
    csn_low();
    HAL_StatusTypeDef st = nrf24_spi_transfer_polling(spi_tx_buf, spi_rx_buf, 2, 50);
    csn_high();
    if (st == HAL_OK && payload_len != NULL) {
        *payload_len = spi_rx_buf[1];
    }
    return st;
}

// ==========================================================
// ===== ĐỌC PAYLOAD (ĐÃ SỬA SANG DÙNG POLLING) =====
// ==========================================================

/*
 * ĐÂY LÀ THAY ĐỔI CUỐI CÙNG
 * Chúng ta TẮT DMA và dùng Polling (SPI thường)
 * để tránh 100% lỗi Overrun (OVR), đúng như cách "ban đầu nó nhận"
 */
HAL_StatusTypeDef nrf24_read_rx_payload_dma(uint8_t *payload, uint8_t len)
{
    if (len > NRF_MAX_PAYLOAD_SIZE) len = NRF_MAX_PAYLOAD_SIZE;

    spi_tx_buf[0] = NRF_CMD_R_RX_PAYLOAD;
    memset(spi_tx_buf + 1, NRF_CMD_NOP, len); // Gửi NOP để đọc
    
    csn_low();
    /*
     * =========================================================
     * THAY ĐỔI: Dùng POLLING (SPI thường) thay vì DMA
     * (Đây là cách "ban đầu nó nhận" mà bạn nói)
     * =========================================================
     */
    HAL_StatusTypeDef st = nrf24_spi_transfer_polling(spi_tx_buf, spi_rx_buf, len + 1, 100);
    csn_high();
    
    if (st != HAL_OK) return st;

    /* * KHÔNG CẦN ĐỢI DMA NỮA
     * vì hàm polling là blocking, nó chạy xong mới trả về
     */
    
    /* Copy kết quả (bỏ qua byte STATUS đầu tiên) */
    memcpy(payload, spi_rx_buf + 1, len);
    return HAL_OK;
}


// ==========================================================
// ===== KHỞI TẠO (Đây là nơi SET THANH GHI) =====
// ==========================================================

void nrf24_driver_init(void)
{
    dwt_delay_init();
    nrf24_ce_low();
    csn_high();
    HAL_Delay(5); 

    // === CÁC THANH GHI ĐƯỢC SET Ở ĐÂY ===
    // (Dùng các hàm _polling an toàn)
    nrf24_write_reg_byte(NRF_REG_CONFIG, 0x0A); // CRC 1-byte, Power Up, TẠM THỜI PTX
    HAL_Delay(2);
    // (Toàn bộ code init của bạn trong rc_receiver.c nên được chuyển về đây)
}

/*
 * Chuyển nRF24 sang chế độ RX (Hàm này rc_receiver.c sẽ gọi)
 */
void nrf24_set_rx_mode(void)
{
    /* Power Up, 1-byte CRC, chế độ PRX */
    uint8_t cfg = NRF_CONFIG_EN_CRC | NRF_CONFIG_PWR_UP | NRF_CONFIG_PRIM_RX;
    nrf24_write_reg_byte(NRF_REG_CONFIG, cfg);
    
    /* Xóa cờ ngắt */
    nrf24_write_reg_byte(NRF_REG_STATUS, NRF_STATUS_RX_DR | NRF_STATUS_TX_DS | NRF_STATUS_MAX_RT);
    nrf24_flush_rx();
    nrf24_flush_tx();
    
    nrf24_ce_high(); // Bắt đầu nghe
    HAL_Delay(1); 
}
