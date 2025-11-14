#include "nrf24_dma_driver.h"
#include "nrf24_conf.h"
#include <string.h>
#include "core_cm3.h"  // để dùng DWT->CYCCNT

// Macro điều khiển GPIO (từ cấu hình gốc)
static inline void csn_low(void)  { HAL_GPIO_WritePin(NRF_CSN_PORT, NRF_CSN_PIN, GPIO_PIN_RESET); }
static inline void csn_high(void) { HAL_GPIO_WritePin(NRF_CSN_PORT, NRF_CSN_PIN, GPIO_PIN_SET); }
static inline void ce_low(void)   { HAL_GPIO_WritePin(NRF_CE_PORT,  NRF_CE_PIN,  GPIO_PIN_RESET); }
static inline void ce_high(void)  { HAL_GPIO_WritePin(NRF_CE_PORT,  NRF_CE_PIN,  GPIO_PIN_SET); }



// ===== DWT-based µs delay (ổn định, không phụ thuộc tối ưu compiler) =====
static inline void dwt_delay_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;   // enable DWT
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;              // enable counter
    DWT->CYCCNT = 0;
}

static inline void delay_us(uint32_t us)
{
    if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0)
    {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
        DWT->CYCCNT = 0;
    }

    uint32_t cycles = (HAL_RCC_GetHCLKFreq() / 1000000U) * us;
    uint32_t start = DWT->CYCCNT;
    while ((uint32_t)(DWT->CYCCNT - start) < cycles) { __NOP(); }
}

// ===== Driver init =====
void nrf24_driver_init(void)
{
    // Chuẩn bị DWT cho delay µs và đưa CE/CSN về trạng thái ban đầu
    dwt_delay_init();
    csn_high();
    ce_low();
    HAL_Delay(5); // ổn định nguồn
}

// ===== CE control =====
void nrf24_ce_high(void) { ce_high(); }
void nrf24_ce_low(void)  { ce_low(); }

void nrf24_ce_pulse(void)
{
    ce_high();
    delay_us(15);  // >=10us theo datasheet
    ce_low();
}

// ===== Register R/W =====
HAL_StatusTypeDef nrf24_read_register(uint8_t reg, uint8_t *buf, uint8_t len)
{
    if (buf == NULL || len == 0 || len > NRF_MAX_PAYLOAD_SIZE) return HAL_ERROR;

    uint8_t tx[33]; // 1 byte cmd + 32 bytes max
    uint8_t rx[33];

    tx[0] = NRF_CMD_R_REGISTER | (reg & 0x1F);
    memset(&tx[1], NRF_CMD_NOP, len);

    csn_low();
    HAL_StatusTypeDef st = nrf24_spi_transfer_dma(tx, rx, len + 1, NRF_DMA_TIMEOUT_MS);
    csn_high();

    if (st == HAL_OK) memcpy(buf, &rx[1], len);
    return st;
}

HAL_StatusTypeDef nrf24_write_register(uint8_t reg, const uint8_t *buf, uint8_t len)
{
    if (buf == NULL || len == 0 || len > NRF_MAX_PAYLOAD_SIZE) return HAL_ERROR;

    uint8_t tx[33];
    uint8_t rx[33];

    tx[0] = NRF_CMD_W_REGISTER | (reg & 0x1F);
    memcpy(&tx[1], buf, len);

    csn_low();
    HAL_StatusTypeDef st = nrf24_spi_transfer_dma(tx, rx, len + 1, NRF_DMA_TIMEOUT_MS);
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

// ===== TX / RX payload =====
HAL_StatusTypeDef nrf24_write_tx_payload_dma(const uint8_t *payload, uint8_t len)
{
    if (payload == NULL || len == 0 || len > NRF_MAX_PAYLOAD_SIZE) return HAL_ERROR;

    uint8_t tx[33];
    uint8_t rx[33];

    ce_low(); // CE phải LOW trước khi ghi payload

    tx[0] = NRF_CMD_W_TX_PAYLOAD;
    memcpy(&tx[1], payload, len);

    csn_low();
    HAL_StatusTypeDef st = nrf24_spi_transfer_dma(tx, rx, len + 1, NRF_DMA_TIMEOUT_MS);
    csn_high();
    if (st != HAL_OK) return st;

    // Bắt đầu truyền
    nrf24_ce_pulse();
    return HAL_OK;
}

HAL_StatusTypeDef nrf24_read_rx_payload_dma(uint8_t *payload, uint8_t len)
{
    if (payload == NULL || len == 0 || len > NRF_MAX_PAYLOAD_SIZE) return HAL_ERROR;

    uint8_t tx[33];
    uint8_t rx[33];

    tx[0] = NRF_CMD_R_RX_PAYLOAD;
    memset(&tx[1], NRF_CMD_NOP, len);

    csn_low();
    HAL_StatusTypeDef st = nrf24_spi_transfer_dma(tx, rx, len + 1, NRF_DMA_TIMEOUT_MS);
    csn_high();

    if (st == HAL_OK) memcpy(payload, &rx[1], len);
    return st;
}

// Đọc chiều rộng payload rồi đọc đúng số byte; nếu width > 32 => FLUSH_RX
HAL_StatusTypeDef nrf24_read_rx_payload_auto(uint8_t *payload, uint8_t *out_len)
{
    if (payload == NULL) return HAL_ERROR;

    uint8_t tx2[2] = {NRF_CMD_R_RX_PL_WID, NRF_CMD_NOP};
    uint8_t rx2[2] = {0};

    csn_low();
    HAL_StatusTypeDef st = nrf24_spi_transfer_dma(tx2, rx2, 2, NRF_DMA_TIMEOUT_MS);
    csn_high();
    if (st != HAL_OK) return st;

    uint8_t wid = rx2[1];
    if (wid == 0 || wid > NRF_MAX_PAYLOAD_SIZE) {
        // width lỗi theo datasheet → flush RX
        (void)nrf24_flush_rx();
        return HAL_ERROR;
    }
    if (out_len) *out_len = wid;
    return nrf24_read_rx_payload_dma(payload, wid);
}

// ===== FIFO / STATUS =====
HAL_StatusTypeDef nrf24_flush_tx(void)
{
    uint8_t tx = NRF_CMD_FLUSH_TX, rx = 0;
    csn_low();
    HAL_StatusTypeDef st = nrf24_spi_transfer_dma(&tx, &rx, 1, NRF_DMA_TIMEOUT_MS);
    csn_high();
    return st;
}

HAL_StatusTypeDef nrf24_flush_rx(void)
{
    uint8_t tx = NRF_CMD_FLUSH_RX, rx = 0;
    csn_low();
    HAL_StatusTypeDef st = nrf24_spi_transfer_dma(&tx, &rx, 1, NRF_DMA_TIMEOUT_MS);
    csn_high();
    return st;
}

HAL_StatusTypeDef nrf24_get_status(uint8_t *status)
{
    uint8_t tx = NRF_CMD_NOP, rx = 0;
    csn_low();
    HAL_StatusTypeDef st = nrf24_spi_transfer_dma(&tx, &rx, 1, NRF_DMA_TIMEOUT_MS);
    csn_high();
    if (st == HAL_OK && status) *status = rx;
    return st;
}
// ====== Chờ TX xong / lỗi ======
// Đợi đến khi bit TX_DS (đã gửi xong) hoặc MAX_RT (retry tối đa) xuất hiện.
// Trả về: HAL_OK (xong), HAL_ERROR (MAX_RT), HAL_TIMEOUT (hết thời gian).
HAL_StatusTypeDef nrf24_wait_tx_done(uint32_t timeout_ms, uint8_t *out_status)
{
    uint32_t start = HAL_GetTick();
    for (;;)
    {
        uint8_t st = 0;
        if (nrf24_get_status(&st) != HAL_OK) {
            return HAL_ERROR;
        }

        if (st & NRF_STATUS_TX_DS) {
            // clear TX_DS
            (void)nrf24_write_reg_byte(NRF_REG_STATUS, NRF_STATUS_TX_DS);
            if (out_status) *out_status = st;
            return HAL_OK;
        }
        if (st & NRF_STATUS_MAX_RT) {
            // clear MAX_RT và flush TX để sẵn sàng lần sau
            (void)nrf24_write_reg_byte(NRF_REG_STATUS, NRF_STATUS_MAX_RT);
            (void)nrf24_flush_tx();
            if (out_status) *out_status = st;
            return HAL_ERROR;
        }

        if ((HAL_GetTick() - start) > timeout_ms) {
            return HAL_TIMEOUT;
        }
        // tránh “quay” quá nhanh, nRF24 xử lý rất nhanh nên chỉ cần delay nhỏ
        // (bạn có thể bỏ HAL_Delay để tăng tốc, tuỳ hệ thống)
        HAL_Delay(1);
    }
}

// ====== Gửi nguyên frame bằng cách chia ≤32B ======
// Gửi tuần tự từng packet 32B, mỗi packet chờ hoàn tất bằng nrf24_wait_tx_done().
HAL_StatusTypeDef nrf24_send_frame_blocking(const uint8_t *frame,
                                            uint16_t len,
                                            uint32_t per_packet_timeout_ms)
{
    if (frame == NULL || len == 0) return HAL_ERROR;

#ifndef NRF_MAX_PAYLOAD_SIZE
# define NRF_MAX_PAYLOAD_SIZE 32
#endif

    uint16_t offset = 0;
    while (offset < len)
    {
        uint8_t chunk = (len - offset > NRF_MAX_PAYLOAD_SIZE)
                        ? NRF_MAX_PAYLOAD_SIZE
                        : (uint8_t)(len - offset);

        // Ghi payload & pulse CE (hàm này của bạn đã pulse CE bên trong)
        HAL_StatusTypeDef st = nrf24_write_tx_payload_dma(&frame[offset], chunk);
        if (st != HAL_OK) {
            return st; // lỗi SPI/DMA
        }

        // Chờ gói này kết thúc
        uint8_t st_last = 0;
        st = nrf24_wait_tx_done(per_packet_timeout_ms, &st_last);
        if (st != HAL_OK) {
            // Nếu lỗi do MAX_RT hoặc TIMEOUT → dừng toàn bộ frame
            return st;
        }

        offset += chunk;
    }
    return HAL_OK;
}
