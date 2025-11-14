#ifndef NRF24_DMA_DRIVER_H
#define NRF24_DMA_DRIVER_H

#include <stdint.h>
#include "nrf24_spi_dma.h"
#include "stm32f1xx_hal.h"


// Các l?nh NRF24
#define NRF_CMD_R_REGISTER      0x00
#define NRF_CMD_W_REGISTER      0x20
#define NRF_CMD_R_RX_PAYLOAD    0x61
#define NRF_CMD_W_TX_PAYLOAD    0xA0
#define NRF_CMD_FLUSH_TX        0xE1
#define NRF_CMD_FLUSH_RX        0xE2
#define NRF_CMD_REUSE_TX_PL     0xE3
#define NRF_CMD_R_RX_PL_WID     0x60   // <— b? sung
#define NRF_CMD_NOP             0xFF

// Các thanh ghi NRF24
#define NRF_REG_CONFIG          0x00
#define NRF_REG_EN_AA           0x01
#define NRF_REG_EN_RXADDR       0x02
#define NRF_REG_SETUP_AW        0x03
#define NRF_REG_SETUP_RETR      0x04
#define NRF_REG_RF_CH           0x05
#define NRF_REG_RF_SETUP        0x06
#define NRF_REG_STATUS          0x07
#define NRF_REG_OBSERVE_TX      0x08
#define NRF_REG_RX_ADDR_P0      0x0A
#define NRF_REG_TX_ADDR         0x10
#define NRF_REG_RX_PW_P0        0x11
#define NRF_REG_FIFO_STATUS     0x17

// Bits trong CONFIG register
#define NRF_CONFIG_PRIM_RX      (1 << 0)
#define NRF_CONFIG_PWR_UP       (1 << 1)
#define NRF_CONFIG_EN_CRC       (1 << 3)

#define NRF_STATUS_RX_DR        (1U << 6)
#define NRF_STATUS_TX_DS        (1U << 5)
#define NRF_STATUS_MAX_RT       (1U << 4)

// Kh?i t?o driver
void nrf24_driver_init(void);

// Ð?c/Ghi register
HAL_StatusTypeDef nrf24_read_register(uint8_t reg, uint8_t *buf, uint8_t len);
HAL_StatusTypeDef nrf24_write_register(uint8_t reg, const uint8_t *buf, uint8_t len);

// Ð?c/Ghi register 1 byte
HAL_StatusTypeDef nrf24_read_reg_byte(uint8_t reg, uint8_t *value);
HAL_StatusTypeDef nrf24_write_reg_byte(uint8_t reg, uint8_t value);

// G?i payload qua TX
HAL_StatusTypeDef nrf24_write_tx_payload_dma(const uint8_t *payload, uint8_t len);

// Ð?c payload t? RX (yêu c?u bi?t tru?c chi?u dài)
HAL_StatusTypeDef nrf24_read_rx_payload_dma(uint8_t *payload, uint8_t len);

// Ð?c payload t? d?ng theo chi?u r?ng th?c t? (R_RX_PL_WID)
HAL_StatusTypeDef nrf24_read_rx_payload_auto(uint8_t *payload, uint8_t *out_len);

// Flush buffers
HAL_StatusTypeDef nrf24_flush_tx(void);
HAL_StatusTypeDef nrf24_flush_rx(void);

// Ð?c STATUS
HAL_StatusTypeDef nrf24_get_status(uint8_t *status);

// Ði?u khi?n CE
void nrf24_ce_high(void);
void nrf24_ce_low(void);
void nrf24_ce_pulse(void);

// Ch? 1 l?n truy?n TX k?t thúc (TX_DS) ho?c l?i (MAX_RT)
HAL_StatusTypeDef nrf24_wait_tx_done(uint32_t timeout_ms, uint8_t *out_status);

// G?i m?t frame b?t k? (len tùy ý) b?ng cách chia thành nhi?u gói =32B
HAL_StatusTypeDef nrf24_send_frame_blocking(const uint8_t *frame,
                                            uint16_t len,
                                            uint32_t per_packet_timeout_ms);

#endif // NRF24_DMA_DRIVER_H
