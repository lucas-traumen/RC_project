#ifndef NRF24_DMA_DRIVER_H
#define NRF24_DMA_DRIVER_H

#include <stdint.h>
#include "stm32f1xx_hal.h"
#include "nrf24_spi_dma.h"

/* L?nh nRF24 */
#define NRF_CMD_R_REGISTER      0x00
#define NRF_CMD_W_REGISTER      0x20
#define NRF_CMD_R_RX_PAYLOAD    0x61
#define NRF_CMD_W_TX_PAYLOAD    0xA0
#define NRF_CMD_FLUSH_TX        0xE1
#define NRF_CMD_FLUSH_RX        0xE2
#define NRF_CMD_REUSE_TX_PL     0xE3
#define NRF_CMD_R_RX_PL_WID     0x60
#define NRF_CMD_NOP             0xFF

/* Thanh ghi nRF24 */
#define NRF_REG_CONFIG          0x00
#define NRF_REG_EN_AA           0x01
#define NRF_REG_EN_RXADDR       0x02
#define NRF_REG_SETUP_AW        0x03
#define NRF_REG_SETUP_RETR      0x04
#define NRF_REG_RF_CH           0x05
#define NRF_REG_RF_SETUP        0x06
#define NRF_REG_STATUS          0x07
#define NRF_REG_OBSERVE_TX      0x08
#define NRF_REG_CD              0x09
#define NRF_REG_RX_ADDR_P0      0x0A
#define NRF_REG_RX_ADDR_P1      0x0B
#define NRF_REG_TX_ADDR         0x10
#define NRF_REG_RX_PW_P0        0x11
#define NRF_REG_FIFO_STATUS     0x17
#define NRF_REG_DYNPD           0x1C
#define NRF_REG_FEATURE         0x1D

/* C? trong thanh ghi STATUS */
#define NRF_STATUS_RX_DR        0x40
#define NRF_STATUS_TX_DS        0x20
#define NRF_STATUS_MAX_RT       0x10

/* C? trong thanh ghi CONFIG */
#define NRF_CONFIG_PWR_UP       0x02
#define NRF_CONFIG_PRIM_RX      0x01
#define NRF_CONFIG_EN_CRC       0x08

/* ===== Kh?i t?o ===== */
void nrf24_driver_init(void);

/* ===== ??c/Ghi Thanh Ghi (S? d?ng Polling) ===== */
HAL_StatusTypeDef nrf24_read_register(uint8_t reg, uint8_t *buf, uint8_t len);
HAL_StatusTypeDef nrf24_write_register(uint8_t reg, const uint8_t *buf, uint8_t len);
HAL_StatusTypeDef nrf24_read_reg_byte(uint8_t reg, uint8_t *value);
HAL_StatusTypeDef nrf24_write_reg_byte(uint8_t reg, uint8_t value);
HAL_StatusTypeDef nrf24_get_status(uint8_t *status);

/* ===== L?nh (S? d?ng Polling) ===== */
HAL_StatusTypeDef nrf24_flush_tx(void);
HAL_StatusTypeDef nrf24_flush_rx(void);
                                            
/* ===== RX side (Payload - S? S?A D?NG POLLING) ===== */
HAL_StatusTypeDef nrf24_read_rx_payload_width(uint8_t *payload_len);
/* ??i t?n h?m d? cho gi?ng v?i rc_receiver.c */
HAL_StatusTypeDef nrf24_read_rx_payload_dma(uint8_t *payload, uint8_t len);
void nrf24_set_rx_mode(void); 

/* ===== GPIO Helpers (cho rc_receiver d?ng) ===== */
void nrf24_ce_low(void);
void nrf24_ce_high(void);

#endif // NRF24_DMA_DRIVER_H
