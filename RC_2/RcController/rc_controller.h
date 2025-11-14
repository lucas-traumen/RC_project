#ifndef RC_CONTROLLER_H
#define RC_CONTROLLER_H

#include <stdint.h>
#include "stm32f1xx_hal.h"
#include "nrf24_dma_driver.h"

// NRF24 defines
#define NRF_MAX_PAYLOAD_SIZE 32
#define NRF_TX_DS_FLAG       0x20  // TX Data Sent
#define NRF_MAX_RT_FLAG      0x10  // Max Retries

// Main mode: Manual / Line-following
typedef enum {
    MAIN_MODE_MANUAL = 0,
    MAIN_MODE_LINE
} MainMode_t;

// Sub mode trong Manual: PWM / Servo
typedef enum {
    SUB_MODE_PWM = 0,
    SUB_MODE_SERVO
} SubMode_t;

// Khung dữ liệu RC (packed)
typedef struct __attribute__((packed)) {
    uint8_t  main_mode;    // 0 = manual, 1 = line
    uint8_t  sub_mode;     // 0 = PWM, 1 = Servo (chỉ dùng khi manual)
    int16_t joyL_x;
    int16_t joyL_y;
    int16_t joyR_x;
    int16_t joyR_y;
    uint8_t  line_bits;    // line sensor bitmap
    //uint8_t  battery_pct;  // % pin
    uint8_t  reserved[6];  // dự phòng (tổng 16 byte)
} RC_Frame_t;
// Trạng thái truyền
typedef enum {
    RC_TX_IDLE = 0,
    RC_TX_BUSY,
    RC_TX_DONE,
    RC_TX_ERROR
} RC_TxStatus_t;

// Khởi tạo controller
void RC_Controller_Init(void);

// Cập nhật joystick + sensor
void RC_Controller_Update(void);

// Non-blocking send (dùng DMA)
HAL_StatusTypeDef RC_Controller_Send_NB(void);

// Lấy trạng thái TX hiện tại
RC_TxStatus_t RC_Controller_GetTxStatus(void);

// Callback gọi khi DMA SPI NRF24 hoàn tất
void RC_Controller_DMA_Callback(void);

#endif
