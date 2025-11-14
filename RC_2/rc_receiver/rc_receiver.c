#include "rc_receiver.h"
#include <string.h>
#include "rc_radio_cfg.h"
/* Nếu chỗ khác chưa define thì define tạm ở đây */
#ifndef NRF_RX_DR_FLAG
#define NRF_RX_DR_FLAG 0x40
#endif

#ifndef NRF_MAX_PAYLOAD_SIZE
#define NRF_MAX_PAYLOAD_SIZE 32
#endif
uint8_t g_nrf_status = 0;
HAL_StatusTypeDef g_nrf_status_ret = HAL_OK;

/* Frame hiện tại nhận được từ tay cầm */
RC_Frame_t rx_frame;

/* Thời điểm nhận gói cuối (ms) – dùng cho failsafe */
static uint32_t last_rx_ms = 0;

/* 2 servo gripper (có thể NULL nếu không dùng) */
static Servo_t* pServoA = NULL;  // ví dụ: servo nâng/hạ
static Servo_t* pServoB = NULL;  // ví dụ: servo gắp

/* Map ADC 0..4095 (MID = 2048) -> -100..+100
   - Kéo xuống (< MID)  => -%
   - Kéo lên   (> MID)  => +%  */
static int16_t map_axis_signed(int16_t v)
{
    const int16_t MID = 2048;
    if (v >= MID) {
        return (int16_t)(((int32_t)(v - MID) * 100) / (4095 - MID)); // 0..+100
    } else {
        return (int16_t)(-((int32_t)(MID - v) * 100) / MID);         // -100..0
    }
}

static int16_t clamp100(int16_t v)
{
    if (v > 100)  return 100;
    if (v < -100) return -100;
    return v;
}

static int16_t deadzone(int16_t v, uint8_t dz)
{
    return (v > -dz && v < dz) ? 0 : v;
}

/* ===== Khởi tạo bên nhận RC ===== */
/* Chu ý: trước đó main() phải gọi:
 *   NRF24_SPI_DMA_Init(&hspiX);
 *   nrf24_driver_init();
 */
void RC_Receiver_Init(Servo_t* sv1, Servo_t* sv2)
{
    pServoA = sv1;
    pServoB = sv2;

    if (sizeof(RC_Frame_t) > NRF_MAX_PAYLOAD_SIZE) {
        return;
    }

    /* 1) CE = 0 trước khi config */
    nrf24_ce_low();

    /* 2) TẮT Auto-ACK, dùng pipe0 */
    nrf24_write_reg_byte(NRF_REG_EN_AA,      0x00);   // ACK off giống TX
    nrf24_write_reg_byte(NRF_REG_EN_RXADDR,  0x01);   // enable pipe0
    nrf24_write_reg_byte(NRF_REG_SETUP_AW,   0x03);   // 5-byte address
    nrf24_write_reg_byte(NRF_REG_SETUP_RETR, 0x15);   // không quan trọng ở RX

    /* 3) RF channel & RF setup – PHẢI GIỐNG TX */
    nrf24_write_reg_byte(NRF_REG_RF_CH,    RC_RF_CH);     // 76
    nrf24_write_reg_byte(NRF_REG_RF_SETUP, RC_RF_SETUP);  // 0x06

    /* 4) Địa chỉ RX_P0 & TX_ADDR – cùng RC_ADDR */
    nrf24_write_register(NRF_REG_RX_ADDR_P0, RC_ADDR, 5);
    nrf24_write_register(NRF_REG_TX_ADDR,    RC_ADDR, 5);

    /* 5) Payload width cố định = sizeof(RC_Frame_t) (17 bytes) */
    nrf24_write_reg_byte(NRF_REG_RX_PW_P0, (uint8_t)sizeof(RC_Frame_t));

    /* 6) Tắt DPL/FEATURE */
    nrf24_write_reg_byte(0x1C /*DYNPD*/,   0x00);
    nrf24_write_reg_byte(0x1D /*FEATURE*/, 0x00);

    /* 7) Clear cờ + flush FIFO */
    nrf24_write_reg_byte(NRF_REG_STATUS, 0x70);
    nrf24_flush_rx();
    nrf24_flush_tx();

    /* 8) Power-up + PRX */
    uint8_t cfg = 0;
    nrf24_read_reg_byte(NRF_REG_CONFIG, &cfg);
    cfg |= NRF_CONFIG_PWR_UP;
    cfg |= NRF_CONFIG_PRIM_RX;   // RX mode
    cfg |= NRF_CONFIG_EN_CRC;
    nrf24_write_reg_byte(NRF_REG_CONFIG, cfg);
    HAL_Delay(5);

    /* 9) CE = 1 để bắt đầu lắng nghe */
    nrf24_ce_high();

    /* 10) Clear frame & thời gian */
    memset(&rx_frame, 0, sizeof(rx_frame));
    last_rx_ms = HAL_GetTick();
}
/* ===== Task chính: gọi liên tục trong while(1) ===== */
void RC_Receiver_Task(void)
{
    /* 1. Kiểm tra nếu có gói mới trong RX FIFO */
   uint8_t status = 0;
    g_nrf_status_ret = nrf24_get_status(&status);
    g_nrf_status = status;

    if (g_nrf_status_ret == HAL_OK && (status & NRF_RX_DR_FLAG))
    {
        uint8_t buf[sizeof(RC_Frame_t)];
        if (nrf24_read_rx_payload_dma(buf, sizeof(buf)) == HAL_OK) {
            memcpy(&rx_frame, buf, sizeof(buf));
            last_rx_ms = HAL_GetTick();
        }
        nrf24_write_reg_byte(NRF_REG_STATUS, NRF_RX_DR_FLAG);
    }

    /* 2. Failsafe: quá 200ms không nhận được gói nào -> dừng xe + servo về giữa */
    if (HAL_GetTick() - last_rx_ms > 200) {
        car_control(CAR_STOP_STATE, 0);

        if (pServoA) Servo_WriteUS(pServoA, pServoA->us_mid);
        if (pServoB) Servo_WriteUS(pServoB, pServoB->us_mid);

        return;
    }

    /* 3. Áp dụng điều khiển theo main_mode + sub_mode */

    /* ===== MAIN MODE: MANUAL ===== */
    if (rx_frame.main_mode == MAIN_MODE_MANUAL)
    {
        /* --- SUB MODE: PWM -> điều khiển bánh xe, bỏ qua servo --- */
        if (rx_frame.sub_mode == SUB_MODE_PWM)
        {
            /* Bánh xe:
               - joyL_y => tiến/lùi (Vy)
               - joyL_x => cua trái/phải (Vx) */
            int16_t vy = deadzone(map_axis_signed((int16_t)rx_frame.joyL_y), 5);
            int16_t vx = deadzone(map_axis_signed((int16_t)rx_frame.joyL_x), 5);

            int16_t left  = clamp100(vy + vx);
            int16_t right = clamp100(vy - vx);

            car_drive_percent((int8_t)left, (int8_t)right);

            /* Servo có thể giữ nguyên vị trí hiện tại,
               hoặc bạn muốn thì đưa về mid ở đây cũng được. */
        }
        /* --- SUB MODE: SERVO -> xe đứng yên, điều khiển servo bằng joystick phải --- */
        else if (rx_frame.sub_mode == SUB_MODE_SERVO)
        {
            /* V1 hiện tại: xe đứng yên, chỉ điều khiển servo
               V2 sau này bạn có thể đổi:
                 - joyL_y điều khiển bánh tiến/lùi
                 - joyR_y điều khiển nâng/hạ
                 - 1 nút toggle gripper auto mở/kẹp
               Chỗ này là chỗ để bạn nâng cấp sau. */

            /* Xe dừng lại cho an toàn */
            car_control(CAR_STOP_STATE, 0);

            /* Servo gripper:
               - joyR_y => servo A (ví dụ nâng/hạ)
               - joyR_x => servo B (ví dụ xoay/mở gắp) */
            if (pServoA) {
                int16_t sv = deadzone(map_axis_signed((int16_t)rx_frame.joyR_y), 5);
                Servo_WritePct(pServoA, sv);
            }

            if (pServoB) {
                int16_t sv = deadzone(map_axis_signed((int16_t)rx_frame.joyR_x), 5);
                Servo_WritePct(pServoB, sv);
            }
        }
        else
        {
            /* sub_mode không hợp lệ -> dừng xe */
            car_control(CAR_STOP_STATE, 0);
        }
    }
    /* ===== MAIN MODE: LINE ===== */
    else if (rx_frame.main_mode == MAIN_MODE_LINE)
    {
        /* Ở chế độ dò line, bỏ qua joystick.
           Sau này bạn có thể dùng:
             - rx_frame.line_bits (nếu sensor ở tay cầm)
             - hoặc sensor line gắn trên xe để điều khiển.

           Hiện tại: tạm cho xe dừng để an toàn. */
        car_control(CAR_STOP_STATE, 0);
    }
    else
    {
        /* main_mode ngoài phạm vi -> dừng xe */
        car_control(CAR_STOP_STATE, 0);
    }
}
