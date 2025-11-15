#include "rc_receiver.h"
#include <string.h>
#include "rc_radio_cfg.h"

/*
 * Lấy cờ ngắt IRQ (được set trong main.c -> HAL_GPIO_EXTI_Callback)
 */
extern volatile uint8_t nrf_irq_flag; 

/* Biến global để chứa frame nhận được */
RC_Frame_t rx_frame;

static uint32_t last_rx_ms = 0;
static Servo_t* pServoA = NULL;
static Servo_t* pServoB = NULL;

/* === (Các hàm helper của bạn: map_axis_signed, deadzone, clamp100) === */
static int16_t map_axis_signed(int16_t v) {
    const int16_t MID = 2048;
    if (v >= MID) {
        return (int16_t)(((int32_t)(v - MID) * 100) / (4095 - MID));
    } else {
        return (int16_t)(-((int32_t)(MID - v) * 100) / MID);
    }
}
static int16_t clamp100(int16_t v) {
    if (v > 100)  return 100;
    if (v < -100) return -100;
    return v;
}
static int16_t deadzone(int16_t v, uint8_t dz) {
    return (v > -dz && v < dz) ? 0 : v;
}

/* === (Hàm xử lý frame của bạn) === */
static void rc_handle_frame(void)
{
    /* ===== MAIN MODE: MANUAL ===== */
    if (rx_frame.main_mode == MAIN_MODE_MANUAL)
    {
        /* --- SUB MODE: PWM -> điều khiển bánh xe --- */
        if (rx_frame.sub_mode == SUB_MODE_PWM)
        {
            int16_t vy = deadzone(map_axis_signed((int16_t)rx_frame.joyL_y), 5);
            int16_t vx = deadzone(map_axis_signed((int16_t)rx_frame.joyL_x), 5);
            int16_t left  = clamp100(vy + vx);
            int16_t right = clamp100(vy - vx);
            car_drive_percent((int8_t)left, (int8_t)right);
        }
        /* --- SUB MODE: SERVO -> điều khiển servo --- */
        else if (rx_frame.sub_mode == SUB_MODE_SERVO)
        {
            car_control(CAR_STOP_STATE, 0); 
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
            car_control(CAR_STOP_STATE, 0);
        }
    }
    /* ===== MAIN MODE: LINE ===== */
    else if (rx_frame.main_mode == MAIN_MODE_LINE)
    {
        car_control(CAR_STOP_STATE, 0); // (Tạm thời dừng, bạn tự thêm logic PID)
    }
}

/*
 * ==========================================================
 * HÀM KHỞI TẠO RECEIVER (gọi 1 lần trong main)
 * ==========================================================
 */
void RC_Receiver_Init(Servo_t* sv1, Servo_t* sv2)
{
    pServoA = sv1;
    pServoB = sv2;
    
    memset(&rx_frame, 0, sizeof(RC_Frame_t));
    last_rx_ms = HAL_GetTick();

    /*
     * KHÔNG GỌI nrf24_driver_init() ở đây nữa
     * vì main.c (phiên bản mới) đã gọi rồi.
     */
     
    /* DI CHUYỂN TOÀN BỘ CODE CẤU HÌNH TỪ FILE CŨ CỦA BẠN VÀO ĐÂY */
    /* (Vì nrf24_driver_init() của tôi không có code) */

    /* 1) CE = 0 trước khi config */
    nrf24_ce_low();

    /* 2) TẮT Auto-ACK, dùng pipe0 */
    nrf24_write_reg_byte(NRF_REG_EN_AA,      0x00);
    nrf24_write_reg_byte(NRF_REG_EN_RXADDR,  0x01);
    nrf24_write_reg_byte(NRF_REG_SETUP_AW,   0x03);
    nrf24_write_reg_byte(NRF_REG_SETUP_RETR, 0x15);

    /* 3) RF channel & RF setup – PHẢI GIỐNG TX */
    nrf24_write_reg_byte(NRF_REG_RF_CH,    RC_RF_CH);
    nrf24_write_reg_byte(NRF_REG_RF_SETUP, RC_RF_SETUP);

    /* 4) Địa chỉ RX_P0 & TX_ADDR – cùng RC_ADDR */
    nrf24_write_register(NRF_REG_RX_ADDR_P0, RC_ADDR, 5);
    nrf24_write_register(NRF_REG_TX_ADDR,    RC_ADDR, 5);

    /* 5) Payload width cố định = sizeof(RC_Frame_t) */
    nrf24_write_reg_byte(NRF_REG_RX_PW_P0, (uint8_t)sizeof(RC_Frame_t));

    /* 6) Tắt DPL/FEATURE */
    nrf24_write_reg_byte(0x1C /*DYNPD*/,   0x00);
    nrf24_write_reg_byte(0x1D /*FEATURE*/, 0x00);

    /* 7) Clear cờ + flush FIFO */
    nrf24_write_reg_byte(NRF_REG_STATUS, 0x70);
    nrf24_flush_rx();
    nrf24_flush_tx();
    
    /* 8) Chuyển nRF24 sang chế độ RX (Dùng hàm trong driver) */
    nrf24_set_rx_mode();
}


/*
 * ==========================================================
 * HÀM TÁC VỤ RECEIVER (gọi liên tục trong while(1))
 * ==========================================================
 */
void RC_Receiver_Task(void)
{
    /*
     * Biến g_nrf_status và g_nrf_status_ret là không cần thiết
     * nếu bạn dùng kiến trúc ngắt IRQ.
     * Chúng ta sẽ dùng cờ nrf_irq_flag.
     */

    /* 1. Kiểm tra cờ ngắt (được set bởi EXTI callback) */
    if (nrf_irq_flag)
    {
        nrf_irq_flag = 0; // Xóa cờ ngay
        uint8_t status = 0;
        
        /* 2. ĐỌC STATUS (Dùng Polling - AN TOÀN) */
        if (nrf24_get_status(&status) == HAL_OK)
        {
            /* 3. Nếu ngắt là do "Đã nhận dữ liệu" (RX_DR) */
            if (status & NRF_STATUS_RX_DR)
            {
                /* 4. ĐỌC PAYLOAD (Dùng Polling - AN TOÀN) */
                // (Hàm này bây giờ đã gọi nrf24_spi_transfer_polling)
                if (nrf24_read_rx_payload_dma((uint8_t*)&rx_frame, sizeof(RC_Frame_t)) == HAL_OK)
                {
                    // === NẾU CODE CHẠY ĐẾN ĐÂY, BẠN ĐÃ THÀNH CÔNG ===
                    last_rx_ms = HAL_GetTick(); 
                    rc_handle_frame();
                }
                
                /* 5. XÓA CỜ NGẮT (Dùng Polling - AN TOÀN) */
                nrf24_write_reg_byte(NRF_REG_STATUS, NRF_STATUS_RX_DR);
            }
            
            /* Xử lý các cờ ngắt khác (nếu cần) */
            if (status & NRF_STATUS_TX_DS) {
                nrf24_write_reg_byte(NRF_REG_STATUS, NRF_STATUS_TX_DS);
            }
            if (status & NRF_STATUS_MAX_RT) {
                nrf24_write_reg_byte(NRF_REG_STATUS, NRF_STATUS_MAX_RT);
                nrf24_flush_tx();
            }
        }
    } // end if(nrf_irq_flag)

    /* 7. Xử lý Failsafe (nếu mất sóng quá 500ms) */
    if (HAL_GetTick() - last_rx_ms > 200) // (Dùng 200ms như file cũ của bạn)
    {
        car_control(CAR_STOP_STATE, 0); // Dừng xe
        if (pServoA) Servo_WriteUS(pServoA, pServoA->us_mid);
        if (pServoB) Servo_WriteUS(pServoB, pServoB->us_mid);
    }
}
