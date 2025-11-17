#include "rc_receiver.h"
#include <string.h>
#include "rc_radio_cfg.h"

/* === BIẾN TOÀN CỤC === */
extern volatile uint8_t nrf_irq_flag; 

volatile RC_Frame_t rx_frame;   
volatile RC_State_t sys_state;  

static uint32_t last_rx_ms = 0;
static Servo_t* pServoLift = NULL;
static Servo_t* pServoGrip = NULL;
static uint8_t grip_is_open = 0;
static uint8_t last_grip_btn = 0;

/* === HÀM HỖ TRỢ === */
static int16_t map_axis(int16_t raw_val) {
    // Logic MAP chuẩn: Input 0..4095 -> Output -100..100
    const int16_t MID = 2048;
    int32_t result;
    
    if (raw_val == -1) return 0; // Lỗi phần cứng -> Về 0

    if (raw_val >= MID) {
        result = ((int32_t)(raw_val - MID) * 100) / (4095 - MID);
    } else {
        result = -((int32_t)(MID - raw_val) * 100) / MID;
    }
    return (int16_t)result;
}

static int16_t clamp100(int16_t v) {
    if (v > 100)  return 100;
    if (v < -100) return -100;
    return v;
}

static int16_t deadzone(int16_t v, uint8_t dz) {
    return (v > -dz && v < dz) ? 0 : v;
}

static uint16_t map_arm_to_servo(int16_t arm_pct) {
    return 1500 + (int16_t)((arm_pct * 1000) / 100);
}

/* ========================================================= */
/* GIẢI MÃ GÓI TIN (RAW -> STATE)                            */
/* ========================================================= */
static void rc_decode_frame(void)
{
    // [QUAN TRỌNG] Dòng này giúp TỰ ĐỘNG KẾT NỐI LẠI khi có dữ liệu
    sys_state.is_connected = 1; 
    
    sys_state.main_mode = (RC_MainMode_t)rx_frame.raw_main_mode;
    sys_state.sub_mode  = (RC_SubMode_t)rx_frame.raw_sub_mode;

    // Lấy và Map Joystick
    int16_t joy_Y = deadzone(map_axis(rx_frame.joyL_x), 5); // Tốc độ (Lên xuống)
    int16_t joy_X = deadzone(map_axis(rx_frame.joyR_y), 5); // Hướng (Trái phải)
    
    sys_state.speed_val = joy_Y;

    if (sys_state.sub_mode == SUBMODE_DRIVING) {
        sys_state.turn_val = joy_X;
        sys_state.arm_val  = 0;
    } else {
        sys_state.turn_val = 0;
        sys_state.arm_val  = joy_X;
        
        uint8_t btn = rx_frame.reserved[0];
        if (btn == 1 && last_grip_btn == 0) {
            grip_is_open = !grip_is_open;
        }
        last_grip_btn = btn;
        sys_state.grip_cmd = grip_is_open;
    }
}

/* ========================================================= */
/* MÁY TRẠNG THÁI ĐIỀU KHIỂN XE                              */
/* ========================================================= */
static void rc_run_state_machine(void)
{
    // [FAILSAFE] Nếu mất kết nối -> Dừng xe ngay lập tức
    if (!sys_state.is_connected) {
        car_control(CAR_STOP_STATE, 0);
        return;
    }

    switch (sys_state.main_mode)
    {
        case MODE_LINE_FOLLOWER:
            car_control(CAR_STOP_STATE, 0); // Chờ code Line
            break;

        case MODE_MANUAL:
            if (sys_state.sub_mode == SUBMODE_DRIVING) {
                // Lái xe
                int16_t L = clamp100(sys_state.speed_val + sys_state.turn_val);
                int16_t R = clamp100(sys_state.speed_val - sys_state.turn_val);
                car_drive_percent((int8_t)L, (int8_t)R);
            } else {
                // Tay máy
                car_drive_percent((int8_t)sys_state.speed_val, (int8_t)sys_state.speed_val);
                
                if (pServoLift) Servo_WriteUS(pServoLift, map_arm_to_servo(sys_state.arm_val));
                if (pServoGrip) Servo_WriteUS(pServoGrip, grip_is_open ? 1800 : 1000);
            }
            break;

        default:
            car_control(CAR_STOP_STATE, 0);
            break;
    }
}

/* ========================================================= */
/* INIT                                                      */
/* ========================================================= */
void RC_Receiver_Init(Servo_t* p_sv_lift, Servo_t* p_sv_grip)
{
    pServoLift = p_sv_lift;
    pServoGrip = p_sv_grip;
    
    // Xóa sạch bộ nhớ đệm khi khởi động
    memset((void*)&rx_frame, 0, sizeof(RC_Frame_t));
    memset((void*)&sys_state, 0, sizeof(RC_State_t));
    
    last_rx_ms = HAL_GetTick();
    grip_is_open = 0;
    last_grip_btn = 0;

    // Config NRF24
    nrf24_ce_low();
    nrf24_write_reg_byte(NRF_REG_CONFIG, 0x0F); 
    nrf24_write_reg_byte(NRF_REG_EN_AA, 0x00);
    nrf24_write_reg_byte(NRF_REG_EN_RXADDR, 0x01);
    nrf24_write_reg_byte(NRF_REG_SETUP_AW, 0x03);
    nrf24_write_reg_byte(NRF_REG_SETUP_RETR, 0x00);
    nrf24_write_reg_byte(NRF_REG_RF_CH, RC_RF_CH);
    nrf24_write_reg_byte(NRF_REG_RF_SETUP, RC_RF_SETUP);
    nrf24_write_register(NRF_REG_RX_ADDR_P0, RC_ADDR, 5);
    nrf24_write_register(NRF_REG_TX_ADDR, RC_ADDR, 5);
    nrf24_write_reg_byte(NRF_REG_RX_PW_P0, (uint8_t)sizeof(RC_Frame_t));
    nrf24_write_reg_byte(NRF_REG_DYNPD, 0x00);
    nrf24_write_reg_byte(NRF_REG_FEATURE, 0x00);
    nrf24_write_reg_byte(NRF_REG_STATUS, 0x70);
    nrf24_flush_rx();
    nrf24_flush_tx();
    HAL_Delay(10);
    nrf24_set_rx_mode();
}

/* ========================================================= */
/* TASK                                                      */
/* ========================================================= */
void RC_Receiver_Task(void)
{
    uint32_t now = HAL_GetTick();
    
    // 1. KIỂM TRA MẤT KẾT NỐI (Time-out)
    // Nếu quá 200ms không nhận được gì -> TỰ ĐỘNG NGẮT
    if (now - last_rx_ms > 200) {
        sys_state.is_connected = 0; // Set về 0
        rc_run_state_machine();     // Dừng xe
        // Không return ở đây để cho phép nó check IRQ ở dưới ngay khi có sóng lại
    }

    // 2. KIỂM TRA CÓ SÓNG LẠI KHÔNG (IRQ)
    if (nrf_irq_flag) {
        nrf_irq_flag = 0;
        
        uint8_t status;
        if (nrf24_get_status(&status) == HAL_OK && (status & NRF_STATUS_RX_DR)) {
            
            uint8_t temp_buf[sizeof(RC_Frame_t)];
            if (nrf24_read_rx_payload_dma(temp_buf, sizeof(RC_Frame_t)) == HAL_OK) {
                
                // Copy dữ liệu mới vào
                memcpy((void*)&rx_frame, temp_buf, sizeof(RC_Frame_t));
                
                // Cập nhật thời gian nhận cuối cùng
                last_rx_ms = now; 
                
                // === TỰ ĐỘNG KẾT NỐI LẠI TẠI ĐÂY ===
                rc_decode_frame();      // Trong hàm này sẽ set is_connected = 1
                rc_run_state_machine(); // Xe chạy lại ngay
            }
            
            nrf24_write_reg_byte(NRF_REG_STATUS, NRF_STATUS_RX_DR);
        }
        
        if (status & NRF_STATUS_TX_DS) 
            nrf24_write_reg_byte(NRF_REG_STATUS, NRF_STATUS_TX_DS);
        if (status & NRF_STATUS_MAX_RT) 
            nrf24_write_reg_byte(NRF_REG_STATUS, NRF_STATUS_MAX_RT);
    }
}
