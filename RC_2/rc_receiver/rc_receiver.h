#ifndef RC_RECEIVER_H
#define RC_RECEIVER_H

#include "stm32f1xx_hal.h"
#include "nrf24_dma_driver.h"
#include "nrf24_conf.h"
#include "servo.h"
#include "car.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ================================================= */
/* 1. Ð?NH NGHIA ENUM CHO CÁC CH? Ð? (R?t rõ ràng)   */
/* ================================================= */

// Ch? d? chính (Main Mode)
typedef enum {
    MODE_MANUAL = 0,        // Ði?u khi?n tay
    MODE_LINE_FOLLOWER = 1, // Dò line t? d?ng
    MODE_OBSTACLE_AVOID = 2 // (D? phòng) Tránh v?t c?n
} RC_MainMode_t;

// Ch? d? ph? trong Manual (Sub Mode)
typedef enum {
    SUBMODE_DRIVING = 0,    // Ch?y xe bình thu?ng
    SUBMODE_ARM_CTRL = 1    // Ði?u khi?n cánh tay/k?p
} RC_SubMode_t;

/* ================================================= */
/* 2. C?U TRÚC GÓI TIN RAW (Kh?p v?i Tay c?m g?i)    */
/* ================================================= */
// Luu ý: V?n dùng uint8_t ? dây d? d?m b?o kích thu?c gói tin không b? l?ch
typedef struct __attribute__((packed)) {
    uint8_t  raw_main_mode; // 0: Manual, 1: Line
    uint8_t  raw_sub_mode;  // 0: Driving, 1: Arm
    int16_t  joyL_x;
    int16_t  joyL_y;
    int16_t  joyR_x;
    int16_t  joyR_y;
    uint8_t  reserved[7];   // Nút b?m g?i ? reserved[0]
} RC_Frame_t;

/* ================================================= */
/* 3. C?U TRÚC TR?NG THÁI H? TH?NG (Processed State) */
/* ================================================= */
typedef struct {
    // --- State (Enum) ---
    RC_MainMode_t main_mode;
    RC_SubMode_t  sub_mode;
    
    uint8_t is_connected;   // 1: Có sóng, 0: M?t sóng

    // --- Control Values (Ðã map sang -100 d?n 100) ---
    int16_t speed_val;      // T?c d? (Throttle)
    int16_t turn_val;       // Hu?ng r? (Steering)
    int16_t arm_val;        // Giá tr? di?u khi?n tay g?p
    uint8_t grip_cmd;       // L?nh k?p (0: M?, 1: Ðóng)
} RC_State_t;

/* Bi?n toàn c?c */
extern volatile  RC_State_t sys_state; // Dùng bi?n này d? Debug xem xe dang ? mode nào

/* Hàm API */
void RC_Receiver_Init(Servo_t* p_sv_lift, Servo_t* p_sv_grip);
void RC_Receiver_Task(void);

#ifdef __cplusplus
}
#endif

#endif /* RC_RECEIVER_H */
