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

/* (C?u trúc frame c?a b?n) */
typedef enum { MAIN_MODE_MANUAL = 0, MAIN_MODE_LINE = 1 } MainMode_t;
typedef enum { SUB_MODE_PWM = 0, SUB_MODE_SERVO = 1 } SubMode_t;
typedef struct __attribute__((packed)) {
    uint8_t  main_mode;
    uint8_t  sub_mode;
    int16_t  joyL_x;
    int16_t  joyL_y;
    int16_t  joyR_x;
    int16_t  joyR_y;
    uint8_t  line_bits;
    uint8_t  reserved[6];
} RC_Frame_t;

/* Bi?n extern ch?a frame nh?n du?c */
extern RC_Frame_t rx_frame;

/* Kh?i t?o Receiver (g?i 1 l?n) */
void RC_Receiver_Init(Servo_t* p_sv_a, Servo_t* p_sv_b);

/* Tác v? Receiver (g?i trong while(1)) */
void RC_Receiver_Task(void);

#ifdef __cplusplus
}
#endif
#endif // RC_RECEIVER_H
