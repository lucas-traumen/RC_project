#ifndef RC_RECEIVER_H
#define RC_RECEIVER_H

#include "stm32f1xx_hal.h"
#include "nrf24_dma_driver.h"
#include "nrf24_conf.h"         /* CE/CSN, timeouts, payload size */
#include "servo.h"
#include "car.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MAIN_MODE_MANUAL = 0,
    MAIN_MODE_LINE   = 1,
} MainMode_t;

typedef enum {
    SUB_MODE_PWM   = 0,
    SUB_MODE_SERVO = 1,
} SubMode_t;

/* Frame chung gi?a TX & RX */
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


/* Truy?n vào 2 servo (dã init 50Hz) d? di?u khi?n gripper */
void RC_Receiver_Init(Servo_t* sv1, Servo_t* sv2);

/* G?i trong while(1): nh?n gói + áp d?ng di?u khi?n (bánh + servo + failsafe) */
void RC_Receiver_Task(void);

#ifdef __cplusplus
}
#endif
#endif /* RC_RECEIVER_H */
