#ifndef SERVO_H
#define SERVO_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

/* Servo 50Hz (period 20ms), xung 1000..2000us (có thể chỉnh) */
typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t channel;
    uint16_t us_min, us_mid, us_max; /* mặc định 1000-1500-2000us */
    uint16_t period_us;              /* mặc định 20000us (50Hz) */
} Servo_t;

/* Timer cho servo phải cấu hình 50Hz (ARR/PSC phù hợp) trước khi attach */
void Servo_Init(Servo_t* s, TIM_HandleTypeDef* htim, uint32_t ch,
                uint16_t us_min, uint16_t us_mid, uint16_t us_max, uint16_t period_us);

/* Đặt trực tiếp độ rộng xung micro-second */
void Servo_WriteUS(Servo_t* s, uint16_t us);

/* Điều khiển theo thang -100..+100: -100->us_min, 0->us_mid, +100->us_max */
void Servo_WritePct(Servo_t* s, int16_t pct);

#endif /* SERVO_H */
