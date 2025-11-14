#ifndef CAR_H
#define CAR_H

#include "motor.h"

typedef enum {
    CAR_STOP_STATE = 0,
    CAR_FORWARD_STATE,
    CAR_BACKWARD_STATE,
    CAR_LEFT_STATE,
    CAR_RIGHT_STATE,
} Car_Typedef;

/* Khởi tạo 2 motor (dùng timer PWM đã cấu hình cho bánh xe) */
void car_init(TIM_HandleTypeDef *htim_pwm);

/* Điều khiển rời rạc (giữ nguyên logic cũ) */
void car_control(Car_Typedef car_state, uint8_t speed);

/* Điều khiển vi-sai theo phần trăm: -100..+100
   + là tiến, - là lùi.
   Mapping giữ nguyên: Bánh phải + => CW, Bánh trái + => CCW. */
void car_drive_percent(int8_t left_pct, int8_t right_pct);

#endif /* CAR_H */
