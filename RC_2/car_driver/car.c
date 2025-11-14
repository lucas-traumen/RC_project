#include "car.h"

static Motor_Typedef motor_right;
static Motor_Typedef motor_left;

void car_control(Car_Typedef car_state, uint8_t speed)
{
    switch (car_state)
    {
        case CAR_STOP_STATE:
            control_motor(&motor_right, MOTOR_STOP, 0);
            control_motor(&motor_left,  MOTOR_STOP, 0);
            break;

        case CAR_FORWARD_STATE:
            control_motor(&motor_right, MOTOR_CW,  speed);   // phải: thuận
            control_motor(&motor_left,  MOTOR_CCW, speed);   // trái: nghịch
            break;

        case CAR_BACKWARD_STATE:
            control_motor(&motor_right, MOTOR_CCW, speed);   // phải: nghịch
            control_motor(&motor_left,  MOTOR_CW,  speed);   // trái: thuận
            break;

        case CAR_LEFT_STATE:
            control_motor(&motor_right, MOTOR_CW,  speed);
            control_motor(&motor_left,  MOTOR_STOP, 0);
            break;

        case CAR_RIGHT_STATE:
            control_motor(&motor_right, MOTOR_STOP, 0);
            control_motor(&motor_left,  MOTOR_CW,  speed);
            break;
    }
}

void car_init(TIM_HandleTypeDef *htim_pwm)
{
    /* Chân điều khiển lấy theo phần cứng hiện tại (INx ở PORTB):
       LEFT: IN3=PB13, IN4=PB12 -> TIM ch2
       RIGHT:IN1=PB14, IN2=PB15 -> TIM ch1
       (giữ nguyên như bạn đang dùng) */
    motor_init(&motor_left,  GPIOB, GPIO_PIN_0, GPIOB, GPIO_PIN_1, htim_pwm, TIM_CHANNEL_3);
    motor_init(&motor_right, GPIOB, GPIO_PIN_10, GPIOB, GPIO_PIN_11, htim_pwm, TIM_CHANNEL_4);
    car_control(CAR_STOP_STATE, 0);
}

/* ====== Điều khiển vi-sai liên tục (-100..+100) ====== */
static uint8_t clamp_u8(int v){ if (v < 0) v = 0; if (v > 100) v = 100; return (uint8_t)v; }

void car_drive_percent(int8_t left_pct, int8_t right_pct)
{
    /* RIGHT: + => CW ; LEFT: + => CCW (khớp logic tiến trong car_control) */

    // RIGHT
    if (right_pct > 0)      control_motor(&motor_right, MOTOR_CW,  clamp_u8(right_pct));
    else if (right_pct < 0) control_motor(&motor_right, MOTOR_CCW, clamp_u8(-right_pct));
    else                    control_motor(&motor_right, MOTOR_STOP, 0);

    // LEFT
    if (left_pct > 0)       control_motor(&motor_left,  MOTOR_CCW, clamp_u8(left_pct));
    else if (left_pct < 0)  control_motor(&motor_left,  MOTOR_CW,  clamp_u8(-left_pct));
    else                    control_motor(&motor_left,  MOTOR_STOP, 0);
}
