#include "servo.h"

static inline uint16_t us_to_ccr(TIM_HandleTypeDef* h, uint16_t us, uint16_t period_us)
{
    uint32_t arr1 = (uint32_t)h->Instance->ARR + 1u;
    return (uint16_t)((us * arr1) / period_us);
}

void Servo_Init(Servo_t* s, TIM_HandleTypeDef* htim, uint32_t ch,
                uint16_t us_min, uint16_t us_mid, uint16_t us_max, uint16_t period_us)
{
    s->htim = htim; s->channel = ch;
    s->us_min = (us_min  ? us_min  : 1000);
    s->us_mid = (us_mid  ? us_mid  : 1500);
    s->us_max = (us_max  ? us_max  : 2000);
    s->period_us = (period_us ? period_us : 20000);   // 50Hz

    HAL_TIM_PWM_Start(htim, ch);
    Servo_WriteUS(s, s->us_mid);   // về giữa
}

void Servo_WriteUS(Servo_t* s, uint16_t us)
{
    if (us < s->us_min) us = s->us_min;
    if (us > s->us_max) us = s->us_max;

    uint16_t ccr = us_to_ccr(s->htim, us, s->period_us);
    switch (s->channel) {
        case TIM_CHANNEL_1: s->htim->Instance->CCR1 = ccr; break;
        case TIM_CHANNEL_2: s->htim->Instance->CCR2 = ccr; break;
        case TIM_CHANNEL_3: s->htim->Instance->CCR3 = ccr; break;
        case TIM_CHANNEL_4: s->htim->Instance->CCR4 = ccr; break;
        default: break;
    }
}

void Servo_WritePct(Servo_t* s, int16_t pct)
{
    if (pct > 100) pct = 100;
    if (pct < -100) pct = -100;

    /* -100 -> us_min ; 0 -> us_mid ; +100 -> us_max */
    int32_t span = (int32_t)(s->us_max - s->us_mid);
    int32_t us = (int32_t)s->us_mid + (span * pct) / 100;
    Servo_WriteUS(s, (uint16_t)us);
}
