#ifndef CAR_H
#define CAR_H

#include "motor.h"

typedef enum {
    CAR_STOP_STATE = 0,
    CAR_FORWARD_STATE,
    CAR_BACKWARD_STATE,
    CAR_LEFT_STATE,     // Quay trái (Spin Left)
    CAR_RIGHT_STATE,    // Quay phải (Spin Right)
    CAR_MIXED_STATE     // (Mới) Trạng thái hỗn hợp (vừa đi vừa rẽ)
} Car_Typedef;

/* Khởi tạo */
void car_init(TIM_HandleTypeDef *htim_pwm);

/* Điều khiển theo trạng thái cố định */
void car_control(Car_Typedef car_state, uint8_t speed);

/* * Điều khiển theo phần trăm (-100 đến 100) 
 * VÀ TRẢ VỀ TRẠNG THÁI HIỆN TẠI 
 */
Car_Typedef car_drive_percent(int8_t left_pct, int8_t right_pct);

#endif /* CAR_H */
