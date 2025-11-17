#include "car.h"

static Motor_Typedef motor_right;
static Motor_Typedef motor_left;

/* Hàm kẹp giá trị (Helper) */
static uint8_t clamp_u8(int v) {
    if (v < 0) v = 0;
    if (v > 100) v = 100;
    return (uint8_t)v;
}

void car_init(TIM_HandleTypeDef *htim_pwm)
{
    /* Cấu hình chân Motor (Dựa trên code cũ của bạn) */
    motor_init(&motor_left,  GPIOB, GPIO_PIN_0, GPIOB, GPIO_PIN_1, htim_pwm, TIM_CHANNEL_3);
    motor_init(&motor_right, GPIOB, GPIO_PIN_10, GPIOB, GPIO_PIN_11, htim_pwm, TIM_CHANNEL_4);
    car_control(CAR_STOP_STATE, 0);
}

/* Điều khiển theo trạng thái cố định (Dùng cho dò line hoặc test) */
void car_control(Car_Typedef car_state, uint8_t speed)
{
    switch (car_state)
    {
        case CAR_STOP_STATE:
            control_motor(&motor_right, MOTOR_STOP, 0);
            control_motor(&motor_left,  MOTOR_STOP, 0);
            break;

        case CAR_FORWARD_STATE:
            // Theo logic cũ: Phải CW, Trái CCW là tiến
            control_motor(&motor_right, MOTOR_CW,  speed);
            control_motor(&motor_left,  MOTOR_CCW, speed);
            break;

        case CAR_BACKWARD_STATE:
            control_motor(&motor_right, MOTOR_CCW, speed);
            control_motor(&motor_left,  MOTOR_CW,  speed);
            break;

        case CAR_LEFT_STATE: // Quay trái tại chỗ
            control_motor(&motor_right, MOTOR_CW,  speed); // Phải tiến
            control_motor(&motor_left,  MOTOR_CW,  speed); // Trái lùi (Logic đảo)
            break;

        case CAR_RIGHT_STATE: // Quay phải tại chỗ
            control_motor(&motor_right, MOTOR_CCW, speed); // Phải lùi
            control_motor(&motor_left,  MOTOR_CCW, speed); // Trái tiến
            break;
            
        default: break;
    }
}

/* * HÀM ĐIỀU KHIỂN TỶ LỆ (JOYSTICK) 
 * Trả về: Trạng thái xe đang thực hiện
 */
Car_Typedef car_drive_percent(int8_t left_pct, int8_t right_pct)
{
    Car_Typedef detected_state = CAR_STOP_STATE;

    // 1. ĐIỀU KHIỂN ĐỘNG CƠ (Logic cũ giữ nguyên)
    
    // --- MOTOR PHẢI ---
    if (right_pct > 0)      control_motor(&motor_right, MOTOR_CW,  clamp_u8(right_pct));
    else if (right_pct < 0) control_motor(&motor_right, MOTOR_CCW, clamp_u8(-right_pct));
    else                    control_motor(&motor_right, MOTOR_STOP, 0);

    // --- MOTOR TRÁI (Lưu ý: Trái tiến là CCW theo logic car_control cũ) ---
    if (left_pct > 0)       control_motor(&motor_left,  MOTOR_CCW, clamp_u8(left_pct));
    else if (left_pct < 0)  control_motor(&motor_left,  MOTOR_CW,  clamp_u8(-left_pct));
    else                    control_motor(&motor_left,  MOTOR_STOP, 0);


    // 2. XÁC ĐỊNH TRẠNG THÁI (Logic mới)
    if (left_pct == 0 && right_pct == 0) 
    {
        detected_state = CAR_STOP_STATE;
    }
    else if (left_pct > 0 && right_pct > 0) 
    {
        detected_state = CAR_FORWARD_STATE; // Cả 2 cùng dương -> Tiến
    }
    else if (left_pct < 0 && right_pct < 0) 
    {
        detected_state = CAR_BACKWARD_STATE; // Cả 2 cùng âm -> Lùi
    }
    else if (left_pct < 0 && right_pct > 0) 
    {
        detected_state = CAR_LEFT_STATE; // Trái lùi, Phải tiến -> Xoay Trái
    }
    else if (left_pct > 0 && right_pct < 0) 
    {
        detected_state = CAR_RIGHT_STATE; // Trái tiến, Phải lùi -> Xoay Phải
    }
    else 
    {
        // Trường hợp: 1 bánh chạy, 1 bánh đứng (Left=100, Right=0)
        // Hoặc các pha cua rộng không xoay tại chỗ.
        detected_state = CAR_MIXED_STATE; 
    }

    return detected_state;
}
