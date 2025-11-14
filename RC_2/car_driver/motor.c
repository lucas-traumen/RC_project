
	#include "motor.h"



void control_motor(Motor_Typedef *motor, MOTOR_STATE state, uint8_t speed)
{
    switch(state)
    {
        case MOTOR_STOP:
            HAL_GPIO_WritePin(motor->in1_port, motor->in1_pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(motor->in2_port, motor->in2_pin, GPIO_PIN_RESET);
            handle_pwm__set_duty(motor->htim, motor->tim_channel, 0);
            break;

        case MOTOR_CW: // quay thu?n
            HAL_GPIO_WritePin(motor->in1_port, motor->in1_pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(motor->in2_port, motor->in2_pin, GPIO_PIN_RESET);
            handle_pwm__set_duty(motor->htim, motor->tim_channel, speed);
            break;

        case MOTOR_CCW: // quay ngu?c
            HAL_GPIO_WritePin(motor->in1_port, motor->in1_pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(motor->in2_port, motor->in2_pin, GPIO_PIN_SET);
            handle_pwm__set_duty(motor->htim, motor->tim_channel, speed);
            break;
    }
}


	void motor_init(Motor_Typedef* motor,
                GPIO_TypeDef* in1_port, uint16_t in1_pin,
                GPIO_TypeDef* in2_port, uint16_t in2_pin,
                TIM_HandleTypeDef* htim, uint32_t tim_channel)
{
    motor->in1_port = in1_port;
    motor->in1_pin  = in1_pin;
    motor->in2_port = in2_port;
    motor->in2_pin  = in2_pin;
    motor->htim     = htim;
    motor->tim_channel = tim_channel;
		HAL_TIM_PWM_Start(htim, tim_channel);
	}



