#ifndef MOTOR_H
#define MOTOR_H


#include "stm32f1xx_hal.h"
#include "pwm.h"

typedef enum
{
	MOTOR_STOP,
	MOTOR_CW, // cung chieu kim dong ho
	MOTOR_CCW // nguoc chieu kim dong ho
	
}MOTOR_STATE;


typedef struct {
    GPIO_TypeDef* in1_port;
    uint16_t      in1_pin;
    GPIO_TypeDef* in2_port;
    uint16_t      in2_pin;
    TIM_HandleTypeDef* htim;
    uint32_t      tim_channel; // PWM
} Motor_Typedef;


void motor_init(Motor_Typedef* motor,
                GPIO_TypeDef* in1_port, uint16_t in1_pin,
                GPIO_TypeDef* in2_port, uint16_t in2_pin,
                TIM_HandleTypeDef* htim, uint32_t tim_channel);
void control_motor(Motor_Typedef *motor,MOTOR_STATE state,uint8_t speed);

#endif


