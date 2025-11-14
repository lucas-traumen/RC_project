#ifndef __JOYSTICK_H
#define __JOYSTICK_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

// S? lu?ng joystick t?i da
#define MAX_JOYSTICKS   2

// --- Tr?c joystick có th? dùng ---
typedef enum {
    JOY_AXIS_NONE = 0,
    JOY_AXIS_X    = 1 << 0,
    JOY_AXIS_Y    = 1 << 1,
    JOY_AXIS_BOTH = (JOY_AXIS_X | JOY_AXIS_Y)
} JoyStick_AxisEnable_t;

// --- C?u hình cho m?i joystick ---
typedef struct {
    ADC_TypeDef* ADC_Instance;     // ADC dùng cho joystick
    uint32_t     ADCx_CH;          // Channel X (0 n?u không dùng)
    uint32_t     ADCy_CH;          // Channel Y (0 n?u không dùng)
    JoyStick_AxisEnable_t axis_enable; // Tr?c s? d?ng
} JoyStick_CfgType;

// --- Struct luu giá tr? 2 tr?c ---
typedef struct {
    uint16_t x;
    uint16_t y;
} JoyValue_t;

// --- Mã l?i ---
typedef enum {
    JOY_OK = 0,                    // Thành công
    JOY_ERR_INVALID_ID,            // ID joystick không h?p l?
    JOY_ERR_INVALID_CHANNEL,       // ADC channel không h?p l?
    JOY_ERR_ADC_NOT_READY,         // ADC chua s?n sàng
    JOY_ERR_DMA_NOT_RUNNING        // DMA không ho?t d?ng
} JoyStick_Error_t;

// ===== HÀM CHÍNH =====

/**
 * @brief  Kh?i t?o ADC và DMA cho joystick
 * @retval HAL_StatusTypeDef - Tr?ng thái kh?i t?o
 */
HAL_StatusTypeDef JoyStick_Init(void);

/**
 * @brief  Ð?c giá tr? joystick
 * @param  joy_id: ID joystick (0 d?n MAX_JOYSTICKS-1)
 * @param  value: M?ng luu giá tr? [0]=X, [1]=Y
 * @retval JoyStick_Error_t - Mã l?i (JOY_OK n?u thành công)
 */
JoyStick_Error_t JoyStick_Read(uint8_t joy_id, uint16_t* value);

// ===== HÀM DEBUG =====

/**
 * @brief  L?y giá tr? raw t? DMA buffer
 * @param  channel_index: Index c?a channel trong DMA (0 ho?c 1)
 * @retval uint16_t - Giá tr? ADC (65535 n?u l?i)
 */
uint16_t JoyStick_GetRawADC(uint8_t channel_index);

/**
 * @brief  L?y s? l?n d?c thành công c?a joystick
 * @param  joy_id: ID joystick
 * @retval uint32_t - S? l?n d?c
 */
uint32_t JoyStick_GetReadCount(uint8_t joy_id);

/**
 * @brief  L?y giá tr? d?c g?n nh?t
 * @param  joy_id: ID joystick
 * @param  value: M?ng luu giá tr? [0]=X, [1]=Y
 */
void JoyStick_GetLastValue(uint8_t joy_id, uint16_t* value);

/**
 * @brief  L?y mã l?i g?n nh?t
 * @retval JoyStick_Error_t - Mã l?i
 */
JoyStick_Error_t JoyStick_GetLastError(void);

/**
 * @brief  Ki?m tra DMA có dang ch?y không
 * @retval uint8_t - 1 n?u dang ch?y, 0 n?u d?ng
 */
uint8_t JoyStick_IsDMARunning(void);

/**
 * @brief  Reset t?t c? debug counters
 */
void JoyStick_ResetDebugCounters(void);

/**
 * @brief  Chuy?n mã l?i sang chu?i (d? printf)
 * @param  error: Mã l?i
 * @retval const char* - Tên l?i d?ng chu?i
 */
const char* JoyStick_ErrorToString(JoyStick_Error_t error);

#endif /* __JOYSTICK_H */
