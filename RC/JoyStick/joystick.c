/* joystick.c */
#include "joystick.h"
#include "stm32f1xx_hal.h"

extern ADC_HandleTypeDef hadc1;

#define TOTAL_ADC_CHANNELS 2
#define JOY_INVALID_VALUE  65535

// H? s? l?c IIR (nh? = mu?t hon). 0.10 h?p lý cho tay di?u khi?n RC.
#define IIR_ALPHA 0.10f

// ====== Mapping kênh ADC -> index buffer DMA ======
// HÃY CH?C CH?N th? t? quét ADC trong CubeMX trùng mapping này
// (ví d? sequence: CH0 tru?c, CH9 sau).
#define DMA_INDEX_OF_ADC_CHANNEL_0   0
#define DMA_INDEX_OF_ADC_CHANNEL_9   1

// DMA buffer ADC
static volatile uint16_t adc_buffer[TOTAL_ADC_CHANNELS] = {0};

// Giá tr? dã l?c IIR
static float adc_filtered[TOTAL_ADC_CHANNELS] = {0};

// Debug counters & last value
static uint32_t     debug_read_count[MAX_JOYSTICKS] = {0};
static JoyValue_t   debug_last_value[MAX_JOYSTICKS] = {0};
static JoyStick_Error_t debug_last_error = JOY_OK;

// C?u hình joystick (gi? nhu b?n g?c)
const JoyStick_CfgType JoyStick_CfgParam[MAX_JOYSTICKS] = {
    {ADC1, ADC_CHANNEL_9, 0, JOY_AXIS_X},  // Joystick 0: ch? X
    {ADC1, 0, ADC_CHANNEL_0, JOY_AXIS_Y}   // Joystick 1: ch? Y
};

// Chuy?n ADC channel -> index DMA buffer
static uint8_t ADC_ChannelToIndex(uint32_t ch)
{
    switch (ch) {
        case ADC_CHANNEL_0: return DMA_INDEX_OF_ADC_CHANNEL_0;
        case ADC_CHANNEL_9: return DMA_INDEX_OF_ADC_CHANNEL_9;
        default:
            debug_last_error = JOY_ERR_INVALID_CHANNEL;
            return 255;
    }
}

// Kh?i t?o ADC + DMA + warm-up IIR
HAL_StatusTypeDef JoyStick_Init(void)
{
    for (int i = 0; i < TOTAL_ADC_CHANNELS; i++) {
        adc_buffer[i]   = 0;
        adc_filtered[i] = 0.0f;
    }
    for (int i = 0; i < MAX_JOYSTICKS; i++) {
        debug_read_count[i] = 0;
        debug_last_value[i].x = 0;
        debug_last_value[i].y = 0;
    }
    debug_last_error = JOY_OK;

    HAL_ADCEx_Calibration_Start(&hadc1);

    HAL_StatusTypeDef status = HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, TOTAL_ADC_CHANNELS);
    if (status != HAL_OK) {
        debug_last_error = JOY_ERR_DMA_NOT_RUNNING;
        return status;
    }

    // Warm-up IIR: d?i vài chu k? r?i set giá tr? d?u vào cho b? l?c
    HAL_Delay(2);
    for (int i = 0; i < TOTAL_ADC_CHANNELS; i++) {
        adc_filtered[i] = (float)adc_buffer[i];
    }
    return HAL_OK;
}

// Ð?c giá tr? joystick v?i IIR filter
JoyStick_Error_t JoyStick_Read(uint8_t joy_id, uint16_t* value)
{
    if (joy_id >= MAX_JOYSTICKS || value == NULL) {
        debug_last_error = JOY_ERR_INVALID_ID;
        return debug_last_error;
    }

    value[0] = JOY_INVALID_VALUE;
    value[1] = JOY_INVALID_VALUE;

    const JoyStick_CfgType* cfg = &JoyStick_CfgParam[joy_id];

    // Tr?c X
    if (cfg->axis_enable & JOY_AXIS_X) {
        uint8_t idx = ADC_ChannelToIndex(cfg->ADCx_CH);
        if (idx != 255) {
            adc_filtered[idx] = (1.0f - IIR_ALPHA) * adc_filtered[idx] + IIR_ALPHA * adc_buffer[idx];
            value[0] = (uint16_t)adc_filtered[idx];
            debug_last_value[joy_id].x = value[0];
        } else {
            return debug_last_error;
        }
    }

    // Tr?c Y
    if (cfg->axis_enable & JOY_AXIS_Y) {
        uint8_t idx = ADC_ChannelToIndex(cfg->ADCy_CH);
        if (idx != 255) {
            adc_filtered[idx] = (1.0f - IIR_ALPHA) * adc_filtered[idx] + IIR_ALPHA * adc_buffer[idx];
            value[1] = (uint16_t)adc_filtered[idx];
            debug_last_value[joy_id].y = value[1];
        } else {
            return debug_last_error;
        }
    }

    debug_read_count[joy_id]++;
    debug_last_error = JOY_OK;
    return JOY_OK;
}

// ===== Hàm debug =====
uint16_t JoyStick_GetRawADC(uint8_t channel_index)
{
    if (channel_index >= TOTAL_ADC_CHANNELS) return JOY_INVALID_VALUE;
    return adc_buffer[channel_index];
}

uint32_t JoyStick_GetReadCount(uint8_t joy_id)
{
    if (joy_id >= MAX_JOYSTICKS) return 0;
    return debug_read_count[joy_id];
}

void JoyStick_GetLastValue(uint8_t joy_id, uint16_t* value)
{
    if (joy_id >= MAX_JOYSTICKS || value == NULL) return;
    value[0] = debug_last_value[joy_id].x;
    value[1] = debug_last_value[joy_id].y;
}

JoyStick_Error_t JoyStick_GetLastError(void)
{
    return debug_last_error;
}

uint8_t JoyStick_IsDMARunning(void)
{
    // An toàn NULL tránh crash tru?c khi DMA kh?i t?o
    if (hadc1.DMA_Handle == NULL || hadc1.DMA_Handle->Instance == NULL) return 0;
    return (hadc1.DMA_Handle->Instance->CCR & DMA_CCR_EN) ? 1 : 0;
}

void JoyStick_ResetDebugCounters(void)
{
    for (int i = 0; i < MAX_JOYSTICKS; i++) debug_read_count[i] = 0;
    debug_last_error = JOY_OK;
}

const char* JoyStick_ErrorToString(JoyStick_Error_t error)
{
    switch (error) {
        case JOY_OK:                  return "OK";
        case JOY_ERR_INVALID_ID:      return "INVALID_ID";
        case JOY_ERR_INVALID_CHANNEL: return "INVALID_CHANNEL";
        case JOY_ERR_ADC_NOT_READY:   return "ADC_NOT_READY";
        case JOY_ERR_DMA_NOT_RUNNING: return "DMA_NOT_RUNNING";
        default:                      return "UNKNOWN";
    }
}
