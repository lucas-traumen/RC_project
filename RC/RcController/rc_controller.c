#include "rc_controller.h"
#include "main.h"
#include <string.h>

/* CONFIG GPIO */
#define BTN_MODE_GPIO_Port GPIOB
#define BTN_MODE_Pin       GPIO_PIN_0
#define BTN_SUB_MODE_GPIO_Port GPIOA
#define BTN_SUB_MODE_Pin       GPIO_PIN_7

#define DEBOUNCE_MS        15
#define SEND_PERIOD_MS     50

// ? Ð?NH NGHIA BI?N TOÀN C?C (không ph?i extern)
RC_Frame_t frame;

// ? KH?I T?O DEBOUNCE
static Debounce_t joy_btn_db;
static Debounce_t btn_mode_db;

/* TX status */
static MainMode_t main_mode = MAIN_MODE_MANUAL;
static SubMode_t sub_mode   = SUB_MODE_PWM;

static RC_TxStatus_t rc_tx_status = RC_TX_IDLE;

/* Stub battery và line sensor */
static uint8_t Read_LineSensors(void) { return 0; }

/* Hàm ki?m tra joystick nút nh?n trái */
__weak uint8_t Joy_Left_Pressed(void) {
    return HAL_GPIO_ReadPin(BTN_SUB_MODE_GPIO_Port, BTN_SUB_MODE_Pin);
}

/* Chuy?n main mode */
static void Toggle_MainMode(void)
{
    main_mode = (main_mode == MAIN_MODE_MANUAL) ? MAIN_MODE_LINE : MAIN_MODE_MANUAL;
    frame.main_mode = main_mode;
}

/* Chuy?n sub mode */
static void Toggle_SubMode(void)
{
    sub_mode = (sub_mode == SUB_MODE_PWM) ? SUB_MODE_SERVO : SUB_MODE_PWM;
    frame.sub_mode = sub_mode;
}

/* Non-blocking send dùng DMA */
HAL_StatusTypeDef RC_Controller_Send_NB(void)
{
    if (rc_tx_status == RC_TX_BUSY)
        return HAL_BUSY;

    rc_tx_status = RC_TX_BUSY;

    // 1) Ðua module sang TX mode
    uint8_t cfg;
    if (nrf24_read_reg_byte(NRF_REG_CONFIG, &cfg) != HAL_OK) {
        rc_tx_status = RC_TX_ERROR;
        return HAL_ERROR;
    }
    cfg &= ~NRF_CONFIG_PRIM_RX;
    if (nrf24_write_reg_byte(NRF_REG_CONFIG, cfg) != HAL_OK) {
        rc_tx_status = RC_TX_ERROR;
        return HAL_ERROR;
    }

    // 2) Ghi payload b?ng DMA
    if (nrf24_write_tx_payload_dma((uint8_t*)&frame, sizeof(frame)) != HAL_OK) {
        nrf24_flush_tx();
        rc_tx_status = RC_TX_ERROR;
        return HAL_ERROR;
    }

    return HAL_OK;
}

/* Tr? tr?ng thái TX hi?n t?i */
RC_TxStatus_t RC_Controller_GetTxStatus(void)
{
    return rc_tx_status;
}

/* Callback g?i khi DMA SPI NRF24 hoàn t?t */
void RC_Controller_DMA_Callback(void)
{
    uint8_t status;
    if (nrf24_get_status(&status) == HAL_OK) {
        if (status & NRF_TX_DS_FLAG) {
            nrf24_write_reg_byte(NRF_REG_STATUS, NRF_TX_DS_FLAG);
            rc_tx_status = RC_TX_DONE;
        } else if (status & NRF_MAX_RT_FLAG) {
            nrf24_write_reg_byte(NRF_REG_STATUS, NRF_MAX_RT_FLAG);
            nrf24_flush_tx();
            rc_tx_status = RC_TX_ERROR;
        } else {
            rc_tx_status = RC_TX_BUSY;
        }
    } else {
        rc_tx_status = RC_TX_ERROR;
    }
}

/* C?p nh?t joystick + sensor */
void RC_Controller_Update(void)
{
    /* ===============================
       1) DEBOUNCE nút MODE PB0
       =============================== */
    Debounce_Update(&btn_mode_db,
                    HAL_GPIO_ReadPin(BTN_MODE_GPIO_Port, BTN_MODE_Pin));

    if (Debounce_IsPressedEdge(&btn_mode_db))
    {
        Toggle_MainMode();
    }

    /* ============================================
       2) DEBOUNCE nút joystick
       ============================================ */
    Debounce_Update(&joy_btn_db, Joy_Left_Pressed());

    if (main_mode == MAIN_MODE_MANUAL)
    {
        if (Debounce_IsPressedEdge(&joy_btn_db))
        {
            Toggle_SubMode();
        }
    }

    /* ===============================
       3) Ð?c Joystick
       =============================== */
    uint16_t joyL[2], joyR[2];
    JoyStick_Read(0, joyL);
    JoyStick_Read(1, joyR);

    frame.joyL_x = joyL[0];
    frame.joyL_y = joyL[1];
    frame.joyR_x = joyR[0];
    frame.joyR_y = joyR[1];

    /* ===============================
       4) Line sensor
       =============================== */
    frame.line_bits = Read_LineSensors();
}

/* Kh?i t?o controller */
void RC_Controller_Init(void)
{
    // ? KH?I T?O DEBOUNCE
    Debounce_Init(&joy_btn_db, 1, DEBOUNCE_MS);  // Pull-up = 1 khi không nh?n
    Debounce_Init(&btn_mode_db, 0, DEBOUNCE_MS); // No pull = 0 khi không nh?n
    
    // Kh?i t?o frame
    memset(&frame, 0, sizeof(frame));
    frame.main_mode = main_mode;
    frame.sub_mode  = sub_mode;
}
