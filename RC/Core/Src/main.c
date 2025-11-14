/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "rc_controller.h"
#include "rc_radio_cfg.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi2_tx;
DMA_HandleTypeDef hdma_spi2_rx;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint16_t joy0_value[2]={0};
uint16_t joy1_value[2]={0};
//RC_Frame_t frame;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI2_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//HAL_StatusTypeDef RC_TxRadio_Init(void)
//{
//    // 1. Khởi tạo driver (CE, CSN, DWT delay)
//    nrf24_driver_init();
//    HAL_Delay(10);

//    // 2. Power down trước khi config
//    uint8_t cfg = 0;
//    if (nrf24_write_reg_byte(NRF_REG_CONFIG, cfg) != HAL_OK) return HAL_ERROR;
//    HAL_Delay(5);

//    // 3. Flush FIFO
//    nrf24_flush_tx();
//    nrf24_flush_rx();

//    // 4. Cấu hình RF channel
//    if (nrf24_write_reg_byte(NRF_REG_RF_CH, RC_RF_CH) != HAL_OK) return HAL_ERROR;

//    // 5. Cấu hình RF setup (1 Mbps, 0 dBm)
//    if (nrf24_write_reg_byte(NRF_REG_RF_SETUP, RC_RF_SETUP) != HAL_OK) return HAL_ERROR;

//    // 6. Cấu hình địa chỉ TX
//    if (nrf24_write_register(NRF_REG_TX_ADDR, RC_ADDR, 5) != HAL_OK) return HAL_ERROR;

//    // 7. Cấu hình địa chỉ RX pipe 0 (cần cho ACK)
//    if (nrf24_write_register(NRF_REG_RX_ADDR_P0, RC_ADDR, 5) != HAL_OK) return HAL_ERROR;

//    // 8. Enable Auto-ACK cho pipe 0
//    if (nrf24_write_reg_byte(NRF_REG_EN_AA, 0x01) != HAL_OK) return HAL_ERROR;

//    // 9. Enable RX address pipe 0
//    if (nrf24_write_reg_byte(NRF_REG_EN_RXADDR, 0x01) != HAL_OK) return HAL_ERROR;

//    // 10. Setup address width = 5 bytes
//    if (nrf24_write_reg_byte(NRF_REG_SETUP_AW, 0x03) != HAL_OK) return HAL_ERROR;

//    // 11. Setup retry: 500µs, 5 lần
//    if (nrf24_write_reg_byte(NRF_REG_SETUP_RETR, 0x15) != HAL_OK) return HAL_ERROR;

//    // 12. RX payload width pipe 0 = 16 bytes (kích thước RC_Frame_t)
//    if (nrf24_write_reg_byte(NRF_REG_RX_PW_P0, sizeof(RC_Frame_t)) != HAL_OK) return HAL_ERROR;

//    // 13. Power up, TX mode (PRIM_RX = 0)
//    cfg = NRF_CONFIG_PWR_UP | NRF_CONFIG_EN_CRC;
//    if (nrf24_write_reg_byte(NRF_REG_CONFIG, cfg) != HAL_OK) return HAL_ERROR;
//    HAL_Delay(5);  // Chờ power up

//    // 14. Clear các cờ status
//    if (nrf24_write_reg_byte(NRF_REG_STATUS, 0x70) != HAL_OK) return HAL_ERROR;

//    return HAL_OK;
//}
HAL_StatusTypeDef RC_TxRadio_Init(void)
{
    // 1) CE/CSN/DWT
    nrf24_driver_init();
    HAL_Delay(10);

    // 2) Power-down trước khi cấu hình
    if (nrf24_write_reg_byte(NRF_REG_CONFIG, 0x00) != HAL_OK) return HAL_ERROR;
    HAL_Delay(5);

    // 3) FIFO sạch
    nrf24_flush_tx();
    nrf24_flush_rx();

    // 4) RF: kênh & tốc độ/công suất
    if (nrf24_write_reg_byte(NRF_REG_RF_CH,    RC_RF_CH)    != HAL_OK) return HAL_ERROR;
    if (nrf24_write_reg_byte(NRF_REG_RF_SETUP, RC_RF_SETUP) != HAL_OK) return HAL_ERROR;

    // 5) Địa chỉ (TX_ADDR = RX_ADDR_P0, giữ nguyên cho tiện chuyển qua ACK sau này)
    if (nrf24_write_register(NRF_REG_TX_ADDR,    RC_ADDR, 5) != HAL_OK) return HAL_ERROR;
    if (nrf24_write_register(NRF_REG_RX_ADDR_P0, RC_ADDR, 5) != HAL_OK) return HAL_ERROR;

    // 6) TẮT ACK/DPL (bắn mù, không cần RX)
    if (nrf24_write_reg_byte(NRF_REG_EN_AA,     0x00) != HAL_OK) return HAL_ERROR; // tắt Auto-ACK
    if (nrf24_write_reg_byte(0x1C /*DYNPD*/,    0x00) != HAL_OK) return HAL_ERROR; // tắt DPL
    if (nrf24_write_reg_byte(0x1D /*FEATURE*/,  0x00) != HAL_OK) return HAL_ERROR; // tắt FEATURE

    // (Tuỳ chọn) Nếu muốn cố định chiều dài ở PRX thì mới cần RX_PW_P0.
    // Ở PTX (TX mode) thanh ghi này không bắt buộc; để 0x00 cũng không sao.
    // if (nrf24_write_reg_byte(NRF_REG_RX_PW_P0, sizeof(RC_Frame_t)) != HAL_OK) return HAL_ERROR;

    // 7) Bật address width=5B, bật pipe0 “đủ thủ tục”
    if (nrf24_write_reg_byte(NRF_REG_SETUP_AW,  0x03) != HAL_OK) return HAL_ERROR; // 5 bytes
    if (nrf24_write_reg_byte(NRF_REG_EN_RXADDR, 0x01) != HAL_OK) return HAL_ERROR; // enable pipe0

    // 8) Retry có thể để mặc kệ (ACK tắt nên không dùng); vẫn set an toàn
    if (nrf24_write_reg_byte(NRF_REG_SETUP_RETR, 0x15) != HAL_OK) return HAL_ERROR;

    // 9) Power-up + TX mode (PRIM_RX=0), bật CRC
    uint8_t cfg = NRF_CONFIG_PWR_UP | NRF_CONFIG_EN_CRC;  // PRIM_RX=0 mặc định
    if (nrf24_write_reg_byte(NRF_REG_CONFIG, cfg) != HAL_OK) return HAL_ERROR;
    HAL_Delay(5); // Tpwrup

    // 10) Clear cờ
    if (nrf24_write_reg_byte(NRF_REG_STATUS, 0x70) != HAL_OK) return HAL_ERROR;

    return HAL_OK;
}


void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == &hspi2) {
        nrf24_spi_dma_complete_cb();
    }
}
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == &hspi2) {
        // Vẫn nhả cờ để không kẹt
        nrf24_spi_dma_complete_cb();
    }
}

/* USER CODE END 0 */

/**	
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI2_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
JoyStick_Init();
    NRF24_SPI_DMA_Init(&hspi2);
    
    // ✅ THÊM KHỞI TẠO NRF24
    if (RC_TxRadio_Init() != HAL_OK) {
        HAL_UART_Transmit(&huart1, (uint8_t*)"NRF24 INIT FAIL\r\n", 17, 100);
        Error_Handler();
    }
    
    RC_Controller_Init();
    
    HAL_UART_Transmit(&huart1, (uint8_t*)"RC TX READY\r\n", 13, 100);
    HAL_Delay(1000);

    /* ===== VÒNG LẶP CHÍNH ===== */
    uint32_t last_send = 0;
    const uint32_t SEND_INTERVAL = 50;  // Gửi mỗi 50ms (20Hz)

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	// 1. Cập nhật joystick và button
        RC_Controller_Update();

        // 2. Gửi dữ liệu theo chu kỳ 50ms
        uint32_t now = HAL_GetTick();
        if ((now - last_send) >= SEND_INTERVAL)
        {
            last_send = now;

            // ✅ GỬI ĐÚNG BUFFER FRAME (16 bytes)
            HAL_StatusTypeDef st = nrf24_send_frame_blocking(
                (uint8_t*)&frame, 
                sizeof(RC_Frame_t), 
                20  // Timeout 20ms
            );

            if (st == HAL_OK) {
                HAL_UART_Transmit(&huart1, (uint8_t*)"OK\r\n", 4, 10);
          //      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
            } 
            else if (st == HAL_TIMEOUT) {
                HAL_UART_Transmit(&huart1, (uint8_t*)"TIMEOUT\r\n", 9, 10);
            }
            else {
                HAL_UART_Transmit(&huart1, (uint8_t*)"ERROR\r\n", 7, 10);
            }
        }

        // 3. Delay nhỏ tránh quay vòng quá nhanh
        HAL_Delay(1);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
