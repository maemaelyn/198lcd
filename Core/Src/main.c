#include "main.h"
#include "main.h"
#include <stdio.h>
#include <string.h>
#include "stm32f4xx_hal.h"
#include "stdlib.h"

#define I2C_ADDR 0x27 // I2C address of the PCF8574
#define RS_BIT 0 // Register select bit
#define EN_BIT 2 // Enable bit
#define BL_BIT 3 // Backlight bit
#define D4_BIT 4 // Data 4 bit
#define D5_BIT 5 // Data 5 bit
#define D6_BIT 6 // Data 6 bit
#define D7_BIT 7 // Data 7 bit
#define LCD_ROWS 2 // Number of rows on the LCD
#define LCD_COLS 16 // Number of columns on the LCD

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);

uint8_t backlight_state = 1; //is this necessary?
/* Private variables */
ADC_HandleTypeDef hadc1;
I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart2;

void lcd_write_nibble(uint8_t nibble, uint8_t rs);
void lcd_send_cmd(uint8_t cmd);
void lcd_send_data(uint8_t data);
void lcd_init();
void lcd_write_string(char *str);
void lcd_set_cursor(uint8_t row, uint8_t column);
void lcd_clear(void);
void lcd_backlight(uint8_t state);

//char int_to_str[10];
//int count=0;

uint16_t ReadPulseSensor(void) {
    ADC_ChannelConfTypeDef sConfig = {0}; //is this necessary?
    ADC_HandleTypeDef hadc1;
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
    return HAL_ADC_GetValue(&hadc1);
}

int main(void)
{
  HAL_Init();

  SystemClock_Config();

  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  lcd_init();
  lcd_backlight(1);
  uint16_t pulseData;
  uint16_t ReadPulseSensor(void) {
      // Implement your pulse sensor reading code here
      // For example, read analog data using ADC
      ADC_HandleTypeDef hadc1;
      HAL_ADC_Start(&hadc1);
      HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
      return HAL_ADC_GetValue(&hadc1);
  }


  while (1)
  {
  	 pulseData= ReadPulseSensor();
  	 char pulseStr[5];
  	  sprintf(pulseStr, "%d", pulseData);
  	  lcd_clear();
  	  lcd_set_cursor(0, 0);
  	  lcd_write_string("BPM: ");
  	  lcd_set_cursor(1, 0);
  	  HAL_GPIO_TogglePin(GPIOA, LD2_Pin);
  	  //lcd_write_string(pulseData);//is this how you do it?
  	  //lcd_write_string(int_to_str);
  	  //count++;
  	  //memset(int_to_str, 0, sizeof(int_to_str));
  	  HAL_Delay(1500);
  }

}


void lcd_write_nibble(uint8_t nibble, uint8_t rs) {
 uint8_t data = nibble << D4_BIT;
 data |= rs << RS_BIT;
 data |= backlight_state << BL_BIT; // Include backlight state in data
 data |= 1 << EN_BIT;
 HAL_I2C_Master_Transmit(&hi2c1, I2C_ADDR << 1, &data, 1, 100);
 HAL_Delay(1);
 data &= ~(1 << EN_BIT);
 HAL_I2C_Master_Transmit(&hi2c1, I2C_ADDR << 1, &data, 1, 100);
}

void lcd_send_cmd(uint8_t cmd) {
 uint8_t upper_nibble = cmd >> 4;
 uint8_t lower_nibble = cmd & 0x0F;
 lcd_write_nibble(upper_nibble, 0);
 lcd_write_nibble(lower_nibble, 0);
 if (cmd == 0x01 || cmd == 0x02) {
   HAL_Delay(2);
 }
}

void lcd_send_data(uint8_t data) {
 uint8_t upper_nibble = data >> 4;
 uint8_t lower_nibble = data & 0x0F;
 lcd_write_nibble(upper_nibble, 1);
 lcd_write_nibble(lower_nibble, 1);
}

void lcd_init() {
 HAL_Delay(50);
 lcd_write_nibble(0x03, 0);
 HAL_Delay(5);
 lcd_write_nibble(0x03, 0);
 HAL_Delay(1);
 lcd_write_nibble(0x03, 0);
 HAL_Delay(1);
 lcd_write_nibble(0x02, 0);
 lcd_send_cmd(0x28);
 lcd_send_cmd(0x0C);
 lcd_send_cmd(0x06);
 lcd_send_cmd(0x01);
 HAL_Delay(2);
}

void lcd_write_string(char *str) {
 while (*str) {
   lcd_send_data(*str++);
 }
}

void lcd_set_cursor(uint8_t row, uint8_t column) {
   uint8_t address;
   switch (row) {
       case 0:
           address = 0x00;
           break;
       case 1:
           address = 0x40;
           break;
       default:
           address = 0x00;
   }
   address += column;
   lcd_send_cmd(0x80 | address);
}

void lcd_clear(void) {
	lcd_send_cmd(0x01);
   HAL_Delay(2);
}

void lcd_backlight(uint8_t state) {
 if (state) {
   backlight_state = 1;
 } else {
   backlight_state = 0;
 }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

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
}

static void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */
  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */
  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */
  /* USER CODE END I2C1_Init 2 */

}

static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }

}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

void Error_Handler(void)
{
 __disable_irq();
 while (1)
 {
 }
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
