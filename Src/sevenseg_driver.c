/**
  ******************************************************************************
  * @file    sevenseg_driver.c
  * @brief   This file implement SPI to driving 7-SEG CA tube of 4 digits.
  ******************************************************************************
  * @attention
  *
  * This software component is created base on STM32F1 HAL library.
  *
  ******************************************************************************
  */

#include "sevenseg_driver.h"

extern SPI_HandleTypeDef hspi2; /* main.c */

/**
  * @brief  Display a digit in a specific position
  * @param  position to display range from {SEGLED_0, SEGLED_1, SEGLED_2, SEGLED_3}
  * @param  value digit to display range from {
  *          DIGIT_DOT, DIGIT_NONE, DIGIT_0, DIGIT_1, DIGIT_2, DIGIT_3,
  *          DIGIT_4, DIGIT_5, DIGIT_6, DIGIT_7, DIGIT_8, DIGIT_9,
  *          DIGIT_A, DIGIT_B, DIGIT_C, DIGIT_D, DIGIT_E, DIGIT_F}
  * @retval None
  */
void displayDigit(uint8_t position, uint8_t value)
{
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi2, &value, 1, 1);     // HIGH BYTE - DIGIT_5 - 0x92
  HAL_SPI_Transmit(&hspi2, &position, 1, 1);  // LOW BYTE - SEGLED_1 - 0x02
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
}
/**
  * @brief  Test 7-SEG tube
  * @retval None
  */
void test_sevenseg_tube(void)
{
  /* SPI2 Latch pin */
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  uint8_t patern[] = {DIGIT_DOT, DIGIT_NONE, DIGIT_0, DIGIT_1, DIGIT_2, DIGIT_3,
                      DIGIT_4, DIGIT_5, DIGIT_6, DIGIT_7, DIGIT_8, DIGIT_9,
                      DIGIT_A, DIGIT_B, DIGIT_C, DIGIT_D, DIGIT_E, DIGIT_F };
  displayDigit(0x0F, 0x00);
  HAL_Delay(500);
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 18; j++) {
      displayDigit(1 << i, patern[j]);
      HAL_Delay(100);
    }
  }
}

/***************************************************************END OF FILE****/
