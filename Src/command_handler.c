/**
  ******************************************************************************
  * @file    command_handler.c
  * @brief   This file implement handler for USB ATS commands
  ******************************************************************************
  * @attention
  *
  * This software component is reference from Long Dang's ATS-FT3 project
  *
  ******************************************************************************
  */

#include "command_handler.h"

cmd_response_t handle_single_command(uint8_t * pBuff, uint32_t len);

/**
  * @brief  Handle Serial Command
  * @param  pdata command buffer pointer
  * @param  len buffer length
  * @retval
  */
cmd_response_t handle_serial_command(uint8_t * pBuff, uint32_t len) {
  // TODO: queue pBuff and trigger handler in main thread
  return handle_single_command(pBuff, len);
}

/**
  * @brief  Handle single command
  *  Prefix  COMMAND  Size     Payload
  * ------------------------------------------//---
  * | 0xAA | 1-byte | 1-byte | Size-byte(s) ..  ..|
  * ------------------------------------------//---
  * * COMMAND:
  *      0xF0 - COMMAND_INIT_GPIO
  *      0xF1 - COMMAND_SET_PIN_STATE
  *      0xF2 - COMMAND_SPI_TRANSFER
  *
  * * Size and payload:
  *      COMMAND_INIT_GPIO
  *      | 0xAA | 0xF0 | 3 |   PORT   +     PIN     |   DIR   | DEFAULT_STATE |
  *                          PA: 0xA.   0,1,2,..,15   0x49 'I'   0x00: LOW
  *                          PB: 0xB.                 0x4F 'O'   0x01: HIGH
  *                          PC: 0xC.
  *      COMMAND_SET_PIN_STATE
  *      | 0xAA | 0xF1 | 2 |   PORT   +     PIN     |   NEW_STATE   |
  *                          PA: 0xA.   0,1,2,..,15     0x00: LOW
  *                          PB: 0xB.                   0x01: HIGH
  *                          PC: 0xC.
  *      COMMAND_SPI_TRANSFER
  *      | 0xAA | 0xF2 | 2 | HIGH-BYTE | LOW-BYTE |
  * @param  pdata command buffer pointer
  * @param  len buffer length
  * @retval
  */
cmd_response_t handle_single_command(uint8_t * pBuff, uint32_t len) {
  if ((pBuff[0] != COMMAND_HEADER) || (len > COMMAND_MAX_LENGTH)) {
    return HANDLER_UNKNOWN_ERROR;
  }

  uint8_t cmdType = pBuff[1];
  uint8_t payloadSize = pBuff[2];

  switch (cmdType) {
    case COMMAND_INIT_GPIO:
      if (payloadSize != COMMAND_INIT_GPIO_PAYLOAD_SIZE) {
        return HANDLER_UNKNOWN_ERROR;
      }
      break;

    case COMMAND_SET_PIN_STATE:
      if (payloadSize != COMMAND_SET_PIN_STATE_PAYLOAD_SIZE) {
        return HANDLER_UNKNOWN_ERROR;
      }

      uint8_t port = pBuff[3] & COMMAND_PORT_MASK;
      uint8_t pin = pBuff[3] & COMMAND_PIN_MASK;
      GPIO_PinState newState = (pBuff[4]) ? (GPIO_PIN_SET) : (GPIO_PIN_RESET);

      GPIO_TypeDef* pGpioBase = NULL;
      if (port == COMMAND_PORT_A) pGpioBase = GPIOA;
      else if (port == COMMAND_PORT_B) pGpioBase = GPIOB;
      else if (port == COMMAND_PORT_C) pGpioBase = GPIOC;
      else
        return HANDLER_UNKNOWN_ERROR;

      HAL_GPIO_WritePin(pGpioBase, (uint16_t)(0x0001 << pin), newState);
      break;

    case COMMAND_SPI_TRANSFER:
      if (payloadSize != COMMAND_SPI_TRANSFER_PAYLOAD_SIZE) {
        return HANDLER_UNKNOWN_ERROR;
      }
      break;

    default:
      return HANDLER_UNKNOWN_ERROR;
  }

  return HANDLER_SUCCESS;
}

/***************************************************************END OF FILE****/
