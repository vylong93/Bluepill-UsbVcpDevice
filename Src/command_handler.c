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
#include "stdbool.h"

typedef struct cmd_gpio {
  GPIO_TypeDef* pGpioBase;
  GPIO_PinState state;
  uint16_t pin;
} cmd_gpio_t;

bool parse_gpio_struct(uint8_t portPin, uint8_t state, cmd_gpio_t * pOut);

cmd_response_t handle_single_command(uint8_t * pBuff, uint32_t len);
cmd_response_t set_pin_output_state(uint8_t portPin, uint8_t state);
cmd_response_t init_port_pin(uint8_t portPin, uint8_t direction, uint8_t state);

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
      return init_port_pin(pBuff[3] /* Port+Pin */, pBuff[4] /* Direction */, pBuff[5] /* New State */);

    case COMMAND_SET_PIN_STATE:
      if (payloadSize != COMMAND_SET_PIN_STATE_PAYLOAD_SIZE) {
        return HANDLER_UNKNOWN_ERROR;
      }
      return set_pin_output_state(pBuff[3] /* Port+Pin */, pBuff[4] /* New State */);

    case COMMAND_SPI_TRANSFER:
      if (payloadSize != COMMAND_SPI_TRANSFER_PAYLOAD_SIZE) {
        return HANDLER_UNKNOWN_ERROR;
      }
      // TODO
      break;

    default:
      return HANDLER_UNKNOWN_ERROR;
  }

  return HANDLER_SUCCESS;
}

cmd_response_t set_pin_output_state(uint8_t portPin, uint8_t state) {
  cmd_gpio_t gpioTarget;
  if (!parse_gpio_struct(portPin, state, &gpioTarget))
    return HANDLER_UNKNOWN_ERROR;

  HAL_GPIO_WritePin(gpioTarget.pGpioBase, gpioTarget.pin, gpioTarget.state);
  return HANDLER_SUCCESS;
}

cmd_response_t init_port_pin(uint8_t portPin, uint8_t direction, uint8_t state) {
  cmd_gpio_t gpioTarget;
  if (!parse_gpio_struct(portPin, state, &gpioTarget))
    return HANDLER_UNKNOWN_ERROR;

  HAL_GPIO_WritePin(gpioTarget.pGpioBase, gpioTarget.pin, gpioTarget.state);

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = gpioTarget.pin;
  if (direction == COMMAND_PIN_DIR_OUTPUT) {
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  } else if (direction == COMMAND_PIN_DIR_INPUT) {
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
  } else {
    return HANDLER_UNKNOWN_ERROR;
  }

  HAL_GPIO_Init(gpioTarget.pGpioBase, &GPIO_InitStruct);
  return HANDLER_SUCCESS;
}

bool parse_gpio_struct(uint8_t portPin, uint8_t state, cmd_gpio_t * pOut) {
  uint8_t port = portPin & COMMAND_PORT_MASK;
  uint8_t pin = portPin & COMMAND_PIN_MASK;

  if (state == COMMAND_PIN_STATE_LOW)
    pOut->state = GPIO_PIN_RESET;
  else if (state == COMMAND_PIN_STATE_HIGH)
    pOut->state = GPIO_PIN_SET;
  else
      return false;

  if (port == COMMAND_PORT_A)
    pOut->pGpioBase = GPIOA;
  else if (port == COMMAND_PORT_B)
    pOut->pGpioBase = GPIOB;
  else if (port == COMMAND_PORT_C)
    pOut->pGpioBase = GPIOC;
  else
    return false;

  pOut->pin = (uint16_t)(0x0001 << pin);
  return true;
}

/***************************************************************END OF FILE****/
