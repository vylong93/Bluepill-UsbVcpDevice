/**
  ******************************************************************************
  * @file           : command_handler.h
  * @brief          : Header for command_handler.c file.
  *                   This file contains the common defines for USB ATS Command
  ******************************************************************************
  * @attention
  *
  * This software component is reference from Long Dang's ATS-FT3 project
  *
  ******************************************************************************
  */

#ifndef __COMMAND_HANDLER_H
#define __COMMAND_HANDLER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

#define COMMAND_HEADER        (0xAA)
#define COMMAND_MAX_LENGTH    (6)

#define COMMAND_INIT_GPIO     (0xF0)
#define COMMAND_SET_PIN_STATE (0xF1)
#define COMMAND_SPI_TRANSFER  (0xF2)

#define COMMAND_INIT_GPIO_PAYLOAD_SIZE     (3)
#define COMMAND_SET_PIN_STATE_PAYLOAD_SIZE (2)
#define COMMAND_SPI_TRANSFER_PAYLOAD_SIZE  (2)

#define COMMAND_PORT_MASK    (0xF0)
#define COMMAND_PORT_A       (0xA0)
#define COMMAND_PORT_B       (0xB0)
#define COMMAND_PORT_C       (0xC0)
#define COMMAND_PIN_MASK     (0x0F)

typedef enum cmd_response {
  HANDLER_SUCCESS = 0,
  HANDLER_UNKNOWN_ERROR = 1
} cmd_response_t;

cmd_response_t handle_serial_command(uint8_t * pBuff, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif /* __COMMAND_HANDLER_H */

/***************************************************************END OF FILE****/
