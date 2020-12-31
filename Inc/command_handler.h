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
