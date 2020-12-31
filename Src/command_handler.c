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
#include "main.h"

/**
  * @brief  Handle Serial Command
  * @param  pdata command buffer pointer
  * @param  len buffer length
  * @retval
  */
cmd_response_t handle_serial_command(uint8_t * pBuff, uint32_t len) {
  return HANDLER_SUCCESS;
}

/***************************************************************END OF FILE****/
