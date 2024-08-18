/**
 * @file tam_usart_commands.h
 * @brief Core interface for controlling the two axis machine with text commands over USART
 */

#ifndef TAM_USART_COMMANDS_H
#define TAM_USART_COMMANDS_H

#include "stm32f4xx_nucleo.h"
#include "tam_error.h"
#include "tam_context.h"

TAM_Status TAM_USART_CheckAppCmd(UART_HandleTypeDef* huart, TAM_Context * const pContex);

#endif