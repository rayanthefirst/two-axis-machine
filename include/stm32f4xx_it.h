/**
  ******************************************************************************
  * @file    stm32f4xx_it.h
  * @date    13/05/2015 09:14:38
  * @brief   This file contains the headers of the interrupt handlers.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2015 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F4xx_IT_H
#define __STM32F4xx_IT_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "stddef.h"
#include "stdint.h"

#include "gpio_defs.h"

#define MAX_EXTI_PINS 16

/**
  * @addtogroup MicrosteppingMotor_Example
  * @{
  */

/**
  * @defgroup STM32F4XX_IT
  * @{
  */

// Define a function pointer type for the interrupt handler
typedef void* EXTI_CallbackArg_t;
typedef void (*EXTI_CallbackFunc_t)(const GPIO_Pin_t, EXTI_CallbackArg_t);

typedef struct {
  EXTI_CallbackFunc_t func;
  EXTI_CallbackArg_t arg;
} EXTI_Callback;

typedef struct {
  uint32_t preemptPriority; 
  uint32_t subPriority;
} EXTI_Config;

typedef void* TIM_CallbackArg_t;
typedef void (*TIM_CallbackFunc_t)(TIM_HandleTypeDef const * const, TIM_CallbackArg_t);

typedef struct {
  TIM_CallbackFunc_t func;
  TIM_CallbackArg_t arg;
  TIM_HandleTypeDef handle;
} TIM_Callback;

typedef struct {
  TIM_Base_InitTypeDef timerConfig; 
  uint32_t preemptPriority; 
  uint32_t subPriority;
} TIM_Callback_Config;

IRQn_Type EXTI_PinToIRQN(const GPIO_Pin_t pin);
void EXTI_Register_Callback(
  const GPIO_Pin_t pin,
  const EXTI_CallbackFunc_t pCallbackFunc,
  const EXTI_CallbackArg_t pCallbackArg,
  EXTI_Config const * const pConfig
);
void EXTI_Unregister_Callback(const GPIO_Pin_t pin);
void EXTI_Enable_IRQ(const GPIO_Pin_t pin);
void EXTI_Disable_IRQ(const GPIO_Pin_t pin);

IRQn_Type TIM_TimerInstanceToIRQN(TIM_TypeDef const * const pTimerInstance);
void TIM_Register_Callback(
  TIM_TypeDef const * const pTimerInstance,
  TIM_CallbackFunc_t pCallbackFunc,
  TIM_CallbackArg_t pCallbackArg, 
  TIM_Callback_Config const * const pConfig
);
void TIM_Unregister_Callback(TIM_TypeDef const * const pTimerInstance);
void TIM_Enable_IRQ(TIM_TypeDef const * const pTimerInstance);
void TIM_Disable_IRQ(TIM_TypeDef const * const pTimerInstance);

/**
  * @defgroup   STM32F4XX_IT_Exported_Functions
  * @brief      Cortex-M4 Processor Interruption and Exception Handlers
  * @{
  */
void EXTI0_IRQHandler(void);
void EXTI1_IRQHandler(void);
void EXTI2_IRQHandler(void);
void EXTI3_IRQHandler(void);
void EXTI4_IRQHandler(void);
void EXTI9_5_IRQHandler(void);
void EXTI15_10_IRQHandler(void);

void SysTick_Handler(void);
void USART2_IRQHandler(void);

/**
  * @}
  */ /* End of STM32F4XX_IT_Exported_Functions */

/**
  * @}
  */ /* End of STM32F4XX_IT */

/**
  * @}
  */ /* End of MicrosteppingMotor_Example */

#ifdef __cplusplus
}
#endif

#endif /* __STM32F4xx_IT_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
