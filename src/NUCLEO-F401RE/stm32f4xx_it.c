/**
 ******************************************************************************
 * @file    stm32f4xx_it.c
 * @date    13/05/2015 09:14:38
 * @brief   Interrupt Service Routines.
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
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
#include "stdbool.h"
#include "stm32f4xx_hal.h"
#include "trace.h"
#include "string.h"

/**
 * @addtogroup MicrosteppingMotor_Example
 * @{
 */

/**
 * @addtogroup STM32F4XX_IT
 * @{
 */

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */
/******************************************************************************/

/**
 * @addtogroup STM32F4XX_IT_Exported_Functions
 * @{
 */

/**
 * @brief This function handles System tick timer.
 */
void SysTick_Handler(void)
{
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

// Global variable to hold the user-provided callback function
EXTI_Callback exti_callbacks[MAX_EXTI_PINS] = {{0}};
TIM_Callback timCallbacks[12] = {{0}};

static const IRQn_Type ERROR_IRQN_VAL = 255;

IRQn_Type EXTI_PinToIRQN(const GPIO_Pin_t pin)
{
  switch (pin) 
  {
    case GPIO_PIN_0: return EXTI0_IRQn;
    case GPIO_PIN_1: return EXTI1_IRQn;
    case GPIO_PIN_2: return EXTI2_IRQn;
    case GPIO_PIN_3: return EXTI3_IRQn;
    case GPIO_PIN_4: return EXTI4_IRQn;
    case GPIO_PIN_5:
    case GPIO_PIN_6:
    case GPIO_PIN_7:
    case GPIO_PIN_8:
    case GPIO_PIN_9: return EXTI9_5_IRQn;
    case GPIO_PIN_10:
    case GPIO_PIN_11:
    case GPIO_PIN_12:
    case GPIO_PIN_13:
    case GPIO_PIN_14:
    case GPIO_PIN_15: return EXTI15_10_IRQn;
    default: 
      LOG_ERROR("Invalid interrupt pin %u\n\r", pin);
      return ERROR_IRQN_VAL; // Invalid pin
  }
}

void EXTI_Register_Callback(
  const GPIO_Pin_t pin,
  const EXTI_CallbackFunc_t pCallbackFunc,
  const EXTI_CallbackArg_t pCallbackArg,
  EXTI_Config const * const pConfig
)
{
  if (pCallbackFunc == NULL)
  {
    LOG_ERROR("Callback function is NULL");
    return;
  }

  EXTI_Unregister_Callback(pin);

  EXTI_Callback* const pCallback = &exti_callbacks[GPIO_GetLineNumberFromPin(pin)];
  pCallback->func = pCallbackFunc;
  pCallback->arg = pCallbackArg;

  const IRQn_Type irqn = EXTI_PinToIRQN(pin);

  if (irqn == ERROR_IRQN_VAL)
  {
    return;
  }

  // Enable and set EXTI Interrupt priority
  HAL_NVIC_SetPriority(irqn, pConfig->preemptPriority, pConfig->subPriority);
  HAL_NVIC_EnableIRQ(irqn);
  
  LOG_INFO("Enabled EXTI and callback for pin %s with preempt priority of %u and subpriority of %u",
    GPIO_GetPinNumberName(pin), pConfig->preemptPriority, pConfig->subPriority);
}

void EXTI_Unregister_Callback(const GPIO_Pin_t pin)
{
  EXTI_Disable_IRQ(pin);
  memset(&exti_callbacks[GPIO_GetLineNumberFromPin(pin)], 0, sizeof(EXTI_Callback));

  LOG_INFO("Disabled EXTI and callback for pin %s", GPIO_GetPinNumberName(pin));
}

void EXTI_Enable_IRQ(const GPIO_Pin_t pin)
{
  const IRQn_Type irqn = EXTI_PinToIRQN(pin);

  if (irqn == ERROR_IRQN_VAL)
  {
    return;
  }

  HAL_NVIC_EnableIRQ(irqn);

  LOG_DEBUG("Enabled EXTI IRQ for pin %s", GPIO_GetPinNumberName(pin));
}

void EXTI_Disable_IRQ(const GPIO_Pin_t pin)
{
  const IRQn_Type irqn = EXTI_PinToIRQN(pin);

  if (irqn == ERROR_IRQN_VAL)
  {
    return;
  }

  HAL_NVIC_DisableIRQ(irqn);

  LOG_DEBUG("Disabled EXTI IRQ for pin %s", GPIO_GetPinNumberName(pin));
}

void HAL_GPIO_EXTI_Callback(GPIO_Pin_t pin)
{
  LOG_DEBUG("EXTI for pin %s Triggered", GPIO_GetPinNumberName(pin));

  EXTI_Callback const * const pCallback = &exti_callbacks[GPIO_GetLineNumberFromPin(pin)];

  if (pCallback->func != NULL)
  {
    pCallback->func(pin, pCallback->arg);
  }
}

static int TIM_GetTimerNumberFromInstance(TIM_TypeDef* pInstance) {
    switch ((uint32_t)pInstance) 
    {
        case (uint32_t)TIM1: return 1;
        case (uint32_t)TIM2: return 2;
        case (uint32_t)TIM3: return 3;
        case (uint32_t)TIM4: return 4;
        case (uint32_t)TIM5: return 5;
        case (uint32_t)TIM9: return 9;
        case (uint32_t)TIM10: return 10;
        case (uint32_t)TIM11: return 11;
        default: return -1; // Invalid timer instance
    }
}

IRQn_Type TIM_TimerInstanceToIRQN(TIM_TypeDef const * const pTimerInstance)
{
  if (pTimerInstance == TIM1)       { return TIM1_UP_TIM10_IRQn; } 
  else if (pTimerInstance == TIM2)  { return TIM2_IRQn; }
  else if (pTimerInstance == TIM3)  { return TIM3_IRQn; }
  else if (pTimerInstance == TIM4)  { return TIM4_IRQn; }
  else if (pTimerInstance == TIM5)  { return TIM5_IRQn; }
  else if (pTimerInstance == TIM9)  { return TIM1_BRK_TIM9_IRQn; }
  else if (pTimerInstance == TIM10) { return TIM1_UP_TIM10_IRQn; }
  else if (pTimerInstance == TIM11) { return TIM1_TRG_COM_TIM11_IRQn; }
  else
  {
    LOG_ERROR("Invalid timer instance %X", pTimerInstance);
    return ERROR_IRQN_VAL;
  }
}

const char* TIM_GetTimerName(TIM_TypeDef const * const pTimerInstance)
{
  #define TIMER_CASE(timer) if (pTimerInstance == timer) { return (#timer);} 

  TIMER_CASE(TIM1);
  TIMER_CASE(TIM2);
  TIMER_CASE(TIM3);
  TIMER_CASE(TIM4);
  TIMER_CASE(TIM5);
  TIMER_CASE(TIM9);
  TIMER_CASE(TIM10);
  TIMER_CASE(TIM11);

  LOG_ERROR("Invalid timer instance %X", pTimerInstance);
  return NULL;

  #undef TIMER_CASE
}

void TIM_Enable_Timer_Clock(TIM_TypeDef const * const pTimerInstance)
{
  if (pTimerInstance == TIM1)       { __HAL_RCC_TIM1_CLK_ENABLE(); } 
  else if (pTimerInstance == TIM2)  { __HAL_RCC_TIM2_CLK_ENABLE(); }
  else if (pTimerInstance == TIM3)  { __HAL_RCC_TIM3_CLK_ENABLE(); }
  else if (pTimerInstance == TIM4)  { __HAL_RCC_TIM4_CLK_ENABLE(); }
  else if (pTimerInstance == TIM5)  { __HAL_RCC_TIM5_CLK_ENABLE(); }
  else if (pTimerInstance == TIM9)  { __HAL_RCC_TIM9_CLK_ENABLE(); }
  else if (pTimerInstance == TIM10) { __HAL_RCC_TIM10_CLK_ENABLE(); }
  else if (pTimerInstance == TIM11) { __HAL_RCC_TIM11_CLK_ENABLE(); }
  else
  {
    LOG_ERROR("Invalid timer instance %X", pTimerInstance);
  }
}

void TIM_Disable_Timer_Clock(TIM_TypeDef const * const pTimerInstance)
{
  if (pTimerInstance == TIM1)       { __HAL_RCC_TIM1_CLK_DISABLE(); } 
  else if (pTimerInstance == TIM2)  { __HAL_RCC_TIM2_CLK_DISABLE(); }
  else if (pTimerInstance == TIM3)  { __HAL_RCC_TIM3_CLK_DISABLE(); }
  else if (pTimerInstance == TIM4)  { __HAL_RCC_TIM4_CLK_DISABLE(); }
  else if (pTimerInstance == TIM5)  { __HAL_RCC_TIM5_CLK_DISABLE(); }
  else if (pTimerInstance == TIM9)  { __HAL_RCC_TIM9_CLK_DISABLE(); }
  else if (pTimerInstance == TIM10) { __HAL_RCC_TIM10_CLK_DISABLE(); }
  else if (pTimerInstance == TIM11) { __HAL_RCC_TIM11_CLK_DISABLE(); }

  LOG_ERROR("Invalid timer instance %X", pTimerInstance);
}

void TIM_Register_Callback(
  TIM_TypeDef const * const pTimerInstance,
  TIM_CallbackFunc_t pCallbackFunc,
  TIM_CallbackArg_t pCallbackArg, 
  TIM_Callback_Config const * const pConfig
)
{
  if (pCallbackFunc == NULL)
  {
    LOG_ERROR("Callback function is NULL");
    return;
  }

  TIM_Unregister_Callback(pTimerInstance);

  TIM_Callback* const pCallback = &timCallbacks[TIM_GetTimerNumberFromInstance(pTimerInstance)];
  TIM_HandleTypeDef * const pHandle = &pCallback->handle;

  pCallback->func = pCallbackFunc;
  pCallback->arg = pCallbackArg;

  pHandle->Instance = pTimerInstance;
  memcpy(&(pHandle->Init), &(pConfig->timerConfig), sizeof(TIM_Base_InitTypeDef));

  TIM_Enable_Timer_Clock(pTimerInstance);

  // Initialize the timer
  if (HAL_TIM_Base_Init(pHandle) != HAL_OK) 
  {
    LOG_ERROR("Failed to initialize timer");
    return;
  }

  TIM_ClockConfigTypeDef sClockSourceConfig = {
    .ClockSource = TIM_CLOCKSOURCE_INTERNAL
  };
  
  if (HAL_TIM_ConfigClockSource(pHandle, &sClockSourceConfig) != HAL_OK)
  {
    LOG_ERROR("Failed to initialize timer clock source");
    return;
  }

  // Start the timer with interrupt
  if (HAL_TIM_Base_Start_IT(pHandle) != HAL_OK) 
  {
    LOG_ERROR("Failed to initialize timer interrupt");
    return;
  }

  const IRQn_Type irqn = TIM_TimerInstanceToIRQN(pTimerInstance);

  if (irqn == ERROR_IRQN_VAL)
  {
    return;
  }

  // Enable and set EXTI Interrupt priority
  HAL_NVIC_SetPriority(irqn, pConfig->preemptPriority, pConfig->subPriority);
  HAL_NVIC_EnableIRQ(irqn);

  LOG_INFO("Enabled TIM interrupts and callback for timer %s with preempt priority of %u and subpriority of %u",
    TIM_GetTimerName(pTimerInstance), pConfig->preemptPriority, pConfig->subPriority);
}

void TIM_Unregister_Callback(TIM_TypeDef const * const pTimerInstance)
{
  TIM_Disable_IRQ(pTimerInstance);

  TIM_Callback* const pCallback = &timCallbacks[TIM_GetTimerNumberFromInstance(pTimerInstance)];
  TIM_HandleTypeDef * const pHandle = &pCallback->handle;

  // Deinitialize Timer Interrupt
  if (pHandle->Instance != NULL)
  {
    TIM_Disable_Timer_Clock(pHandle->Instance);

    // Stop the timer
    if (HAL_TIM_Base_Stop_IT(pHandle) != HAL_OK) 
    {
      LOG_ERROR("Failed to deinitialize timer interrupt");
      return;
    }

    // De-Initialize the timer
    if (HAL_TIM_Base_DeInit(pHandle) != HAL_OK) 
    {
      LOG_ERROR("Failed to deinitialize timer");
      return;
    }
  }

  memset(pCallback, 0, sizeof(TIM_Callback));

  LOG_INFO("Deinitialized TIM interrupts and callback for timer %s", TIM_GetTimerName(pTimerInstance));
}

void TIM_Enable_IRQ(TIM_TypeDef const * const pTimerInstance)
{
  const IRQn_Type irqn = TIM_TimerInstanceToIRQN(pTimerInstance);

  if (irqn == ERROR_IRQN_VAL)
  {
    return;
  }

  HAL_NVIC_EnableIRQ(irqn);

  LOG_DEBUG("Enabled TIM IRQ for timer %s", TIM_GetTimerName(pTimerInstance));
}


void TIM_Disable_IRQ(TIM_TypeDef const * const pTimerInstance)
{
  const IRQn_Type irqn = TIM_TimerInstanceToIRQN(pTimerInstance);

  if (irqn == ERROR_IRQN_VAL)
  {
    return;
  }

  HAL_NVIC_DisableIRQ(irqn);

  LOG_DEBUG("Disabled TIM IRQ for timer %s", TIM_GetTimerName(pTimerInstance));
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  const unsigned int timerNumber = TIM_GetTimerNumberFromInstance(htim->Instance);
  LOG_DEBUG_VERBOSE("Timer %s period elapsed interrupt triggered", TIM_GetTimerName(htim->Instance));

  TIM_Callback const * const pCallback = &timCallbacks[timerNumber];

  if (pCallback->func != NULL)
  {
    pCallback->func(htim, pCallback->arg);
  }
}

/**
 * @brief This function handles EXTI Line0 interrupt.
 */
void EXTI0_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
}

/**
 * @brief This function handles EXTI Line1 interrupt.
 */
void EXTI1_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
}

/**
 * @brief This function handles EXTI Line2 interrupt.
 */
void EXTI2_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
}

/**
 * @brief This function handles EXTI Line3 interrupt.
 */
void EXTI3_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);
}

/**
 * @brief This function handles EXTI Line4 interrupt.
 */
void EXTI4_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
}

/**
 * @brief This function handles EXTI Line[9:5] interrupts.
 */
void EXTI9_5_IRQHandler(void)
{
  for (uint16_t line = 5; line < 10; ++line)
  {
    HAL_GPIO_EXTI_IRQHandler((1 << line));
  }
}

/**
 * @brief This function handles EXTI Line[15:10] interrupts.
 */
void EXTI15_10_IRQHandler(void)
{
  for (uint16_t line = 10; line < 16; ++line)
  {
    HAL_GPIO_EXTI_IRQHandler((1 << line));
  }
}

/**
 * @brief This function handles USART2 global interrupt.
 */
void USART2_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart);
  USART_ITCharManager(&huart);
}

/** 
 * @brief This function handles TIM2 interrupt
*/
void TIM2_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&(timCallbacks[2].handle));
}

/** 
 * @brief This function handles TIM3 interrupt
*/
void TIM3_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&(timCallbacks[3].handle));
}

/** 
 * @brief This function handles TIM4 interrupt
*/
void TIM4_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&(timCallbacks[4].handle));
}

/** 
 * @brief This function handles TIM5 interrupt
*/
void TIM5_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&(timCallbacks[5].handle));
}

/** 
 * @brief This function handles TIM1_BRK_TIM9 interrupt
*/
void TIM1_BRK_TIM9_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&(timCallbacks[1].handle));
  HAL_TIM_IRQHandler(&(timCallbacks[9].handle));
}

/** 
 * @brief This function handles TIM1_UP_TIM10 interrupt
*/
void TIM1_UP_TIM10_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&(timCallbacks[1].handle));
  HAL_TIM_IRQHandler(&(timCallbacks[10].handle));
}


/** 
 * @brief This function handles TIM1_TRG_COM_TIM11 interrupt
*/
void TIM1_TRG_COM_TIM11_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&(timCallbacks[1].handle));
  HAL_TIM_IRQHandler(&(timCallbacks[11].handle));
}


/**
 * @}
  */ /* End of STM32F4XX_IT_Exported_Functions */

/**
 * @}
  */ /* End of STM32F4XX_IT */

/**
  * @}
  */ /* End of MicrosteppingMotor_Example */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
