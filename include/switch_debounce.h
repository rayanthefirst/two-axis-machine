#ifndef SWITCH_UTILS_H
#define SWITCH_UTILS_H

#include "stm32f4xx_it.h"
#include "stdbool.h"

#define SWDB_EXTI_DEBOUNCE_TIMER TIM3

typedef void* SWDB_CallbackArg_t;
typedef void (*SWDB_CallbackFunc_t)(GPIO_Pin const * const, const GPIO_PinState, SWDB_CallbackArg_t);

/**
 * @brief Contains the switch debounce callback configuration 
 */
typedef struct {
  SWDB_CallbackFunc_t onTriggerFunc;
  SWDB_CallbackFunc_t onStabilizeFunc;
  SWDB_CallbackArg_t arg;
} SWDB_Callback;

/**
 * @brief Contains the switch debounce context for a specific switch
 */
typedef struct {
  GPIO_Pin pin;
  uint32_t stableTimeTolerance_ms;
  uint32_t unstableTimeout_ms;
  SWDB_Callback callback;
  
  GPIO_PinState stableState;
  uint32_t lastStableTime;
  GPIO_PinState lastState;
  uint32_t lastChangeTime;
  bool bouncing;
} SWDB_EXTI_Context;

void SWDB_Init_EXTI(
  GPIO_Pin const * const pPin,
  const GPIO_PullMode pull,
  const GPIO_IT_Trigger_Mode it_trigger_mode,
  const SWDB_CallbackFunc_t pOnTriggerCallbackFunc,
  const SWDB_CallbackFunc_t pOnStabilizeCallbackFunc,
  const SWDB_CallbackArg_t pCallbackArg,
  const uint32_t stableTimeTolerance_ms,
  const uint32_t unstableTimeout_ms,
  EXTI_Config const * const pConfig
);
void SWDB_DeInit(GPIO_Pin const * const pPin);

#endif