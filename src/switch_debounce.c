#include "switch_debounce.h"
#include "stdbool.h"
#include "string.h"
#include "stdlib.h"
#include "trace.h"
#include "gpio_utils.h"

SWDB_EXTI_Context* pDebouncedInterrupts[MAX_EXTI_PINS] = {0};
bool swdbExtidbInitialized = false;

void SWDB_DebounceSwitch(SWDB_EXTI_Context * const pContext);

/**
 * @brief The switch EXTI callback
 * 
 * @param pin   The GPIO pin id that the interrupt triggered on
 * @param pArg  The callback arg
 */
void SWDB_EXTI_Callback(const GPIO_Pin_t pin, EXTI_CallbackArg_t pArg)
{
  // The strategy here is to disable the IRQ for this pin until we ensure it is stabilized in the timer
  EXTI_Disable_IRQ(pin);

  SWDB_EXTI_Context * const pContext = pDebouncedInterrupts[GPIO_GetLineNumberFromPin(pin)];
  const GPIO_PinState state = HAL_GPIO_ReadPin(pContext->pin.port, pContext->pin.pin);
  pContext->lastStableTime = HAL_GetTick();
  pContext->lastChangeTime = pContext->lastStableTime;
  LOG_DEBUG("Switch interrupt triggered on pin %s", GPIO_GetPinName(&(pContext->pin)));

  // Invoke the onTrigger callback function if specified
  if (pContext->callback.onTriggerFunc != NULL)
  {
    pContext->callback.onTriggerFunc(&(pContext->pin), state, pContext->callback.arg);
  }
  
  // Update the switch state
  pContext->bouncing = true;
}

/**
 * @brief Callback function for the debouncing timer period
 * 
 * @param pHandle   The timer handle
 * @param pArg      Callback arg
 */
void SWDB_EXTIDT_PeriodElapsedCallback(TIM_HandleTypeDef const * const pHandle, TIM_CallbackArg_t pArg)
{
  // Debounce all the switches (if applicable)
  // We can improve the efficiency of this by only checking the debouncing state of 
  // the initialized pins or keeping track of which pins are in the debouncing state
  // but i'll leave that as a TODO. This shouldn't really matter too much since its a for
  // loop with maximum of 16 if statement checks which for a 80MHz clock should take negligible time
  for (unsigned int i = 0; i < MAX_EXTI_PINS; ++i)
  {
    if (pDebouncedInterrupts[i] != NULL)
    {
      SWDB_DebounceSwitch(pDebouncedInterrupts[i]);
    }
  }
}

/**
 * @brief Debounce the switch
 * 
 * @param pContext  The switch debouncing context
 */
void SWDB_DebounceSwitch(SWDB_EXTI_Context * const pContext) 
{
  // No need to do anything if not bouncing
  if (pContext->bouncing == false)
  {
    return;
  }

  const uint32_t currentTime = HAL_GetTick();
  const GPIO_PinState currentState = HAL_GPIO_ReadPin(pContext->pin.port, pContext->pin.pin);
  
  // Did we change states?
  if (currentState != pContext->lastState)
  {
    LOG_DEBUG("Switch bounced on %d", pContext->pin.pin);
    pContext->lastChangeTime = currentTime;
    pContext->lastState = currentState;
  } 
  // Are we past the stable time tolerance? 
  else if ((currentTime - pContext->lastChangeTime) >= pContext->stableTimeTolerance_ms)
  {
    LOG_DEBUG("Switch stabilized on %d", pContext->pin.pin);
    pContext->lastStableTime = currentTime;

    pContext->stableState = currentState;

    // Handle switch state change and call the onStabilize callback if specified
    if (pContext->callback.onStabilizeFunc != NULL)
    {
      pContext->callback.onStabilizeFunc(&(pContext->pin), pContext->stableState, pContext->callback.arg);
    }

    pContext->bouncing = false;
    // Re-enable the corresponding EXTI line
    EXTI_Enable_IRQ(pContext->pin.pin);
    return;
  }

  // Have we been unstable past the timeout?
  if ((currentTime - pContext->lastStableTime) >= pContext->unstableTimeout_ms)
  {
    // Renable the EXTI and force the switch into "stabilized" state. But since we didn't confirm stabilization,
    // callback is not called.
    LOG_WARNING("Timed out waiting for switch on pin %d to stabilize", pContext->pin.pin);
    pContext->bouncing = false;
    EXTI_Enable_IRQ(pContext->pin.pin);
    return;
  }
}

/**
 * @brief Initializes the switch debouncer timer for the switches
 */
void SWDB_Setup_EXTIDT()
{
  // If the timer is already initialized, no need to do anything
  if (swdbExtidbInitialized)
  {
    return;
  }

  TIM_Callback_Config config = {
    .preemptPriority = PRIORITY_LEVEL_ABOVE_NORMAL_CONTROL,
    .subPriority = 0,
    .timerConfig = {
      .Prescaler = (uint32_t)((SystemCoreClock / 1000) - 1), // 1 ms tick
      .CounterMode = TIM_COUNTERMODE_UP,
      .Period = 2 - 1, // Check every 1 ms
      .ClockDivision = TIM_CLOCKDIVISION_DIV1,
      .AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE,
    }
  };

  TIM_Register_Callback(SWDB_EXTI_DEBOUNCE_TIMER, SWDB_EXTIDT_PeriodElapsedCallback, NULL, &config);

  swdbExtidbInitialized = true;
}

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
)
{
  SWDB_DeInit(pPin);

  SWDB_EXTI_Context* pContext = (SWDB_EXTI_Context*)malloc(sizeof(SWDB_EXTI_Context));
  pDebouncedInterrupts[GPIO_GetLineNumberFromPin(pPin->pin)] = pContext; 
  
  memset(pContext, 0, sizeof(SWDB_EXTI_Context));
  
  pContext->pin.pin = pPin->pin;
  pContext->pin.port = pPin->port;
  pContext->callback.onTriggerFunc = pOnTriggerCallbackFunc;
  pContext->callback.onStabilizeFunc = pOnStabilizeCallbackFunc;
  pContext->callback.arg = pCallbackArg;
  pContext->stableTimeTolerance_ms = stableTimeTolerance_ms;
  pContext->unstableTimeout_ms = unstableTimeout_ms;
  pContext->bouncing = false;

  SWDB_Setup_EXTIDT();
  GPIO_Init_Basic_EXTI(pPin, pull, it_trigger_mode, SWDB_EXTI_Callback, NULL, pConfig);

  pContext->stableState = HAL_GPIO_ReadPin(pPin->port, pPin->pin);
  pContext->lastStableTime = HAL_GetTick();
  pContext->lastChangeTime = pContext->lastStableTime;

  LOG_INFO("Initialized debounced switch on pin %s", GPIO_GetPinName(pPin));
}

/**
 * @brief Deinitializes the switch debouncing for a pin
 * 
 * @param pPin  The pin to deinitialize the switch debouncing for
 */
void SWDB_DeInit(GPIO_Pin const * const pPin)
{
  const GPIO_Pin_t lineNumber = GPIO_GetLineNumberFromPin(pPin->pin);
  if (pDebouncedInterrupts[lineNumber] != NULL)
  {
    free(pDebouncedInterrupts[lineNumber]);
    pDebouncedInterrupts[lineNumber] = NULL;
  }

  LOG_INFO("Deinitialized debounced switch on pin %s", GPIO_GetPinName(pPin));
}