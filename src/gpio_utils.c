#include "gpio_utils.h"
#include <stm32f4xx_it.h>
#include "trace.h"

/**
 * @brief Initializes a basic GPIO output
 * 
 * @param pPin            The pin to initialize the output for
 */
void GPIO_Init_Basic_Output(GPIO_Pin const * const pPin)
{
  GPIO_InitTypeDef initStruct = {
    .Pin = pPin->pin,
    .Mode = GPIO_MODE_OUTPUT_PP,
    .Pull = GPIO_NOPULL,
    .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
    .Alternate = 0,
  };

  HAL_GPIO_Init(pPin->port, &initStruct);

  LOG_INFO("GPIO pin %s configured as %s", 
    GPIO_GetPinName(pPin), 
    GPIO_GetModeName(initStruct.Mode)
  );
}

/**
 * @brief Initializes a basic GPIO input
 * 
 * @param pPin            The pin to initialize for
 * @param pull            The pull mode (down, up, none)
 * @param itTriggerMode   The interrupt trigger mode (none, rising, falling, change)
 */
void GPIO_Init_Basic_Input(GPIO_Pin const * const pPin, const GPIO_PullMode pull, const GPIO_IT_Trigger_Mode itTriggerMode)
{
  uint32_t mode = GPIO_MODE_INPUT;
  if (itTriggerMode != GPIO_IT_TRIGGER_MODE_NONE)
  {
    mode |= EXTI_IT | itTriggerMode;
  }

  GPIO_InitTypeDef initStruct = {
    .Pin = pPin->pin,
    .Mode = mode,
    .Pull = pull,
    .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
    .Alternate = 0,
  };
  HAL_GPIO_Init(pPin->port, &initStruct);

  LOG_INFO("GPIO pin %s configured as input with pull mode %s and interrupt trigger mode %s", 
    GPIO_GetPinName(pPin), 
    GPIO_GetPullModeName(pull),
    GPIO_GetItTriggerModeName(itTriggerMode)
  );
}

/**
 * @brief Initialize a basic GPIO input with an EXTI interrupt
 * 
 * @param pPin              The pin to initialize the input for
 * @param pull              The input pull mode
 * @param it_trigger_mode   The interrupt trigger mode
 * @param pCallbackFunc     The EXTI interrupt callback function
 * @param pCallbackArg      The EXTI interrupt callback function argument
 * @param pEXTIConfig       The EXTI configuration
 */
void GPIO_Init_Basic_EXTI(GPIO_Pin const * const pPin,
                              const GPIO_PullMode pull,
                              const GPIO_IT_Trigger_Mode it_trigger_mode,
                              const EXTI_CallbackFunc_t pCallbackFunc,
                              const EXTI_CallbackArg_t pCallbackArg,
                              EXTI_Config const * const pEXTIConfig)
{
  if (it_trigger_mode == GPIO_IT_TRIGGER_MODE_NONE)
  {
    LOG_ERROR("Error trying to initialize basic EXTI pin with no trigger modes");
    return;
  } 

  GPIO_Init_Basic_Input(pPin, pull, it_trigger_mode);
  EXTI_Register_Callback(pPin->pin, pCallbackFunc, pCallbackArg, pEXTIConfig);
}
