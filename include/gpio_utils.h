#ifndef GPIO_UTILS_H
#define GPIO_UTILS_H

#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
#include "gpio_defs.h"

void GPIO_Init_Basic_Output(GPIO_Pin const * const pPin);
void GPIO_Init_Basic_Input(GPIO_Pin const * const pPin, const GPIO_PullMode pull, const GPIO_IT_Trigger_Mode itTriggerMode);
void GPIO_Init_Basic_EXTI(GPIO_Pin const * const pPin,
                              const GPIO_PullMode pull,
                              const GPIO_IT_Trigger_Mode it_trigger_mode,
                              const EXTI_CallbackFunc_t pCallbackFunc,
                              const EXTI_CallbackArg_t pCallbackArg,
                              EXTI_Config const * const pEXTIConfig);

#endif