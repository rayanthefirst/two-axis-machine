#include "synchronization_test_example.h"
#include "stdlib.h"
#include "string.h"
#include "stm32f4xx_it.h"
#include "trace.h"

void input_controlled_output_polling_loop(InputOutputPins const * const pPins)
{
  GPIO_Init_Basic_Input(&(pPins->input_pin), GPIO_NOPULL, 0);
  GPIO_Init_Basic_Output(&(pPins->output_pin));

  while (1)
  {
    GPIO_PinState input_state = HAL_GPIO_ReadPin(pPins->input_pin.port, pPins->input_pin.pin);
    LOG_DEBUG("Input state is %d\n\r", (int)input_state);
    HAL_GPIO_WritePin(pPins->output_pin.port, pPins->output_pin.pin, input_state);
  }
}

void input_controlled_output_interrupt_handler(const GPIO_Pin_t exti_pin, void* arg)
{
  InputOutputPins const * const pPins  = (InputOutputPins*)arg;

  // Lab 4 Exercise 4.2 Handler
  GPIO_PinState input_state = HAL_GPIO_ReadPin(pPins->input_pin.port, pPins->input_pin.pin);
  LOG_DEBUG("Input state is %d\n\r", (int)input_state);
  HAL_GPIO_WritePin(pPins->output_pin.port, pPins->output_pin.pin, input_state);
}

void input_controlled_output_interrupt(InputOutputPins const * const pPins)
{
  // Need to persist the interrupt args on heap
  InputOutputPins* const pPinsInterrupt = (InputOutputPins*)malloc(sizeof(InputOutputPins));
  memcpy(pPinsInterrupt, pPins, sizeof(InputOutputPins));

  const EXTI_Config extiConfig = { .preemptPriority = 0x0F, .subPriority = 0x00};

  GPIO_Init_Basic_EXTI(
    &(pPins->input_pin),
    GPIO_PULLMODE_NOPULL, 
    GPIO_IT_TRIGGER_MODE_CHANGE, 
    input_controlled_output_interrupt_handler, 
    pPinsInterrupt,
    &extiConfig
  );
  GPIO_Init_Basic_Output(&(pPins->output_pin));

  while (1)
  {

  }

  free(pPinsInterrupt);
}