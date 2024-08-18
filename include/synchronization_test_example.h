#ifndef SYNCHRONIZATION_TEST_EXAMPLE_H
#define SYNCHRONIZATION_TEST_EXAMPLE_H

#include "gpio_utils.h"

typedef struct {
  GPIO_Pin input_pin;
  GPIO_Pin output_pin;
} InputOutputPins;

void input_controlled_output_polling_loop(InputOutputPins const * const pPins);
void input_controlled_output_interrupt_handler(const GPIO_Pin_t exti_pin, void* arg);
void input_controlled_output_interrupt(InputOutputPins const * const pPins);

#endif