#include "gpio_defs.h"
#include "trace.h"

// Faster with __builtin_ctz if available
// However, switch case is fast enouph for 16 cases 
#define GPIO_PIN_SWITCH_CASE(pin, caseFunc)       \
{                                                 \
  switch ((pin))                                    \
  {                                               \
    caseFunc(0)                                   \
    caseFunc(1)                                   \
    caseFunc(2)                                   \
    caseFunc(3)                                   \
    caseFunc(4)                                   \
    caseFunc(5)                                   \
    caseFunc(6)                                   \
    caseFunc(7)                                   \
    caseFunc(8)                                   \
    caseFunc(9)                                   \
    caseFunc(10)                                  \
    caseFunc(11)                                  \
    caseFunc(12)                                  \
    caseFunc(13)                                  \
    caseFunc(14)                                  \
    caseFunc(15)                                  \
  }                                               \
}

/**
 * @brief Get the GPIO line number (0, 1, 2, ...) from the pin id
 * 
 * @param pin           The GPIO pin id
 * @return GPIO_Pin_t   The GPIO pin line number
 */
GPIO_Pin_t GPIO_GetLineNumberFromPin(const GPIO_Pin_t pin)
{
  #define GPIO_PIN_CASE(pin) case GPIO_PIN_##pin: return pin;

  GPIO_PIN_SWITCH_CASE(pin, GPIO_PIN_CASE);

  #undef GPIO_PIN_CASE

  LOG_ERROR("Invalid GPIO pin number %u", pin);
  return (GPIO_Pin_t)-1;
}

/**
 * @brief Get the GPIO port name from port id (e.g, GPIOA, GPIOB, ...)
 * 
 * @param port            The GPIO port id
 * @return const char*    The GPIO port name
 */
const char* GPIO_GetPortName(const GPIO_Port_t port)
{
  #define NAME_CASE(val) if (port == val) return(#val);

  NAME_CASE(GPIOA);
  NAME_CASE(GPIOB);
  NAME_CASE(GPIOC);
  NAME_CASE(GPIOD);

  #undef NAME_CASE

  LOG_ERROR("Invalid port %X", port);
  return NULL;
}

/**
 * @brief Get the pin number name from pin id (e.g, GPIO_PIN_7, GPIO_PIN_5, ...)
 * 
 * @param pin             The GPIO pin id
 * @return const char*    The GPIO pin name
 */
const char* GPIO_GetPinNumberName(const GPIO_Pin_t pin)
{
  #define GPIO_PIN_CASE(pin) case GPIO_PIN_##pin: return "GPIO_PIN_"#pin;
  GPIO_PIN_SWITCH_CASE(pin, GPIO_PIN_CASE);
  #undef GPIO_PIN_CASE

  LOG_ERROR("Invalid GPIO pin number %u", pin);
  return NULL;
}

/**
 * @brief Get the friendly GPIO pin name (e.g. PB7, PA7, ...)
 * 
 * @param pPin            The GPIO pin
 * @return const char*    The friendly GPIO pin name
 */
const char* GPIO_GetPinName(GPIO_Pin const * const pPin)
{
  #define GPIO_PIN_CASE(port, pin) case GPIO_PIN_##pin: return "P"#port#pin;
  #define GPIOA_PIN_CASE(pin) GPIO_PIN_CASE(A, pin)
  #define GPIOB_PIN_CASE(pin) GPIO_PIN_CASE(B, pin)
  #define GPIOC_PIN_CASE(pin) GPIO_PIN_CASE(C, pin)
  #define GPIOD_PIN_CASE(pin) GPIO_PIN_CASE(D, pin)

  if (pPin->port == GPIOA)
  {
    GPIO_PIN_SWITCH_CASE(pPin->pin, GPIOA_PIN_CASE);
    LOG_ERROR("Invalid GPIO pin number %u", pPin->pin);
    return NULL;
  }
  
  if (pPin->port == GPIOB)
  {
    GPIO_PIN_SWITCH_CASE(pPin->pin, GPIOB_PIN_CASE);
    LOG_ERROR("Invalid GPIO pin number %u", pPin->pin);
    return NULL;
  }

  if (pPin->port == GPIOC)
  {
    GPIO_PIN_SWITCH_CASE(pPin->pin, GPIOC_PIN_CASE);
    LOG_ERROR("Invalid GPIO pin number %u", pPin->pin);
    return NULL;
  }

  if (pPin->port == GPIOD)
  {
    GPIO_PIN_SWITCH_CASE(pPin->pin, GPIOD_PIN_CASE);
    LOG_ERROR("Invalid GPIO pin number %u", pPin->pin);
    return NULL;
  }

  LOG_ERROR("Invalid GPIO port %X", pPin->port);
  return NULL;

  #undef GPIO_PIN_CASE
  #undef GPIOA_PIN_CASE
  #undef GPIOA_PIN_CASE
  #undef GPIOA_PIN_CASE
  #undef GPIOA_PIN_CASE
}

/**
 * @brief Get the GPIO mode name from id
 * 
 * @param mode            The GPIO mode id
 * @return const char*    The GPIO mode name
 */
const char* GPIO_GetModeName(const uint32_t mode)
{
  #define NAME_CASE(val) case (val): return(#val)

  switch (mode)
  {
    NAME_CASE(GPIO_MODE_OUTPUT_OD);
    NAME_CASE(GPIO_MODE_OUTPUT_PP);
    NAME_CASE(GPIO_MODE_INPUT);
    NAME_CASE(GPIO_MODE_IT_RISING);
    NAME_CASE(GPIO_MODE_IT_FALLING);
    NAME_CASE(GPIO_MODE_IT_RISING_FALLING);
    NAME_CASE(GPIO_MODE_ANALOG);
    default:
      LOG_ERROR("Invalid GPIO mode %d", mode);
      return NULL;
  }

  #undef NAME_CASE
}

/**
 * @brief Get the GPIO pull mode name from id
 * 
 * @param pull            The GPIO pull mode id
 * @return const char*    The GPIO pull mode name
 */
const char* GPIO_GetPullModeName(const GPIO_PullMode pull)
{
  #define NAME_CASE(val) case (val): return(#val)

  switch (pull)
  {
    NAME_CASE(GPIO_PULLMODE_NOPULL);
    NAME_CASE(GPIO_PULLMODE_PULLDOWN);
    NAME_CASE(GPIO_PULLMODE_PULLUP);
    default:
      LOG_ERROR("Invalid GPIO pull mode %u", pull);
      return NULL;
  }

  #undef NAME_CASE
}

/**
 * @brief Get the interrupt trigger mode name from id
 * 
 * @param itTriggerMode   The trigger mode id
 * @return const char*    The trigger mode name
 */
const char* GPIO_GetItTriggerModeName(const GPIO_IT_Trigger_Mode itTriggerMode)
{
  #define NAME_CASE(val) case (val): return(#val)

  switch (itTriggerMode)
  {
    NAME_CASE(GPIO_IT_TRIGGER_MODE_NONE);
    NAME_CASE(GPIO_IT_TRIGGER_MODE_RISING);
    NAME_CASE(GPIO_IT_TRIGGER_MODE_FALLING);
    NAME_CASE(GPIO_IT_TRIGGER_MODE_CHANGE);
    default:
      LOG_ERROR("Invalid GPIO interrupt trigger mode %d", itTriggerMode);
      return NULL;
  }

  #undef NAME_CASE
}

#undef GPIO_PIN_SWITCH_CASE
