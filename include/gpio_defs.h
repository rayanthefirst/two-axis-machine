#ifndef GPIO_PIN_H
#define GPIO_PIN_H

#include "stm32f4xx.h"

typedef GPIO_TypeDef* GPIO_Port_t;
typedef uint16_t GPIO_Pin_t;

/**
 * @brief Describes a GPIO connection
 */
typedef struct {
    GPIO_Port_t port;                           //!< The GPIO port
    GPIO_Pin_t pin;                             //!< The GPIO pin ID (not number!!)
} GPIO_Pin;

/**
 * @brief Numerical ids for the PULL modes
 */
typedef enum {
    GPIO_PULLMODE_NOPULL = GPIO_NOPULL,
    GPIO_PULLMODE_PULLDOWN = GPIO_PULLDOWN,
    GPIO_PULLMODE_PULLUP = GPIO_PULLUP
} GPIO_PullMode;

/**
 * @brief Numerical ids for the trigger modes
 * 
 */
typedef enum {
    GPIO_IT_TRIGGER_MODE_NONE = 0,
    GPIO_IT_TRIGGER_MODE_RISING = TRIGGER_RISING,
    GPIO_IT_TRIGGER_MODE_FALLING = TRIGGER_FALLING,
    GPIO_IT_TRIGGER_MODE_CHANGE = TRIGGER_RISING | TRIGGER_FALLING,
} GPIO_IT_Trigger_Mode;

GPIO_Pin_t GPIO_GetLineNumberFromPin(const GPIO_Pin_t pin);

const char* GPIO_GetPortName(const GPIO_Port_t port);
const char* GPIO_GetPinNumberName(const GPIO_Pin_t pin);
const char* GPIO_GetPinName(GPIO_Pin const * const pPin);
const char* GPIO_GetModeName(const uint32_t mode);
const char* GPIO_GetPullModeName(const GPIO_PullMode pull);
const char* GPIO_GetItTriggerModeName(const GPIO_IT_Trigger_Mode itTriggerMode);


#endif