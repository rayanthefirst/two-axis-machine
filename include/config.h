#ifndef MASTER_CONFIG_H
#define MASTER_CONFIG_H

#define NUCLEO_USE_USER_LED     //!< Uncomment to enable the NUCLEO User LED feature.
#define NUCLEO_USE_USER_BUTTON  //!< Uncomment to enable the NUCLEO User Button feature.
#define NUCLEO_USE_USART        //!< Uncomment to enable the NUCLEO USART feature.
#define GLOBAL_LOG_LEVEL LOG_LEVEL_INFO
#define TRACE_UART_HANDLE USART2

#ifdef STM32F072xB
#define NUCLEO_BOARD_NAME       "NUCLEO-F072RB"
#endif
#ifdef STM32F302x8
#define NUCLEO_BOARD_NAME       "NUCLEO-F302R8"
#endif
#ifdef STM32F401xE
#define NUCLEO_BOARD_NAME       "NUCLEO-F401RE"
#endif

/**
 * @brief Useful description of interrupt pre-empt priority levels
 */
typedef enum {
    PRIORITY_LEVEL_CRITICAL_SYSTEM = 0,       //!< Critical system tasks, highest priority
    PRIORITY_LEVEL_CRITICAL_REALTIME,         //!< Critical real-time processing
    PRIORITY_LEVEL_HIGH_SAFETY,               //!< High priority for safety-related tasks
    PRIORITY_LEVEL_HIGH_IO,                   //!< High priority for I/O operations
    PRIORITY_LEVEL_HIGH_COMMUNICATION,        //!< High priority for communication tasks
    PRIORITY_LEVEL_HIGH_SENSOR,               //!< High priority for sensor data processing
    PRIORITY_LEVEL_ABOVE_NORMAL_CONTROL,      //!< Above normal for control algorithms
    PRIORITY_LEVEL_ABOVE_NORMAL_MONITORING,   //!< Above normal for system monitoring
    PRIORITY_LEVEL_NORMAL_APP,                //!< Normal priority for standard application tasks
    PRIORITY_LEVEL_BELOW_NORMAL_BACKGROUND,   //!< Below normal for background tasks
    PRIORITY_LEVEL_BELOW_NORMAL_DIAGNOSTIC,   //!< Below normal for diagnostic tasks
    PRIORITY_LEVEL_LOW_MAINTENANCE,           //!< Low priority for maintenance tasks
    PRIORITY_LEVEL_LOW_LOGGING,               //!< Low priority for logging tasks
    PRIORITY_LEVEL_LOWEST_IDLE,               //!< Lowest priority, idle tasks
} InterruptPriorityLevel;

#endif