#ifndef LOG_H
#define LOG_H

#include "stdbool.h"
#include "stm32f4xx_nucleo.h"
#include "config.h"

#ifndef GLOBAL_LOG_LEVEL
#define GLOBAL_LOG_LEVEL LOG_LEVEL_INFO
#endif

#ifndef TRACE_UART_HANDLE
#define TRACE_UART_HANDLE USART2
#endif

/**
 * @brief Enumeration for log levels
 */
typedef enum {
    LOG_LEVEL_DEBUG_VERBOSE = 0,
    LOG_LEVEL_DEBUG,
    LOG_LEVEL_INFO,
    LOG_LEVEL_WARNING,
    LOG_LEVEL_ERROR
} LogLevel;


extern UART_HandleTypeDef huart;

void Trace_Init();
bool LogLevelEnabled(LogLevel level);
void LogPrint(LogLevel level, const char *file, int line, const char *func, const char *fmt, ...);
void Println(const char* fmt, ...);
void USART_ITCharManager(UART_HandleTypeDef* huart);

#ifdef NUCLEO_USE_USART

#define LOG(level, fmt, ...) \
{\
    if (LogLevelEnabled(level))\
    {\
        LogPrint(level, __FILE__, __LINE__, __func__, (fmt), ##__VA_ARGS__); \
    }\
}

#define LOG_DEBUG_VERBOSE(fmt, ...)     LOG(LOG_LEVEL_DEBUG_VERBOSE, (fmt), ##__VA_ARGS__)
#define LOG_DEBUG(fmt, ...)             LOG(LOG_LEVEL_DEBUG, (fmt), ##__VA_ARGS__)
#define LOG_INFO(fmt, ...)              LOG(LOG_LEVEL_INFO, (fmt), ##__VA_ARGS__)
#define LOG_WARNING(fmt, ...)           LOG(LOG_LEVEL_WARNING, (fmt), ##__VA_ARGS__)
#define LOG_ERROR(fmt, ...)             LOG(LOG_LEVEL_WARNING, (fmt), ##__VA_ARGS__)

#define PRINTLN(fmt, ...)               Println((const char*)(fmt), ##__VA_ARGS__)

#else // ifdef NUCLEO_USE_USART

#define LOG_DEBUG_VERBOSE(fmt, ...)
#define LOG(level, fmt, ...) 
#define LOG_DEBUG(fmt, ...)    
#define LOG_INFO(fmt, ...)     
#define LOG_WARNING(fmt, ...)   
#define LOG_ERROR(fmt, ...)     

#define PRINTLN(fmt, ...)

#endif

#endif