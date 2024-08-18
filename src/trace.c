#include "trace.h"
#include "stdarg.h"
#include "string.h"
#include "stdlib.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_nucleo.h"

#ifdef NUCLEO_USE_USART
#include "stdio.h"
#endif

#ifdef STM32F072xB
#define USART_STATUS_REGISTER   ISR                     //!< HAL USART status register name adapter.
#define USART_DATA_REGISTER     RDR                     //!< HAL UART data register name adapter.
#endif
#ifdef STM32F302x8
#define USART_STATUS_REGISTER   ISR                     //!< HAL USART status register name adapter.
#define USART_DATA_REGISTER     RDR                     //!< HAL UART data register name adapter.
#endif
#ifdef STM32F401xE
#define USART_STATUS_REGISTER   SR                      //!< HAL USART status register name adapter.
#define USART_DATA_REGISTER     DR                      //!< HAL UART data register name adapter.
#endif

UART_HandleTypeDef huart; //!< The data structure for all further instances to USART.

/**
 * @brief Transmit a string text over USART
 * 
 * @param pHuart    The USART handle
 * @param text      The null-terminated text to transmit
 */
void UART_Transmit_String(UART_HandleTypeDef* pHuart, const char * text)
{
#ifdef NUCLEO_USE_USART
	/* Use the HAL function to send the text string via USART */
	HAL_UART_Transmit(pHuart, (uint8_t*)text, (uint16_t)strlen(text), 100);
#else
    __NOP();
#endif
}

/**
 * @brief Retargets the C library printf function to the USART
 * 
 * @param ch    The character to print
 * @return int  The same character?
 */
int __io_putchar(int ch)
{
#ifdef NUCLEO_USE_USART
	/* Use the HAL function to send the text string via USART */
	HAL_UART_Transmit(&huart, &ch, 1, 10);
#else
    __NOP();
#endif
    return ch;
}

/**
 * @brief Retarget the C library printf function to the USART
 * 
 * @param file      The file being written to (stdout, stederr), irrelevent for microcontroller 
 * @param ptr       The ptr to the string text to transmit
 * @param len       The length of the message
 * @return int 
 */
int _write(int file, char *ptr, int len)
{
#ifdef NUCLEO_USE_USART
    if ((file != 1) && (file != 2))
    {
        return -1; // Only accept stdout (1) or stderr (2)
    }

    HAL_UART_Transmit(&huart, (uint8_t*)ptr, (uint16_t)len, 100);
#endif
    return len;
}

/**
 * @brief Initialize the tracing function by initializing the USART
 * 
 */
void Trace_Init(void)
{
#ifdef NUCLEO_USE_USART
  huart.Instance = TRACE_UART_HANDLE;
  huart.Init.BaudRate = 115200;
  huart.Init.WordLength = UART_WORDLENGTH_8B;
  huart.Init.StopBits = UART_STOPBITS_1;
  huart.Init.Parity = UART_PARITY_NONE;
  huart.Init.Mode = UART_MODE_TX_RX;
  huart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart);
#else
  __NOP();
#endif
}

/**
 * @brief Returns whether the given log level is enabled
 * 
 * @param level     The log level to check
 * @return true     Returned if the log level is enabled
 * @return false    Returned if the log level is disabled
 */
bool LogLevelEnabled(LogLevel level)
{
    return level >= GLOBAL_LOG_LEVEL;
}

/**
 * @brief Log message through USART 
 * 
 * @param level     The log level of the message
 * @param file      The source file name
 * @param line      The source line number
 * @param func      The source function name
 * @param fmt       The message format, same as printf
 * @param ...       Format args, same as printf
 */
void LogPrint(LogLevel level, const char *file, int line, const char *func, const char *fmt, ...)
{
#ifdef NUCLEO_USE_USART
    if (!LogLevelEnabled(level))
    {
        return;
    }

    const uint32_t timestamp = HAL_GetTick();

    const char *level_str = NULL;
    switch (level) {
        case LOG_LEVEL_DEBUG:   level_str = "DEBUG";   break;
        case LOG_LEVEL_INFO:    level_str = "INFO";    break;
        case LOG_LEVEL_WARNING: level_str = "WARNING"; break;
        case LOG_LEVEL_ERROR:   level_str = "ERROR";   break;
        default:                level_str = "UNKNOWN"; break;
    }

    // Buffer to hold the formatted message
    const size_t MESSAGE_BUFFER_SIZE = 128;
    char message[MESSAGE_BUFFER_SIZE];

    memset(message, 0, MESSAGE_BUFFER_SIZE);

    // Handle the variable arguments
    va_list args;
    va_start(args, fmt);
    int ret = vsnprintf(message, MESSAGE_BUFFER_SIZE, fmt, args);
    va_end(args);

    // Check for buffer overflow
    if (ret < 0 || (size_t)ret >= MESSAGE_BUFFER_SIZE) 
    {
        // Truncate the message and indicate truncation
        strncpy(message + MESSAGE_BUFFER_SIZE - 14, " [truncated]", 13);
        message[MESSAGE_BUFFER_SIZE - 1] = '\0'; // Ensure null-termination
    }

    // Print the complete log message in one call to printf
    printf("[%d][%s][%s() in %s:%d]: %s\n\r", timestamp, level_str, func, file, line, message);
#else
    __NOP();
#endif
}

/**
 * @brief Log message over USART like LogPrint except no headers are printed
 * 
 * @param fmt   The message format, same as printf
 * @param ...   The message format args, same as printf
 */
void Println(const char *fmt, ...)
{
#ifdef NUCLEO_USE_USART

   // Buffer to hold the formatted message
    const size_t FMT_BUFFER_SIZE = 128;
    char fmtBuffer[FMT_BUFFER_SIZE];

    memset(fmtBuffer, 0, FMT_BUFFER_SIZE);

    int ret = snprintf(fmtBuffer, FMT_BUFFER_SIZE, "%s\n\r", fmt);

    // Check for buffer overflow
    if (ret < 0 || (size_t)ret >= FMT_BUFFER_SIZE) 
    {
        // Truncate the message and indicate truncation
        strncpy(fmtBuffer + FMT_BUFFER_SIZE - 14, " [truncated]", 13);
        fmtBuffer[FMT_BUFFER_SIZE - 1] = '\0'; // Ensure null-termination
    }

    va_list args;
    va_start(args, fmt);
    vprintf(fmtBuffer, args);
    va_end(args);
#else
    __NOP();
#endif
}

/**
 * @brief  Handle text character just received.
 * @param  huart pointer to a UART_HandleTypeDef structure that contains
 *               the configuration information for the specified UART module.
 * @note To use inside USART2_IRQHandler function.
 */
void USART_ITCharManager(UART_HandleTypeDef* pHuart) 
{
    uint8_t UART_Receive_IT_Char;

    UART_Receive_IT_Char = (uint8_t) (pHuart->Instance->USART_DATA_REGISTER);
    /* Checks the buffer full or retur carriage  */
    if ((pHuart->RxXferCount == 1) || (UART_Receive_IT_Char == '\r')) {
        pHuart->RxXferCount += 1;
        pHuart->pRxBuffPtr -= 1;
        *(pHuart->pRxBuffPtr) = '\0';

        UART_Transmit_String(pHuart, "\n\r");

        uint32_t START = DWT->CYCCNT;
        while(DWT->CYCCNT - START < 8000); //DELAY FOR ~1MS 

            while (HAL_IS_BIT_SET(pHuart->Instance->USART_STATUS_REGISTER,
                UART_FLAG_RXNE)) {
            }
        __HAL_UART_DISABLE_IT(pHuart, UART_IT_RXNE);

        /* Check if a transmit process is ongoing or not */
        if (pHuart->gState == HAL_UART_STATE_BUSY_TX_RX) {
            pHuart->gState = HAL_UART_STATE_BUSY_TX;
        } else {
            /* Disable the UART Parity Error Interrupt */
            __HAL_UART_DISABLE_IT(pHuart, UART_IT_PE);

            /* Disable the UART Error Interrupt: (Frame error, noise error, overrun error) */
            __HAL_UART_DISABLE_IT(pHuart, UART_IT_ERR);

            pHuart->gState = HAL_UART_STATE_READY;
            pHuart->RxState = HAL_UART_STATE_READY;
        }
    }
}
