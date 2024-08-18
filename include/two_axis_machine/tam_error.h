/**
 * @file tam_error.h
 * @brief Utilitis and definitions for error/status handling for the two axis machine interface
 */

#ifndef TAM_ERROR_H
#define TAM_ERROR_H

#include "trace.h"
#include "error.h"

/**
 * @brief Two axis machine error status codes
 */
typedef enum {
  TAM_OK = 0,
  TAM_INVALID_PARAM,
  TAM_INVALID_OPERATION,
  TAM_NULL_POINTER,
  TAM_UNSAFE_OPERATION,
  TAM_TRAVEL_BLOCKED,
  TAM_INVALID_STRING_COMMAND,
} TAM_Status;

inline bool TAM_StatusFailed(TAM_Status status) { return status != TAM_OK; }
inline bool TAM_StatusSuccess(TAM_Status status) { return !TAM_StatusFailed(status); }
const char* TAM_StatusGetName(const TAM_Status status);

#define TAM_CHECK_STATUS_RETURN(status) \
  CHECK_STATUS_RETURN_BASE(TAM_StatusSuccess, (status))

#define TAM_LOG_ERROR_STATUS(status, fmt, ...) LOG_ERROR_STATUS_BASE(TAM_StatusGetName, (status), (fmt), ##__VA_ARGS__)

#define TAM_CHECK_STATUS_LOG_RETURN(status, fmt, ...) \
  CHECK_STATUS_LOG_RETURN_BASE(TAM_StatusSuccess, TAM_LOG_ERROR_STATUS, (status), (fmt), ##__VA_ARGS__)

#define TAM_CHECK_STATUS_LOG(status, fmt, ...) \
  CHECK_STATUS_LOG_BASE(TAM_StatusSuccess, TAM_LOG_ERROR_STATUS, (status), (fmt), ##__VA_ARGS__)

#define TAM_CHECK_NULL_POINTER_RETURN(pointer) \
  ASSERT_CHECK_RETURN((pointer) != NULL, TAM_NULL_POINTER)

#define TAM_CHECK_NULL_POINTER_LOG_RETURN(pointer, fmt, ...) \
  ASSERT_CHECK_LOG_RETURN((pointer) != NULL, TAM_NULL_POINTER, fmt, ##__VA_ARGS__)

#endif