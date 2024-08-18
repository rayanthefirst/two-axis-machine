#ifndef ERROR_H
#define ERROR_H

#include "stdbool.h"
#include "trace.h"

#define _VOID_RETURN 0

#define ASSERT_CHECK_RETURN(check, returnVal) \
{\
  const bool checkStatus = (check);\
  if (checkStatus == false)\
  {\
    return returnVal;\
  }\
}

#define ASSERT_CHECK_RETURN_VOID(check, returnVal) ASSERT_CHECK_RETURN((check), _VOID_RETURN)

#define ASSERT_CHECK_LOG_RETURN(check, returnVal, fmt, ...) \
{\
  const bool checkStatus = (check);\
  if (checkStatus == false)\
  {\
    LOG_ERROR((fmt), ##__VA_ARGS__);\
    return returnVal;\
  }\
}

#define ASSERT_CHECK_LOG_RETURN_VOID(check, fmt, ...) ASSERT_CHECK_LOG_RETURN((check), _VOID_RETURN, fmt, ##__VA_ARGS__)

char const * const _AppendErrorStatusName(char const * const pStatusName, char const* const pFmt);

#define CHECK_STATUS_RETURN_BASE(statusSuccessFunc, statusValue) \
  ASSERT_CHECK_RETURN((statusSuccessFunc)((statusValue)), (statusValue))


#define LOG_ERROR_STATUS_BASE(statusNameFunc, statusValue, fmt, ...) \
{\
  const char* updatedFmt = _AppendErrorStatusName((statusNameFunc)((statusValue)), (fmt));\
  LOG_ERROR(updatedFmt, ##__VA_ARGS__);\
  free((char*)updatedFmt);\
}

#define CHECK_STATUS_LOG_RETURN_BASE(statusSuccessFunc, logFunc, statusValue, fmt, ...) \
{\
  if ((statusSuccessFunc)((statusValue)) == false)\
  {\
    logFunc((statusValue), (fmt), ##__VA_ARGS__);\
    return (statusValue);\
  }\
}

/**
 * @brief Generic error status
 */
typedef enum {
  STATUS_OK = 0,
  STATUS_INVALID_PARAM,
  STATUS_INVALID_OPERATION,
  STATUS_NULL_POINTER,

} ErrStatus;

inline bool ErrorStatusSuccess(const ErrStatus status) { return status == STATUS_OK; }
inline bool ErrorStatusFailed(const ErrStatus status) { return status != STATUS_OK; }
const char* ErrorStatusGetName(const ErrStatus status); 


#define CHECK_STATUS_RETURN(status) \
  CHECK_STATUS_RETURN_BASE(ErrorStatusSuccess, (status))

#define LOG_ERROR_STATUS(status, fmt, ...) LOG_ERROR_STATUS_BASE(ErrorStatusGetName, (status), (fmt), ##__VA_ARGS__)

#define CHECK_STATUS_LOG_RETURN(status, fmt, ...) \
  CHECK_STATUS_LOG_RETURN_BASE(ErrorStatusSuccess, LOG_ERROR_STATUS, (status), (fmt), ##__VA_ARGS__)

#define CHECK_STATUS_LOG(status, fmt, ...) \
  CHECK_STATUS_LOG_BASE(ErrorStatusSuccess, LOG_ERROR_STATUS, (status), (fmt), ##__VA_ARGS__)

#define CHECK_NULL_POINTER_RETURN(pointer) \
  ASSERT_CHECK_RETURN((pointer) != NULL, STATUS_NULL_POINTER)

#define CHECK_NULL_POINTER_LOG_RETURN(pointer, fmt, ...) \
  ASSERT_CHECK_LOG_RETURN((pointer) != NULL, STATUS_NULL_POINTER, fmt, ##__VA_ARGS__)

#endif