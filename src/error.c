#include "error.h"
#include "stdio.h"
#include "trace.h"
#include "string.h"

/**
 * @brief Helper function used to add the error status name to error messages
 * 
 * @param pStatusName         The status name to add
 * @param pFmt                The error message format
 * @return char const* const  The modified format message. Must be freed by the user.
 */
char const * const _AppendErrorStatusName(char const * const pStatusName, char const* const pFmt)
{
  size_t bufferSize = snprintf(NULL, 0, "%s - %s", pStatusName, pFmt);

  char * const pBuffer = (char*)malloc(bufferSize);
  snprintf(pBuffer, bufferSize, "%s - %s", pStatusName, pFmt);

  return pBuffer;
}

/**
 * @brief Get the error status name from the id
 * 
 * @param status        The error status id
 * @return const char*  The friendly error status name
 */
const char* ErrorStatusGetName(const ErrStatus status)
{
  #define NAME_CASE(val) case (val): return #val

  switch (status)
  {
    NAME_CASE(STATUS_OK);
    NAME_CASE(STATUS_INVALID_PARAM);
    NAME_CASE(STATUS_INVALID_OPERATION);
    NAME_CASE(STATUS_NULL_POINTER);
    default:
      LOG_ERROR("Invalid error status %d", status);
      return NULL;
  }

  #undef NAME_CASE
}
