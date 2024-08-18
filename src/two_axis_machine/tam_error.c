#include "two_axis_machine/tam_error.h"

/**
 * @brief Returns the name of the two axis machine error status
 * 
 * @param status        The status to get the name of
 * @return const char*  Pointer to the status name
 */
const char* TAM_StatusGetName(const TAM_Status status)
{
  #define NAME_CASE(val) case (val): return #val

  switch (status)
  {
    NAME_CASE(TAM_OK);
    NAME_CASE(TAM_INVALID_PARAM);
    NAME_CASE(TAM_INVALID_OPERATION);
    NAME_CASE(TAM_NULL_POINTER);
    NAME_CASE(TAM_UNSAFE_OPERATION);
    NAME_CASE(TAM_TRAVEL_BLOCKED);
    NAME_CASE(TAM_INVALID_STRING_COMMAND);
    default:
      LOG_ERROR("Invalid error status %d", status);
      return NULL;
  }

  #undef NAME_CASE
}