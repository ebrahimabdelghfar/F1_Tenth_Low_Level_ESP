#include "rtos_utils_lib.h"

void logInMutex(SemaphoreHandle_t *mutex, const char *taskName)
{
  if (*mutex != NULL)
  {
    xSemaphoreTake(*mutex, portMAX_DELAY);
  }
}

void logOutMutex(SemaphoreHandle_t *mutex, const char *taskName)
{
  if (*mutex != NULL)
  {
    xSemaphoreGive(*mutex);
  }
}