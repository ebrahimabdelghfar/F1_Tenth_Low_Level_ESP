/**
 * @file rtos_utils_lib.h
 * @brief Utility functions for FreeRTOS synchronization and logging
 * @author Ebrahim Abdelghfar Ebrahim
 */

#ifndef __RTOS_UTILS_LIB_H__
#define __RTOS_UTILS_LIB_H__
#include <Arduino.h>

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief logInMutex - Logs and takes a mutex
     * @param mutex Pointer to the mutex handle to take
     * @param taskName Name of the task taking the mutex (for logging)
     * @returns void
     */
    void logInMutex(SemaphoreHandle_t *mutex, const char *taskName);

    /**
     * @brief logOutMutex - Logs and releases a mutex
     * @param mutex Pointer to the mutex handle to release
     * @param taskName Name of the task releasing the mutex (for logging)
     * @returns void
     */
    void logOutMutex(SemaphoreHandle_t *mutex, const char *taskName);

#ifdef __cplusplus
}
#endif

#endif /* __RTOS_UTILS_LIB_H__ */