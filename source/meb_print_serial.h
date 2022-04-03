/**
 * @file meb_print_serial.h
 * @author Mit Bailey (mitbailey99@gmail.com)
 * @brief Arduino serial version of meb_print.h
 * @version See Git tags for version information.
 * @date 2022.04.02
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef MEB_PRINT_SERIAL_H
#define MEB_PRINT_SERIAL_H

#include <stdio.h>

#ifndef sdbprintlf
#ifdef SERIAL_DEBUG_PRINT_ENABLE
#define sdbprintlf(format, ...)                                                                                             \
    {                                                                                                                       \
        char buffer[snprintf(NULL, 0, "[%s:%d | %s] " format "\n", __FILE__, __LINE__, __func__, ##__VA_ARGS__) + 1] = {0}; \
        sprintf(buffer, "[%s:%d | %s] " format "\n", __FILE__, __LINE__, __func__, ##__VA_ARGS__);                          \
        Serial.print(buffer);                                                                                               \
    }
#else // SERIAL_DEBUG_PRINT_ENABLE
#define sdbprintlf(format, ...){}
#endif // SERIAL_DEBUG_PRINT_ENABLE
#endif // sdbprintf

#ifndef sdbprintf
#ifdef SERIAL_DEBUG_PRINT_ENABLE
#define sdbprintf(format, ...)                                                                                             \
    {                                                                                                                       \
        char buffer[snprintf(NULL, 0, "[%s:%d | %s] " format "\n", __FILE__, __LINE__, __func__, ##__VA_ARGS__) + 1] = {0}; \
        sprintf(buffer, "[%s:%d | %s] " format, __FILE__, __LINE__, __func__, ##__VA_ARGS__);                          \
        Serial.print(buffer);                                                                                               \
    }
#else // SERIAL_DEBUG_PRINT_ENABLE
#define sdbprintf(format, ...){}
#endif // SERIAL_DEBUG_PRINT_ENABLE
#endif // sdbprintf

#endif // MEB_PRINT_SERIAL_H