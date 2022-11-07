/**
 * @file sx1272.h
 * @author Mit Bailey (mitbailey@outlook.com)
 * @brief 
 * @version See Git tags for version information.
 * @date 2022.11.07
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef SX1272_H
#define SX1272_H

// Radio Defines
// From: https://github.com/GLEE2023/GLEE2023_Lunar/blob/main/src/GLEE_Radio_Lunar.h
#define SX1272_NSS_PIN   10
#define SX1272_DIO1_PIN  3
#define SX1272_DIO0_PIN  2
#define SX1272_RESET_PIN 9
#define SX1272_FREQ      915.0 // Frequency (MHz)
#define SX1272_PWR       17 // Power
#define SX1272_BW        250.0 // Bandwidth
#define SX1272_SF        12 // Spreading Factor
#define SX1272_CR        8 // Coding Rate

#include "i2c.h"

int8_t sx1272_init()
{
    return -1;
}

#endif // SX1272_H