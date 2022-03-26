/**
 * @file luna.cpp
 * @author Mit Bailey (mitbailey99@gmail.com)
 * @brief 
 * @version See Git tags for version information.
 * @date 2022.03.25
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <Wire.h>

#include "luna.hpp"

void I2C_scan(bool thorough)
{
    uint8_t addr, err, n_devices;
    uint8_t i_max = NUM_I2C_DEV;

    if (thorough)
    {
        i_max = 127;
    }

    for (uint8_t i = 0; i <= i_max; i++)
    {
        Wire.beginTransmission(thorough ? i : sensAddr[i]);

        switch (Wire.endTransmission())
        {
        case 0:
            n_devices++;
            Serial.print("Address ");
            Serial.print(thorough ? i : sensAddr[i], HEX);
            Serial.print(".\n");
            break;
        case 1:
            Serial.print("Error 1: Data too long to fit in transmit buffer.\n");
            break;
        case 2:
            if (!thorough)
                Serial.print("Error 2: Received NACK on transmit of address.\n");
            break;
        case 3:
            Serial.print("Error 3: Received NACK on transmit of data.\n");
            break;
        case 4:
            Serial.print("Error value 4 at address ");
            Serial.print(thorough ? i : sensAddr[i], HEX);
            Serial.print(".\n");
            break;
        default:
            break;
        }
    }

    Serial.print("I2C scan complete: ");
    Serial.print(n_devices);
    Serial.print(" devices found.\n");
}