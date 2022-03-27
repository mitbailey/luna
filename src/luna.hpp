/**
 * @file luna.hpp
 * @author Mit Bailey (mitbailey99@gmail.com)
 * @brief 
 * @version See Git tags for version information.
 * @date 2022.03.25
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef LUNA_HPP
#define LUNA_HPP

// Pins
#define PIN_LED_1 4
#define PIN_LED_2 5
#define PIN_SOLAR_V A1

#define NUM_I2C_DEV 4
#define AN_TO_V(An) An * (3.3 / 1023.f)
#define CADENCE 500 // loop cadence in milliseconds
#define MAX_PACKET_SIZE 256 // bytes
// TODO: Update SAT_ID to our team number when one is issued.
#define SAT_ID "42" // EXAMPLE
#define SAT_ID_B1 0x34 // ASCII 4 in binary EXAMPLE
#define SAT_ID_B2 0x32 // ASCII 2 in binary EXAMPLE


// I2C Information
// LunaSat V4 Sensors
// AK09940 -- Magnetometer -- 0x0c
// TMP117 --- Thermometer --- 0x48
// TPIS1385 - Thermopile ---- 0x0d
// ICM20602 - Accelerometer - 0x69
static uint8_t sensAddr[4] = {0x0c, 0x48, 0x0D, 0x69};

/**
 * @brief
 *
 * @param thorough If TRUE, checks all possible addresses for I2C.
 */
void I2C_scan(bool thorough = false);

#endif // LUNA_HPP