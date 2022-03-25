/**
 * @file main.ino
 * @author Mit Bailey (mitbailey99@gmail.com)
 * @brief
 * @version See Git tags for version information.
 * @date 2022.03.25
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <Wire.h>

#define NUM_I2C_DEV 4

// LunaSat V4 Sensors
// AK09940 -- Magnetometer -- 0x0c
// TMP117 --- Thermometer --- 0x48
// TPIS1385 - Thermopile ---- 0x0d
// ICM20602 - Accelerometer - 0x69

// I2C Information
uint8_t sensAddr[4] = {0x0c, 0x48, 0x0D, 0x69};
uint8_t addr, err, n_devices;

/**
 * @brief
 *
 * @param thorough If TRUE, checks all possible addresses for I2C.
 */
void I2C_scan(bool thorough)
{
    uint8_t i_max = NUM_I2C_DEV;

    if (thorough)
    {
        i_max = 127;
    }

    for (uint8_t i = 0; i <= i_max; i++)
    {
        Wire.beginTransmission(i);

        switch (Wire.endTransmission())
        {
        case 0:
            n_devices++;
            Serial.print("Address ");
            Serial.print(sensAddr[i], HEX);
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
            Serial.print(sensAddr[i], HEX);
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

void setup()
{
    // I2C Initialization
    Wire.begin();
    Wire.setClock(100000); // I2C frequency in Hertz (100kHz standard).
    Serial.begin(9600);

    if (!Serial)
    {
        delay(100); // Delay if serial not yet ready.
    }

    Serial.print("I2C and Serial initialization complete.\n");

    // Thermopile Initialization
    Wire.beginTransmission(0x00); // TX buffer.
    Wire.write(0x04);             // Add register address to TX buffer.
    Wire.write(0x00);             // Add initialization data to TX.
    Wire.endTransmission();       // Send.

    delay(50);

    Serial.print("Thermopile init command sent.\n"); // Note that we havent actually checked if its working.

    // No idea what is on this pin, assuming an LED maybe.
    pinMode(A3, OUTPUT);
    digitalWrite(A3, HIGH);
}

void loop()
{
}