/**
 * @file sketch_apr04a.cpp
 * @author Mit Bailey (mitbailey99@gmail.com)
 * @brief Successfully initializes (basic init) and reads accelerometer data from MPU6050; translates data into g-units.
 * @version See Git tags for version information.
 * @date 2022.04.04
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <Wire.h>

void setup()
{
    Serial.begin(9600);
    delay(1000);

    Serial.println("Beginning test.");

    // Read WhoAmI? register.
    Wire.beginTransmission(0x68);
    Wire.write(0x75);
    Wire.endTransmission();

    Wire.requestFrom(0x68, 1);

    for (uint16_t data = 0, i = 0; Wire.available() && i < UINT16_MAX; i++)
    {
        data = Wire.read();
        Serial.println(data, HEX);
    }

    // Wake up the MPU.
    Wire.beginTransmission(0x68);
    Wire.write(0x6B);
    Wire.write(0x0);
    Wire.endTransmission();

    // Read ACCEL_CONFIG register.
    Wire.beginTransmission(0x68);
    Wire.write(0x1C);
    Wire.endTransmission();

    Wire.requestFrom(0x68, 1);

    for (uint16_t data = 0, i = 0; Wire.available() && i < UINT16_MAX; i++)
    {
        data = Wire.read();
        Serial.println(data, HEX);
    }

    // Set ACCEL_CONFIG register.
    Wire.beginTransmission(0x68);
    Wire.write(0x1C);
    Wire.write(0b00000000);
    Wire.endTransmission();

    // Read ACCEL_CONFIG register.
    Wire.beginTransmission(0x68);
    Wire.write(0x1C);
    Wire.endTransmission();

    Wire.requestFrom(0x68, 1);

    for (uint16_t data = 0, i = 0; Wire.available() && i < UINT16_MAX; i++)
    {
        data = Wire.read();
        Serial.println(data, HEX);
    }
}

void loop()
{
    // Read ACCEL data.
    Wire.beginTransmission(0x68);
    Wire.write(0x3B);
    Wire.endTransmission();

    Wire.requestFrom(0x68, 6);

    uint8_t data[6] = {0};
    for (uint16_t i = 0; Wire.available() && i < 6; i++)
    {
        data[i] = Wire.read();
        Serial.print(data[i], HEX);
        Serial.print(" ");
    }
    Serial.println();

    uint16_t acc_xyz[3] = {0};
    acc_xyz[0] = data[0] << 8 | data[1];
    acc_xyz[1] = data[2] << 8 | data[3];
    acc_xyz[2] = data[4] << 8 | data[5];
    Serial.print(acc_xyz[0]);
    Serial.print(" ");
    Serial.print(acc_xyz[1]);
    Serial.print(" ");
    Serial.print(acc_xyz[2]);
    Serial.println();

    float acc_xyz_f[3] = {0};
    for (int i = 0; i < 3; i++)
    {
        if (acc_xyz[i] > 32767)
            acc_xyz_f[i] = 4.f - (((float)acc_xyz[i]) / (16384.f));
        else
            acc_xyz_f[i] = 0.f - (((float)acc_xyz[i]) / (16384.f));
    }

    Serial.print(acc_xyz_f[0]);
    Serial.print(" ");
    Serial.print(acc_xyz_f[1]);
    Serial.print(" ");
    Serial.print(acc_xyz_f[2]);
    Serial.println("\n");

    delay(500);
}