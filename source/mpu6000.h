/**
 * @file mpu6000.h
 * @author Mit Bailey (mitbailey@outlook.com)
 * @brief 
 * @version See Git tags for version information.
 * @date 2022.11.07
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef MPU6000_H
#define MPU6000_H

/// MPU
#define MPU6000_I2CADDR_DEFAULT 0x69    // MPU6000 default i2c address w/ AD0 high
#define MPU6000_DEVICE_ID 0x68          // The correct MPU6000_WHO_AM_I value
#define MPU6000_SELF_TEST_X 0x0D        // Self test factory calibrated values register
#define MPU6000_SELF_TEST_Y 0x0E 
#define MPU6000_SELF_TEST_Z 0x0F 
#define MPU6000_SELF_TEST_A 0x10 
#define MPU6000_SMPLRT_DIV 0x19         // sample rate divisor register
#define MPU6000_CONFIG 0x1A             // General configuration register
#define MPU6000_GYRO_CONFIG 0x1B        // Gyro specfic configuration register
#define MPU6000_ACCEL_CONFIG 0x1C       // Accelerometer specific configration register
#define MPU6000_INT_PIN_CONFIG 0x37     // Interrupt pin configuration register
#define MPU6000_WHO_AM_I 0x75           // Divice ID register
#define MPU6000_SIGNAL_PATH_RESET 0x68  // Signal path reset register
#define MPU6000_USER_CTRL 0x6A          // FIFO and I2C Master control register
#define MPU6000_PWR_MGMT_1 0x6B         // Primary power/sleep control register
#define MPU6000_PWR_MGMT_2 0x6C         // Secondary power/sleep control register
#define MPU6000_TEMP_H 0x41             // Temperature data high byte register
#define MPU6000_TEMP_L 0x42             // Temperature data low byte register
#define MPU6000_ACCEL_OUT 0x3B          // base raw accel address (6 bytes for 3 axis)
#define MPU6000_TEMP_OUT 0x41           // base raw temp address
#define MPU6000_GYRO_OUT 0x43           // base raw gyro address (6 bytes for 3 axis)
#define MPU6000_CONFIG_FS_SEL_BIT 4
#define MPU6000_CONFIG_FS_SEL_LEN 2
#define MPU_ONE_G 9.80665

// From Adafruit Lib
#define MPU6000_BAND_260_HZ 0x0 //< Docs imply this disables the filter
#define MPU6000_BAND_184_HZ 0x1 //< 184 Hz
#define MPU6000_BAND_94_HZ 0x2  //< 94 Hz
#define MPU6000_BAND_44_HZ 0x3  //< 44 Hz
#define MPU6000_BAND_21_HZ 0x4  //< 21 Hz
#define MPU6000_BAND_10_HZ 0x5  //< 10 Hz
#define MPU6000_BAND_5_HZ 0x6   //< 5 Hz

#define MPU6000_RANGE_2_G 0x0
#define MPU6000_RANGE_4_G 0x8
#define MPU6000_RANGE_8_G 0x10
#define MPU6000_RANGE_16_G 0x18

#include "i2c.h"
#include "utilities.h"

int8_t mpu6000_init()
{    
    // NOTE: See sketch_apr04a.cpp for how to properly read and write with MPU6050.
    // Declare our I2C write and read buffers.
    uint8_t wr_buf[2] = {0};
    uint8_t rd_buf[2] = {0};

    // Read from the WhoAmI? register.
    wr_buf[0] = MPU6000_WHO_AM_I;
    i2cbus_transfer(ADDR_MPU6000, wr_buf, 1, rd_buf, 1);

    // Serial.print("WhoAmI: ");
    // Serial.println(rd_buf[0], HEX);

    if (rd_buf[0] != ADDR_MPU6000)
    {
        return -1;
    }

    // Wake the MPU6000.
    wr_buf[0] = MPU6000_PWR_MGMT_1;
    wr_buf[1] = 0x0;
    i2cbus_write(ADDR_MPU6000, wr_buf, 2);

    /* NOTE: We may not want to do this if it changes the value of the wake-up register bit. May not be necessary.
    // Begin resetting of all registers to defaults.
    wr_buf[0] = MPU6000_PWR_MGMT_1;
    wr_buf[1] = 0b10000000;
    i2cbus_write(ADDR_MPU6000, wr_buf, 2);

    // sbprintlf("Send reset-all-registers command."); */

    /* NOTE: Uncomment after testing.
    // Reset analog and digital signal paths of the gyro, accel, and temp sensors.
    wr_buf[0] = MPU6000_SIGNAL_PATH_RESET;
    wr_buf[1] = 0b00000111;
    i2cbus_write(ADDR_MPU6000, wr_buf, 2);

    // sbprintlf("Reset analog and digital signal paths of the gyroscope, accelerometer, and temperature sensors."); */

    // Set to -2G to 2G range.
    wr_buf[0] = MPU6000_ACCEL_CONFIG;
    wr_buf[1] = MPU6000_RANGE_4_G;
    i2cbus_write(ADDR_MPU6000, wr_buf, 2);

    // Check to see if the range is set properly.
    i2cbus_transfer(ADDR_MPU6000, wr_buf, 1, rd_buf, 1);

    if (rd_buf[0] != wr_buf[1])
    {
        return -2;
    }

    /* NOTE: Uncomment after testing.
    // Set sample rate divisor.
    // Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
    wr_buf[0] = MPU6000_SMPLRT_DIV;
    wr_buf[1] = 0x0; // 
    i2cbus_write(ADDR_MPU6000, wr_buf, 2);

    // Check to see if the sample rate was set properly.
    i2cbus_transfer(ADDR_MPU6000, wr_buf, 1, rd_buf, 1);

    // sbprintlf("Set sample rate divisor. Register reads: 0x%02X", rd_buf[0]); */

    /* NOTE: Uncomment after testing.
    // Set filter bandwidth.
    // First read what the current value is before clobbering anything.
    wr_buf[0] = MPU6000_CONFIG;
    i2cbus_transfer(ADDR_MPU6000, wr_buf, 1, rd_buf, 1);

    // sbprintlf("Read CONFIG register as: 0x%02X", rd_buf[0]);

    // We want the register to be DDDDD000, where Ds are don't-cares.
    // Therefore, we must take the current register value, and bitwise AND it with the result of 0b11111000 (DDDDD000) bitwise ORed with our desired band value.
    wr_buf[1] = rd_buf[0] & (0b11111000 | MPU6000_BAND_260_HZ);
    i2cbus_write(ADDR_MPU6000, wr_buf, 2);

    // sbprintlf("Attempted to set CONFIG register to: 0x%02X", wr_buf[1]);

    // Check to make sure we set the register correctly.
    i2cbus_transfer(ADDR_MPU6000, wr_buf, 1, rd_buf, 1);

    // sbprintlf("CONFIG register now reads as: 0x%02X", rd_buf[0]); */
    
    /* NOTE: Uncomment after testing. 
    // Select PLL with X axis gyroscope reference as our clock (defaults to internal oscillator).
    wr_buf[0] = MPU6000_PWR_MGMT_1;
    wr_buf[1] = 0x1;
    i2cbus_write(ADDR_MPU6000, wr_buf, 2);

    // Ensure PWR_MGMT_1 was set properly.
    i2cbus_transfer(ADDR_MPU6000, wr_buf, 1, rd_buf, 1);

    // sbprintlf("PWR_MGMT_1 register reads: 0x%02X", rd_buf[0]); */

    // sbprintlf("MPU6000 boot complete."); 

    return 0;
}

int16_t mpu6000_sample_magnitude()
{
    // Declare I2C read/write buffers.
    uint8_t wr_buf[1] = {0};
    uint8_t rd_buf[6] = {0};

    // Request six bytes of accelerometer data.
    wr_buf[0] = MPU6000_ACCEL_OUT; // For some reason we can get all six registers of accel. data (0x3B - 0x40) just by querying the first one.
    i2cbus_transfer(ADDR_MPU6000, wr_buf, 1, rd_buf, 6);

    uint32_t mag2 = 0;
    for (byte i = 6; i > 0; i-=2)
    {
        int32_t val = (rd_buf[i - 2] << 8 | rd_buf[i - 1]);
        mag2 += val * val;
    }

    return isqrt(mag2);
}

#endif // MPU6000_H