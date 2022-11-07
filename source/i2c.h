/**
 * @file i2c.h
 * @author Mit Bailey (mitbailey@outlook.com)
 * @brief 
 * @version See Git tags for version information.
 * @date 2022.11.07
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef I2C_H
#define I2C_H

#include <Wire.h>

/**
 * @brief Write bytes to the i2c device.
 * Note: Bus access by this function is protected by a recursive
 * pthread mutex.
 * 
 * @param dev_addr I2C device address.
 * @param buf Pointer to byte array to write (MSB first)
 * @param len Length of byte array
 * @return uint8_t Length of bytes written.
 */
static inline uint8_t i2cbus_write(uint8_t dev_addr, uint8_t *buf, uint8_t len)
{
    Wire.beginTransmission(dev_addr);
    uint8_t bytes = Wire.write(buf, len);
    Wire.endTransmission();
    return bytes;
}

/**
 * @brief Read bytes from the i2c device.
 * Note: Bus access by this function is protected by a recursive
 * pthread mutex.
 * 
 * @param dev_addr I2C device address.
 * @param buf Pointer to byte array to read to (MSB first)
 * @param len Length of byte array
 * @return uint8_t Length of bytes read.
 */
static inline uint8_t i2cbus_read(uint8_t dev_addr, uint8_t *buf, uint8_t len)
{
    Wire.requestFrom(dev_addr, len);
    uint8_t i = 0;
    for(; i < len; i++)
        buf[i] = Wire.read();
    return i;
}

/**
 * @brief Function to do a write, and get the reply in one operation. 
 * 
 * Avoid using this function if you have read or write buffer lengths zero.
 * 
 * @param dev_addr I2C device address.
 * @param outbuf Pointer to byte array to write (MSB first).
 * @param outlen Length of output byte array.
 * @param inbuf Pointer to byte array to read to (MSB first), can be the same as out_buf.
 * @param inlen Length of input byte array.
 * @return uint8_t Length of bytes read.
 */
static inline uint8_t i2cbus_transfer(uint8_t dev_addr, uint8_t *out_buf, uint8_t out_len, uint8_t *in_buf, uint8_t in_len)
{
    Wire.beginTransmission(dev_addr);
    Wire.write(out_buf, out_len);
    Wire.endTransmission();

    Wire.requestFrom(dev_addr, in_len);
    uint8_t i = 0;
    for(; i < in_len; i++)
        in_buf[i] = Wire.read();
    return i;
}

#endif // I2C_H