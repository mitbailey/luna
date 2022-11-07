/**
 * @file mlx90393.h
 * @author Mit Bailey (mitbailey@outlook.com)
 * @brief 
 * @version See Git tags for version information.
 * @date 2022.11.07
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef MLX90393_H
#define MLX90393_H

/// MLX
#define MLX90393_AXIS_ALL 0x0E      /**< X+Y+Z axis bits for commands. */
#define MLX90393_CONF1 0x00         /**< Gain */
#define MLX90393_CONF2 0x01         /**< Burst, comm mode */
#define MLX90393_CONF3 0x02         /**< Oversampling, filter, res. */
#define MLX90393_CONF4 0x03         /**< Sensitivty drift. */
#define MLX90393_GAIN_SHIFT 0x04       /**< Left-shift for gain bits. */
#define MLX90393_HALL_CONF 0x0C     /**< Hall plate spinning rate adj. */
#define MLX90393_STATUS_OK 0x00     /**< OK value for status response. */
#define MLX90393_STATUS_SMMODE 0x08 /**< SM Mode status response. */
#define MLX90393_STATUS_RESET 0x01  /**< Reset value for status response. */
#define MLX90393_STATUS_ERROR 0xFF  /**< OK value for status response. */
#define MLX90393_STATUS_MASK 0xFC   /**< Mask for status OK checks. */

/** Register map. */
#define MLX90393_REG_SB 0x10  /**< Start burst mode. */
#define MLX90393_REG_SW 0x20  /**< Start wakeup on change mode. */
#define MLX90393_REG_SM 0x30  /**> Start single-meas mode. */
#define MLX90393_REG_RM 0x40  /**> Read measurement. */
#define MLX90393_REG_RR 0x50  /**< Read register. */
#define MLX90393_REG_WR 0x60  /**< Write register. */
#define MLX90393_REG_EX 0x80  /**> Exit moode. */
#define MLX90393_REG_HR 0xD0  /**< Memory recall. */
#define MLX90393_REG_HS 0x70  /**< Memory store. */
#define MLX90393_REG_RT 0xF0  /**< Reset. */
#define MLX90393_REG_NOP 0x00 /**< NOP. */

/** Gain settings for CONF1 register. (Modified) */
#define MLX90393_GAIN_2_5X 0x00
#define MLX90393_GAIN_1X 0x01

/** Resolution settings for CONF3 register. */
#define MLX90393_RES_16 0x0
#define MLX90393_RES_17 0x1
#define MLX90393_RES_18 0x2
#define MLX90393_RES_19 0x3

/** Digital filter settings for CONF3 register (Modified) */
// #define MLX90393_FILTER_6 0x0
// #define MLX90393_FILTER_7 0x1
#define MLX90393_FILTER_1 0x0
#define MLX90393_FILTER_2 0x1
#define MLX90393_FILTER_3 0x2
#define MLX90393_FILTER_4 0x3
#define MLX90393_FILTER_5 0x4
#define MLX90393_FILTER_6 0x5
#define MLX90393_FILTER_7 0x6
#define MLX90393_FILTER_8 0x7

/** Oversampling settings for CONF3 register. */
#define MLX90393_OSR_0 0x0
#define MLX90393_OSR_1 0x1
#define MLX90393_OSR_2 0x2
#define MLX90393_OSR_3 0x3

#include "i2c.h"

int8_t mlx90393_init()
{
    // magnetometer.begin_I2C(); // Pointless
    // inplaceof: magnetometer.setGain(MLX90393_GAIN_2_5X);
    {
        uint8_t wr_buf[4] = {0};
        uint8_t rd_buf[3] = {0};

        // Retrieve configuration data bytes.
        wr_buf[0] = MLX90393_REG_RR;
        wr_buf[1] = (MLX90393_CONF1 << 2); // MLX90393 requires addresses to be left shifted 2.
        i2cbus_transfer(ADDR_MLX90393, wr_buf, 2, rd_buf, 2);

        // Mask off gain bits from low byte.
        wr_buf[2] = rd_buf[0];
        wr_buf[3] = rd_buf[1] & 0b10001111;

        // Set gain bits.
        wr_buf[3] |= 0b01110000;

        // Simultaneously sets the CONF1 register and retrieves status byte.
        wr_buf[0] = MLX90393_REG_WR;
        i2cbus_transfer(ADDR_MLX90393, wr_buf, 4, rd_buf, 1);

        // TODO: Check the status byte.
    }
    
    // Settings resolutions of X, Y, and Z axes have been combined (same register).
    // inplaceof: magnetometer.setResolution(MLX90393_X, MLX90393_RES_19),
    //            magnetometer.setResolution(MLX90393_Y, MLX90393_RES_19),
    //        and magnetometer.setResolution(MLX90393_Z, MLX90393_RES_19);
    {
        uint8_t wr_buf[4] = {0};
        uint8_t rd_buf[3] = {0};

        // Retrieve CONF3 data.
        wr_buf[0] = MLX90393_REG_RR;
        wr_buf[1] = (MLX90393_CONF3 << 2); // MLX90393 requires addresses to be left shifted 2.
        i2cbus_transfer(ADDR_MLX90393, wr_buf, 2, rd_buf, 2);

        // ANDing w/ 0s clears the current values of the XYZ resolutions so we can OR in our desired values.
        wr_buf[3] = rd_buf[1] & 0b00011111; // Bits 7-0
        wr_buf[2] = rd_buf[0] & 0b11111000; // Bits 15-8

        // Shifting these over more than 8 bits works because it flows over into wr_buf[2].
        wr_buf[3] |= (MLX90393_RES_19 << 5); // Sets the X resolution.
        wr_buf[3] |= (MLX90393_RES_19 << 7); // Sets the Y resolution.
        wr_buf[3] |= (MLX90393_RES_19 << 9); // Sets the Z resolution.

        // Simultaneously sets the register and retrieves status byte.
        wr_buf[0] = MLX90393_REG_WR;
        i2cbus_transfer(ADDR_MLX90393, wr_buf, 4, rd_buf, 1);

        // TODO: Check the status byte.
    }

    // inplaceof: magnetometer.setOversampling(MLX90393_OSR_2);
    {
        uint8_t wr_buf[4] = {0};
        uint8_t rd_buf[3] = {0};

        // Read data from CONF3.
        wr_buf[0] = MLX90393_REG_RR;
        wr_buf[1] = (MLX90393_CONF3 << 2); // MLX90393 requires addresses to be left shifted 2.
        i2cbus_transfer(ADDR_MLX90393, wr_buf, 2, rd_buf, 2);

        wr_buf[3] = rd_buf[1] & 0b11111100; // Opens up the OSR bits (magnetic sensor oversampling).
        wr_buf[2] = rd_buf[0] & 0b11100111; // Opens up the OSR2 bits (temperature sensor oversampling).

        wr_buf[3] |= (MLX90393_OSR_2); // Sets the magnetometer oversampling bits.
        wr_buf[2] |= (MLX90393_OSR_2 << 3); // Sets the temperature sensor oversampling bits.

        // Simultaneously sets the register and retrieves status byte.
        wr_buf[0] = MLX90393_REG_WR;
        i2cbus_transfer(ADDR_MLX90393, wr_buf, 4, rd_buf, 1);

        // TODO: Check the status byte.
    }

    // inplaceof: magnetometer.setFilter(MLX90393_FILTER_6);
    {
        uint8_t wr_buf[4] = {0};
        uint8_t rd_buf[3] = {0};

        // Read data from CONF3.
        wr_buf[0] = MLX90393_REG_RR;
        wr_buf[1] = (MLX90393_CONF3 << 2); // MLX90393 requires addresses to be left shifted 2.
        i2cbus_transfer(ADDR_MLX90393, wr_buf, 2, rd_buf, 2);

        wr_buf[3] = rd_buf[1] & 0b11100011;
        wr_buf[2] = rd_buf[0]; // Do nothing.

        wr_buf[3] |= (MLX90393_FILTER_4 << 2);

        // Simultaneously sets the register and retrieves status byte.
        wr_buf[0] = MLX90393_REG_WR;
        i2cbus_transfer(ADDR_MLX90393, wr_buf, 4, rd_buf, 1);

        // TODO: Check the status byte.
    }

    // inplaceof: magnetometer.setTrigInt(false);
    {
        uint8_t wr_buf[4] = {0};
        uint8_t rd_buf[3] = {0};

        // Read data from CONF3.
        wr_buf[0] = MLX90393_REG_RR;
        wr_buf[1] = (MLX90393_CONF2 << 2); // MLX90393 requires addresses to be left shifted 2.
        i2cbus_transfer(ADDR_MLX90393, wr_buf, 2, rd_buf, 2);

        wr_buf[3] = rd_buf[1] & 0b01111111; // Effectively sets the trigger bit to 0 (false).
        wr_buf[2] = rd_buf[0]; // Do nothing.

        // Simultaneously sets the register and retrieves status byte.
        wr_buf[0] = MLX90393_REG_WR;
        i2cbus_transfer(ADDR_MLX90393, wr_buf, 4, rd_buf, 1);

        // TODO: Check the status byte.
    }
}

#endif // MLX90393