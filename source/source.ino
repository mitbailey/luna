/**
 * @file source.ino
 * @author Mit Bailey (mitbailey99@gmail.com)
 * @brief Flight software for LunaSat.
 * @version See Git tags for version information.
 * @date 2022.03.25
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <Wire.h>

#define SERIAL_PRINT_EN // Enables printouts.

// #include "meb_print_serial.h"

// The program loop is guaranteed to run at most this many times per second.
#define MAX_FREQUENCY ((uint8_t)(60)) // 60

/// SENSOR USAGE CONDITIONALS
// #define USING_MLX90393
#define USING_MPU6000
// #define USING_SX1272
// #define USING_TMP117
// #define USING_TPIS1385
// #define USING_CAP

// TODO: Change to maximum possible length.
// Accelerometer Data Array Size = ACC_DATA_LEN * 6Bytes
// LENGTH CANNOT EXCEED 254 (255 - 1) due to uint8_t limits.
#define ACC_DATA_LEN 210 // If ACC_AVG_SAMP == MAX_FREQUENCY then we will have ACC_DATA_LEN seconds worth of recording time.
// How many samples do we average together before moving on?
#define ACC_AVG_SAMP ((uint8_t)(3)) // 10 // How many samples go into each average.


// Event ACTIVE if >=ACC_EVENT_ON_NUM of the last ACC_EVENT_ON_CONSIDER_NUM averages exceed the threshold.
// Event INACTIVE if >=ACC_EVENT_OFF_NUM of the last ACC_EVENT_ON_CONSIDER_NUM averages do not exceed the threshold.
#define ACC_EVENT_ON_CONSIDER_NUM 10
#define ACC_EVENT_ON_NUM 1
#define ACC_EVENT_OFF_CONSIDER_NUM 20
#define ACC_EVENT_OFF_NUM 20

// How many samples are considered protected/recorded prior to declaration of an event?
#define ACC_EVENT_BACKLOG 30

#define ACC_RECALIBRATE ((uint8_t)(32))

///////////////////////////////
// SENSOR-SPECIFIC CONSTANTS //
///////////////////////////////

#define ADDR_MLX90393 0x0C
#define ADDR_MPU6000 0x68
// #define ADDR_MPU6000_2 0x69
#define ADDR_SX1272 0x00 // UNKNOWN
#define ADDR_TMP117 0x00 // UNKNOWN
#define ADDR_TPIS1385 0x00 // UNKNOWN
#define PIN_CAP A0 // Hardcoded analog pin on LunaSat.

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

/// TMP
#define TMP117_TEMP_REG 0X00
#define TMP117_CONFIG_REG 0x01
#define TMP117_HIGH_LIMIT_REG 0X02
#define TMP117_LOW_LIMIT_REG 0X03
#define TMP117_EEPROM_UL_REG 0X04
#define TMP117_EEPROM1_REG 0X05
#define TMP117_EEPROM2_REG 0X06
#define TMP117_TEMP_OFFSET_REG 0X07
#define TMP117_EEPROM3_REG 0X08
#define TMP117_DEVICE_ID 0X0F
#define TMP117_RESOLUTION 0.0078125

/// TP
#define TP_OBJECT 1                             // Object Temperature reg                         (3xBytes) 17bit value [read] 
#define TP_AMBIENT 3                            // Ambient Temp Reg                               (2xBytes) 15bit value [read]
#define TP_OBJECT_LP1 5                         // Low Pass filter of object signal value 1 reg   (3xBytes) 20bit value [read] comparison: 8
#define TP_OBJECT_LP2 7                         // Low Pass filter of object signal value 2 reg   (3xBytes) 20bit value [read] comparison: 8
#define TP_AMBIENT_LP3 10                       // Low Pass filter of ambient signal value 3 reg  (2xBytes) 16bit value [read] comparison: 2
#define TP_OBJECT_LP2_FROZEN 12                 // Low pass filter of object on motion reg        (3xbytes) 24bit
#define TP_PRESENCE 15                          // READ
#define TP_MOTION 16                            // READ
#define TP_AMBIENT_SHOCK 17                     // READ
#define TP_INTERUPT_STATUS 18                   // READ
#define TP_CHIP_STATUS 19                       // READ
#define TP_LOW_PASS_TIME_CONST_LP1_LP2 20       // READ/WRITE
#define TP_LOW_PASS_TIME_CONST_LP3 21           // READ/WRITE
#define TP_PRESENCE_THRESHOLD 22                // READ/WRITE
#define TP_MOTION_THRESHOLD 23                  // READ/WRITE
#define TP_AMBIENT_SHOCK_THRESHOLD 24           // READ/WRITE
#define TP_INTERUPT_MASK 25                     // READ/WRITE
#define TP_MULTIPLE 26                          // READ/WRITE
#define TP_TIMER_INTERRUPT 27                   // READ/WRITE
#define TP_OT_THRESHOLD_HIGH 28                 // READ/WRITE
#define TP_EEPROM_CONTROL 31                    // 1byte READ/WRITE
#define TP_PROTOCOL 32                          // READ/WRITE
#define TP_CHSUM 33                             // READ
#define TP_LOOKUP 41                            // READ
#define TP_PTAT25 42                            // 2xbytes 15 bit value READ
#define TP_M 44                                 // 2xbytes 16 bit value requires RegVal/100 offset typically 172 counts / K READ
#define TP_U0 46                                // 2xbytes 16 bit value requires RegVal + 32768 offset READ
#define TP_UOUT1 48                             // 2xbytes 16 bit value
#define TP_T_OBJ_1 50                           // 1xbytes 
#define TP_I2C_ADDR 63  
#define TPIS1385_I2C_ADDR 0x0D

// This type is used for storing processed accelerometer data into the acc_data array. Note: Processed meaning rolling averaged.
typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} int16_3_t;

/**
 * @brief Writes slave device and register addresses.
 * 
 * First writes to I2C bus the address of a slave device, then a register address.
 * 
 * @param sensor_addr The address of the slave device.
 * @param reg_addr The address of the register.
 * @return uint8_t I2C bus status.
 */
static inline uint8_t i2c_write_blank(uint8_t slave_addr, uint8_t reg_addr)
{
    Wire.beginTransmission(slave_addr);
    Wire.write(reg_addr);
    return Wire.endTransmission();
}

/**
 * @brief Writes slave device and register addresses, and one data byte.
 * 
 * First writes to I2C bus the address of a slave device, then a register address, followed by one byte of data.
 * 
 * @param slave_addr The address of the slave device.
 * @param reg_addr The address of the register.
 * @param data Value to be written.
 * @return uint8_t I2C bus status. 
 */
static inline uint8_t i2c_write_byte(uint8_t slave_addr, uint8_t reg_addr, uint8_t data)
{
    Wire.beginTransmission(slave_addr);
    Wire.write(reg_addr);
    Wire.write(data);
    return Wire.endTransmission();
}

/**
 * @brief Writes slave device and register addresses, and some number of data bytes.
 * 
 * First writes to I2C bus the address of a slave device, then a register address, followed by nbytes of data.
 * 
 * @param slave_addr The address of the slave device.
 * @param reg_addr The address of the register.
 * @param data Pointer to the beginning of nbytes of data.
 * @param nbytes Number of bytes to send of data.
 * @return uint8_t I2C bus status. 
 */
static inline uint8_t i2c_write_bytes(uint8_t slave_addr, uint8_t reg_addr, uint8_t *data, uint8_t nbytes = 1)
{
    Wire.beginTransmission(slave_addr);
    Wire.write(reg_addr);
    Wire.write(data, nbytes);
    return Wire.endTransmission();
}

/**
 * @brief Writes slave device and register addresses, and then requests nbytes of data.
 * 
 * First writes to I2C bus the address of a slave device, then a register address. It then requests n-bytes of data, and
 * reads data until either there are no more bytes to read or nbytes of data are read, whichever occurs first. 
 * 
 * @param slave_addr The address of the slave device.
 * @param reg_addr The address of the register.
 * @param data Pointer to the beginning of nbytes of memory for data storage.
 * @param nbytes Maximum number of bytes to be received.
 * @return uint8_t Bytes read from I2C.
 */
static inline uint8_t i2c_read_bytes(uint8_t slave_addr, uint8_t reg_addr, uint8_t *data, uint8_t nbytes = 1)
{
    Wire.beginTransmission(slave_addr);
    Wire.write(reg_addr);
    Wire.endTransmission();
    Wire.requestFrom(slave_addr, nbytes);
    uint16_t i = 0;
    for (; Wire.available() && i < UINT16_MAX; i++)
        data[i] = Wire.read();
    return i;
}

/**
 * @brief Requests nbytes of data without first writing anything to the I2C bus.
 * 
 * @param slave_addr The address of the slave device.
 * @param data Pointer to the beginning of nbytes of memory for data storage.
 * @param nbytes Maximum number of bytes to be received.
 * @return uint8_t 
 */
static inline uint8_t i2c_read_bytes_passive(uint8_t slave_addr, uint8_t *data, uint8_t nbytes = 1)
{
    Wire.requestFrom(slave_addr, nbytes);
    uint16_t i = 0;
    for (; Wire.available() && i < UINT16_MAX; i++)
        data[i] = Wire.read();
    return i;
}

// TODO: Migrate to the i2cbus_* functions: remove all of the above functions.
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

// TEMP
// MLX90393 magnetometer = MLX90393(1,false);
// MPU6000 accelerometer(1, false); // Sets sensor ID to 1 and debugging to false
// TMP117 thermometer(1,false);
// TPIS1385 thermopile(1);

// Actually important global variables.
float TPIS_cal_K = 0.f; // TPIS calibration constant.
uint16_t MPU6000_accel_scale = 0;
void setup()
{
    // NOTE: Setup sequences taken from the basic setup examples found in: 
    //       github.com/GLEE2023/GLEE2023/examples/Sensor_Examples

#ifdef SERIAL_PRINT_EN
    Serial.begin(9600);
#endif // SERIAL_PRINT_EN

    Wire.begin();
    Wire.setClock(100000);
    
    // CAP initialization and setup.
    // None required.

// #ifdef USING_MLX90393
    // MLX90393 initialization and setup.  
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
// #endif // USING_MLX90393

#ifdef USING_MPU6000
    // MPU6000 initialization and setup.
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
            exit(1);
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
            exit(1);
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
    }
#endif // USING_MPU6000

#ifdef USING_TMP117
    // TMP117 initialization and setup.
    // None required.

    // TPIS1385 initialization and setup.
    {
        // inplaceof: thermopile.begin(); // Thermopile start-up
        {
            Wire.begin(); // Begin i2c coms at standard speed
            Wire.beginTransmission(0x00); // Reload all call   
            Wire.write(0x04);
            Wire.write(0x00);         
            // if(Wire.endTransmission() != 0) 
            //     Serial.println(F("Init call failiure"));
            delay(50);  // Wait on i2c transmission
        }
        
        // inplaceof: thermopile.readEEprom(); // Prints eeprom and updates calibration constants
        {
            uint8_t eeprom_data[2] = {0};
            // Set EEPROM control to read.
            i2c_write_byte(ADDR_TPIS1385, TP_EEPROM_CONTROL, 0x80);
            // Read EEPROM protocol.
            i2c_read_bytes(ADDR_TPIS1385, TP_PROTOCOL, eeprom_data);
            // Read PTAT25 calibration value.
            i2c_read_bytes(ADDR_TPIS1385, TP_PTAT25, eeprom_data, 0x2);
            uint16_t TPIS_cal_PTAT25 = ((uint16_t) eeprom_data[0] << 8) | eeprom_data[1]; 
            // Read M calibration value.
            i2c_read_bytes(ADDR_TPIS1385, TP_M, eeprom_data, 0x2);
            uint16_t TPIS_cal_M = ((uint16_t) eeprom_data[0] << 8) | eeprom_data[1]; 
            TPIS_cal_M /= 100; // Apply appropriate offset
            // Read U0 calibration value.
            i2c_read_bytes(ADDR_TPIS1385, TP_U0, eeprom_data, 0x2);
            uint16_t TPIS_cal_U0 = ((uint16_t) eeprom_data[0] << 8) | eeprom_data[1];
            TPIS_cal_U0 += 32768;
            // Read Uout1 calibration value.
            i2c_read_bytes(ADDR_TPIS1385, TP_UOUT1, eeprom_data, 0x2);
            uint32_t TPIS_cal_UOut1 = ((uint16_t) eeprom_data[0] << 8) | eeprom_data[1];
            TPIS_cal_UOut1 *= 2;
            // Read TObj1 calibration value.
            i2c_read_bytes(ADDR_TPIS1385, TP_T_OBJ_1, eeprom_data);
            uint8_t TPIS_cal_TObj1 = eeprom_data[0];
            // Stop reading from EEPROM.
            i2c_write_byte(ADDR_TPIS1385, TP_EEPROM_CONTROL, 0x0);
            // Calculate the calibration constant, K (see: Section 8.4).
            TPIS_cal_K = ((float) (TPIS_cal_UOut1 - TPIS_cal_U0) / (pow((float) (TPIS_cal_TObj1 + 273.15f), 3.8f) - pow(25.0f + 273.15f,3.8f)));
        }
    }
#endif // USING_TMP117

#ifdef USING_SX1272
    // SX1272 initialization and setup.
    // TODO: Manual initialization for the radio transceiver system.
#endif // USING_SX1272
}

int16_3_t acc_data[ACC_DATA_LEN];

// If ACC_DATA_LEN * ACC_AVG_SAMP is <= 255, use uint8_t.
// If ACC_DATA_LEN * ACC_AVG_SAMP is <= 65535, use uint16_t.
uint16_t acc_i = 0; // NOTE: acc_i IS TO BE USED FOR ACCELEROMETER INDEXING PURPOSES ONLY! DO NOT MISUSE.

// If we are currently recording an 'event,' this will be set to the beginning index and end index of the data recorded during the event. New data recorded should not overwrite this.
uint8_t overwrite_deny_start = ACC_DATA_LEN + 1;
uint8_t overwrite_deny_end = ACC_DATA_LEN + 1;
uint8_t accel_event = 0; // 0 = No Event; 1 = Event; 2 = Recalibration

// The threshold where an event is declared and recording begins.
// The threshold is determined by, at startup, 
// (1) filling an entire buffer full of accelerometer data
// (2) determining the average value for all values 0
// (3) multiplying (2) by 1.5
int16_3_t acc_event_threshold = { .x = 0, .y = 0, .z = 0 };;
int16_3_t acc_calib_mean = { .x = 0, .y = 0, .z = 0 };;
int16_3_t acc_calib_max = { .x = -32768, .y = -32768, .z = -32768 };
int16_3_t acc_calib_min = { .x = 32767, .y = 32767, .z = 32767};
int16_3_t curr_val = { .x = 0, .y = 0, .z = 0 };
uint8_t acc_loop_count = 0;

void loop()
{
    // Mark the beginning of the loop.
    // time = millis();

#ifdef USING_SX1272
    // TODO: Manual radio T/RX, probably with an interrupt handler.
#endif // USING_SX1272

#ifdef USING_CAP
    // CAP read.
    int CAP_data = analogRead(PIN_CAP);
#endif // USING_CAP

#ifdef USING_MLX90393
    // MLX read.
    // inplaceof: mlx_sample_t mlx_data = magnetometer.getSample();
    // TODO: Manual MLX90393 magnetometer reading.
#endif // USING_MLX90393

    // MPU6000 accelerometer read.
#ifdef USING_MPU6000
    {      
        // We have looped back to the zeroth element of the circular buffer.
        if ((acc_i/ACC_AVG_SAMP) == 0)
        {
            // Recalibrate every ACC_RECALIBRATE times we fill the array.
            if (acc_loop_count == 0)
            {
                accel_event = 2;
                acc_calib_max = { .x = -32768, .y = -32768, .z = -32768 };
                acc_calib_min = { .x = 32767, .y = 32767, .z = 32767};
            }

            acc_loop_count = (acc_loop_count + 1) % ACC_RECALIBRATE;
        }

        // NOTE: The data coming out of the MPU6000 is in a weird 16-bit format. Performing the (rd_buf[0] << 8 | rd_buf[1]) operation on each X, Y, and Z element converts it into int16_t.

        /////////////////////
        // DATA COLLECTION //
        /////////////////////

        // Declare I2C read/write buffers.
        uint8_t wr_buf[1] = {0};
        uint8_t rd_buf[6] = {0};

        // Request six bytes of accelerometer data.
        wr_buf[0] = MPU6000_ACCEL_OUT; // For some reason we can get all six registers of accel. data (0x3B - 0x40) just by querying the first one.
        i2cbus_transfer(ADDR_MPU6000, wr_buf, 1, rd_buf, 6);

        // Converts the raw data into int16_t, ensures its positive, then rolling-averages it with the data in the current index.
        curr_val.x = (rd_buf[0] << 8 | rd_buf[1]);
        if (curr_val.x < 0) { curr_val.x *= -1; }
#ifdef SERIAL_PRINT_EN
        Serial.print("I_X:");
        Serial.print(curr_val.x);
        Serial.print(" ");
#endif // SERIAL_PRINT_EN
        // Welford's Method: M[k] = M[k-1] + ( ( x[k] - M[k-1] ) / ( k ) )
        acc_data[(acc_i/ACC_AVG_SAMP)].x = acc_data[(acc_i/ACC_AVG_SAMP)].x + ( ( curr_val.x - acc_data[(acc_i/ACC_AVG_SAMP)].x ) / ( (int16_t)acc_i%ACC_AVG_SAMP ) );

        curr_val.y = (rd_buf[2] << 8 | rd_buf[3]);
        if (curr_val.y < 0) { curr_val.y *= -1; }
        // Welford's Method: M[k] = M[k-1] + ( ( x[k] - M[k-1] ) / ( k ) )
        acc_data[(acc_i/ACC_AVG_SAMP)].y = acc_data[(acc_i/ACC_AVG_SAMP)].y + ( ( curr_val.y - acc_data[(acc_i/ACC_AVG_SAMP)].y ) / ( (int16_t)acc_i%ACC_AVG_SAMP ) );

        curr_val.z = (rd_buf[4] << 8 | rd_buf[5]);
        if (curr_val.z < 0) { curr_val.z *= -1; }
        // Welford's Method: M[k] = M[k-1] + ( ( x[k] - M[k-1] ) / ( k ) )
        acc_data[(acc_i/ACC_AVG_SAMP)].z = acc_data[(acc_i/ACC_AVG_SAMP)].z + ( ( curr_val.z - acc_data[(acc_i/ACC_AVG_SAMP)].z ) / ( (int16_t)acc_i%ACC_AVG_SAMP ) );

        ////////////////////
        // EVENT HANDLING //
        ////////////////////

#ifdef SERIAL_PRINT_EN
        Serial.print("Event:");
        Serial.print(3000 + (accel_event * 1000));
        Serial.print(" ");
#endif // SERIAL_PRINT_EN

        // TODO: Investigate: some kind of issue where the event flag stays 'ON' after a permanent orientation shift even once the average has caught up to the new position.

        // If we are not currently in an accelerometer event, then we are SAMPLING data but not RECORDING.
        // However, there exists the possibility that data from a previously RECORDED event has not yet been TRANSMITTED.
        // UNTRANSMITTED RECORDED data is marked by the overwrite_deny_start and overwrite_deny_end bounds.
        // While sampling, we should skip over UNTRANSMITTED RECORDED data.
        if (accel_event == 1)
        { // Currently experiencing an accelerometer event.
            if ((acc_i / ACC_AVG_SAMP) >= overwrite_deny_start)
            {
                // We just reached, while RECORDING, the beginning of the UNTRANSMITTED RECORDED data section.
                // TODO: Determine what we do; do we overwrite old data or cancel the event recording?
            }

            // Event determination. 
            // Event declared inactive if _OFF_NUM-many samples do not exceed the threshold out of the last _CONSIDER_NUM-many samples.

            // If subtracting ACC_EVENT_CONSIDER_NUM from the current index results in <0, make it loop properly.
            uint8_t i = 0;
            if ((acc_i/ACC_AVG_SAMP) < ACC_EVENT_OFF_CONSIDER_NUM)
            {
                i = ACC_DATA_LEN - (ACC_EVENT_OFF_CONSIDER_NUM - (acc_i/ACC_AVG_SAMP));
            }
            else
            {
                i = (acc_i/ACC_AVG_SAMP) - ACC_EVENT_OFF_CONSIDER_NUM;
            }

            uint8_t event_qualifiers = 0;
            for (; i < (acc_i/ACC_AVG_SAMP); i++)
            {
                // Detect threshold violations.
                if (acc_data[i].x < acc_calib_mean.x + acc_event_threshold.x
                 && acc_data[i].y < acc_calib_mean.y + acc_event_threshold.y
                 && acc_data[i].z < acc_calib_mean.z + acc_event_threshold.z
                 && acc_data[i].x > acc_calib_mean.x - acc_event_threshold.x
                 && acc_data[i].y > acc_calib_mean.y - acc_event_threshold.y
                 && acc_data[i].z > acc_calib_mean.z - acc_event_threshold.z)
                {
                    event_qualifiers++;
                }
            }

#ifdef SERIAL_PRINT_EN
            Serial.print("E1_Q:");
            Serial.print(3000 + (event_qualifiers * 10));
            Serial.print(" ");
#endif // SERIAL_PRINT_EN

            if (event_qualifiers >= ACC_EVENT_OFF_NUM)
            {
                accel_event = 0;
            }
        }
        else if (accel_event == 0)
        { // Not currently experiencing an accelerometer event.
            if ((acc_i / ACC_AVG_SAMP) >= overwrite_deny_start)
            {
                // We just reached, while SAMPLING, the beginning of the UNTRANSMITTED RECORDED data section.
                // We should skip over this area and continue sampling.
                acc_i = ((overwrite_deny_end + 1) % ACC_DATA_LEN) * ACC_AVG_SAMP;
            }

            // Event determination.
            // Event declared active if _ON_NUM-many samples exceed the threshold out of the last _CONSIDER_NUM-many samples.
            
            // If subtracting ACC_EVENT_CONSIDER_NUM from the current index results in <0, make it loop properly.
            uint8_t i = 0;
            if ((acc_i/ACC_AVG_SAMP) < ACC_EVENT_ON_CONSIDER_NUM)
            {
                i = ACC_DATA_LEN - (ACC_EVENT_ON_CONSIDER_NUM - (acc_i/ACC_AVG_SAMP));
            }
            else
            {
                i = (acc_i/ACC_AVG_SAMP) - ACC_EVENT_ON_CONSIDER_NUM;
            }

            uint8_t event_qualifiers = 0;
            for (; i < (acc_i/ACC_AVG_SAMP); i++)
            {
                // Detect threshold violations.
                if (acc_data[i].x > acc_calib_mean.x + acc_event_threshold.x
                 || acc_data[i].y > acc_calib_mean.y + acc_event_threshold.y
                 || acc_data[i].z > acc_calib_mean.z + acc_event_threshold.z
                 || acc_data[i].x < acc_calib_mean.x - acc_event_threshold.x
                 || acc_data[i].y < acc_calib_mean.y - acc_event_threshold.y
                 || acc_data[i].z < acc_calib_mean.z - acc_event_threshold.z)
                {
                    event_qualifiers++;
                }
            }

#ifdef SERIAL_PRINT_EN
            Serial.print("E0_Q:");
            Serial.print(3000 + (event_qualifiers * 10));
            Serial.print(" ");
#endif // SERIAL_PRINT_EN

            if (event_qualifiers >= ACC_EVENT_ON_NUM)
            {
                accel_event = 1;
            }
        }
        else if (accel_event == 2) // Lunar Noise Calibration Required
        {   
            if (acc_i/ACC_AVG_SAMP >= ACC_DATA_LEN-1)
            {
                // Once we fill the buffer, set accel_event from CALIBRATE to FALSE,
                // and set the thresholds to our average acceleration magnitude * 1.5.
                // Calculate poor man's standard deviation. std_dev.x = (int16_t)((acc_calib_max.x - acc_calib_min.x) / 4);
                accel_event = 0;
                acc_event_threshold.x = (int16_t)((acc_calib_max.x - acc_calib_min.x) / 2);
                acc_event_threshold.y = (int16_t)((acc_calib_max.y - acc_calib_min.y) / 2);
                acc_event_threshold.z = (int16_t)((acc_calib_max.z - acc_calib_min.z) / 2);
            }
            else if ((acc_i%ACC_AVG_SAMP) == 0)
            { // This ensures that we only add to the array average once the index average has been taken. 
              // acc_data[(acc_i/ACC_AVG_SAMP)-1].x should ensure we take the previously averaged one once we move on.
                // Keep a rolling average of collected data in the last index as we go.
                
                // Welford's Method: M[k] = M[k-1] + ( ( x[k] - M[k-1] ) / ( k ) )
                acc_calib_mean.x = acc_calib_mean.x + ( ( acc_data[(acc_i/ACC_AVG_SAMP)-1].x - acc_calib_mean.x ) / ( (int16_t)acc_i/ACC_AVG_SAMP ) );
                acc_calib_mean.y = acc_calib_mean.y + ( ( acc_data[(acc_i/ACC_AVG_SAMP)-1].y - acc_calib_mean.y ) / ( (int16_t)acc_i/ACC_AVG_SAMP ) );
                acc_calib_mean.z = acc_calib_mean.z + ( ( acc_data[(acc_i/ACC_AVG_SAMP)-1].z - acc_calib_mean.z ) / ( (int16_t)acc_i/ACC_AVG_SAMP ) );

                // Keep track of min/max during calibration.
                // TODO: Maybe when a new max is discovered, set the max to the average of the OldMax and NewMax.
                if (curr_val.x > acc_calib_max.x)
                {
                    acc_calib_max.x = curr_val.x;
                    // acc_calib_max.x = (int16_t)((acc_calib_max.x + curr_val.x) / 2);
                }
                else if (curr_val.x < acc_calib_min.x)
                {
                    acc_calib_min.x = curr_val.x;
                    // acc_calib_min.x = (int16_t)((acc_calib_min.x + curr_val.x) / 2);
                }
                if (curr_val.y > acc_calib_max.y)
                {
                    acc_calib_max.y = curr_val.y;
                }
                else if (curr_val.y < acc_calib_min.y)
                {
                    acc_calib_min.y = curr_val.y;
                }
                if (curr_val.z > acc_calib_max.z)
                {
                    acc_calib_max.z = curr_val.z;
                }
                else if (curr_val.z < acc_calib_min.z)
                {
                    acc_calib_min.z = curr_val.z;
                }
            }
        }

        // OOOOOOO|XXXXXXXX|OOO
        // XXX|OOOOOOOOOOO|XXXX

        uint8_t idx = 0;
        if (acc_i/ACC_AVG_SAMP == 0)
        {
            idx = ACC_DATA_LEN - 1;
        }
        else
        {
            idx = (acc_i/ACC_AVG_SAMP) - 1;
        }

        if (accel_event != 2 && ((acc_i%ACC_AVG_SAMP) == 0))
        {
            // Adjust the calibrated mean to follow the changing trends.
            if (acc_data[idx].x > acc_calib_mean.x)
            {
                acc_calib_mean.x += ((acc_data[idx].x - acc_calib_mean.x) / ((uint16_t)(MAX_FREQUENCY/2))) + 1;
            }
            else if (acc_data[idx].x < acc_calib_mean.x)
            {
                acc_calib_mean.x -= ((acc_calib_mean.x - acc_data[idx].x) / ((uint16_t)(MAX_FREQUENCY/2))) + 1;
            }

            if (acc_data[idx].y > acc_calib_mean.y)
            {
                acc_calib_mean.y += ((acc_data[idx].y - acc_calib_mean.y) / ((uint16_t)(MAX_FREQUENCY/2))) + 1;
            }
            else if (acc_data[idx].y < acc_calib_mean.y)
            {
                acc_calib_mean.y -= ((acc_calib_mean.y - acc_data[idx].y) / ((uint16_t)(MAX_FREQUENCY/2))) + 1;
            }

            if (acc_data[idx].z > acc_calib_mean.z)
            {
                acc_calib_mean.z += ((acc_data[idx].z - acc_calib_mean.z) / ((uint16_t)(MAX_FREQUENCY/2))) + 1;
            }
            else if (acc_data[idx].z < acc_calib_mean.z)
            {
                acc_calib_mean.z -= ((acc_calib_mean.z - acc_data[idx].z) / ((uint16_t)(MAX_FREQUENCY/2))) + 1;
            }
        }

        // acc_i iterates once each loop, but data is only finalized once every ACC_AVG_SAMP loops. So thats why we then have to divide acc_i by ACC_AVG_SAMP all the time.
        acc_i = (acc_i + 1) % (ACC_DATA_LEN * ACC_AVG_SAMP);

#ifdef SERIAL_PRINT_EN
        Serial.print("A_X:");
        Serial.print(acc_data[idx].x);
        Serial.print(" ");

        Serial.print("MEAN:");
        Serial.print(acc_calib_mean.x);
        Serial.print(" ");

        Serial.print("MEAN_P:");
        Serial.print(acc_calib_mean.x + acc_event_threshold.x);
        Serial.print(" ");

        Serial.print("MEAN_M:");
        Serial.print(acc_calib_mean.x - acc_event_threshold.x);
        Serial.print(" ");

        Serial.print("MAX:");
        Serial.print(acc_calib_max.x);
        Serial.print(" ");
        
        Serial.print("MIN:");
        Serial.print(acc_calib_min.x);
        Serial.print(" ");

        Serial.print("\n");
#endif // SERIAL_PRINT_EN
    }
#endif // USING_MPU6000

#ifdef USING_MPU6000
    // MPU6000 gyrometer read.
    {
        // Declare I2C read/write buffers.
        uint8_t wr_buf[1] = {0};
        uint8_t rd_buf[6] = {0};

        // Request six bytes of gyrometer data.
        wr_buf[0] = MPU6000_GYRO_OUT;
        i2cbus_transfer(ADDR_MPU6000, wr_buf, 1, rd_buf, 6);

        // Convert the data to a usable format.
        uint16_t gyro_xyz[3] = {0};
        gyro_xyz[0] = rd_buf[0] << 8 | rd_buf[1];
        gyro_xyz[1] = rd_buf[2] << 8 | rd_buf[3];
        gyro_xyz[2] = rd_buf[4] << 8 | rd_buf[5];
    }
#endif // USING_MPU6000

#ifdef USING_MPU6000
    // MPU6000 temperature read.
    {
        // Declare I2C read / write buffers.
        uint8_t wr_buf[1] = {0};
        uint8_t rd_buf[2] = {0};

        // Request two bytes of thermal data.
        wr_buf[0] = MPU6000_TEMP_OUT;
        i2cbus_transfer(ADDR_MPU6000, wr_buf, 1, rd_buf, 2);

        // Convert the data to a usable format.
        uint16_t temp = rd_buf[0] << 8 | rd_buf[1];
    }
#endif // USING_MPU6000
    
#ifdef USING_TMP117
    // TMP117 thermometer read.
    {
        uint8_t wr_buf[1] = {0};
        uint8_t rd_buf[2] = {0};

        // Read temperature register.
        wr_buf[0] = TMP117_TEMP_REG;
        i2cbus_transfer(ADDR_TMP117, wr_buf, 1, rd_buf, 2);

        // Convert to degrees C.
        float temp = (rd_buf[0] << 8 | rd_buf[1]) * TMP117_RESOLUTION;
    }
#endif // USING_TMP117
    
#ifdef USING_TPIS1385
    // TPile read.
    // inplaceof: TPsample_t temperatures = thermopile.getSample();
    // TODO: Manual thermopile reading.
#endif // USING_TPIS1385

#ifdef USING_SX1272
    // TODO: Figure out transmissions.
#endif // USING_SX1272

    // While this does not ensure a constant cadence, it is the most efficient manner of ensuring the program loop runs at most some-many times per second.
    delay(1000 / MAX_FREQUENCY);
}
