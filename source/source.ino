/**
 * @file source.ino
 * @author Mit Bailey (mitbailey99@gmail.com)
 * @brief
 * @version See Git tags for version information.
 * @date 2022.03.25
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <Wire.h>

// #define SERIAL_DEBUG_PRINT_ENABLE // Allows sdbprintlf(...) & sdbprintf(...) printouts if defined.

#include "meb_print_serial.h"

// #include <GLEE_Sensor.h>
// #include <TMP117.h>
// #include <MPU6000.h>
// #include <MLX90393.h>
// #include <CAP.h>
// #include <TPIS1385.h>
// // #include <GLEE_Radio.h>
// #include "GLEE_Radio_v2.h"

// #include "luna.hpp"

/// SENSOR-SPECIFIC CONSTANTS
#define ADDR_MLX90393 0x0C
#define ADDR_MPU6000 0x68
#define ADDR_MPU6000_2 0x69
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
#define MLX90393_FILTER_6 0x0
#define MLX90393_FILTER_7 0x1

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
// #define MPU6000_RANGE_4_G 0x8
// #define MPU6000_RANGE_8_G 0x10
// #define MPU6000_RANGE_16_G 0x18

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

#ifdef SERIAL_DEBUG_PRINT_ENABLE
    Serial.begin(9600);
#endif // SERIAL_DEBUG_PRINT_ENABLE

    Wire.begin();
    Wire.setClock(100000);
    
    // CAP initialization and setup.
    // None required.

    // MLX90393 initialization and setup.  
    {
        // magnetometer.begin_I2C(); // Pointless
        // inplaceof: magnetometer.setGain(MLX90393_GAIN_2_5X);
        {
            uint8_t data[3] = {0}; // [0] is High byte, [1] is Low byte. (HHHH HHHH LLLL LLLL)

            data[0] = (MLX90393_CONF1 << 0x2);
            i2c_write_bytes(ADDR_MLX90393, MLX90393_REG_RR, data, 0x1); // Pokes the read register asking for data from CONF1.
            memset(data, 0x0, sizeof(data));
            
            i2c_read_bytes_passive(ADDR_MLX90393, data, 2); // Actually grab the two bytes.
            delay(15); // Wait 15ms after transactions as per the datasheet.

            // uint16_t data16 = ((uint16_t)data[0] << 8 | data[1]);
            // data16 &= ~0x0070;
            // data16 &= 0b10001111;

            // Mask off gain bits.
            // [0] is High byte, [1] is Low byte.
            // data[0] &= 0b11111111;
            data[1] &= 0b10001111;
            
            // Set gain bits.
            // data16 |= (0x07 << MLX90393_GAIN_SHIFT);
            // data16 |= (0b01110000);
            // data[0] |= 0b00000000;
            data[1] |= 0b01110000;

            data[2] = (MLX90393_CONF1 << 0x2);
            i2c_write_bytes(ADDR_MLX90393, MLX90393_REG_WR, data, 0x2);
            delay(15);
            // Read status byte.
            uint8_t status_buffer[2] = {0};
            i2c_read_bytes_passive(ADDR_MLX90393, status_buffer, 0x2);
        }

        // inplaceof: magnetometer.setResolution(MLX90393_X, MLX90393_RES_19);
        {
            // uint16_t data;
            uint8_t data[3] = {0};
            // readRegister(MLX90393_CONF3, &data);
            data[0] = (MLX90393_CONF3 << 0x2);
            i2c_write_bytes(ADDR_MLX90393, MLX90393_REG_RR, data, 0x1); // Pokes the read register asking for data from CONF1.
            memset(data, 0x0, sizeof(data));

            i2c_read_bytes_passive(ADDR_MLX90393, data, 0x2); // Reads the data.
            delay(15);

            // [0] is High byte, [1] is Low byte.
            // data[0] &= 0b11111111;
            data[1] &= 0b10011111;

            // data[0] |= (MLX90393_RES_19 << 5) >> 8;
            data[1] |= MLX90393_RES_19 << 5;

            data[2] = (MLX90393_CONF3 << 2);
            i2c_write_bytes(ADDR_MLX90393, MLX90393_REG_WR, data, 0x3);
            delay(15);

            // Read status byte.
            uint8_t status_buffer[2] = {0};
            i2c_read_bytes_passive(ADDR_MLX90393, status_buffer, 2);
        }

        // inplaceof: magnetometer.setResolution(MLX90393_Y, MLX90393_RES_19);
        {
            uint8_t data[3] = {0};

            data[0] = (MLX90393_CONF3 << 0x2);
            i2c_write_bytes(ADDR_MLX90393, MLX90393_REG_RR, data, 0x1); // Pokes the read register asking for data from CONF1.
            memset(data, 0x0, sizeof(data));

            i2c_read_bytes_passive(ADDR_MLX90393, data, 0x2); // Reads the data.
            delay(15);

            // [0] is High byte, [1] is Low byte.
            data[0] &= 0b11111110;
            data[1] &= 0b01111111;

            // data[0] |= (MLX90393_RES_19 << 7) >> 8;
            data[1] |= MLX90393_RES_19 << 7;

            data[2] = (MLX90393_CONF3 << 2);
            i2c_write_bytes(ADDR_MLX90393, MLX90393_REG_WR, data, 0x3);
            delay(15);

            uint8_t status_buffer[2] = {0};
            i2c_read_bytes_passive(ADDR_MLX90393, status_buffer, 2);
        }

        // inplaceof: magnetometer.setResolution(MLX90393_Z, MLX90393_RES_16);
        {
            uint8_t data[3] = {0};

            data[0] = (MLX90393_CONF3 << 0x2);
            i2c_write_bytes(ADDR_MLX90393, MLX90393_REG_RR, data, 0x1); // Pokes the read register asking for data from CONF1.
            memset(data, 0x0, sizeof(data));

            i2c_read_bytes_passive(ADDR_MLX90393, data, 0x2); // Reads the data.
            delay(15);

            // [0] is High byte, [1] is Low byte.
            data[0] &= 0b11111001;
            // data[1] &= 0b11111111;

            // data[0] |= (MLX90393_RES_19 << 9) >> 8; aka:
            data[0] |= (MLX90393_RES_19 << 1);
            data[1] |= MLX90393_RES_19 << 9;

            data[2] = (MLX90393_CONF3 << 2);
            i2c_write_bytes(ADDR_MLX90393, MLX90393_REG_WR, data, 0x3);
            delay(15);

            uint8_t status_buffer[2] = {0};
            i2c_read_bytes_passive(ADDR_MLX90393, status_buffer, 2);
        }

        // inplaceof: magnetometer.setOversampling(MLX90393_OSR_2);
        {
            uint8_t data[3] = {0};

            data[0] = (MLX90393_CONF3 << 0x2);
            i2c_write_bytes(ADDR_MLX90393, MLX90393_REG_RR, data, 0x1); // Pokes the read register asking for data from CONF1.
            memset(data, 0x0, sizeof(data));

            i2c_read_bytes_passive(ADDR_MLX90393, data, 0x2); // Reads the data.
            delay(15);

            // [0] is High byte, [1] is Low byte.
            // data[0] &= 0b11111111;
            data[1] &= 0b11111100;

            // data[0] |= MLX90393_OSR_2;
            data[1] |= MLX90393_OSR_2;

            data[2] = (MLX90393_CONF3 << 2);
            i2c_write_bytes(ADDR_MLX90393, MLX90393_REG_WR, data, 0x3);
            delay(15);

            uint8_t status_buffer[2] = {0};
            i2c_read_bytes_passive(ADDR_MLX90393, status_buffer, 2);
        }

        // inplaceof: magnetometer.setFilter(MLX90393_FILTER_6);
        {
            uint8_t data[3] = {0};

            data[0] = (MLX90393_CONF3 << 0x2);
            i2c_write_bytes(ADDR_MLX90393, MLX90393_REG_RR, data, 0x1); // Pokes the read register asking for data from CONF1.
            memset(data, 0x0, sizeof(data));

            i2c_read_bytes_passive(ADDR_MLX90393, data, 0x2); // Reads the data.
            delay(15);

            // [0] is High byte, [1] is Low byte.
            // data[0] &= 0b11111111;
            data[1] &= 0b11100011;

            // data[0] |= ((MLX90393_FILTER_6 + 0x6) << 2);
            data[1] |= ((MLX90393_FILTER_6 + 0x6) << 2);

            data[2] = (MLX90393_CONF3 << 2);
            i2c_write_bytes(ADDR_MLX90393, MLX90393_REG_WR, data, 0x3);
            delay(15);

            uint8_t status_buffer[2] = {0};
            i2c_read_bytes_passive(ADDR_MLX90393, status_buffer, 2);
        }

        // inplaceof: magnetometer.setTrigInt(false);
        {
            uint8_t data[3] = {0};

            data[0] = (MLX90393_CONF2 << 0x2);
            i2c_write_bytes(ADDR_MLX90393, MLX90393_REG_RR, data, 0x1); // Pokes the read register asking for data from CONF2.
            memset(data, 0x0, sizeof(data));

            i2c_read_bytes_passive(ADDR_MLX90393, data, 0x2); // Reads the data.
            delay(15);

            // [0] is High byte, [1] is Low byte.
            data[0] &= 0b01111111;
            // data[1] &= 0b11111111;

            // ONLY IF setTrigInt(true)
            // data[0] |= 0b10000000;
            // data[1] |= 0b00000000; // Never uncomment, just here for clarity.

            data[2] = (MLX90393_CONF2 << 2);
            i2c_write_bytes(ADDR_MLX90393, MLX90393_REG_WR, data, 0x3);
            delay(15);

            uint8_t status_buffer[2] = {0};
            i2c_read_bytes_passive(ADDR_MLX90393, status_buffer, 2);
        }
    }

    // MPU6000 initialization and setup.
    {    
        // accelerometer.begin(); // Pointless
        // inplaceof: accelerometer.initialize();
        {
            // Reset all internal registers to defaults.
            i2c_write_byte(ADDR_MPU6000, MPU6000_PWR_MGMT_1, 0b10000000);
            uint8_t data[1] = {0};
            do
            {
                i2c_read_bytes(ADDR_MPU6000, MPU6000_PWR_MGMT_1, data);
                delay(1);
            } while (data[0] == 0b10000000);
            delay(100);
            i2c_write_byte(ADDR_MPU6000, MPU6000_SIGNAL_PATH_RESET, 0b00000111);
            delay(100);
            // Set sample rate divisor.
            i2c_write_byte(ADDR_MPU6000, MPU6000_SMPLRT_DIV, 0x0);
            // Set filter bandwidth.
            i2c_write_byte(ADDR_MPU6000, MPU6000_CONFIG, MPU6000_BAND_260_HZ);
            // Set acceleration range to 2 Gs.
            i2c_write_byte(ADDR_MPU6000, MPU6000_ACCEL_CONFIG, MPU6000_RANGE_2_G); // 0x0 = 2G; 0x8 = 4G; 0x10 = 8G; 0x18 = 16G
            MPU6000_accel_scale = 16384; // 16384 = 2G; 8192 = 4G; 4096 = 8G; 2048 = 16G
            // IDK what this does but its in the initializer.
            i2c_write_byte(ADDR_MPU6000, MPU6000_PWR_MGMT_1, 0x1);
        }
        // accelerometer.setAccelRange(MPU6000_RANGE_2_G); // Redundant, already called in the initializer.
    }

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

    // SX1272 initialization and setup.
    // TODO: Manual initialization for the radio transceiver system.
}

void loop()
{
    // TODO: Manual radio T/RX.

    // CAP read.
    int CAP_data = analogRead(PIN_CAP);

    // MLX read.
    // inplaceof: mlx_sample_t mlx_data = magnetometer.getSample();
    // TODO: Manual MLX90393 magnetometer reading.
    
    // MPU6000 Accel read.
    // inplaceof: sensor_float_vec_t acc_data = accelerometer.getSample();
    {
        uint8_t rd_buf[6] = {0};
        // Reads the accelerometer data in LSB/g
        i2c_read_bytes(ADDR_MPU6000, MPU6000_ACCEL_OUT, rd_buf, 0x6);
        uint16_t acc_xyz[3] = {0};
        acc_xyz[0] = rd_buf[0] << 8 | rd_buf[1];
        acc_xyz[1] = rd_buf[0] << 8 | rd_buf[1];
        acc_xyz[2] = rd_buf[0] << 8 | rd_buf[1];
    }

    // MPU6000 Gyro read.
    {
        uint8_t rd_buf[6] = {0};
        // Reads the accelerometer data in LSB/g
        i2c_read_bytes(ADDR_MPU6000, MPU6000_GYRO_OUT, rd_buf, 0x6);
        uint16_t gyro_xyz[3] = {0};
        gyro_xyz[0] = rd_buf[0] << 8 | rd_buf[1];
        gyro_xyz[1] = rd_buf[0] << 8 | rd_buf[1];
        gyro_xyz[2] = rd_buf[0] << 8 | rd_buf[1];
    }
    
    // Thermo read.
    // inplaceof: float temp_data = thermometer.getTemperatureC();
    {
        uint8_t rd_buf[2] = {0};
        i2c_read_bytes(ADDR_TMP117, TMP117_TEMP_REG, rd_buf, 2);
        uint16_t temperature = ((rd_buf[0] << 8) | rd_buf[1]); // Swaps MSB and LSB, casts to uint16_t.
        float temp_degC = temperature * TMP117_RESOLUTION; // Converts integer value to degC.
    }
    
    // TPile read.
    // inplaceof: TPsample_t temperatures = thermopile.getSample();
    // TODO: Manual thermopile reading.
}