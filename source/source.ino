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

#include <GLEE_Sensor.h>
#include <TMP117.h>
#include <MPU6000.h>
#include <MLX90393.h>
#include <CAP.h>
#include <TPIS1385.h>
// #include <GLEE_Radio.h>
#include "GLEE_Radio_v2.h"

#include "luna.hpp"

// #define DEBUG_PRINTS

/// SENSOR-SPECIFIC CONSTANTS
#define ADDR_MLX90393 0x0C
#define ADDR_MPU6000 0x00 // UNKOWN
#define ADDR_SX1272 0x00 // UNKNOWN
#define ADDR_TMP117 0x00 // UNKNOWN
#define ADDR_TPIS1385 0x00 // UNKNOWN
#define PIN_CAP A0 // Hardcoded analog pin on LunaSat.

#define MLX90393_AXIS_ALL (0x0E)      /**< X+Y+Z axis bits for commands. */
#define MLX90393_CONF1 (0x00)         /**< Gain */
#define MLX90393_CONF2 (0x01)         /**< Burst, comm mode */
#define MLX90393_CONF3 (0x02)         /**< Oversampling, filter, res. */
#define MLX90393_CONF4 (0x03)         /**< Sensitivty drift. */
#define MLX90393_GAIN_SHIFT (0x04)       /**< Left-shift for gain bits. */
#define MLX90393_HALL_CONF (0x0C)     /**< Hall plate spinning rate adj. */
#define MLX90393_STATUS_OK (0x00)     /**< OK value for status response. */
#define MLX90393_STATUS_SMMODE (0x08) /**< SM Mode status response. */
#define MLX90393_STATUS_RESET (0x01)  /**< Reset value for status response. */
#define MLX90393_STATUS_ERROR (0xFF)  /**< OK value for status response. */
#define MLX90393_STATUS_MASK (0xFC)   /**< Mask for status OK checks. */

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

/// FUNCTION-LIKE MACROS ///
#define WRITE_BYTE(sensor_addr, reg_addr, data) \
    Wire.beginTransmission(sensor_addr); \
    Wire.write(reg_addr); \
    Wire.write(data); \
    Wire.endTransmission();
#define READ_BYTE(sensor_addr, reg_addr, data) \ 
    Wire.beginTransmission(sensor_addr); \
    Wire.write(reg_addr); \
    Wire.endTransmission(); \
    Wire.requestFrom(sensor_addr, 0x1); \
    data[0] = Wire.read();
#define READ_BYTES(sensor_addr, reg_addr, data, nbytes) \ 
    Wire.beginTransmission(sensor_addr); \
    Wire.write(reg_addr); \
    Wire.endTransmission(); \
    Wire.requestFrom(sensor_addr, nbytes); \
    for (uint16_t i = 0; Wire.available() && i < UINT16_MAX; i++) \
        data[i] = Wire.read();

// TEMP
MLX90393 magnetometer = MLX90393(1,false);
MPU6000 accelerometer(1, false); // Sets sensor ID to 1 and debugging to false
// TMP117 thermometer(1,false);
// TPIS1385 thermopile(1);

// Actually important global variables.
float TPIS_cal_K = 0.f; // TPIS calibration constant.
void setup()
{
    Wire.begin();
    Wire.setClock(100000);
    
    // CAP initialization and setup.
    // None required.

    // MLX90393 initialization and setup.  
    {
        // magnetometer.begin_I2C(); // Pointless
#define READ_REGISTER16()
        magnetometer.setGain(MLX90393_GAIN_2_5X);  
        magnetometer.setResolution(MLX90393_X, MLX90393_RES_19);
        magnetometer.setResolution(MLX90393_Y, MLX90393_RES_19);
        magnetometer.setResolution(MLX90393_Z, MLX90393_RES_16);
        magnetometer.setOversampling(MLX90393_OSR_2);
        magnetometer.setFilter(MLX90393_FILTER_6);
        magnetometer.setTrigInt(false);
    }

    // MPU6000 initialization and setup.
    {    
        // accelerometer.begin(); // Pointless
        // accelerometer.initialize();
        // Reset all internal registers to defaults.
        WRITE_BYTE(ADDR_MPU6000, MPU6000_PWR_MGMT_1, 0b10000000);
        uint8_t data[1] = {0};
        do
        {
            READ_BYTE(ADDR_MPU6000, MPU6000_PWR_MGMT_1, data);
            delay(1);
        } while (data[0] == 0b10000000);
        delay(100);
        WRITE_BYTE(ADDR_MPU6000, MPU6000_SIGNAL_PATH_RESET, 0b00000111);
        delay(100);
        // Set sample rate divisor.
        WRITE_BYTE(ADDR_MPU6000, MPU6000_SMPLRT_DIV, 0x0);
        // Set filter bandwidth.
        WRITE_BYTE(ADDR_MPU6000, MPU6000_CONFIG, MPU6000_BAND_260_HZ);
        // Set acceleration range to 2 Gs.
        WRITE_BYTE(ADDR_MPU6000, MPU6000_ACCEL_CONFIG, 0x0);
        // IDK what this does but its in the initializer.
        WRITE_BYTE(ADDR_MPU6000, MPU6000_PWR_MGMT_1, 0x1);
        // accelerometer.setAccelRange(MPU6000_RANGE_2_G); // Redundant, already called in the initializer.
    }


    // TMP117 initialization and setup.
    // None required.

    // TPIS1385 initialization and setup.
    {
        // thermopile.begin(); // Thermopile start-up
        Wire.begin(); // Begin i2c coms at standard speed
        Wire.beginTransmission(0x00); // Reload all call   
        Wire.write(0x04);
        Wire.write(0x00);         
        if(Wire.endTransmission() != 0) 
            Serial.println(F("Init call failiure"));
        delay(50);  // Wait on i2c transmission
        
        // thermopile.readEEprom(); // Prints eeprom and updates calibration constants
        uint8_t eeprom_data[2] = {0};
        // Set EEPROM control to read.
        WRITE_BYTE(ADDR_TPIS1385, TP_EEPROM_CONTROL, 0x80);
        // Read EEPROM protocol.
        READ_BYTE(ADDR_TPIS1385, TP_PROTOCOL, eeprom_data);
        // Read PTAT25 calibration value.
        READ_BYTES(ADDR_TPIS1385, TP_PTAT25, eeprom_data, 0x2);
        uint16_t TPIS_cal_PTAT25 = ((uint16_t) eeprom_data[0] << 8) | eeprom_data[1]; 
        // Read M calibration value.
        READ_BYTES(ADDR_TPIS1385, TP_M, eeprom_data, 0x2);
        uint16_t TPIS_cal_M = ((uint16_t) eeprom_data[0] << 8) | eeprom_data[1]; 
        TPIS_cal_M /= 100; // Apply appropriate offset
        // Read U0 calibration value.
        READ_BYTES(ADDR_TPIS1385, TP_U0, eeprom_data, 0x2);
        uint16_t TPIS_cal_U0 = ((uint16_t) eeprom_data[0] << 8) | eeprom_data[1];
        TPIS_cal_U0 += 32768;
        // Read Uout1 calibration value.
        READ_BYTES(ADDR_TPIS1385, TP_UOUT1, eeprom_data, 0x2);
        uint32_t TPIS_cal_UOut1 = ((uint16_t) eeprom_data[0] << 8) | eeprom_data[1];
        TPIS_cal_UOut1 *= 2;
        // Read TObj1 calibration value.
        READ_BYTE(ADDR_TPIS1385, TP_T_OBJ_1, eeprom_data);
        uint8_t TPIS_cal_TObj1 = eeprom_data[0];
        // Stop reading from EEPROM.
        WRITE_BYTE(ADDR_TPIS1385, TP_EEPROM_CONTROL, 0x0);
        // Calculate the calibration constant, K (see: Section 8.4).
        TPIS_cal_K = ((float) (TPIS_cal_UOut1 - TPIS_cal_U0) / (pow((float) (TPIS_cal_TObj1 + 273.15f), 3.8f) - pow(25.0f + 273.15f,3.8f)));
    }

    // SX1272 initialization and setup.
    // TODO: 
}

void loop()
{
    // CAP read.
    int CAP_data = analogRead(PIN_CAP);
    mlx_sample_t mlx_data = magnetometer.getSample();
    sensor_float_vec_t acc_data = accelerometer.getSample();
    float temp_data = thermometer.getTemperatureC();
    TPsample_t temperatures = thermopile.getSample();
}