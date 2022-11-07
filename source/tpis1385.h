/**
 * @file tpis1385.h
 * @author Mit Bailey (mitbailey@outlook.com)
 * @brief 
 * @version See Git tags for version information.
 * @date 2022.11.07
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef TPIS1385_H
#define TPIS1385_H

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

#include "i2c.h"

static inline int8_t tpis1385_init()
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
    
    /*
    // inplaceof: thermopile.readEEprom(); // Prints eeprom and updates calibration constants
    {
        uint8_t eeprom_data[2] = {0};

        uint8_t wr_buf[2] = {0};
        uint8_t rd_buf[2] = {0};

        // Set EEPROM control to read.
        wr_buf[0] = TP_EEPROM_CONTROL;
        wr_buf[1] = 0x80;
        i2cbus_write(ADDR_TPIS1385, wr_buf, 2);
        // Read EEPROM protocol.
        i2cbus_read(ADDR_TPIS1385, rd_buf, 2);
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
    */
}

#endif // TPIS1385_H