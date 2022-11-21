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

#define SERIAL_PRINT_EN // Enables printouts.

#define FREQUENCY ((uint8_t)(60)) // Accurate to +- millisecond; guaranteed to not run faster.

/// SENSOR USAGE CONDITIONALS
// #define USING_MLX90393
#define USING_MPU6000
// #define USING_SX1272
// #define USING_TMP117
// #define USING_TPIS1385
// #define USING_CAP

/// ACCELEROMETER LOOP VALUES
#define ACC_DATA_LEN 128 // Accelerometer data array size: used during an event. RecordingTime = (ACC_DATA_LEN * ACC_AVG_SAMP) / (FREQUENCY)
#define ACC_PEEK_DATA_LEN 32 // Accelerometer peek array size: used when there is no event.
#define ACC_AVG_SAMP ((uint8_t)(3)) // Each element of the data and peek arrays will be this many averaged samples.
#define ACC_EVENT_OFF_NUM 20 // How many consecutive elements in the data array must be below threshold to constitute the end of an event.
#define ACC_EVENT_BACKLOG 10 // The number of samples we copy from the peek array to the data array on declaration of an event.
#define ACC_RECALIBRATE ((uint8_t)(32)) // The accelerometer standard deviation is recalibrated every this many times the peek array is filled.

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

// Machine States
#define STATE_POST      0 // Power On Self Test
#define STATE_RECAL     1 // Recalibration
#define STATE_RECAL_1   11 // Recalibration
#define STATE_NO_EVENT  2 // No Seismic Event
#define STATE_EVENT     3 // Seismic Event
#define STATE_COMMAND   4 // External Command Execution

#include <Wire.h>
#include <RadioLib.h>
#include "i2c.h"
#include "mlx90393.h"
#include "mpu6000.h"
#include "tmp117.h"
#include "tpis1385.h"
#include "sx1272.h"
#include "utilities.h"

// Actually important global variables.
float TPIS_cal_K = 0.f; // TPIS calibration constant.
uint16_t MPU6000_accel_scale = 0;

///////////
// SETUP //
///////////

SX1272 radio = NULL;

void setup()
{
    // NOTE: Setup sequences taken from the basic setup examples found in: 
    // github.com/GLEE2023/GLEE2023/examples/Sensor_Examples

#ifdef SERIAL_PRINT_EN
    Serial.begin(9600);
#endif // SERIAL_PRINT_EN

    Wire.begin();
    Wire.setClock(100000);
    
    // CAP initialization and setup.
    // None required.

#ifdef USING_MLX90393
    // MLX90393 initialization and setup.  
    mlx90393_init();
#endif // USING_MLX90393

#ifdef USING_MPU6000
    // MPU6000 initialization and setup.
    mpu6000_init();
#endif // USING_MPU6000

#ifdef USING_TMP117
    // TMP117 initialization and setup.
    // None required.
#endif // USING_TMP117

#ifdef USING_TPIS1385
    // TPIS1385 initialization and setup.
    tpis1385_init();
#endif // USING_TPIS1385

#ifdef USING_SX1272
    // SX1272 initialization and setup.
    radio = sx1272_init()
#endif // USING_SX1272
}

////////////////////
// LOOP VARIABLES //
////////////////////

int16_t acc_data[ACC_DATA_LEN];
int16_t acc_peek_data[ACC_PEEK_DATA_LEN];

// If ACC_DATA_LEN * ACC_AVG_SAMP is <= 255, use uint8_t.
// If ACC_DATA_LEN * ACC_AVG_SAMP is <= 65535, use uint16_t.
uint16_t acc_i = 0; // NOTE: acc_i IS TO BE USED FOR ACCELEROMETER INDEXING PURPOSES ONLY! DO NOT MISUSE.
uint16_t acc_peek_i = 0;
uint16_t last_rec_end_i = 0;

// If we are currently recording an 'event,' this will be set to the beginning index and end index of the data recorded during the event. New data recorded should not overwrite this.
uint8_t overwrite_deny_start = ACC_DATA_LEN + 1;
uint8_t overwrite_deny_end = ACC_DATA_LEN + 1;

uint8_t machine_state = STATE_RECAL;

// The threshold where an event is declared and recording begins.
// The threshold is determined by, at startup, 
// (1) filling an entire buffer full of accelerometer data
// (2) determining the average value for all values 0
// (3) multiplying (2) by 1.5
int16_t acc_event_threshold = 0;
int16_t acc_calib_mean = 0;
int16_t acc_calib_max = 0;
int16_t acc_calib_min = 0;
uint8_t acc_loop_count = 0;
int16_t magnitude = 0;

uint8_t last_peek_idx = 0;
uint8_t last_idx = 0;

unsigned long time = 0;
unsigned long time_2 = 0;
int global_violations = 0;
uint16_t since_last_violation = 0;

//////////
// LOOP //
//////////

void loop()
{
    time = millis();
    time += (1000/FREQUENCY);

    ////////////////////////////
    /// TRANSCEIVER (SX1272) ///
    ////////////////////////////

#ifdef USING_SX1272
    // TODO: Manual radio T/RX, probably with an interrupt handler.
    
#endif // USING_SX1272

    ///////////////////////////////
    /// CAPACITIVE SENSOR (CAP) ///
    ///////////////////////////////

#ifdef USING_CAP
    // CAP read.
    int CAP_data = analogRead(PIN_CAP);
#endif // USING_CAP

    ///////////////////////////////
    /// MAGNETOMETER (MLX90393) ///
    ///////////////////////////////

#ifdef USING_MLX90393
    // MLX read.
    // inplaceof: mlx_sample_t mlx_data = magnetometer.getSample();
    // TODO: Manual MLX90393 magnetometer reading.
#endif // USING_MLX90393

    ///////////////////////////////
    /// ACCELEROMETER (MPU6000) ///
    ///////////////////////////////

    // MPU6000 accelerometer read.
#ifdef USING_MPU6000
    {      
        // NOTE: The data coming out of the MPU6000 is in a weird 16-bit format. Performing the (rd_buf[0] << 8 | rd_buf[1]) operation on each X, Y, and Z element converts it into int16_t.

        magnitude = mpu6000_sample_magnitude();

#ifdef SERIAL_PRINT_EN
        Serial.print("MAGNITUDE:");
        Serial.print(magnitude);
        Serial.print(" ");
#endif // SERIAL_PRINT_EN
        
        //////////////////////
        // SET LAST INDICES //
        //////////////////////

        if (acc_peek_i/ACC_AVG_SAMP == 0)
        {
            last_peek_idx = ACC_PEEK_DATA_LEN - 1;
        }
        else
        {
            last_peek_idx = (acc_peek_i/ACC_AVG_SAMP) - 1;
        }

        if (acc_i/ACC_AVG_SAMP == 0)
        {
            last_idx = ACC_PEEK_DATA_LEN - 1;
        }
        else
        {
            last_idx = (acc_i/ACC_AVG_SAMP) - 1;
        }
        
        ////////////////////
        // EVENT HANDLING //
        ////////////////////

        switch (machine_state)
        {
            case STATE_NO_EVENT: // Not currently in an accelerometer event nor recalibrating.
            {
                //////////////////
                // DATA LOGGING //
                //////////////////

                // Welford's Method: M[k] = M[k-1] + ( ( x[k] - M[k-1] ) / ( k ) )
                acc_peek_data[(acc_peek_i/ACC_AVG_SAMP)] += ( ( magnitude - acc_peek_data[(acc_peek_i/ACC_AVG_SAMP)] ) / ( (int16_t)acc_peek_i%ACC_AVG_SAMP ) );

                /////////////////////////
                // EVENT DETERMINATION //
                /////////////////////////

                // Determines if an event is detected. If not, see if we need to recalibrate.
                // We have looped back to the zeroth element of the peek circular buffer.
                if (magnitude > acc_calib_mean + acc_event_threshold
                 || magnitude < acc_calib_mean - acc_event_threshold)
                {
                    machine_state = STATE_EVENT;
                }
                else if ((acc_peek_i/ACC_AVG_SAMP) == 0)
                {
                    // Recalibrate every ACC_RECALIBRATE times we've looped back.
                    if (acc_loop_count == 0)
                    {
                        machine_state = STATE_RECAL;
                    }

                    acc_loop_count = (acc_loop_count + 1) % ACC_RECALIBRATE;
                }

                ///////////////
                // ITERATION //
                ///////////////

                acc_peek_i = (acc_peek_i + 1) % (ACC_PEEK_DATA_LEN * ACC_AVG_SAMP);
                break;
            }
            case STATE_EVENT: // Currently experiencing an accelerometer event.
            {
                //////////////////
                // DATA LOGGING //
                //////////////////

                // Welford's Method: M[k] = M[k-1] + ( ( x[k] - M[k-1] ) / ( k ) )
                acc_data[(acc_i/ACC_AVG_SAMP)] += ( ( magnitude - acc_data[(acc_i/ACC_AVG_SAMP)] ) / ( (int16_t)acc_i%ACC_AVG_SAMP ) );
                
                /////////////////////////
                // EVENT DETERMINATION //
                /////////////////////////

                if ((acc_i%ACC_AVG_SAMP) == 0) 
                {
                    if (acc_data[last_idx] > acc_calib_mean + acc_event_threshold
                    || acc_data[last_idx] < acc_calib_mean - acc_event_threshold)
                    {
                        Serial.println("Violation detected!");
                        since_last_violation = 0;
                    }
                    else
                    {
                        since_last_violation++;
                    }
                }

                if (since_last_violation > ACC_EVENT_OFF_NUM)
                {
                    since_last_violation = 0;
                    last_rec_end_i = (acc_i/ACC_AVG_SAMP);
                    machine_state = STATE_NO_EVENT;
                    // acc_peek_i = 0;
                }

                ///////////////
                // ITERATION //
                ///////////////

                // acc_i iterates once each loop, but data is only finalized once every ACC_AVG_SAMP loops. So thats why we then have to divide acc_i by ACC_AVG_SAMP all the time.
                acc_i = (acc_i + 1) % (ACC_DATA_LEN * ACC_AVG_SAMP);
                break;
            }
            case STATE_RECAL:
            {
                acc_calib_max = -32768;
                acc_calib_min = 32767;
                machine_state = STATE_RECAL_1;
            }
            case STATE_RECAL_1: // (Re-)calibrating the standard deviation.
            {
                //////////////////
                // DATA LOGGING //
                //////////////////

                // Welford's Method: M[k] = M[k-1] + ( ( x[k] - M[k-1] ) / ( k ) )
                acc_peek_data[(acc_peek_i/ACC_AVG_SAMP)] += ( ( magnitude - acc_peek_data[(acc_peek_i/ACC_AVG_SAMP)] ) / ( (int16_t)acc_peek_i%ACC_AVG_SAMP ) );

                /////////////////
                // CALIBRATION //
                /////////////////

                if (acc_peek_i/ACC_AVG_SAMP >= ACC_PEEK_DATA_LEN-1)
                {
                    // Once we fill the buffer, set accel_event from CALIBRATE to FALSE,
                    // and set the thresholds to our average acceleration magnitude * 1.5.
                    // Calculate poor man's standard deviation. std_dev.x = (int16_t)((acc_calib_max.x - acc_calib_min.x) / 4);
                    machine_state = STATE_NO_EVENT;
                    acc_event_threshold = (int16_t)((acc_calib_max - acc_calib_min) / 2);
                }
                else if ((acc_peek_i%ACC_AVG_SAMP) == 0)
                { // This ensures that we only add to the array average once the index average has been taken. 
                // acc_data[(acc_i/ACC_AVG_SAMP)-1].x should ensure we take the previously averaged one once we move on.
                    // Keep a rolling average of collected data in the last index as we go.
                    
                    // Welford's Method: M[k] = M[k-1] + ( ( x[k] - M[k-1] ) / ( k ) )
                    acc_calib_mean += ( ( acc_peek_data[(acc_peek_i/ACC_AVG_SAMP)-1] - acc_calib_mean ) / ( (int16_t)acc_peek_i/ACC_AVG_SAMP ) );

                    // Serial.print("acc_calib_mean ");
                    // Serial.println(acc_calib_mean);

                    // Keep track of min/max during calibration.
                    if (magnitude > acc_calib_max)
                    {
                        acc_calib_max = magnitude;
                    }
                    else if (magnitude < acc_calib_min)
                    {
                        acc_calib_min = magnitude;
                    }
                }

                ///////////////
                // ITERATION //
                ///////////////

                acc_peek_i = (acc_peek_i + 1) % (ACC_PEEK_DATA_LEN * ACC_AVG_SAMP);
                break;
            }

            ////////////////////////////////
            // CALIBRATED MEAN ADJUSTMENT //
            ////////////////////////////////

            if ((acc_peek_i%ACC_AVG_SAMP) == 0)
            {
                // Adjust the calibrated mean to follow the changing trends.
                if (acc_peek_data[last_peek_idx] > acc_calib_mean)
                {
                    acc_calib_mean += ((acc_peek_data[last_peek_idx] - acc_calib_mean) / ((uint16_t)(FREQUENCY/2))) + 1;
                }
                else if (acc_peek_data[last_peek_idx] < acc_calib_mean)
                {
                    acc_calib_mean -= ((acc_calib_mean - acc_peek_data[last_peek_idx]) / ((uint16_t)(FREQUENCY/2))) + 1;
                }
            }
        }

#ifdef SERIAL_PRINT_EN
        Serial.print(F("acc_i:"));
        Serial.print(((acc_i)+7600));
        Serial.print(F(" "));

        Serial.print(F("acc_peek_i:"));
        Serial.print(((acc_peek_i)*5)+7600);
        Serial.print(F(" "));

        Serial.print(F("SinceLastViolation:"));
        Serial.print(((since_last_violation)*10)+7600);
        Serial.print(F(" "));

        Serial.print(F("FState:"));
        Serial.print((machine_state * 200)+7600);
        Serial.print(F(" "));

        Serial.print(F("Last-acc_peek_data:"));
        Serial.print(acc_peek_data[last_peek_idx]);
        Serial.print(F(" "));

        // Serial.print(F("Last-acc_data:"));
        // Serial.print(acc_data[last_idx]);
        // Serial.print(F(" "));

        Serial.print(F("MEAN:"));
        Serial.print(acc_calib_mean);
        Serial.print(F(" "));

        Serial.print(F("MEAN_P:"));
        Serial.print(acc_calib_mean + acc_event_threshold);
        Serial.print(F(" "));

        Serial.print(F("MEAN_M:"));
        Serial.print(acc_calib_mean - acc_event_threshold);
        Serial.print(F(" "));

        // if (acc_calib_max > 5000)
        // {
        //     Serial.print(F("MAX:"));
        //     Serial.print(acc_calib_max);
        //     Serial.print(F(" "));
        // }
        
        // if (acc_calib_min < 10000)
        // {
        //     Serial.print(F("MIN:"));
        //     Serial.print(acc_calib_min);
        //     Serial.print(F(" "));
        // }

        Serial.print(F("\n"));
#endif // SERIAL_PRINT_EN
    }
#endif // USING_MPU6000

    ///////////////////////////
    /// GYROMETER (MPU6000) ///
    ///////////////////////////

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

    ////////////////////////////////////
    /// TEMPERATURE SENSOR (MPU6000) ///
    ////////////////////////////////////

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
    
    ///////////////////////////////////
    /// TEMPERATURE SENSOR (TMP117) ///
    ///////////////////////////////////

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
    
    /////////////////////////////
    /// THERMOPILE (TPIS1385) ///
    /////////////////////////////

#ifdef USING_TPIS1385
    // TPile read.
    // inplaceof: TPsample_t temperatures = thermopile.getSample();
    // TODO: Manual thermopile reading.
#endif // USING_TPIS1385

    ////////////////////////////
    /// TRANSCEIVER (SX1272) ///
    ////////////////////////////

#ifdef USING_SX1272
    // TODO: Figure out transmissions.
#endif // USING_SX1272

    time_2 = millis();
    if (time_2 < time)
    {
        delay(time - time_2);
    }
}
