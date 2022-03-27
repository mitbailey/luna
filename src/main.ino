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

TMP117 *thermo = nullptr;
MPU6000 *accel = nullptr;
MLX90393 *magne = nullptr;
CAP *cap = nullptr;
TPIS1385 *pile = nullptr;
// LunaRadio *radio = nullptr;
LunaRadio_v2 *radio = nullptr;

// Standard Arduino initialization function.
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

#ifdef DEBUG_PRINTS
    Serial.print("I2C and Serial initialization complete.\n");
#endif

    I2C_scan(false);

    // Thermopile Initialization
    Wire.beginTransmission(0x00); // TX buffer.
    Wire.write(0x04);             // Add register address to TX buffer.
    Wire.write(0x00);             // Add initialization data to TX.
    Wire.endTransmission();       // Send.

    delay(50);

#ifdef DEBUG_PRINTS
    Serial.print("Thermopile init command sent.\n"); // Note that we havent actually checked if its working.
#endif

    // Sensors Initialization
    thermo = new TMP117(1, false);
    delay(100);
#ifdef DEBUG_PRINTS
    if (!thermo->isConnected())
        Serial.println("TMP117 init failure.");
    else
        Serial.println("TMP117 init success.");
#endif

    accel = new MPU6000(1, false);
    accel->begin();
    delay(100);
    accel->initialize();
    delay(100);
    accel->setAccelRange(MPU6000_RANGE_4_G);
#ifdef DEBUG_PRINTS
    if (accel->isConnected())
        Serial.println("MPU6000 init failure.");
    else
        Serial.println("MPU6000 init success.");
#endif

    magne = new MLX90393(1, false);
    magne->begin_I2C();
    delay(100);
    magne->setGain(MLX90393_GAIN_2_5X);
    magne->setResolution(MLX90393_X, MLX90393_RES_19);
    magne->setResolution(MLX90393_Y, MLX90393_RES_19);
    magne->setResolution(MLX90393_Z, MLX90393_RES_16);
    magne->setOversampling(MLX90393_OSR_2);
    magne->setFilter(MLX90393_FILTER_6);
#ifdef DEBUG_PRINTS
    if (!magne->isConnected())
        Serial.println("MLX90393 init failure.");
    else
        Serial.println("MLX90393 init success.");
#endif

    cap = new CAP(2, false);
    delay(100);
    cap->begin();
    delay(100);
#ifdef DEBUG_PRINTS
    if (!cap->isConnected())
        Serial.println("CAP init failure.");
    else
        Serial.println("CAP init success.");
#endif

    pile = new TPIS1385(1);
    delay(100);
    pile->begin();
    pile->readEEprom();
#ifdef DEBUG_PRINTS
    if (!pile->isConnected())
        Serial.println("TPIS1385 init failure.");
    else
        Serial.println("TPIS1385 init success.");
#endif

    // Radio Initialization
    // radio = new LunaRadio();
    radio = new LunaRadio_v2();
    // Initialize the radio settings by using the initialize_radio function
    //  Argument 1: Set frequency to 915
    //  Argument 2: Set output power to 17
    //  Argument 3: Set Bandwidth to 250
    //  Argument 4: Set spreading factor to 12
    //  Argument 5: Set coding rate to 8
    radio->initialize_radio(915.0, 17, 250.0, 12, 8);

    // No idea what is on this pin, assuming an LED maybe.
    pinMode(A3, OUTPUT);
    digitalWrite(A3, HIGH);

    // These two are LEDs.
    pinMode(PIN_LED_1, OUTPUT);
    pinMode(PIN_LED_2, OUTPUT);
}

unsigned long loop_count = 0;
unsigned long delta_time = 0;
sensor_float_vec_t accel_data = {0};
mlx_sample_t magne_data = {0};
TPsample_t pile_data = {0};
bool transmit = false;
uint8_t tx_buf[MAX_PACKET_SIZE + 1] = {0};
uint8_t rx_buf[MAX_PACKET_SIZE + 1] = {0};
float voltage = 0;
float temp_c = 0;
float cap_data = 0;

// Standard Arduino run-time function.
void loop()
{
    // Records loop epoch.
    delta_time = millis();
    String mystr;

    // Check for received radio data.
    /*     String rx_msg = radio->receive_data_string();
        String rx_msg_id = rx_msg.substring(0, 2);

        // Ignore transmissions that do not apply to us.
        if (rx_msg && rx_msg_id == SAT_ID)
        {
            digitalWrite(PIN_LED_2, HIGH);
    #ifdef DEBUG_PRINTS
            Serial.print("(RSSI ");
            Serial.print(radio->getRSSI());
            Serial.print("): ");
            Serial.println(rx_msg);
    #endif
            // Convert the message to uint8_t array.
            rx_msg.getBytes(rx_buf, sizeof(rx_buf));

            digitalWrite(PIN_LED_2, LOW);
        } */

    // !WARN! This might not work properly if its converted to an Arduino String when received by the lander. If that happens, then the data will be truncated at the first 0x0.
    // Check for received radio data; binary method.
    memset(rx_buf, 0x0, sizeof(rx_buf));
    radio->receive_data_binary(rx_buf, sizeof(rx_buf));
    // Check if its our ID.
    if (rx_buf[0] == SAT_ID_B1 && rx_buf[1] == SAT_ID_B2)
    {
        digitalWrite(PIN_LED_2, HIGH);
#ifdef DEBUG_PRINTS
        Serial.print("(RSSI ");
        Serial.print(radio->getRSSI());
        Serial.print("): ");
        // Serial.println(rx_msg);
#endif
        
        // Received message now in rx_buf.
        // TODO: Parse or do something with rx_buf.

        digitalWrite(PIN_LED_2, LOW);
    }
    // END !WARN!

    // Prints out current solar panel voltage.
    voltage = AN_TO_V(analogRead(PIN_SOLAR_V));
#ifdef DEBUG_PRINTS
    Serial.print("Panel V: ");
    Serial.println(voltage, 2);
#endif

    // Prints temperature.
    temp_c = thermo->getTemperatureC();
#ifdef DEBUG_PRINTS
    Serial.print("Temp ('C): ");
    Serial.println(temp_c); // BIG NOTE: getTemperatureF just calls getTemperatureC and converts.
#endif

    // Prints accelerometer data.
    accel_data = accel->getSample();
#ifdef DEBUG_PRINTS
    Serial.print("Accel XYZ (Gs): ");
    Serial.print(accel_data.x, 2);
    Serial.print(" ");
    Serial.print(accel_data.y, 2);
    Serial.print(" ");
    Serial.println(accel_data.z, 2);
#endif

    // Prints magnetometer data.
    magne_data = magne->getSample();
#ifdef DEBUG_PRINTS
    Serial.print("Mag XYZ (uT): ");
    Serial.print(magne_data.magnetic.x, 2);
    Serial.print(" ");
    Serial.print(magne_data.magnetic.y, 2);
    Serial.print(" ");
    Serial.println(magne_data.magnetic.z, 2);
    Serial.print("Mag strength (uT): ");
    Serial.println(magne_data.strength, 2);
#endif

    // Prints raw CAP data.
    cap_data = cap->getRawData();
#ifdef DEBUG_PRINTS
    Serial.print("CAP: ");
    Serial.println(cap_data);
#endif

    // Prints thermopile data.
    pile_data = pile->getSample();
#ifdef DEBUG_PRINTS
    Serial.print("Pile Obj Temp ('C): ");
    Serial.println(pile_data.object);
    Serial.print("Pile Amb Temp ('C): ");
    Serial.println(pile_data.ambient);
#endif

    // The next two lines are only for testing purposes.
    if (!(loop_count % 1000))
        transmit = true;

    if (transmit)
    {
        digitalWrite(PIN_LED_1, HIGH);
        memset(tx_buf, 0x0, MAX_PACKET_SIZE);
        // Make sure to set our SAT_ID preamble properly!
        tx_buf[0] = SAT_ID_B1;
        tx_buf[1] = SAT_ID_B2;

        // NOTE: There may be no way to transmit nor receive binary data due to the way GLEE appears to be handling
        // data transmission and receiving. Assuming the lander's system works the same as the system presented here,
        // the data is converted to an Arduino String prior to being return to the caller. This means that if the
        // data contained any 0x0 values, the newly created String will truncate itself there.
        // It is possible for us to bypass this issue, but it is unclear if that will create issues since, if they
        // use the String system, it will truncate itself somewhere between our LunaSat and the Earth-based database.
        // The receive_data_binary(...) function in luna.cpp attempts to fix this.

        // When 'radio->transmit_data' is called, it requires a 'char *' buffer. This is then passed to
        // 'LunaRadio::radio.transmit(...)' as a 'const char *'. This function then casts it to a 'uint8_t *' before
        // calling 'transmit(...)', where it is then, presumably, transmitted.
        // TX: char * -> const char * -> uint8_t *

        // On the receive path, it seems to be cast from 'uint8_t *' to 'char *' and then made into a 'String'.
        // RX: uint8_t * -> char * -> String

        // We are allowed a maximum transmit size of 256.

        // 256:
        // _---------------------------------------------------------------
        // _---------------------------------------------------------------
        // _---------------------------------------------------------------
        // _---------------------------------------------------------------

        // radio->transmit_data("This is a test transmission.");

        // !WARN! This might not work properly if its converted to an Arduino String when received by the lander. If that happens, then the data will be truncated at the first 0x0.
        // Set our tx_buf with data.
        memcpy(&tx_buf[2], &magne_data, sizeof(mlx_sample_t));
        // Transmits using the binary method!
        radio->transmit_data_binary(tx_buf, sizeof(tx_buf));
        // END !WARN!

        transmit = false;
        digitalWrite(PIN_LED_1, LOW);
    }

    loop_count++;

    // Calculates time since loop epoch.
    delta_time = millis() - delta_time;
    delay(CADENCE - delta_time); // Ensures loop is run once per cadence.
}