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
#include <GLEE_Radio.h>

#include "luna.hpp"

TMP117 *thermo = nullptr;
MPU6000 *accel = nullptr;
MLX90393 *magne = nullptr;
CAP *cap = nullptr;
TPIS1385 *pile = nullptr;
LunaRadio *radio = nullptr;

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

    Serial.print("I2C and Serial initialization complete.\n");

    I2C_scan();

    // Thermopile Initialization
    Wire.beginTransmission(0x00); // TX buffer.
    Wire.write(0x04);             // Add register address to TX buffer.
    Wire.write(0x00);             // Add initialization data to TX.
    Wire.endTransmission();       // Send.

    delay(50);

    Serial.print("Thermopile init command sent.\n"); // Note that we havent actually checked if its working.

    // Sensors Initialization
    thermo = new TMP117(1, false);
    delay(100);
    if(!thermo->isConnected())
        Serial.println("TMP117 init failure.");
    else
        Serial.println("TMP117 init success.");

    accel = new MPU6000(1, false);
    accel->begin();
    delay(100);
    accel->initialize();
    delay(100);
    accel->setAccelRange(MPU6000_RANGE_4_G);
    if(accel->isConnected())
        Serial.println("MPU6000 init failure.");
    else
        Serial.println("MPU6000 init success.");

    magne = new MLX90393(1, false);
    magne->begin_I2C();
    delay(100);
    magne->setGain(MLX90393_GAIN_2_5X);
    magne->setResolution(MLX90393_X, MLX90393_RES_19);
    magne->setResolution(MLX90393_Y, MLX90393_RES_19);
    magne->setResolution(MLX90393_Z, MLX90393_RES_16);
    magne->setOversampling(MLX90393_OSR_2);
    magne->setFilter(MLX90393_FILTER_6);
    if(!magne->isConnected())
        Serial.println("MLX90393 init failure.");
    else
        Serial.println("MLX90393 init success.");

    cap = new CAP(2, false);
    delay(100);
    cap->begin();
    delay(100);
    if(!cap->isConnected())
        Serial.println("CAP init failure.");
    else
        Serial.println("CAP init success.");

    pile = new TPIS1385(1);
    delay(100);
    pile->begin();
    pile->readEEprom();
    if(!pile->isConnected())
        Serial.println("TPIS1385 init failure.");
    else
        Serial.println("TPIS1385 init success.");

    // Radio Initialization
    radio = new LunaRadio();
    //Initialize the radio settings by using the initialize_radio function
	// Argument 1: Set frequency to 915
	// Argument 2: Set output power to 17
	// Argument 3: Set Bandwidth to 250
	// Argument 4: Set spreading factor to 12
	// Argument 5: Set coding rate to 8
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
char tx_msg[MAX_TX_SIZE] = {0};
float voltage = 0;
float temp_c = 0;
float cap_data = 0;

// Standard Arduino run-time function.
void loop()
{
    // Records loop epoch.
    delta_time = millis();

    // Check for received radio data.
    String rx_msg = radio->receive_data_string();
    String rx_msg_id = rx_msg.substring(0, 2);
    String rx_msg_content = rx_msg.substring(3);

    // Ignore transmissions that do not apply to us.
    if (rx_msg && rx_msg_id == SAT_ID)
    {
        digitalWrite(PIN_LED_2, HIGH);
        Serial.print("(RSSI ");
        Serial.print(radio->getRSSI());
        Serial.print("): ");
        Serial.println(rx_msg_content);
        digitalWrite(PIN_LED_2, LOW);
    }

    // Prints out current solar panel voltage.
    Serial.print("Panel V: ");
    voltage = AN_TO_V(analogRead(PIN_SOLAR_V));
    Serial.println(voltage, 2);

    // Prints temperature.
    Serial.print("Temp ('C): ");
    temp_c = thermo->getTemperatureC();
    Serial.println(temp_c); // BIG NOTE: getTemperatureF just calls getTemperatureC and converts.

    // Prints accelerometer data.
    accel_data = accel->getSample();
    Serial.print("Accel XYZ (Gs): ");
    Serial.print(accel_data.x, 2);
    Serial.print(" ");
    Serial.print(accel_data.y, 2);
    Serial.print(" ");
    Serial.println(accel_data.z, 2);

    // Prints magnetometer data.
    magne_data = magne->getSample();
    Serial.print("Mag XYZ (uT): ");
    Serial.print(magne_data.magnetic.x, 2);
    Serial.print(" ");
    Serial.print(magne_data.magnetic.y, 2);
    Serial.print(" ");
    Serial.println(magne_data.magnetic.z, 2);
    Serial.print("Mag strength (uT): ");
    Serial.println(magne_data.strength, 2);

    // Prints raw CAP data.
    Serial.print("CAP: ");
    cap_data = cap->getRawData();
    Serial.println(cap_data);

    // Prints thermopile data.
    pile_data = pile->getSample();
    Serial.print("Pile Obj Temp ('C): ");
    Serial.println(pile_data.object);
    Serial.print("Pile Amb Temp ('C): ");
    Serial.println(pile_data.ambient);

    // The next two lines are only for testing purposes.
    if (!(loop_count % 1000))
        transmit = true;

    if (transmit)
    {
        digitalWrite(PIN_LED_1, HIGH);
        memset(tx_msg, 0x0, MAX_TX_SIZE);
        
        radio->transmit_data("This is a test transmission.");

        transmit = false;
        digitalWrite(PIN_LED_1, LOW);
    }

    loop_count++;

    // Calculates time since loop epoch.
    delta_time = millis() - delta_time;
    delay(CADENCE - delta_time); // Ensures loop is run once per cadence.
}