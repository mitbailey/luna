/**
 * @file GLEE_Radio_v2.cpp
 * @author Mit Bailey (mitbailey99@gmail.com)
 * @brief Edited version of GLEE_Radio.cpp, since the original did not allow for binary data TRX.
 * @version See Git tags for version information.
 * @date 2022.03.27
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "GLEE_Radio_v2.h"

/**
 * Parameters: frequency as a float, power as an unsigned 8 bit integer, bandwith as a float,
 * spreading factoctor as an unsigned 8 bit integer, and coding rate as an an unsigned 8 bit integer
 * Returns: none
 * This function begins communications with the radio and notifies if the communication
 * was successful or not. Then, it sets the frequency, bandwith, output power, spreading
 * factor, and coding rate if successful.
 **/
void LunaRadio_v2::initialize_radio(float freq, uint8_t pwr, float bw, uint8_t sf, uint8_t cr)
{

    // Serial.print(F("Radio Initializing ... "));
    LunaRadio_v2::err_state = LunaRadio_v2::radio.begin(); // begin communication with radio
    if (LunaRadio_v2::err_state == RADIOLIB_ERR_NONE)
    {
        // if no error
        // Serial.println(F("success!"));
    }
    else
    {
        // if failed and error
        // Serial.print(F("failed, code "));
        // Serial.println(LunaRadio_v2::err_state);
        while (true)
            ;
    }

    // set frequency
    LunaRadio_v2::setFreq(freq);

    // set bandwidth
    LunaRadio_v2::setBandwidth(bw);

    // set output power
    LunaRadio_v2::setPWR(pwr);

    // set spreading factor
    LunaRadio_v2::setSF(sf);

    // set coding rate
    LunaRadio_v2::setCR(cr);

    // Serial.println(F("The radio is ready for use!"));
}

/**
 * Parameters: frequency as a float
 * Returns: none
 * This function sets the frequency and if successful, it will construct a
 * display message and send that message to serial to notify the user. If
 * the frequency failed, it will output a message.
 **/
void LunaRadio_v2::setFreq(float freq)
{
    LunaRadio_v2::err_state = LunaRadio_v2::radio.setFrequency(freq); // saves the error state and sets frequency
    if (LunaRadio_v2::err_state == RADIOLIB_ERR_NONE)
    {
        // constructing display message, then sends message to serial if no error
        snprintf(LunaRadio_v2::disp_buff, sizeof(LunaRadio_v2::disp_buff), "Frequency set to %d.%01d MHz, success!", (int)freq, (int)(freq * 10) % 10);
        // Serial.println(LunaRadio_v2::disp_buff);
    }
    else
    {
        // if failed and error
        // Serial.print(F("Frequency set failed, code "));
        // Serial.println(LunaRadio_v2::err_state);
        while (true)
            ;
    }
}

/**
 * Parameters: coding rate as an 8-bit unsigned integer
 * Returns: none
 * This function sets the Coding Rate and if successful, it will construct a
 * display message and send that message to serial to notify the user. If
 * the frequency failed, it will output a message.
 **/
void LunaRadio_v2::setCR(uint8_t cr)
{
    LunaRadio_v2::err_state = LunaRadio_v2::radio.setCodingRate(cr); // sets coding rate and saves error state
    if (LunaRadio_v2::err_state == RADIOLIB_ERR_NONE)
    {
        // constructing display message, then sends message to serial if no error
        snprintf(LunaRadio_v2::disp_buff, sizeof(LunaRadio_v2::disp_buff), "Coding Rate set to %d, success!", cr);
        // Serial.println(LunaRadio_v2::disp_buff);
    }
    else
    {
        // if failed and error
        // Serial.print(F("Coding Rate set failed, code "));
        // Serial.println(LunaRadio_v2::err_state);
        while (true)
            ;
    }
}

/**
 * Parameters: spreading factor as an 8-bit unsigned integer
 * Returns: none
 * This function sets the Spreading Factor and if successful, it will construct a
 * display message and send that message to serial to notify the user. If
 * the frequency failed, it will output a message.
 **/
void LunaRadio_v2::setSF(uint8_t sf)
{
    LunaRadio_v2::err_state = LunaRadio_v2::radio.setSpreadingFactor(sf); // saves the error state and sets spreading factor
    if (LunaRadio_v2::err_state == RADIOLIB_ERR_NONE)
    {
        // constructing display message, then sends message to serial if no error
        snprintf(LunaRadio_v2::disp_buff, sizeof(LunaRadio_v2::disp_buff), "Spreading factor set to %d, success!", sf);
        // Serial.println(LunaRadio_v2::disp_buff);
    }
    else
    {
        // if failed and error
        // Serial.print(F("Spreading Factor set failed, code "));
        // Serial.println(LunaRadio_v2::err_state);
        while (true)
            ;
    }
}

/**
 * Parameters: bandwidth as a float
 * Returns: none
 * This function sets the bandwith and if successful, it will construct a
 * display message and send that message to serial to notify the user. If
 * the frequency failed, it will output a message.
 **/
void LunaRadio_v2::setBandwidth(float bw)
{
    LunaRadio_v2::err_state = LunaRadio_v2::radio.setBandwidth(bw); // saves the error state and sets bandwith
    if (LunaRadio_v2::err_state == RADIOLIB_ERR_NONE)
    {
        // constructing display message, then sends message to serial if no error
        snprintf(LunaRadio_v2::disp_buff, sizeof(LunaRadio_v2::disp_buff), "Bandwidth set to %d.%01d, success!", (int)bw, (int)(bw * 10) % 10);
        // Serial.println(LunaRadio_v2::disp_buff);
    }
    else
    {
        // if failed and error
        // Serial.print(F("Bandwidth set failed, code "));
        // Serial.println(LunaRadio_v2::err_state);
        while (true)
            ;
    }
}

/**
 * Parameters: output power as an 8-bit unsigned integer
 * Returns: none
 * This function sets the output power and if successful, it will construct a
 * display message and send that message to serial to notify the user. If
 * the frequency failed, it will output a message.
 **/
void LunaRadio_v2::setPWR(uint8_t pwr)
{
    LunaRadio_v2::err_state = LunaRadio_v2::radio.setOutputPower(pwr); // sets output and saves error state
    if (LunaRadio_v2::err_state == RADIOLIB_ERR_NONE)
    {
        // constructing display message, then sends message to serial if no error
        snprintf(LunaRadio_v2::disp_buff, sizeof(LunaRadio_v2::disp_buff), "Output Power set to %d, success!", pwr); //
        // Serial.println(LunaRadio_v2::disp_buff);
    }
    else
    {
        // if error and failed
        // Serial.print(F("Output Power set failed, code "));
        // Serial.println(LunaRadio_v2::err_state);
        while (true)
            ;
    }
}

/**
 * Parameters: the buffer as a char pointer
 * Returns: none
 * This function transmits data from what is given as the argument and prints
 * out the  easured data rate if successful and the error message if unsuccessful.
 **/
void LunaRadio_v2::transmit_data(char *buff)
{
    // Serial.println(F("Transmitting Data"));
    LunaRadio_v2::err_state = LunaRadio_v2::radio.transmit(buff); // saves error state when trasnmitting
    if (LunaRadio_v2::err_state == RADIOLIB_ERR_NONE)
    {
        // the packet was successfully transmitted
        // Serial.println(F("Tranmission success!"));
        // print measured data rate
        // Serial.print(F("Datarate: "));
        // Serial.print(LunaRadio_v2::radio.getDataRate());
        // Serial.println(F(" bps"));
    }
    else if (LunaRadio_v2::err_state == RADIOLIB_ERR_PACKET_TOO_LONG)
    {
        // the supplied packet was longer than 256 bytes
        // Serial.println(F("too long!"));
    }
    else if (LunaRadio_v2::err_state == RADIOLIB_ERR_TX_TIMEOUT)
    {
        // timeout occurred while transmitting packet
        // Serial.println(F("timeout!"));
    }
    else
    {
        // some other error occurred
        // Serial.print(F("failed, code "));
        // Serial.println(LunaRadio_v2::err_state);
    }
}

void LunaRadio_v2::transmit_data_binary(uint8_t *buf, size_t buf_len)
{
    // Serial.println(F("Transmitting Data"));
    LunaRadio_v2::err_state = LunaRadio_v2::radio.transmit(buf, buf_len);

    if (LunaRadio_v2::err_state == RADIOLIB_ERR_NONE)
    {
        // the packet was successfully transmitted
        // Serial.println(F("Tranmission success!"));
        // print measured data rate
        // Serial.print(F("Datarate: "));
        // Serial.print(LunaRadio_v2::radio.getDataRate());
        // Serial.println(F(" bps"));
    }
    else if (LunaRadio_v2::err_state == RADIOLIB_ERR_PACKET_TOO_LONG)
    {
        // the supplied packet was longer than 256 bytes
        // Serial.println(F("too long!"));
    }
    else if (LunaRadio_v2::err_state == RADIOLIB_ERR_TX_TIMEOUT)
    {
        // timeout occurred while transmitting packet
        // Serial.println(F("timeout!"));
    }
    else
    {
        // some other error occurred
        // Serial.print(F("failed, code "));
        // Serial.println(LunaRadio_v2::err_state);
    }
}

/**
 * Parameters: none
 * Returns: a string that is the data received or a message of the error.
 * This function receives data while checking and outputting
 * error if there is any.
 **/
String LunaRadio_v2::receive_data_string(void)
{
    // Serial.println(F("Waiting for Data"));
    String tmp_str;
    LunaRadio_v2::err_state = LunaRadio_v2::radio.receive(tmp_str); // saves error state when receiving
    if (LunaRadio_v2::err_state == RADIOLIB_ERR_NONE)
    {
        // if no eror
        // Serial.println(F("Data received"));
        return tmp_str;
    }
    else if (LunaRadio_v2::err_state == RADIOLIB_ERR_RX_TIMEOUT)
    {
        // timeout error
        // Serial.println("Receive Failed");
        return "Receiver Timeout";
    }
    else if (LunaRadio_v2::err_state == RADIOLIB_ERR_CRC_MISMATCH)
    {
        // CTC error
        // Serial.println("Receive Failed");
        return "CTC Error";
    }
    else
    {
        // Serial.print(F("failed, code "));
        // code error
        // Serial.println(LunaRadio_v2::err_state);
        return "Check Code";
    }
}

// Bypasses GLEE's receive_data_string() method which clobbers any attempt at receiving binary data. Specifically, the
// uint8_t * to Arduino String cast in RadioLib causes all 0s to be interpreted as '\0', truncating the data.
uint8_t LunaRadio_v2::receive_data_binary(uint8_t *buf, size_t buf_len)
{
    // Serial.println(F("Waiting for Data"));

    LunaRadio_v2::err_state = LunaRadio_v2::radio.receive(buf, buf_len); // Saves error state when RXing.

    if (LunaRadio_v2::err_state == RADIOLIB_ERR_NONE)
    {
        // If no error.
        // Serial.println(F("Data received"));
        return 0;
    }
    else if (LunaRadio_v2::err_state == RADIOLIB_ERR_RX_TIMEOUT)
    {
        // Timeout error.
        // Serial.println("Receive Failed");
        return 1; // Receiver timeout
    }
    else if (LunaRadio_v2::err_state == RADIOLIB_ERR_CRC_MISMATCH)
    {
        // CTC error.
        // Serial.println("Receive Failed");
        return 2; // "CTC Error";
    }
    else
    {
        // Serial.print(F("failed, code "));
        // Code error.
        // Serial.println(LunaRadio_v2::err_state);
        return 3; // "Check Code";
    }
}

/**
 * Parameters: none
 * Returns: string that states the error
 * This function received data while checking and outputting
 * error if there is any and is used for plotting.
 **/
String LunaRadio_v2::receive_data_string_plotting(void)
{
    String tmp_str;
    LunaRadio_v2::err_state = LunaRadio_v2::radio.receive(tmp_str); // saves error state when receiving
    if (LunaRadio_v2::err_state == RADIOLIB_ERR_NONE)
    {
        // if no eror
        return tmp_str;
    }
    else if (LunaRadio_v2::err_state == RADIOLIB_ERR_RX_TIMEOUT)
    {
        // timeout error
        // Serial.println(F("Receive Failed"));
        return "Receiver Timeout";
    }
    else if (LunaRadio_v2::err_state == RADIOLIB_ERR_CRC_MISMATCH)
    {
        // CTC error
        // Serial.println(F("Receive Failed"));
        return "CTC Error";
    }
    else
    {
        // Serial.print(F("failed, code "));
        // code error
        // Serial.println(LunaRadio_v2::err_state);
        return "Check Code";
    }
}

/**
 * Parameters: pointer to recieve interupt callback function
 * Returns: none
 * Sets recieve
 **/
/* void LunaRadio_v2::enable_recieve_interupt(void (*func)){

    radio.setDio0Action(func);

    int state = radio.startReceive();
    if(state == RADIOLIB_ERR_NONE){
        // Serial.println(F("Now listening for transmissions"));
    } else{
        // Serial.print(F("failed, code "));
        // Serial.println(state);
        while(true);
  }
} */

/**
 * Parameters: none
 * Returns: returns if transmission started or not
 * This function starts the radio transmission or the code fails.
 **/
void LunaRadio_v2::startRecieve(void)
{
    int state = radio.startReceive();
    if (state == RADIOLIB_ERR_NONE)
    {
        // Serial.println(F("Now listening for transmissions"));
    }
    else
    {
        // Serial.print(F("failed, code "));
        // Serial.println(state);
        while (true)
            ;
    }
}

/**
 * Parameters: uint8_t* data and size_t len
 * Returns: if packet was sucessfully received or not
 * This function determines if the data contains an error
 * with the given outputs.
 **/
void LunaRadio_v2::readData(uint8_t *data, size_t len)
{
    int state = radio.readData(data, len);
    if (state == RADIOLIB_ERR_NONE)
    {
        // packet was successfully received
        // Serial.println(F("Packet received!"));
    }
    else if (state == RADIOLIB_ERR_CRC_MISMATCH)
    {
        // malformed packet reception
        // Serial.println(F("Malformed Packet!"));
    }
    else
    {
        // Print unhandled error coad
        // Serial.print(F("Failed, code "));
        // Serial.println(state);
    }
}

/**
 * Parameters: None
 * Returns: Luna Radio RSSI
 * This function returns locally stored RSSI
 **/
float LunaRadio_v2::getRSSI(void)
{
    return (LunaRadio_v2::radio.getRSSI());
}

/**
 * Parameters: None
 * Returns: Luna Radio SNR
 * This function returns locally stored SNR
 **/
float LunaRadio_v2::getSNR(void)
{
    return (LunaRadio_v2::radio.getSNR());
}
