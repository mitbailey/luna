/**
 * @file radio_rx.ino
 * @author Mit Bailey (mitbailey99@gmail.com)
 * @brief RX for LunaSat 1 & 2.
 * @version See Git tags for version information.
 * @date 2022.11.21
 *
 * @copyright Copyright (c) 2022
 *
 */

// include the library
#include <RadioLib.h>

int LED1 = 4;
int LED2 = 5;

// SX1272 has the following connections:
// NSS pin:   10
// DIO0 pin:  2
// NRST pin:  9
// DIO1 pin:  3
SX1272 radio = new Module(10, 2, 9, 3);

String ID;

// flag to indicate that a packet was received
volatile bool freshPacket = false;

// disable interrupt when it's not needed
volatile bool enableInterrupt = false;

// this function is called when a complete packet
// is received by the module
// IMPORTANT: this function MUST be 'void' type
//            and MUST NOT have any arguments!
#if defined(ESP8266) || defined(ESP32)
  ICACHE_RAM_ATTR
#endif
void setFlag(void) {
    Serial.println("setFlag called!");

    // check if the interrupt is enabled
    if(!enableInterrupt) 
    {
        return;
    }

    // we got a packet, set the flag
    freshPacket = true;
}

void setup() {
    pinMode(LED1, OUTPUT);
    pinMode(LED2, OUTPUT);

    Serial.begin(9600);

    // initialize SX1272 with default settings
    Serial.print(F("[SX1272] Initializing ... "));
    int state = radio.begin();
    if (state == RADIOLIB_ERR_NONE) {
        Serial.println(F("success!"));
    } else {
        Serial.print(F("failed, code "));
        Serial.println(state);
        while (true);
    }

    ID = "RECEIVER";

    radio.setFrequency(915.0);
    radio.setCodingRate(8);
    radio.setSpreadingFactor(12);
    radio.setBandwidth(250.0);
    radio.setOutputPower(17);

    radio.setDio0Action(setFlag);
    state = radio.startReceive();
    if (state == RADIOLIB_ERR_NONE)
    {
        Serial.println("Now listening...");
    }
    else
    {
        Serial.println("FATAL: Failed to listen!");
        for(;;);
    }

    enableInterrupt = true;
}

void loop()
{   
    if (freshPacket)
    {
        Serial.println("Got fresh packet!");

        digitalWrite(LED1, LOW);
        digitalWrite(LED2, HIGH);

        enableInterrupt = false;
        freshPacket = false;

        String str;
        int state = radio.readData(str);
        // you can also read received data as byte array
        /*
        byte byteArr[8];
        int state = radio.readData(byteArr, 8);
        */

        if (state == RADIOLIB_ERR_NONE) 
        {
            // packet was successfully received
            Serial.println(F("[SX1272] Received packet!"));

            // print data of the packet
            Serial.print(F("[SX1272] Data:\t\t"));
            Serial.println(str);

            // print RSSI (Received Signal Strength Indicator)
            // of the last received packet
            Serial.print(F("[SX1272] RSSI:\t\t"));
            Serial.print(radio.getRSSI());
            Serial.println(F(" dBm"));
        } 
        else if (state == RADIOLIB_ERR_CRC_MISMATCH) 
        {
            // packet was received, but is malformed
            Serial.println(F("CRC error!"));
        } 
        else 
        {
            // some other error occurred
            Serial.print(F("failed, code "));
            Serial.println(state);
        }

        // put module back to listen mode
        radio.startReceive();

        // we're ready to receive more packets,
        // enable interrupt service routine
        enableInterrupt = true;
    }
}
