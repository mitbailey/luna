/**
 * @file adv_pingpong2.c
 * @author Mit Bailey (mitbailey99@gmail.com)
 * @brief Advanced radio ping-pong for LunaSat 1 & 2.
 * @version See Git tags for version information.
 * @date 2022.11.21
 *
 * @copyright Copyright (c) 2022
 *
 */

// Successful back-and-forth b/w LunaSat 1 & 2
// 09:54 2022.11.21

/*
   RadioLib SX127x Ping-Pong Example
   For default module settings, see the wiki page
   https://github.com/jgromes/RadioLib/wiki/Default-configuration#sx127xrfm9x---lora-modem
   For full API reference, see the GitHub Pages
   https://jgromes.github.io/RadioLib/
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

// Change this boolean to 'true' on the initiator.
bool transmitter = false;
bool transmitting = transmitter;
String ID;

unsigned long last_tx = 0;
#define TX_TIMER 10000

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

    if (transmitter)
    {
        ID = "TRANSMITTER";
    }
    else
    {
        ID = "RECEIVER";
    }

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

bool timedout = false;
void loop()
{   
    if (transmitting || timedout)
    {
        timedout = false;

        // Transmitting
        digitalWrite(LED1, HIGH);
        digitalWrite(LED2, LOW); 

        delay(500);

        Serial.println("Beginning transmit...");
        last_tx = millis();
        enableInterrupt = false;
        int state = radio.transmit("Hello from " + ID + "!");

        if (state == RADIOLIB_ERR_NONE) {
            // the packet was successfully transmitted
            Serial.println(F(" success!"));

            // print measured data rate
            Serial.print(F("[SX1272] Datarate:\t"));
            Serial.print(radio.getDataRate());
            Serial.println(F(" bps"));

            transmitting = false;

        } else if (state == RADIOLIB_ERR_PACKET_TOO_LONG) {
            // the supplied packet was longer than 256 bytes
            Serial.println(F("too long!"));

        } else if (state == RADIOLIB_ERR_TX_TIMEOUT) {
            // timeout occurred while transmitting packet
            Serial.println(F("timeout!"));

        } else {
            // some other error occurred
            Serial.print(F("failed, code "));
            Serial.println(state);
        }

        enableInterrupt = true;
        radio.startReceive();
    
    }
    else if (freshPacket)
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
        transmitting = true;
    }
    else
    {
        if (millis() - last_tx > TX_TIMER)
        {
            timedout = true;
        }
    }
}
