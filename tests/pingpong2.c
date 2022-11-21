// Successful back-and-forth b/w LunaSat 1 & 2
// 08:33 2022.11.21

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
}

void loop()
{
    if (transmitting)
    {
        // Transmitting
        digitalWrite(LED1, HIGH);
        digitalWrite(LED2, LOW); 

        delay(500);

        Serial.println("Beginning transmit...");
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
    }
    else
    {
        // Receiving
        digitalWrite(LED1, LOW);
        digitalWrite(LED2, HIGH); 

        String str;
        Serial.println("Beginning receive...");
        int state = radio.receive(str);

        if (state == RADIOLIB_ERR_NONE) 
        {
            // packet was successfully received
            Serial.println(F("success!"));

            // print the data of the packet
            Serial.print(F("[SX1272] Data:\t\t\t"));
            Serial.println(str);

            // print the RSSI (Received Signal Strength Indicator)
            // of the last received packet
            Serial.print(F("[SX1272] RSSI:\t\t\t"));
            Serial.print(radio.getRSSI());
            Serial.println(F(" dBm"));

            // print the SNR (Signal-to-Noise Ratio)
            // of the last received packet
            Serial.print(F("[SX1272] SNR:\t\t\t"));
            Serial.print(radio.getSNR());
            Serial.println(F(" dB"));

            // print frequency error
            // of the last received packet
            Serial.print(F("[SX1272] Frequency error:\t"));
            Serial.print(radio.getFrequencyError());
            Serial.println(F(" Hz"));

            transmitting = true;

        } else if (state == RADIOLIB_ERR_RX_TIMEOUT) {
            // timeout occurred while waiting for a packet
            Serial.println(F("timeout!"));

            if (transmitter)
            {
                transmitting = true;
            }

        } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
            // packet was received, but is malformed
            Serial.println(F("CRC error!"));

        } else {
            // some other error occurred
            Serial.print(F("failed, code "));
            Serial.println(state);
        }
    }

    // digitalWrite(LED1, LOW);
    // digitalWrite(LED2, LOW);    
}
