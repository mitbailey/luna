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

// uncomment the following only on one
// of the nodes to initiate the pings
//#define INITIATING_NODE

// SX1272 has the following connections:
// NSS pin:   10
// DIO0 pin:  2
// NRST pin:  9
// DIO1 pin:  3
SX1272 radio = new Module(10, 2, 9, 3);

// or using RadioShield
// https://github.com/jgromes/RadioShield
//SX1272 radio = RadioShield.ModuleA;

// save transmission states between loops
int transmissionState = RADIOLIB_ERR_NONE;

// flag to indicate transmission or reception state
bool transmitFlag = false;

// disable interrupt when it's not needed
volatile bool enableInterrupt = true;

// flag to indicate that a packet was sent or received
volatile bool operationDone = false;

// this function is called when a complete packet
// is transmitted or received by the module
// IMPORTANT: this function MUST be 'void' type
//            and MUST NOT have any arguments!
void setFlag(void) {
  // check if the interrupt is enabled
  if(!enableInterrupt) {
    return;
  }

  // we sent or received  packet, set the flag
  operationDone = true;
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

    // set the function that will be called
    // when new packet is received
    radio.setDio0Action(setFlag);

#if defined(INITIATING_NODE)
    // send the first packet on this node
    Serial.print(F("[SX1272] Sending first packet ... "));
    transmissionState = radio.startTransmit("Hello World!");
    transmitFlag = true;
#else
    // start listening for LoRa packets on this node
    Serial.print(F("[SX1272] Starting to listen ... "));
    state = radio.startReceive();
    if (state == RADIOLIB_ERR_NONE) {
      Serial.println(F("success!"));
    } else {
      Serial.print(F("failed, code "));
      Serial.println(state);
      while (true);
    }
#endif
}

void loop() {
    // check if the previous operation finished
    if(operationDone) 
    {
        // disable the interrupt service routine while
        // processing the data
        enableInterrupt = false;

        // reset flag
        operationDone = false;

        if(transmitFlag) 
        {
            digitalWrite(LED1, HIGH);
            digitalWrite(LED2, LOW);

            // the previous operation was transmission, listen for response
            // print the result
            if (transmissionState == RADIOLIB_ERR_NONE) 
            {
                // packet was successfully sent
                Serial.println(F("transmission finished!"));

            } else 
            {
                Serial.print(F("failed, code "));
                Serial.println(transmissionState);
            }

        // listen for response
        radio.startReceive();
        transmitFlag = false;

        } 
        else 
        {
            digitalWrite(LED2, HIGH);
            digitalWrite(LED1, LOW);

            // the previous operation was reception
            // print data and send another packet
            String str;
            int state = radio.readData(str);

            if (state == RADIOLIB_ERR_NONE) 
            {
                // packet was successfully received
                Serial.println(F("[SX1272] Received packet!"));

                // print data of the packet
                Serial.print(F("[SX1272] Data:\t\t"));
                Serial.println(str);

                // print RSSI (Received Signal Strength Indicator)
                Serial.print(F("[SX1272] RSSI:\t\t"));
                Serial.print(radio.getRSSI());
                Serial.println(F(" dBm"));

                // print SNR (Signal-to-Noise Ratio)
                Serial.print(F("[SX1272] SNR:\t\t"));
                Serial.print(radio.getSNR());
                Serial.println(F(" dB"));

            }

            // wait a second before transmitting again
            delay(1000);

            // send another one
            Serial.print(F("[SX1272] Sending another packet ... "));
            transmissionState = radio.startTransmit("Hello World!");
            transmitFlag = true;
        }

        // we're ready to process more packets,
        // enable interrupt service routine
        enableInterrupt = true;

    }

    digitalWrite(LED1, LOW);
    digitalWrite(LED2, LOW);
}