
/*
    Please note the long strings of data sent mean the *RTS* pin is
    required with UART to slow down data sent to the Bluefruit LE!
*/

#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"

#if SOFTWARE_SERIAL_AVAILABLE
  #include <SoftwareSerial.h>
#endif

// Create the bluefruit object, either software serial...uncomment these lines
/*
SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
                      BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);
*/

/* ...or hardware serial, which does not need the RTS/CTS pins. Uncomment this line */
// Adafruit_BluefruitLE_UART ble(BLUEFRUIT_HWSERIAL_NAME, BLUEFRUIT_UART_MODE_PIN);

/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

/* ...software SPI, using SCK/MOSI/MISO user-defined SPI pins and then user selected CS/IRQ/RST */
//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_SCK, BLUEFRUIT_SPI_MISO,
//                             BLUEFRUIT_SPI_MOSI, BLUEFRUIT_SPI_CS,
//                             BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);


// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

/* The service information */
int32_t clickServiceId;
//int32_t hrmMeasureCharId;
//int32_t hrmLocationCharId;
int32_t clickCharId;

/* Circuit initial variables */
const int buttonPin = 6;
int ledState = HIGH;         // the current state of the output pin
int buttonState;             // the current reading from the input pin
int lastButtonState = LOW;   // the previous reading from the input pin

// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 5;    // the debounce time; increase if the output flickers
/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/
void setup(void)
{
  boolean success;

  Serial.begin(115200);
  Serial.println(F("HotButton"));
  Serial.println(F("---------------------------------------------------"));

  randomSeed(micros());

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  /* Perform a factory reset to make sure everything is in a known state */
  Serial.println(F("Performing a factory reset: "));
  if (! ble.factoryReset() ){
       error(F("Couldn't factory reset"));
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  // this line is particularly required for Flora, but is a good idea
  // anyways for the super long lines ahead!
  // ble.setInterCharWriteDelay(5); // 5 ms

  /* Change the device name to make it easier to find */
  Serial.println(F("Setting device name to 'HotButton': "));

  if (! ble.sendCommandCheckOK(F("AT+GAPDEVNAME=HotButton")) ) {
    error(F("Could not set device name?"));
  }

  /* Add the Click Service definition */
  /* Service ID should be 1 */
  Serial.println(F("Adding the Click Service definition (UUID = 0x9e5c00cc-7541-4205-8df1-74f41e2fb968): "));
  success = ble.sendCommandWithIntReply( F("AT+GATTADDSERVICE=UUID128=9e-5c-00-cc-75-41-42-05-8d-f1-74-f4-1e-2f-b9-68"), &clickServiceId);
  if (! success) {
    error(F("Could not add click service"));
  }

  /* Add the Heart Rate Measurement characteristic */
  /* Chars ID for Measurement should be 1 */
//  Serial.println(F("Adding the Heart Rate Measurement characteristic (UUID = 0x2A37): "));
//  success = ble.sendCommandWithIntReply( F("AT+GATTADDCHAR=UUID=0x2A37, PROPERTIES=0x10, MIN_LEN=2, MAX_LEN=3, VALUE=00-40"), &hrmMeasureCharId);
//    if (! success) {
//    error(F("Could not add HRM characteristic"));
//  }

  /* Add the Body Sensor Location characteristic */
  /* Chars ID for Body should be 2 */
//  Serial.println(F("Adding the Body Sensor Location characteristic (UUID = 0x2A38): "));
//  success = ble.sendCommandWithIntReply( F("AT+GATTADDCHAR=UUID=0x2A38, PROPERTIES=0x02, MIN_LEN=1, VALUE=3"), &hrmLocationCharId);
//    if (! success) {
//    error(F("Could not add BSL characteristic"));
//  }

  /* Add click timestamp to heart rate transmission */
 
  Serial.println(F("Adding the timestamp characteristic (UUID = 0x0001): "));
  success = ble.sendCommandWithIntReply(F("AT+GATTADDCHAR=UUID=0x0001, PROPERTIES=0x10, MIN_LEN=1, MAX_LEN=20, VALUE=INITIAL"), &clickCharId);
    if (! success) {
    error(F("Could not add timestamp characteristic"));
  }
  
  /* Add the Heart Rate Service to the advertising data (needed for Nordic apps to detect the service) */
  Serial.print(F("Adding Click Service UUID to the advertising payload: "));
  ble.sendCommandCheckOK( F("AT+GAPSETADVDATA=02-01-06-05-02-0d-18-0a-18") );

  /* Reset the device for the new service setting changes to take effect */
  Serial.print(F("Performing a SW reset (service changes require a reset): "));
  ble.reset();

  Serial.println();
}

/** Send randomized heart rate data continuously **/
void loop(void)
{


  /* Listen for and debounce clicks */
  uint32_t start, stop, sent;
  start = stop = sent = 0;

  // read the state of the switch into a local variable:
  int reading = digitalRead(buttonPin);
//  Serial.println(reading);
  
  // If the switch changed, due to noise or pressing:
  if (reading != lastButtonState) {
    // reset the debouncing timer
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:

    // if the button state has changed:
    if (reading != buttonState) {
      buttonState = reading;

      // only toggle the LED if the new button state is HIGH
      if (buttonState == HIGH) { 
        // lastDebounceTime is the start time of the last valid click, do stuff with it here
        Serial.println(lastDebounceTime); 
        ble.print( F("AT+GATTCHAR=") );
        ble.print( clickCharId );
        ble.print( F(",00-") );
        ble.println(lastDebounceTime, HEX);
        
        /* Check if command executed OK */
        if ( !ble.waitForOK() )
        {
          Serial.println(F("Failed to get response!"));
        }       
      }
    }
  }
  
  // save the reading. Next time through the loop, it'll be the lastButtonState:
  lastButtonState = reading;
 
}
