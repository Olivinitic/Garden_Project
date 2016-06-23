/*************************************************************************************
*   Project:      Automated Garden Monitor
*   Author:       Oliver Salmeron
*   Date Began:   May, 2016
*   
*   Description:  This program monitors and controls the parameters of a garden space. 
*                 All inputs and outputs are modular by design and can be selected as desired.
*
*   Steps:
*     1. Get Parts
*     2. Set Up sensors individually
*     3. Set Up sensors under 1 code
*     4. Enclose
*     5. Make a projected Business Plan
*       - Executive Summary
*         -- Expected Revenues, Expenses, Profits (5 years)
*         -- Funding needed
*       - Company Overview
*       - Industry Analysis
*         -- Market Overview
*         -- Relevant Market Size
*       - Customer Analysis
*         -- Target Customers
*         -- Customer Needs
*       - Competitive Analysis
*         -- Direct Competitors
*         -- Indirect Competitors
*         -- Competitive Advantages
*       - Marketing Plan
*         -- Products & Services
*         -- Pricing
*         -- Promotions Plan
*         -- Distributions Plan
*       - Price of Production
*       - Price of Retail
*       - 
*     
*
*   Psuedoinstructions:
*   - Inputs (Settings and Sensors)
*       Config/Settings   <-- Presets or User "Recipes"
*                               Defines the Parameters (# of plants, strains, area)
*                               Controls the Outputs and Automation
*       Documentation     <-- Create a new portfolio or Select an existing one
*         | Portfolio Name:       Name                    |
*         | Strain/Plant Name:    Name                    |
*         | Setting:              Name                    |
*         | # of plants:          1                       |
*         | Sensors Connected:    4/5                     |
*         |   Humidity        -- ok                       |
*         |   Temperature     -- ok                       |
*         |   Soil Moisture   -- ok                       |
*         |   Light Intensity -- ok                       |
*         |   Water Flow      -- error, not connected     |
*         | Sensor Data:          Saved to SD card        |
*         |   Humidity        -- %                        |
*         |   Temperature     -- Fahrenheit               |         
*         |   Heat Index      -- Fahrenheit               |
*         |   Soil Moisture   -- ?                        |
*         |   Light Intensity -- lux                      |
*         |   Water Flow      -- lps                      |                   
*         |   Water Used      -- Liters or Gallons        |
*         |   Water Tank      -- Liters or Gallons        |
*         | Picture:              Timelapse               |
*         | Graphs:               Select Timeframe        |
*         |   Past 24 Hours                               |
*         |   Past 2 Weeks                                |
*         |   Past Month                                  |
*         |   Full Length                                 |
*       Sensors           <-- Modular and Wireless          
*         | Humidity          -- DHT11 Humidity & Temp Sensor              |
*         | Temperature       -- DHT11 Humidity & Temp Sensor              |
*         |                      Amazon Digital Temp Sensor                |
*         | Soil Moisture     -- Sparkfun Soil Moisture Sensor             |
*         | Lights            -- Photocell Diode                           |
*         | Water Flow        -- Adafruit Water Flow Sensor                |
*         | Camera            -- FLIR, Webcam                              |
*         | Display Interface -- Joystick, Button and Text                 |                                 
*         | Water pH          -- eTape Chemical Sensor (Need to purchase)  |
*         | Soil pH           -- (Need to Purchase)                        |
*         | pH Level          -- (Need to purchase)                        |
*         | Nutrients         -- (Need to purchase)                        |
*         |                      Foliar Feeding or Injection to Water line |
*  
*   - Outputs (Controls and Displays)
*       Automation        <-- Defined in the Settings
*         | 1. Light Cycle and Intensity Controls
*         |     LED's           -- Light & Temperature Sensors  |
*         | 2. Fan Controls                                     |
*         |     Computer Fan    -- Humidity & Temperature       |
*         | 3. Water Controls                                   |
*         |     Solenoid Valve  -- Soil Moisture                |
*         | 5. [Nutrient Controls]                              |
*         | 6. [pH Controls]                                    
*       Micro OLED        <-- [System Information]
*         | Internet Connection:  Yes (If No, info is saved on SD)                  |
*         | Datalogging:          Yes                                               | 
*         | Plant Groups:         5                                                 |
*         |   1. Cannabis                                                           |
*         |   2. Succulents                                                         |
*         |   3. Jalapenos/ Veggies                                                 |
*         |   4. Herbs                                                              |
*         |   5. Houseplants                                                        |
*         | Group 1:              Name (each plant/section has a unique Portfolio)  |
*         | Strain:               Name                                              |
*         | Settings:             Preset 1/ mySettings 1                            |
*         | # of plants:          1                                                 |
*         | Sensors Connected:    4/5 (display errors if any)                       |
*         |   Humidity        -   ok                                                |
*         |   Temperature     -   ok                                                |
*         |   Soil Moisture   -   ok                                                |
*         |   Light Intensity -   ok                                                |
*         |   Water Flow      -   error, not connected                              |
*         | Sensor Data:          Averaged over 30 second intervals                 |
*         |   Humidity        -   %                                                 |
*         |   Temperature     -   Fahrenheit & Heat Index                           |
*         |   Soil Moisture   -                                                     |
*         |   Light Intensity -   lux                                               |
*         |   Water Flow      -   lps                                               |
*       Wireless Hardware <--                                                       |
*         | Bluetooth chip:   Connects the Modular Devices, broadcast Website Link  |
*         | WiFi chip:        Connects the System with the Internet                 |
*       Wireless Software <-- Any software used in the project
*         | Dweet.io          - https://dweet.io/
*         | Freeboard.io      - https://freeboard.io/board/Rqzu2j
*         | Sparkfun Phant    - https://learn.sparkfun.com/tutorials/pushing-data-to-datasparkfuncom/what-is-phant
*         
*       Data Logs         <-- Each Plant/Group has a Portfolio
*         | 1. Plant/Strain Name                                |
*         | 2. Log # (auto check for last log)                  |
*         | 3. Date                                             |
*         | 4. Setting Name                                     |
*         | 5. Picture                                          |
*         | 6. Data (last 24 hrs, 1 month, beggining of time)   |
*         | 7. Notes                                            |
*                             
*******************************************************************
*  To-do:  Add User Input for initial setup Configuration
*          Add Inputs (Sensors and Settings)
*            - Digital Temp sensor (from amazon)
*            - [Nutrients Portfolio]
*            - [Connect 5 led level display for each sensor (7x = 35 leds)]
*              -- Average range of values
*          Add Outputs (Controls, Communications and Displays)
*            - Light Controls
*              -- Linked with Light Intensity and settings
*              -- Add if/else statements to integrate light schedule
*              -- Preset Cycle Settings
*            - Fan Controls
*              -- Linked with Humidity and Temp
*              -- Add if/else statements to integrate fan schedule
*              -- Preset Ideal Heat Index (humidity & temp) Settings
*            - Water Controls
*              -- Linked with Soil Moisture
*              -- Add if/else statements to integrate watering schedule
*              -- Preset Ideal Water Thresholds Settings
*              -- Adjust for soil
*            - [Nutrient Controls]
*              -- Linked with [Nutrient Sensor]
*              -- Preset Strain Nutrient Recipes
*            - Communication (BLE)
*              -- Sensor Data Transmission
*              -- API
*              -- GUI 
*            - Graphs and Displays
*              -- Light Intensity vs Time
*              -- Humidity, Temp, Fan Cycles vs Time
*              -- Soil Moisture vs Time
*              -- Average range of values
*          Update entire code to Feather M0, SAMD21 support
*          
*
*       MIT license, check LICENSE for more information
*       All text above, and the splash screen below must be included in
*       any redistribution
*********************************************************************/

/*********************************************************************
* void setup() {
*   // Initialize Input Sensors
*     // Camera
*     // Temperature (Digital Temp)
*     // Humidity (Digital)
*     // Soil Moisture (Analog)
* 
*     // Light Sensor (Analog)
*     // Water Level Sensor ()
*     // pH Sensor (Digital)
* }
* 
* void loop() {
*   // Read Sensors
*   // Display Sensor Readings (Micro OLED)
*   // Transmit Sensor Readings (Bluetooth)
*   // Output Controls
*     // Light Controls
*     // Water Controls
*     // Fan Controls
* }
*********************************************************************/


#include <Arduino.h>
#include <SPI.h>
#if not defined (_VARIANT_ARDUINO_DUE_X_) && not defined (_VARIANT_ARDUINO_ZERO_)
  #include <SoftwareSerial.h>
#endif

#include <Wire.h>                       // Adafruit I2C Library
#include <Adafruit_Sensor.h>            // Adafruit Sensor Library

#include "DHT.h"                        // DHT11 Sensor Library

// Adafruit Bluefruit Libraries
#include "Adafruit_BLE.h"               // Adafruit Bluetooth Library
#include "Adafruit_BluefruitLE_SPI.h"   // Adafruit Bluefruit I2C Definition
#include "Adafruit_BluefruitLE_UART.h"  // Adafruit Bluefruit UART Definition


#include "BluefruitConfig.h"            // Adafruit Bluefruit Config file

/*=========================================================================
    APPLICATION SETTINGS

    FACTORYRESET_ENABLE     Perform a factory reset when running this sketch
   
                            Enabling this will put your Bluefruit LE module
                            in a 'known good' state and clear any config
                            data set in previous sketches or projects, so
                            running this at least once is a good idea.
   
                            When deploying your project, however, you will
                            want to disable factory reset by setting this
                            value to 0.  If you are making changes to your
                            Bluefruit LE device via AT commands, and those
                            changes aren't persisting across resets, this
                            is the reason why.  Factory reset will erase
                            the non-volatile memory where config data is
                            stored, setting it back to factory default
                            values.
       
                            Some sketches that require you to bond to a
                            central device (HID mouse, keyboard, etc.)
                            won't work at all with this feature enabled
                            since the factory reset will clear all of the
                            bonding data stored on the chip, meaning the
                            central device won't be able to reconnect.
    -----------------------------------------------------------------------*/
    #define FACTORYRESET_ENABLE      1
    #define MINIMUM_FIRMWARE_VERSION    "0.6.7"
    #define URL                         "http://www.freeboard.io"
/*=========================================================================*/

// DEFINITIONS
/*
// Soil Moisture Sensor Definitions
int sensorPin1 = A0;      // sensorPin1 <-- Soil Moisture (Analog pin A0)

int thresholdUp = 400;    // High threshold that is used in the Water Cycle
int thresholdDown = 250;  // Low threshold that is used in the Water Cycle

// Photocell Light Intensity Definitions
int photocellPin = 1;     // the cell and 10K pulldown are connected to a1
int photocellReading;     // the analog reading from the sensor divider
int LEDpin = 3;           // connect Red LED to pin 3 (PWM pin)
int LEDbrightness;        // Used when controlling brightness

// DHT11 Humidity and Temperature Sensor Definitions
#define DHTPIN 2          // what digital pin we're connected to
#define DHTTYPE DHT11     // DHT 11 type

DHT dht(DHTPIN, DHTTYPE); // Initialize DHT sensor.

// DHT11 pinout
// Connect pin 1 (on the left) of the sensor to +5V
// NOTE: If using a board with 3.3V logic like an Arduino Due connect pin 1
// to 3.3V instead of 5V!
// Connect pin 2 of the sensor to whatever your DHTPIN is
// Connect pin 4 (on the right) of the sensor to GROUND
// Connect a 10K resistor from pin 2 (data) to pin 1 (power) of the sensor

// Water Flow Sensor Definitions
#define FLOWSENSORPIN 4   // Digital Pin 4 <-- Flow Sensor

// count how many pulses!
volatile uint16_t pulses = 0;
// track the state of the pulse pin
volatile uint8_t lastflowpinstate;
// you can try to keep time of how long it is between pulses
volatile uint32_t lastflowratetimer = 0;
// and use that to calculate a flow rate
volatile float flowrate;
// Interrupt is called once a millisecond, looks for any pulses from the sensor!
SIGNAL(TIMER0_COMPA_vect) {
  uint8_t x = digitalRead(FLOWSENSORPIN);
  
  if (x == lastflowpinstate) {
    lastflowratetimer++;
    return; // nothing changed!
  }
  
  if (x == HIGH) {
    //low to high transition!
    pulses++;
  }
  lastflowpinstate = x;
  flowrate = 1000.0;
  flowrate /= lastflowratetimer;  // in hertz
  lastflowratetimer = 0;
}

// Create useInterrupt function tool for Water Flow Sensor
void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
  }
}
*/

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

/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/
void setup(void)
{
  while (!Serial);          // required for Flora & Micro
  delay(500);               // Wait 0.5 sec before checking again

  Serial.begin(115200);     // Set the Baud Rate
  // System Information
  Serial.println(F("Automated Garden Code"));
  Serial.println(F("------------------------------------"));
  delay(500);               // wait for display to boot up

/*
  pinMode(FLOWSENSORPIN, INPUT);                  // Define digital pin 4 as input
  digitalWrite(FLOWSENSORPIN, HIGH);              // Set digital pin 4 to high
  lastflowpinstate = digitalRead(FLOWSENSORPIN);  // Set the read digital value to lastflowpinstate
  useInterrupt(true);
  
  dht.begin();
*/

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ){
      error(F("Couldn't factory reset"));
    }
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

// EddyStone commands are added from firmware 0.6.6
  if ( !ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    error(F("EddyStone is only available from 0.6.6. Please perform firmware upgrade"));
  }

  /* Set EddyStone URL beacon data */
  Serial.println(F("Setting EddyStone-url to Freeboard website: "));

  if (! ble.sendCommandCheckOK(F( "AT+EDDYSTONEURL=" URL ))) {
    error(F("Couldnt set, is URL too long !?"));
  }

  Serial.println(F("**************************************************"));
  Serial.println(F("Please use Google Physical Web application to test"));
  Serial.println(F("**************************************************"));
}

/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/
void loop(void)
{
  // Print user's option
  Serial.println(F("Please choose:"));
  Serial.println(F("0 : Disable EddyStone URL"));
  Serial.println(F("1 : Enable EddyStone URL"));
  Serial.println(F("2 : Put EddyStone URL to Config Mode"));

  // Get User's input
  char option[BUFSIZE+1];
  getUserInput(option, BUFSIZE);

  // Proccess option
  switch ( option[0] - '0' )
  {
    case 0:
      ble.sendCommandCheckOK(F("AT+EDDYSTONEENABLE=off"));
      break;

    case 1:
      ble.sendCommandCheckOK(F("AT+EDDYSTONEENABLE=on"));
      break;

    case 2:
      Serial.println(F("EddyStone config's mode is enabled for 300 seconds"));
      Serial.println(F("Please use Physical Web app to edit URL"));
      ble.sendCommandCheckOK(F("AT+EDDYSTONECONFIGEN=300"));
      break;

    default:
      Serial.print(F("Invalid input; "));
      Serial.println(option);
      break;
  }
/*
  // Wait a few seconds between measurements.
  delay(2000);
  // Add the reading # and timestamp to each reading measured

  /////////////////////////////////////////////////////////////////////
  // Reading # and Timestamp loop
  /////////////////////////////////////////////////////////////////////
  // Add the reading # and timestamp to each measurement recorded
  int reading;
  int i;
  String timestamp;
  String ts;
  
  timestamp = ("May 3, 2015 ");
  ts = timestamp;
     
  i = 1;
  reading = reading + i;
  
  Serial.print(reading);
  Serial.print("\t");
  Serial.println(ts);
  
  /////////////////////////////////////////////////////////////////////
  // Soil Moisture Sensor loop
  /////////////////////////////////////////////////////////////////////
  // Define sensorValue as the pin's analog value.
  int sensorValue1;
  sensorValue1 = analogRead(sensorPin1);
  
  // Print the Soil Moisture Level info.
  Serial.print("Soil Moisture Level: ");
  Serial.print(sensorValue1);
  Serial.print("\t");
  
  /////////////////////////////////////////////////////////////////////
  // Photocell Light Intensity loop
  /////////////////////////////////////////////////////////////////////
  photocellReading = analogRead(photocellPin);  
 
  // Print the raw analog light intensity value
  Serial.print("| LUX light intensity: ");
  Serial.print(photocellReading);     // the raw analog reading
  Serial.println("\t| ");
  // Convert to Lux
  // ??????????????
 
  // LED gets brighter the darker it is at the sensor
  // that means we have to -invert- the reading from 0-1023 back to 1023-0
  photocellReading = 1023 - photocellReading;
  //now we have to map 0-1023 to 0-255 since thats the range analogWrite uses
  LEDbrightness = map(photocellReading, 0, 1023, 0, 255);
  analogWrite(LEDpin, LEDbrightness);
 
  /////////////////////////////////////////////////////////////////////
  // DHT11 Humidity and Temperature loop
  /////////////////////////////////////////////////////////////////////
  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();
  // Read temperature as Fahrenheit (isFahrenheit = true)
  float f = dht.readTemperature(true);

  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  // Compute heat index in Fahrenheit (the default)
  float hif = dht.computeHeatIndex(f, h);
  // Compute heat index in Celsius (isFahreheit = false)
  float hic = dht.computeHeatIndex(t, h, false);
  
  // Print Humidity/Temp Info
  Serial.print("Humidity: ");
  Serial.print(h);
  Serial.print(" %\t");
  Serial.print("| Temperature: ");
  Serial.print(t);
  Serial.print(" *C ");
  Serial.print(f);
  Serial.print(" *F");
  Serial.print("| Heat index: ");
  Serial.print(hic);
  Serial.print(" *C ");
  Serial.print(hif);
  Serial.println(" *F");
  
  /////////////////////////////////////////////////////////////////////
  // Water Flow sensor loop
  /////////////////////////////////////////////////////////////////////
  // Print Water Flow sensor info
  Serial.print("Freq: "); Serial.print(flowrate); Serial.print(" \t\t");
  Serial.print("| Pulses: "); Serial.print(pulses, DEC); Serial.print(" \t\t\t");
  
  // if a plastic sensor use the following calculation
  // Sensor Frequency (Hz) = 7.5 * Q (Liters/min)
  // Liters = Q * time elapsed (seconds) / 60 (seconds/minute)
  // Liters = (Frequency (Pulses/second) / 7.5) * time elapsed (seconds) / 60
  // Liters = Pulses / (7.5 * 60)
  float liters = pulses;
  liters /= 7.5;
  liters /= 60.0;

  // Print the amount of liquid passed in liters
  Serial.print("| Liters: "); Serial.println(liters);
  
  /////////////////////////////////////////////////////////////////////
  // Light Intensity sensor loop
  /////////////////////////////////////////////////////////////////////
  
  /////////////////////////////////////////////////////////////////////
  // Water Resevoir Level sensor loop
  /////////////////////////////////////////////////////////////////////
  
*/
}

/**************************************************************************/
/*!
    @brief  Checks for user input (via the Serial Monitor)
*/
/**************************************************************************/
void getUserInput(char buffer[], uint8_t maxSize)
{
  memset(buffer, 0, maxSize);
  while( Serial.available() == 0 ) {
    delay(1);
  }

  uint8_t count=0;

  do
  {
    count += Serial.readBytes(buffer+count, maxSize);
    delay(2);
  } while( (count < maxSize) && !(Serial.available() == 0) );



}
