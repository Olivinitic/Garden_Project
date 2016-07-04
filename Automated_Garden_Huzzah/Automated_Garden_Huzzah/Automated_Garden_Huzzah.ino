/*************************************************************************************
*   Project:      Automated Garden Monitor
*   Author:       Oliver Salmeron
*   Date Began:   May, 2016
*   Version:      1.1
*   
*   Description:  This program monitors and controls the parameters of a garden space. 
*                 All inputs and outputs are modular by design and can be selected as desired.
*   
*   AIO Username:   oligon
*   AIO Key:        9b9ec3768ccd4ebf83a67d2223e6e2b8
*   
*   Hardware Hook Up:
*     - Indicator LED
*       -- Digital Pin 2 -> 1k Resistor -> LED -> GND (Series)
*       -- LED: Positive Lead is the longer lead
*     - DHT11 Humidity and Temperature Sensor
*       -- 3.3V               -   pin 1
*       -- GND                -   pin 4
*       -- Digital Pin 3      -   pin 2
*     - Soil Moisture Sensor
*       -- 3.3V               -   Red Wire
*       -- GND                -   Black Wire
*       -- Analog Pin 0       -   Green Wire
*     - Photocell
*       -- 3.3V               -   Either End
*       -- Analog Pin 1       -   Opposite End + 5kOhm Pull Down Resistor (to GND)
*     - Digital Temperature Sensor
*       -- ??
*     - Water Flow Sensor
*       -- 5V Power Source    -   Red Wire
*       -- Digital Pin 4      -   Yellow Wire
*       -- GND                -   Black Wire
*     - Water Solenoid Valve
*       -- Digital Pin 5      -   Transistor: 12V -> Both Terminals
*     
*   Wireless Communication:
*     - Bluetooth (Feather)
*       -- Eddystone URL:   https://freeboard.io/board/Rqzu2j
*       -- Description:     The bluetooth nRF51 chip broadcasts the link above to any nearby devices. This website is where all the information is displayed visually.
*     - WiFi (HUZZAH)
*       -- IP Address:      
*       -- SSID:            
*       -- Password:        
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
*         | 6. [pH Controls]                                    |
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
*       Wireless Hardware <-- Any IoT hardware used in the project 
*         | Bluetooth chip:   Connects the Modular Devices, broadcast Website Link  |
*         | WiFi chip:        Connects the System with the Internet                 |
*       Wireless Software <-- Any IoT software used in the project
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

#include <ESP8266WiFi.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"

/*
// Adafruit Bluefruit Libraries
#include "Adafruit_BLE.h"               // Adafruit Bluetooth Library
#include "Adafruit_BluefruitLE_SPI.h"   // Adafruit Bluefruit I2C Definition
#include "Adafruit_BluefruitLE_UART.h"  // Adafruit Bluefruit UART Definition


#include "BluefruitConfig.h"            // Adafruit Bluefruit Config file
*/

// DEFINITIONS
/*  // Soil Moisture Sensor DEFINITIONS
  int soilMoisturePin = A0;      // soilMoisturePin     <-- Analog 0 Data Signal 
  int soilMoistureControl = 4;   // soilMoistureControl <-- Digital 4 Control Signal
  
  int thresholdUp = 400;         // High threshold that is used in the Water Cycle
  int thresholdDown = 250;       // Low threshold that is used in the Water Cycle
  
  // Photocell DEFINITIONS       
  int photocellPin = 1;         // photocellPin      <-- Analog 1 Data Signal
  int photocellReading;          // photocellReading  <--the analog reading from the sensor divider
  int photocellControl = 8;      // photocellControll <-- Digital 8 Control Signal
*/
  // DHT11 Humidity and Temperature Sensor DEFINITIONS
  #define DHTPIN 4         // what digital pin we're connected to
  #define DHTTYPE DHT11     // DHT 11 type
  //#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
  //#define DHTTYPE DHT21   // DHT 21 (AM2301)
  
  DHT dht(DHTPIN, DHTTYPE); // Initialize DHT sensor.
  
  // DHT11 pinout
  // Connect pin 1 (on the left) of the sensor to +5V
  // NOTE: If using a board with 3.3V logic like an Arduino Due connect pin 1
  // to 3.3V instead of 5V!
  // Connect pin 2 of the sensor to whatever your DHTPIN is
  // Connect pin 4 (on the right) of the sensor to GROUND
  // Connect a 10K resistor from pin 2 (data) to pin 1 (power) of the sensor
  // Photocell Light Intensity Definitions
/*  
  // LED Indicator Definitions
  int REDpin = 10;           // connect Red LED to Digital pin 10 (PWM pin)
  int REDbrightness;         // Used when controlling brightness
  
  int GRNpin = 11;           // connect Green LED to Digital pin 11 (PWM pin)
  int GRNbrightness;         // Used when controlling brightness
  
  // Water Solenoid Valve Definition
  int solenoidPin = 2;       // Digital Pin 2 on Arduino that controls the Transistor


  // Water Flow Sensor Definitions
  int flowPin = 2;    //This is the input pin on the Arduino
  double flowRate;    //This is the value we intend to calculate. 
  volatile int count; //This integer needs to be set as volatile to ensure it updates correctly during the interrupt process.  
*/
  /*
  #define FLOWSENSORPIN 7  // Digital Pin 7 <-- Flow Sensor
  
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
*/

// WiFi DEFINITIONS
#define WLAN_SSID       "HOME-0057-2.4"
#define WLAN_PASS       "award8619eighty"

// Adafruit IO
#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    "oligon"
#define AIO_KEY         "9b9ec3768ccd4ebf83a67d2223e6e2b8"


// FUNCTIONS
void connect();

// Create an ESP8266 WiFiClient class to connect to the MQTT server.
WiFiClient client;

// Store the MQTT server, client ID, username, and password in flash memory.
const char MQTT_SERVER[] PROGMEM    = AIO_SERVER;

// Set a unique MQTT client ID using the AIO key + the date and time the sketch
// was compiled (so this should be unique across multiple devices for a user,
// alternatively you can manually set this to a GUID or other random value).
const char MQTT_CLIENTID[] PROGMEM  = AIO_KEY __DATE__ __TIME__;
const char MQTT_USERNAME[] PROGMEM  = AIO_USERNAME;
const char MQTT_PASSWORD[] PROGMEM  = AIO_KEY;

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.
Adafruit_MQTT_Client mqtt(&client, MQTT_SERVER, AIO_SERVERPORT, MQTT_CLIENTID, MQTT_USERNAME, MQTT_PASSWORD);

/****************************** Feeds ***************************************/

// Setup feeds for temperature & humidity
const char TEMPERATURE_FEED[] PROGMEM = AIO_USERNAME "/feeds/temperature";
Adafruit_MQTT_Publish temperature = Adafruit_MQTT_Publish(&mqtt, TEMPERATURE_FEED);

const char HUMIDITY_FEED[] PROGMEM = AIO_USERNAME "/feeds/humidity";
Adafruit_MQTT_Publish humidity = Adafruit_MQTT_Publish(&mqtt, HUMIDITY_FEED);

/* 
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
/*
void Flow()
{
   count++; //Every time this function is called, increment "count" by 1
}
*/

void setup(void)
  {
    while (!Serial);          // required for Flora & Micro
    //delay(2000);               // Wait 0.5 sec before checking again
  
    Serial.begin(115200);     // Set the Baud Rate
    
    // System Information
    Serial.println(F("Garden Code Test!"));
    Serial.println(F(""));

    // Connect to WiFi access point.
    Serial.println(); Serial.println();
    delay(10);
    Serial.print(F("Connecting to "));
    Serial.println(WLAN_SSID);

    WiFi.begin(WLAN_SSID, WLAN_PASS);
    while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(F("."));
  }
    Serial.println();

    Serial.println(F("WiFi connected"));
    Serial.println(F("IP address: "));
    Serial.println(WiFi.localIP());

    // connect to adafruit io
    connect();
  
    // Print the Data String Format referance
    Serial.println("Moisture,Light,WaterValve,WaterUsed,REDIndicator,GRNIndicator,Humidity,Temperature,HeatIndex,");
    Serial.println("--------------------------------------------------------------------------------------------");
    //delay(500);               // wait for display to boot up
  /*
    // Soil Moisture setup
    pinMode(soilMoistureControl, OUTPUT);           // Sets Digital 4 pin as Output

    // Photocell setup
    pinMode(photocellControl, OUTPUT);              // Sets Digital 8 pin as Output
*/
    // DHT11 setup
    dht.begin();
/*
    // Water Solenoid Valve setup
    pinMode(solenoidPin, OUTPUT);                   // Sets Digital 2 pin as output
    
    // Water Flow Sensor setup
    pinMode(flowPin, INPUT);           //Sets the pin as an input
    attachInterrupt(0, Flow, RISING);  //Configures interrupt 0 (pin 2 on the Arduino Uno) to run the function "Flow"  

/*
    pinMode(FLOWSENSORPIN, INPUT);                  // Define digital pin 7 as input
    digitalWrite(FLOWSENSORPIN, HIGH);              // Set digital pin 7 to high
    lastflowpinstate = digitalRead(FLOWSENSORPIN);  // Set the read digital value to lastflowpinstate
    useInterrupt(true);
 

    // LED Indicator setup
    pinMode(GRNpin, OUTPUT);              // Sets Digital 11 pin as Output
*/
  }
  
void loop(void)
  {
   
    // Wait a few seconds between measurements.
    delay(2000);

    
  // ping adafruit io a few times to make sure we remain connected
    if(! mqtt.ping(3)) {
      // reconnect to adafruit io
      if(! mqtt.connected())
        connect();
  }

    // Grab the current state of the sensor
    int humidity_data = (int)dht.readHumidity();
    int temperature_data = (int)dht.readTemperature();

    // Publish data
    if (! temperature.publish(temperature_data))
      Serial.println(F("Failed to publish temperature"));
    else
      Serial.println(F("Temperature published!"));
      Serial.println(F(temperature_data));

    if (! humidity.publish(humidity_data))
      Serial.println(F("Failed to publish humidity"));
    else
      Serial.println(F("Humidity published!"));
      Serial.println(F(humidity_data));

    // Repeat every 10 seconds
    delay(10000);

    /////////////////////////////////////////////////////////////////////
    // Reading # and Timestamp loop
    /////////////////////////////////////////////////////////////////////
    // Add the reading # and timestamp to each reading measured
    Serial.println("--------------------------------------------------------------------------------------------");

    /*
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
    Serial.print("Humidity: \t\t");
    Serial.print(h);
    Serial.println(" %");           // Humidity Units
    Serial.print("Temperature: \t\t");
    Serial.print(t);
    Serial.print(" *C \t");           // Temperature Units
    Serial.print(f);
    Serial.println(" *F");            // Temperature Units
    Serial.print("Heat index: \t\t");
    Serial.print(hic);              // Heat Index Units
    Serial.print(" *C \t");
    Serial.print(hif);
    Serial.println(" *F");          // Heat Index Units
    
    /////////////////////////////////////////////////////////////////////
    // Soil Moisture Sensor loop
    /////////////////////////////////////////////////////////////////////
    // Define sensorValue as the pin's analog value.
    int soilMoistureReading;
    digitalWrite(soilMoistureControl, HIGH);            // Switch Sensor On
    delay(1000);                                        // Wait for Sensor to charge
    soilMoistureReading = analogRead(soilMoisturePin);  // Take Sensor Reading
    digitalWrite(soilMoistureControl, LOW);             // Switch Sensor Off
    
    // Print the Soil Moisture Level info.
    Serial.print("Soil Moisture Level: \t");
    Serial.println(soilMoistureReading);
    
    /////////////////////////////////////////////////////////////////////
    // Photocell Light Intensity loop
    /////////////////////////////////////////////////////////////////////
    digitalWrite(photocellControl, HIGH);           // Switch Sensor On
    delay(1000);                                    // Wait for Sensor to charge
    photocellReading = analogRead(photocellPin);    // Take Sensor Reading
    //digitalWrite(photocellControl, LOW);            // Switch Sensor Off
   
    // Print the raw analog light intensity value
    Serial.print("Analog light reading: \t");
    Serial.print(photocellReading);                 // Print the raw analog reading
    // We'll have a few threshholds, qualitatively determined
    if (photocellReading < 10) {
      Serial.println(" - Dark");
    } else if (photocellReading < 200) {
      Serial.println(" - Dim");
    } else if (photocellReading < 500) {
      Serial.println(" - Light");
    } else if (photocellReading < 800) {
      Serial.println(" - Bright");
    } else {
      Serial.println(" - Very bright");
    }
    
    /////////////////////////////////////////////////////////////////////
    // Water Solenoid Valve loop
    /////////////////////////////////////////////////////////////////////
    digitalWrite(solenoidPin, HIGH);    //Switch Solenoid ON
    delay(1000);                        //Wait 1 Second
    Serial.println("Valve: \t\t\tOpen");
    
    digitalWrite(solenoidPin, LOW);     //Switch Solenoid OFF
    delay(1000);                        //Wait 1 Second
    Serial.println("Valve: \t\t\tClose");

    /////////////////////////////////////////////////////////////////////
    // Water Flow sensor loop
    /////////////////////////////////////////////////////////////////////
    count = 0;      // Reset the counter so we start counting from 0 again
    interrupts();   //Enables interrupts on the Arduino
    delay (1000);   //Wait 1 second 
    noInterrupts(); //Disable the interrupts on the Arduino

    //Start the math
    flowRate = (count * 2.25);        //Take counted pulses in the last second and multiply by 2.25mL 
    flowRate = flowRate * 60;         //Convert seconds to minutes, giving you mL / Minute
    flowRate = flowRate / 1000;       //Convert mL to Liters, giving you Liters / Minute

    Serial.print("Flow Rate: \t\t");
    Serial.println(flowRate);         //Print the variable flowRate to Serial
    delay(1000);
   
    /*
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
    ////
    
    /////////////////////////////////////////////////////////////////////
    // LED Indicators loop
    /////////////////////////////////////////////////////////////////////
    // Red LED Indicator
    REDbrightness = map(photocellReading, 0, 1023, 0, 255);
    Serial.print("Red LED Brightness: \t");
    Serial.println(REDbrightness);
    analogWrite(REDpin,REDbrightness);
    Serial.println("Red LED:      Check");
    delay(1000);
    
    // Green LED Indicator
    digitalWrite(GRNpin, HIGH);       // Turn Green LED On
    Serial.println("Green LED:    On");
    delay(100000);
    
    digitalWrite(GRNpin, LOW);        // Turn Green LED Off
    Serial.println("Green LED:    Off");    
    delay(10000);
*/
    
    /*// Control the brightness of an LED in response to photocell
    // LED gets brighter the darker it is at the sensor
    // that means we have to -invert- the reading from 0-1023 back to 1023-0
    photocellReading = 1023 - photocellReading;
    //now we have to map 0-1023 to 0-255 since thats the range analogWrite uses
    LEDbrightness = map(photocellReading, 0, 1023, 0, 255);
    analogWrite(LEDpin, LEDbrightness);
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


// connect to adafruit io via MQTT
void connect() {

  Serial.print(F("Connecting to Adafruit IO... "));

  int8_t ret;

  while ((ret = mqtt.connect()) != 0) {

    switch (ret) {
      case 1: Serial.println(F("Wrong protocol")); break;
      case 2: Serial.println(F("ID rejected")); break;
      case 3: Serial.println(F("Server unavail")); break;
      case 4: Serial.println(F("Bad user/pass")); break;
      case 5: Serial.println(F("Not authed")); break;
      case 6: Serial.println(F("Failed to subscribe")); break;
      default: Serial.println(F("Connection failed")); break;
    }

    if(ret >= 0)
      mqtt.disconnect();

    Serial.println(F("Retrying connection..."));
    delay(5000);

  }

  Serial.println(F("Adafruit IO Connected!"));

}
