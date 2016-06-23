/*******************************************************************
*  Sensor:       Sparkfun's Soil Moisture Sensor
*  Author:       Sparkfun
*  Contributer:  Oliver
*  Description:  This is an example sketch to test the soil moisture sensor.
*
*  To-do:        Add if/else statements to integrate watering schedule
*                
*******************************************************************/

// Here we are setting up some water thersholds that we will 
// use later. Note that you will need to change these to match
// your soil type and environment. 

int thresholdUp = 400;
int thresholdDown = 250;

// We are setting up the pin A0 on the redboard to be our sensor
// pin input:

int sensorPin = A0;


void setup(){
  mySerial.begin(9600); // set up serial port for 9600 baud (speed)
  delay(500); // wait for display to boot up
}

void loop() {
  // We need to set up a pin to get the value that the soil 
  // moisture sensor is outputting, so sensorValue will get the
  // analog value from the sensor pin A0 on the redboard that we 
  // set up earlier.

  int sensorValue;
  sensorValue = analogRead(sensorPin);

  Serial.print("Soil Moisture Level: ");
  Serial.println(sensorValue);
}
