/*************************************************************************************
*   Project Name:   Water Solenoid Valve
*   Author:         Oliver Salmeron
*   Date Began:     May, 2016
*   
*   Description:  This program monitors and controls the parameters of a garden space. 
*                 All inputs and outputs are modular by design and can be selected as desired.
*   
*   AIO Username:   oligon
*   AIO Key:        9b9ec3768ccd4ebf83a67d2223e6e2b8
*   
*   Hardware Hook Up:
*     - Water Solenoid Valve
*       -- Digital Pin 5      ->  1.2K Ohm + TIP120 Transistor
*       -- TIP120 Transistor  ->  Base: Digital 5, Emitter: Ground, Collector: Solenoid + Diode in parallel to 5V
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
*************************************************************************************/

int solenoidPin = 2;    //This is the output pin on the Arduino we are using

void setup() {
  Serial.begin(115200);     // Set the Baud Rate
  
  // put your setup code here, to run once:
  pinMode(solenoidPin, OUTPUT);           //Sets the pin as an output
}

void loop() {
  // put your main code here, to run repeatedly:  
  digitalWrite(solenoidPin, HIGH);    //Switch Solenoid ON
  Serial.println("on");
  delay(10000);                      //Wait 1 Second
  digitalWrite(solenoidPin, LOW);     //Switch Solenoid OFF
  Serial.println("off");
  delay(10000);                      //Wait 1 Second
}
