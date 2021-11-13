#include "DHT.h"

// Define Port E Register Pointers
volatile unsigned char* port_e = (unsigned char*) 0x2E; 
volatile unsigned char* ddr_e  = (unsigned char*) 0x2D; 
volatile unsigned char* pin_e  = (unsigned char*) 0x2C; 

// ~~ Port E ~~~
// Pin 2 - PE4 - Temperature and Humidity Input
// Pin 3 - PE5 - Water Level Sensor Input

// Pin 5 - PE3 - Servo ~library~

// Define Port H Register Pointers
volatile unsigned char* port_h = (unsigned char*) 0x102; 
volatile unsigned char* ddr_h  = (unsigned char*) 0x101; 
volatile unsigned char* pin_h  = (unsigned char*) 0x100; 

// ~~ Port H ~~~
// Fan Motor:
// Pin 16 - PH4 - In 1 OutPut
// Pin 15 - PH3 - In 2 Output





void setup() {

// ~~ Port E ~~ 
//    Inputs
*ddr_e |= 0x0 << 5; // Temperature
*ddr_e |= 0x0 << 4; // Water
//    Outputs
*ddr_e |= 0x1 << 3; // Servo

// ~~ Port H ~~ 
//    Outputs
*ddr_h |= 0x01 << 4; // Fan
*ddr_h |= 0x01 << 3; // Fan


// setup the UART
Serial.begin(9600);



}

void loop() {





}







int getTemperature(){











  
}







void write_ph(unsigned char pin_num, unsigned char state)
{
  if(state == 0)
  {
    *port_h &= ~(0x01 << pin_num);
  }
  else
  {
    *port_h |= 0x01 << pin_num;
  }
}
