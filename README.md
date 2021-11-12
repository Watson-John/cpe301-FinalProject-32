# **CPE 301 Semester Project**

## Role Distribution

* Erin Marden  
     * Motor On/ Off
* Kenneth Escovilla
     * Water Level
* John Watson
     * Humidity and Temperature
* **Group**
    * LCD
    * Servo
    * Real Time Clock
    * LED
    * On/Off

## Links
 [Atmega 2560 Data Sheet](https://ww1.microchip.com/downloads/en/devicedoc/atmel-2549-8-bit-avr-microcontroller-atmega640-1280-1281-2560-2561_datasheet.pdf)
 
 [Arduino Pinout Diagram](http://domoticx.com/wp-content/uploads/2016/05/Arduino-MEGA-pinout.jpg)
 
## Quick Tips
```
  //set PB 0-3 to INPUT (input = 0)
  *ddr_b &= 0x0;

  //set PK 0-7 to OUTPUT (output = 1)
  *ddr_k = 0xFF;

   // enable the pullup resistor on PB 0 - 3
  *port_b |= 0xFF;
```
```
// if the pin is high
  if(*pin_b & 0x10){
       
  }
 ```
 
