#include <Arduino.h>
#include <Servo.h>
#include <LiquidCrystal.h>
#include <DS3231.h>
#include <dht_nonblocking.h>

// Define ADC 

volatile unsigned char* my_ADMUX = 0x7C;
volatile unsigned char* my_ADCSRB = 0x7B;
volatile unsigned char* my_ADCSRA = 0x7A;
volatile unsigned int* my_ADC_DATA = 0x78;

// Define Port E Register Pointers
volatile unsigned char* port_e = (unsigned char*) 0x2E;
volatile unsigned char* ddr_e  = (unsigned char*) 0x2D;
volatile unsigned char* pin_e  = (unsigned char*) 0x2C;

// ~~ Port E ~~~
// Pin 2 - PE4 - Temperature and Humidity Input
#define DHT_SENSOR_TYPE DHT_TYPE_11
static const int DHT_SENSOR_PIN = 2;
DHT_nonblocking dht_sensor( DHT_SENSOR_PIN, DHT_SENSOR_TYPE );


// Pin 3 - PE5 - Water Level Sensor Input

// Pin 5 - PE3 - Servo ~library~

// Define Port H Register Pointers
volatile unsigned char* port_h = (unsigned char*) 0x102;
volatile unsigned char* ddr_h  = (unsigned char*) 0x101;
volatile unsigned char* pin_h  = (unsigned char*) 0x100;

// ~~ Port H ~~~
// Fan Motor:
// Pin 17 - PH5 - Enable from L293D
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
  *ddr_h |= 0x01 << 5; // Fan enable
  *ddr_h |= 0x01 << 4; // Fan
  *ddr_h |= 0x01 << 3; // Fan


  // setup the UART
  Serial.begin(9600);



}

void loop() {


  float temperature;
  float humidity;


  if ( getTemperature( &temperature, &humidity ) == true )
  {
    Serial.print( "T = " );
    Serial.println( temperature, 1 );

  }
  else{
    dc_fan();
  }


}

void dc_fan(){
//getTemp = true - in range ??
 float temperature;
 float humidity;

if ( getTemperature( &temperature, &humidity ) == true ){
  //Fan off\
  // 5 and 4 and 3 off
  *port_h &= 0x00;
  }
else{
//Fan on 
//5 and 4 on 3 off 
  *port_h |= 0x30;
}

 
 }






static bool getTemperature( float *temperature, float *humidity ){

    if ( dht_sensor.measure( temperature, humidity ) == true )
    {
      return ( true );
    }
  

  return ( false );
}












void write_ph(unsigned char pin_num, unsigned char state)
{
  if (state == 0)
  {
    *port_h &= ~(0x01 << pin_num);
  }
  else
  {
    *port_h |= 0x01 << pin_num;
  }
}


void adc_init()
{
  // set up the A register
  *my_ADCSRA |= 0b10000000; // set bit   7 to 1 to enable the ADC
  *my_ADCSRA &= 0b11011111; // clear bit 5 to 0 to disable the ADC trigger mode
  *my_ADCSRA &= 0b11110111; // clear bit 3 to 0 to disable the ADC interrupt
  *my_ADCSRA &= 0b11111000; // clear bit 2-0 to 0 to set prescaler selection to slow reading
  
  // set up the B register
  *my_ADCSRB &= 0b11110111; // clear bit 3 to 0 to reset the channel and gain bits
  *my_ADCSRB &= 0b11110111; // clear bit 2-0 to 0 to set free running mode
  
  // set up the MUX Register
  *my_ADMUX  &= 0b00000000; // clear bit 7 to 0 for AVCC analog reference
  *my_ADMUX  |= 0b01000000; // set bit   6 to 1 for AVCC analog reference
  *my_ADMUX  &= 0b11011111; // clear bit 5 to 0 for right adjust result
  *my_ADMUX  &= 0b11100000; // clear bit 4-0 to 0 to reset the channel and gain bits
}
unsigned int adc_read(unsigned char adc_channel_num)
{
  // reset the channel and gain bits
  *my_ADMUX &= 0b11100000;
  
  // clear the channel selection bits
  *my_ADCSRB &= 0b11110111;
  
  // set the channel number
  if(adc_channel_num > 7)
  {
    // set the channel selection bits, but remove the most significant bit (bit 3)
    adc_channel_num -= 8;
    
    // set MUX bit 
    *my_ADCSRB |= 0b00001000;
  }
  
  // set the channel selection bits
  *my_ADMUX  += adc_channel_num;
  
  // set bit ?? of ADCSRA to 1 to start a conversion
  *my_ADCSRA |= 0b01000000;
  
  // wait for the conversion to complete
  while(*my_ADCSRA & 0b000000000);
  
  // return the result in the ADC data register
  return *my_ADC_DATA;
}