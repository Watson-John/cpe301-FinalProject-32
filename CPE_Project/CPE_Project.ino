#include <Arduino.h>
#include <Servo.h>
#include <LiquidCrystal.h>
#include <DS3231.h>
#include <dht_nonblocking.h>

// ~~~~~~~~~~~~~~~~~~~~~ Define ADC ~~~~~~~~~~~~~~~~~~~~~~~~
volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// ~~~~~~~~~~~ Define Port E Register Pointers ~~~~~~~~~~~~~
// Temperature and Humidity Input ~ Pin 2 - PE4
volatile unsigned char* port_e = (unsigned char*) 0x2E;
volatile unsigned char* ddr_e  = (unsigned char*) 0x2D;
volatile unsigned char* pin_e  = (unsigned char*) 0x2C;

#define DHT_SENSOR_TYPE DHT_TYPE_11
static const int DHT_SENSOR_PIN = 2;
DHT_nonblocking dht_sensor( DHT_SENSOR_PIN, DHT_SENSOR_TYPE );

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// ~~~~~~~~~~~ Define Port H Register Pointers ~~~~~~~~~~~~~
// Fan Motor:
// Pin 17 - PH5 - Enable from L293D
// Pin 16 - PH4 - In 1 OutPut
// Pin 15 - PH3 - In 2 Output
volatile unsigned char* port_h = (unsigned char*) 0x102;
volatile unsigned char* ddr_h  = (unsigned char*) 0x101;
volatile unsigned char* pin_h  = (unsigned char*) 0x100;
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// ~~~~~~~~~~~ Define Port L Register Pointers ~~~~~~~~~~~~~
// RGB LED Control
volatile unsigned char* port_L = (unsigned char*) 0x10B;
volatile unsigned char* ddr_L  = (unsigned char*) 0x10A;
volatile unsigned char* pin_L  = (unsigned char*) 0x109;

int Yellow = 5; // 44
int Green = 4; // 45
int Blue = 3; // 46
int Red = 2; // 47
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// ~~ PWM Set Up ~~
// Timer/ Counter 5 Register A ~  Pointer to Timer 5
volatile unsigned char* TCCR_5A  = (unsigned char*) 0x120;

// Output Compare Pins ~ Turns on Alternatve Functionality ↓
int COM_5_B1 = 5;
// Wave Form Generation Mode ~ Turns on PWM in 8 bit mode (mode 1);
int WGM_5_1 = 1;

// Output Compare Register 5 B ~ This is where we store the Duty Cycle
volatile unsigned char* OCR_5B  = (unsigned char*) 0x12A;
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// ~~~~~~~~~~~~~~~~~~~ Analog Inputs ~~~~~~~~~~~~~~~~~~~~~
// Water Level Sensor Input
int water_level_pin = 0;

// Servo Potentiometer
int servo_pot_pin = 8;
Servo myservo;
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// ~~~~~~~~~~~~~~~~~~~ Define LCD Pins ~~~~~~~~~~~~~~~~~~~~~
// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(22, 23, 24, 25, 26, 27);
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// ~~~~~~~~~~~~~~~~~~~ Global Thresholds ~~~~~~~~~~~~~~~~~~~~~
float temperature_threshold = 20.0;
int   water_level_threshold = 400;
int water_level;

enum all_states {IDLE, DISABLED, ERROR, RUNNING};
volatile all_states state = DISABLED;
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


// ~~~~~~~~~~~~~~~~~~~ Real Time Clock ~~~~~~~~~~~~~~~~~~~~~
DS3231 clock;
RTCDateTime dt;
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

// ~~~~~~~~~~~~~~~~~~~ Button  ~~~~~~~~~~~~~~~~~~~~~
int buttonPushCounter = 0;   // counter for the number of button presses
int buttonState = 0;         // current state of the button
int lastButtonState = 0;

int button_pin = 5; // 3
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


// ~~~~~~~~~~~~~~~~~~~ Pot Button  ~~~~~~~~~~~~~~~~~~~~~
unsigned long lastDebounceTime = 0; 
unsigned long debounceDelay = 50;

int pot_lastButtonState;
int pot_buttonState;

int pot_button_pin = 7; // 42
int counter = 0;
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~





void setup() {

  // ~~ Port E ~~
  //    Inputs
  //*ddr_e |= 0x0 << 4; // Temperature
  *port_e |= 0x1 << button_pin; // Button Pin w/ Pull Up
  //    Outputs
  *ddr_e |= 0x1 << 3; // Servo

  // ~~ Port H ~~
  //    Outputs
  *ddr_h |= 0x01 << 5; // Fan enable
  *ddr_h |= 0x01 << 4; // Fan
  *ddr_h |= 0x01 << 3; // Fan

  // ~~ Port L ~~
  //    Outputs
  *ddr_L |= 0x01 << Red;
  *ddr_L |= 0x01 << Green;
  *ddr_L |= 0x01 << Blue;
  *ddr_L |= 0x01 << Yellow;
  //    Inputs
  *port_L |= 0x1 << pot_button_pin; // Button Pin w/ Pull Up


  // setup the LCD
  lcd.begin(16, 2);

  // setup Servo
  myservo.attach(5);

  // setup the ADC
  adc_init();

  // setup the UART
  Serial.begin(9600);

  // setup Clock
  clock.begin();
  clock.setDateTime(__DATE__, __TIME__);
}


void loop() {

  float temperature;
  float humidity;


  switch (state) {
    case IDLE:

      //Serial.println("Idle");
      LED_color('g');


      // Servo Active
      pot_to_servo(1);

      // Motor OFF
      dc_fan(0);

      getTemperature( &temperature, &humidity);
      water_level = adc_read(water_level_pin);
      getButtonPress();

      print_temp_humid(temperature, humidity);

      //      ~~ Exit Condition ~~
      if ( temperature > temperature_threshold) {
        state = RUNNING;
      } else if ( water_level < water_level_threshold) {
        state = ERROR;
      } else if (buttonState == 0) {
        state = DISABLED;
      }


      break;

    case DISABLED:
      // Serial.println("Disabled");
      lcd.clear();
      LED_color('y');

      // Servo Disabled
      pot_to_servo(0);

      // Motor OFF
      dc_fan(0);

      getButtonPress();
      if (buttonState == 0) {
        state = IDLE;
      }

      break;
    case RUNNING:
      //Serial.println("Running");
      LED_color('b');

      // Servo Active
      pot_to_servo(1);

      //Motor ON
      dc_fan(1);

      getTemperature( &temperature, &humidity);
      water_level = adc_read(water_level_pin);
      getButtonPress();

      print_temp_humid(temperature, humidity);

      //Serial.println( water_level);
      // ~~ Exit Condition ~~
      if ( temperature <= temperature_threshold) {
        state = IDLE;

      } else if ( water_level < water_level_threshold) {
        state = ERROR;

      } else if (buttonState == 0) {
        state = DISABLED;
      }

      break;
    case ERROR:
      //Serial.println("Error");
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(" ERROR ");
      lcd.setCursor(0, 1);
      lcd.print("Water Empty :(   ");
      LED_color('r');

      // Servo Active
      pot_to_servo(1);

      // Motor OFF
      dc_fan(0);


      getTemperature( &temperature, &humidity);
      water_level = adc_read(water_level_pin);
      getButtonPress();

      // ~~ Exit Condition ~~
      if ( water_level > water_level_threshold) {
        state = IDLE;
      } else if (buttonState == 0) {
        state = DISABLED;
      }

      break;
  }
}


// Get Debounced Button Press
void getButtonPress() {

  if (*pin_e & (1 << button_pin)) {
    buttonState = 1;
  } else {
    buttonState = 0;
  }

  if (buttonState != lastButtonState) {

    if (buttonState == HIGH) {
      buttonPushCounter++;
    }
    delay(75);
  }
  lastButtonState = buttonState;

}

//  ~~ Display Temp/Humidity ~~
void print_temp_humid(float temperature, float humidity) {

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Temp: ");
  lcd.print(temperature);
  lcd.setCursor(0, 1);
  lcd.print("Humidity: ");
  lcd.print(humidity);

}

bool lastState;

// ~~ DC Fan Code ~~
void dc_fan(bool state) {

  if ( state  == false ) {
    Serial.print("Motor Off: ");
    // 5 and 4 and 3 off
    *port_h &= 0x00;

  lastState = false;
    
  } else {
     Serial.print("Motor On: ");
    //5 and 4 on 3 off
    *port_h |= 0x30;
      lastState = true;
  }

    dt = clock.getDateTime();
    Serial.print(dt.year);   Serial.print("-");
    Serial.print(dt.month);  Serial.print("-");
    Serial.print(dt.day);    Serial.print(" ");
    Serial.print(dt.hour);   Serial.print(":");
    Serial.print(dt.minute); Serial.print(":");
    Serial.print(dt.second); Serial.println("");
    
}

// ~~ Potentiometer to Servo Enable/Disable ~~
void pot_to_servo(int state) {
  if (state == 1) {
    int reading;

    if (*pin_L & (1 << pot_button_pin)) {
      reading = 1;
    } else {
      reading = 0;
    }


    if (reading == 0) {

      if (counter < 180) {
        counter = counter + 60;
      } else if (counter = 180) {
        counter = 0;
      }
    }



    myservo.write(counter);

    delay(15);
  }

}




// ~~ DH11 Temperature Sensor ~~
static bool getTemperature( float * temperature, float * humidity ) {

  if ( dht_sensor.measure( temperature, humidity ) == true )
  {
    return ( true );
  }


  return ( false );
}

// ~~ RGB LED Color Code ~~
void LED_color(char color) {
  // r = Red
  // g = Green
  // b = Blue
  // y = Yellow

  write_pL(Red, 0);
  write_pL(Green, 0);
  write_pL(Blue, 0);
  write_pL(Yellow, 0);


  switch (color) {

    case 'r':
      write_pL(Red, 1);
      break;
    case 'g':
      write_pL(Green, 1);
      break;
    case 'b':
      write_pL(Blue, 1);
      break;
    case 'y':
      write_pL(Yellow, 1);
      break;

  }
  delay(75);
}
void write_pL(unsigned char pin_num, unsigned char state) {
  if (state == 0) {
    *port_L &= ~(0x01 << pin_num);

  } else {
    *port_L |= 0x01 << pin_num;

  }
}

// ~~ Analog to Digital Coversion - Water Level Sensor And Servo Potentiometer ~~
void adc_init() {
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
unsigned int adc_read(unsigned char adc_channel_num) {
  // reset the channel and gain bits
  *my_ADMUX &= 0b11100000;

  // clear the channel selection bits
  *my_ADCSRB &= 0b11110111;

  // set the channel number
  if (adc_channel_num > 7)
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
  while (*my_ADCSRA & 0b000000000);

  // return the result in the ADC data register
  return *my_ADC_DATA;
}