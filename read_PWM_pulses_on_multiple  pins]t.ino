/*
pulse_reader_w_pin_change_interrupt_singleCh.ino
-read in any pulsing signal on Arduino pin INPUT_PIN1 (defined below), to get its period (us) & freq (Hz), with a time resolution of 0.5us
--you can read in any pulse, including standard PWM signals, Radio Control (RC) PWM signals, etc. 
-I am using some low-level AVR code, which requires using some built-in Arduino macros to do pin-mapping. 
-this code only reads in a single channel at a time, though it could be expanded to read in signals on every Arduino pin, digital and analog, simultaneously.
--this would be lots of work, so for now I'll leave that up to you.
-this code should be able to read in any pulse between approximately 10~20us and 35.79 minutes; I'll let you experiment
 to find the actual shortest pulse you can measure with it

By Gabriel Staples
http://www.ElectricRCAircraftGuy.com/
-My contact info is available by clicking the "Contact Me" tab at the top of my website.
Written: 28 Nov. 2013
Upd1ated: 21 March 2015

Some References:
-to learn how to manipulate some of the low-level AVR code, pin change interrupts, etc, these links will help
--http://www.gammon.com.au/interrupts
--ATmega328 datasheet: http://www.atmel.com/Images/Atmel-8271-8-bit-AVR-Microcontroller-ATmega48A-48PA-88A-88PA-168A-168PA-328-328P_datasheet.pd1f
--http://playground.arduino.cc/Main/TimerPWMCheatsheet
--See the *many* very helpful links at bottom of this article: http://www.electricrcaircraftguy.com/2014/01/the-power-of-arduino.html
--reference the Arduino source code, ex:
---C:\Program Files (x86)\Arduino\hardware\arduino\avr\cores\arduino\Arduino.h
---C:\Program Files (x86)\Arduino\hardware\arduino\avr\cores\arduino\wiring_digital.c
---C:\Program Files (x86)\Arduino\hardware\arduino\avr\variants\standard\pins_arduino.h
*/

/*
Circuits:

Option 1) To measure the pulses from an Arduino PWM pin, ex: pin 5 or 9, connect Pin 5 or 9 (a PWM output) to INPUT_PIN1 (the pulse reader input)
          -see the setup() function for commanding the PWM output to begin, so you can have something to read in

Option 2) To measure an RC PWM servo-type signal coming from an RC Rx:
-Power the Rx by connecting 5V to + on the Rx, and GND to - on the Rx
-Connect the channel signal you want to measure on the Rx to the INPUT_PIN1 on the Arduino
*/

#include <eRCaGuy_Timer2_Counter.h>
#include "Wire.h"
//macros
#define fastDigitalRead(p_inputRegister, bitMask) ((*p_inputRegister & bitMask) ? HIGH : LOW)
// #define fastDigitalRead2(p_inputRegister2, bitMask2) ((*p_inputRegister2 & bitMask2) ? HIGH : LOW)

//Global Variables & defines
const byte INPUT_PIN1 = 2; //you can change this to ANY digital or analog pin, ex: 10, 8, A0, A5, etc, 
const byte INPUT_PIN2 = 8;
                           //EXCEPT A6 and A7 (which exists on the Nano and Pro Mini, for example, and are NOT capable of digital operations)
byte input_pin_bitMask1;
volatile byte* p_input_pin_register1;
byte input_pin_bitMask2;
volatile byte* p_input_pin_register2;
 
//volatile variables for use in the ISR (Interrupt Service Routine)
volatile boolean output_data1 = false; //the main loop will try to output data each time a new pulse comes in, which is when this gets set true
volatile boolean output_data2 = false; //the main loop will try to output data each time a new pulse comes in, which is when this gets set true
volatile unsigned long pulseCounts1 = 0; //units of 0.5us; the input signal high pulse time
volatile unsigned int pd1 = 0; //units of 0.5us; the pulse period (ie: time from start of high pulse to start of next high pulse)
volatile unsigned long pulseCounts2 = 0; //units of 0.5us; the input signal high pulse time
volatile unsigned int pd2 = 0; //units of 0.5us; the pulse period (ie: time from start of high pulse to start of next high pulse)

void setup() 
{
  pinMode(INPUT_PIN1,INPUT_PULLUP); //use INPUT_PULLUP to keep the pin from floating and jumping around when nothing is connected
  pinMode(INPUT_PIN2,INPUT_PULLUP); //use INPUT_PULLUP to keep the pin from floating and jumping around when nothing is connected
  //configure timer2
  timer2.setup();

  //prepare for FAST digital reads on INPUT_PIN1, by mapping to the input register (ex: PINB, PINC, or PIND), and creating a bitMask
  //using this method, I can do digital reads in approx. 0.148us/reading, rather than using digitalRead, which takes 4.623us/reading (31x speed increase)
  input_pin_bitMask1 = digitalPinToBitMask(INPUT_PIN1);
  p_input_pin_register1 = portInputRegister(digitalPinToPort(INPUT_PIN1));
  input_pin_bitMask2 = digitalPinToBitMask(INPUT_PIN2);
  p_input_pin_register2 = portInputRegister(digitalPinToPort(INPUT_PIN2));
  
  configurePinChangeInterrupts();
  
  
  
  
  Serial.begin(115200);
  Serial.print(F("Begin waiting for pulses on pin 1 ")); Serial.print(INPUT_PIN1);
  Serial.println(F(".\nData will be printed after each pulse is received."));
  Serial.print(F("Begin waiting for pulses on pin 2 ")); Serial.print(INPUT_PIN2);
  Serial.println(F(".\nData will be printed after each pulse is received."));
}

void loop() 
{
  //local variables
  static float pulseTime1 = 0; //us; the most recent input signal high pulse time
  static float pd1_us = 0; //us; the most recent input signal period between pulses
  static float pulseFreq1 = 0; //Hz, the most recent input signal pulse frequency
  static float pulseTime2 = 0; //us; the most recent input signal high pulse time
  static float pd2_us = 0; //us; the most recent input signal period between pulses
  static float pulseFreq2 = 0; //Hz, the most recent input signal pulse frequency
  
  if (output_data1==true) //if a pulse just came in
  {
    //turn off interrupts, grab copies of volatile data, and re-enable interrupts
    noInterrupts();
    output_data1 = false; //reset
    unsigned long pulseCounts2Copy1 = pulseCounts1; //0.5us units
    unsigned long pd1Copy = pd1; //0.5us units
    interrupts();
    
    //do calculations
    pulseTime1 = pulseCounts2Copy1/2.0; //us
    pd1_us = pd1Copy/2.0; //us
    pulseFreq1 = 1000000.0/pd1_us; //Hz
    
    //print values 
    //(optionally, add extra code here to not print after EVERY pulse is received, as this can result in serial data coming in excessively fast when pulses come in at a high freq)
    Serial.print(F("pulsetime(us) = ")); Serial.print(pulseTime1);
    Serial.print(F(", pd1_us(us) = ")); Serial.print(pd1_us);
    Serial.print(F(", pulseFreq1(Hz) = ")); Serial.println(pulseFreq1);
  }

  if (output_data2==true) //if a pulse just came in
  {
    //turn off interrupts, grab copies of volatile data, and re-enable interrupts
    noInterrupts();
    output_data2 = false; //reset
    unsigned long pulseCounts2Copy2 = pulseCounts2; //0.5us units
    unsigned long pd2Copy = pd2; //0.5us units
    interrupts();
    
    //do calculations
    pulseTime2 = pulseCounts2Copy2/2.0; //us
    pd2_us = pd2Copy/2.0; //us
    pulseFreq2 = 1000000.0/pd2_us; //Hz
    
    //print values 
    //(optionally, add extra code here to not print after EVERY pulse is received, as this can result in serial data coming in excessively fast when pulses come in at a high freq)
    Serial.print(F("pulsetime2(us) = ")); Serial.print(pulseTime2);
    Serial.print(F(", pd2_us(us) = ")); Serial.print(pd2_us);
    Serial.print(F(", pulseFreq2(Hz) = ")); Serial.println(pulseFreq2);
  }
delay(100);
} //end of loop()

////Use macro instead
//boolean fastDigitalRead(volatile byte* p_inputRegister,byte bitMask)
//{
//  return (*p_inputRegister & bitMask) ? HIGH : LOW;
//}

void pinChangeIntISR1()
{
  //local variables
  static boolean pin_state_new = LOW; //initialize
  static boolean pin_state_old = LOW; //initialize
  static unsigned long t_start = 0; //units of 0.5us
  static unsigned long t_start_old = 0; //units of 0.5us
  
  pin_state_new = fastDigitalRead(p_input_pin_register1,input_pin_bitMask1);
  if (pin_state_old != pin_state_new)
  {
    //if the pin state actualy changed, & it was not just noise lasting < ~2~4us
    pin_state_old = pin_state_new; //update the state
    if (pin_state_new == HIGH)
    {
      t_start = timer2.get_count(); //0.5us units
      pd1 = t_start - t_start_old; //0.5us units, the incoming pulse period
      t_start_old = t_start; //0.5us units; update
    }
    else //pin_state_new == LOW
    {
      unsigned long t_end = timer2.get_count(); //0.5us units
      pulseCounts1 = t_end - t_start; //0.5us units
      output_data1 = true;
    }
  }
}
void pinChangeIntISR2()
{
  //local variables
  static boolean pin_state_new = LOW; //initialize
  static boolean pin_state_old = LOW; //initialize
  static unsigned long t_start = 0; //units of 0.5us
  static unsigned long t_start_old = 0; //units of 0.5us
  
  pin_state_new = fastDigitalRead(p_input_pin_register2,input_pin_bitMask2);
  if (pin_state_old != pin_state_new)
  {
    //if the pin state actualy changed, & it was not just noise lasting < ~2~4us
    pin_state_old = pin_state_new; //update the state
    if (pin_state_new == HIGH)
    {
      t_start = timer2.get_count(); //0.5us units
      pd2 = t_start - t_start_old; //0.5us units, the incoming pulse period
      t_start_old = t_start; //0.5us units; update
    }
    else //pin_state_new == LOW
    {
      unsigned long t_end = timer2.get_count(); //0.5us units
      pulseCounts2 = t_end - t_start; //0.5us units
      output_data2 = true;
    }
  }
}


//PCINT0_vect is for pins D8 to D13
ISR(PCINT0_vect)
{
  pinChangeIntISR1();
}

//PCINT1_vect is for pins A0 to A5
ISR(PCINT1_vect)
{
  //pinChangeIntISR();
}

//PCINT2_vect is for pins D0 to D7
ISR(PCINT2_vect)
{
  pinChangeIntISR2();
}

void configurePinChangeInterrupts()
{
  //Pin Change Interrupt Configuration
  //see ATmega328 datasheet, ex: pgs. 73-75
  //also see: C:\Program Files (x86)\Arduino\hardware\arduino\avr\variants\standard\pins_arduino.h for the macros used here
  //1st, set flags on the proper Pin Change Mask Register (PCMSK)
  volatile byte* p_PCMSK1 = (volatile byte*)digitalPinToPCMSK(INPUT_PIN1); //pointer to the proper PCMSK register
  *p_PCMSK1= _BV(digitalPinToPCMSKbit(INPUT_PIN1));
  //2nd, set flags in the Pin Change Interrupt Control Register (PCICR)
  volatile byte* p_PCICR1 = (volatile byte*)digitalPinToPCICR(INPUT_PIN1); //pointer to PCICR
  *p_PCICR1 |= _BV(digitalPinToPCICRbit(INPUT_PIN1));

  volatile byte* p_PCMSK2 = (volatile byte*)digitalPinToPCMSK(INPUT_PIN2); //pointer to the proper PCMSK register
  *p_PCMSK2 = _BV(digitalPinToPCMSKbit(INPUT_PIN2));
  //2nd, set flags in the Pin Change Interrupt Control Register (PCICR)
  volatile byte* p_PCICR2 = (volatile byte*)digitalPinToPCICR(INPUT_PIN2); //pointer to PCICR
  *p_PCICR2|= _BV(digitalPinToPCICRbit(INPUT_PIN2));
}
