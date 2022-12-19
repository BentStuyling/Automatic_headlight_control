#include <Arduino.h>
#include <SpeedyStepper.h>
#include <EEPROM.h>
#include <eRCaGuy_Timer2_Counter.h>

//macros
#define fastDigitalRead(p_inputRegister, bitMask) ((*p_inputRegister & bitMask) ? HIGH : LOW)

//Global Variables & defines for PWMREADING
const byte PWM_INPUT_PIN = A1; //you can change this to ANY digital or analog pin, ex: 10, 8, A0, A5, etc, 
                           //EXCEPT A6 and A7 (which exists on the Nano and Pro Mini, for example, and are NOT capable of digital operations)
byte input_pin_bitMask;
volatile byte* p_input_pin_register;
 
//volatile variables for use in the ISR (Interrupt Service Routine)
volatile boolean output_data = false; //the main loop will try to output data each time a new pulse comes in, which is when this gets set true
volatile unsigned long pulseCounts = 0; //units of 0.5us; the input signal high pulse time
volatile unsigned int pd = 0; //units of 0.5us; the pulse period (ie: time from start of high pulse to start of next high pulse)
//

// Define constants
const int Calibrate_pin = 2; 
const int MOTOR_STEP_PIN = 3;
const int MOTOR_DIRECTION_PIN = 4;
const float stroke = 11.5; // [mm]
const float holeToHoleDistance = 97.3; //[mm]
const int defaultStepperposition = 9; // [mm]
const float minimumStepsize = 0.10; // [mm] Corresponds to a Headlight tilt angle of 0.06 deg
float baseRearAngleSensorAngle; // [deg]
const int minimumRearCarAngle = 83;  // [deg]
const int maximumRearCarAngle = 97; // [deg]
const float ewmaAlpha = 0.90;  // the EWMA alpha value (α) for smoothing
float smoothedNewStepperPositionGoal = defaultStepperposition;
float carAngle = 0; // Set initial value for car Angle
long lastPWMSignalTime = 0; 


// create the stepper motor object
SpeedyStepper stepper;
//

// Functions:
float headlightAngleToStroke(float alpha) {
  // Convert stepper stroke into headlight tilt angle
  return tan(alpha/57.2958)*holeToHoleDistance;
}
void configurePinChangeInterrupts(); // call function before void setup

float rearPWMSenorCarAngle() {
  //local variables
  static float pulseTime = 0; //us; the most recent input signal high pulse time
  static float pd_us = 0; //us; the most recent input signal period between pulses
  static float dutyCycle = 0;
  long timeCurrentPWMread = micros();
  

  if (output_data==true) //if a pulse just came in
  {
    //turn off interrupts, grab copies of volatile data, and re-enable interrupts
    noInterrupts();
    output_data = false; //reset
    unsigned long pulseCountsCopy = pulseCounts; //0.5us units
    unsigned long pdCopy = pd; //0.5us units
    interrupts();
    
    //do calculations
    pulseTime = pulseCountsCopy/2.0; // Pulse HIGH time [us]
    pd_us = pdCopy/2.0; // Period of signal (HIGH+LOW) [us]
    dutyCycle = 100*(pulseTime/pd_us); // Duty cycle in [%]
    carAngle = 0.06*dutyCycle + 86; // Convert from dutycycle to angles
    lastPWMSignalTime = micros();
  }

  // Set limits to output values in case of faulty measurement
  if(carAngle < minimumRearCarAngle ){
    carAngle = maximumRearCarAngle; // Default to most conservative setting
    Serial.println(F("Car angle below minimum setpoint, default to max")); 
  }
  else if(carAngle > maximumRearCarAngle){
    carAngle = maximumRearCarAngle; // Default to most conservative realistic setting
    Serial.println(F("Car angle above maximum setpoint, default to max")); 
  }
  else if(isnan(carAngle)){ // Check if the calculated value is a NaN
    carAngle = maximumRearCarAngle; // Default to most conservative realistic setting
    Serial.println(F("No valid car angle value, default to max"));
  }  
  else if((timeCurrentPWMread - lastPWMSignalTime)> 3000000){
    carAngle = maximumRearCarAngle; // Default to most conservative realistic setting
    Serial.println(F("No new PWM signal received for 3s, default to max "));
  }
  else{
    carAngle = carAngle; // Just return the calculated value
  }
  return carAngle;
}

void setup() {
  Serial.begin(115200);
  Serial.println(F("Start setup void"));
  
  // Setup PWM Reaing
  pinMode(PWM_INPUT_PIN,INPUT_PULLUP); //use INPUT_PULLUP to keep the pin from floating and jumping around when nothing is connected
  //configure timer2
  timer2.setup();
  //prepare for FAST digital reads on INPUT_PIN, by mapping to the input register (ex: PINB, PINC, or PIND), and creating a bitMask
  //using this method, I can do digital reads in approx. 0.148us/reading, rather than using digitalRead, which takes 4.623us/reading (31x speed increase)
  input_pin_bitMask = digitalPinToBitMask(PWM_INPUT_PIN);
  p_input_pin_register = portInputRegister(digitalPinToPort(PWM_INPUT_PIN));
  configurePinChangeInterrupts();
  
  // connect and configure the stepper motor to its IO pins
  stepper.connectToPins(MOTOR_STEP_PIN, MOTOR_DIRECTION_PIN);
  stepper.setStepsPerMillimeter(60.75); // 1/2x microstepping, 0.033mm per step
  // set the speed and acceleration rates for the stepper motor
  stepper.setSpeedInStepsPerSecond(1000); //PPS Pulses Per Second [P/s]
  stepper.setAccelerationInStepsPerSecondPerSecond(6000); //Pulses per second per second [P/s^2]
  
  //Do full headlight vertical sweep to zero steppermotor. Then sweep up and to default startpoint.
  stepper.moveRelativeInMillimeters(-stroke-2); // Move full stroke down (in) plust 2 mmm extra for good measure
  delay(10);
  stepper.moveRelativeInMillimeters(0.5);
  delay(10);
  stepper.moveRelativeInMillimeters(-0.75);
  delay(10);
  stepper.setCurrentPositionInMillimeters(0.0); // Set current (full retracted) position to zero
  delay(5000);
  stepper.moveToPositionInMillimeters(stroke); // Move all the way up (out)
  delay(5100);

  // Read baseline rearaxleanglesensor value from eeprom. If calibration mode = Active, read value from sensor and store.
  int eeAddress = 0; //EEPROM address to read and put the default rear sensor value
  //float defaultRearAngleSensorValue = 0.00f;
  pinMode(Calibrate_pin, INPUT_PULLUP); //Set calibration pin to input and pull High

  if(digitalRead(Calibrate_pin) == HIGH) { // If the Calibration button is NOT pushed
    //Read  from EEPROM.
    EEPROM.get(eeAddress, baseRearAngleSensorAngle);
  }
  else {    // If the calibration buttin IS pushed, calibration mode active
    while (digitalRead(Calibrate_pin) == LOW) {
            //One simple call, with the address first and the object second.
      Serial.println(F("Calibration mode Active"));
      stepper.moveToPositionInMillimeters(defaultStepperposition); 
      Serial.println(F("Stepper moved to default position"));
      float f = rearPWMSenorCarAngle(); 
      EEPROM.put(eeAddress, f);
      baseRearAngleSensorAngle = f;
      Serial.print(F("Calibrated baseRearAngleSensorAngle= ")); Serial.println(baseRearAngleSensorAngle);/* code */
      delay(2000);
    }
  }
  Serial.print(F("baseRearAngleSensorAngle= ")); Serial.println(baseRearAngleSensorAngle);

}

void loop() {
  float currentRearCarAngle = rearPWMSenorCarAngle();
  
  // Adjustment run
  float currentCarRearAngleOffset = currentRearCarAngle - baseRearAngleSensorAngle; // Calculate offset between baseline car angle and current angle. Positive is nose further up than baseline. 
  float requiredHeadlightCorrectionInmm = headlightAngleToStroke(currentCarRearAngleOffset); //Calculate the required stepper stroke to correct for the current car angle offset, positive means the headlight needs to move down.
  float newStepperPositionGoal = defaultStepperposition - requiredHeadlightCorrectionInmm;
  Serial.print(F("Current car angle [deg] = ")); Serial.print(currentRearCarAngle); 
  Serial.print(F(", offset [deg] = "));  Serial.print(currentCarRearAngleOffset);
  
  
  // Check of the stepper goal is not out of bounds
  if(newStepperPositionGoal < 0.0) {
    newStepperPositionGoal = 0.0;
    Serial.println(F("NewStepperPositionGoal out of bounds (too low)"));
  }
  else if (newStepperPositionGoal > stroke)
  {
    newStepperPositionGoal = stroke;
    Serial.println(F("NewStepperPositionGoal out of bounds (too high)"));
  }  

  // Smooth signal new stepper goal signal
  smoothedNewStepperPositionGoal = (ewmaAlpha * newStepperPositionGoal) + (1 - ewmaAlpha) * smoothedNewStepperPositionGoal;
  Serial.print(F(", stepper Goal [mm] = ")); Serial.println(smoothedNewStepperPositionGoal);
  
  // Check if required stepsize is not below minimum
  float requiredStepSize = abs(stepper.getCurrentPositionInMillimeters() - smoothedNewStepperPositionGoal); 
  if(requiredStepSize >= minimumStepsize){
    // Move stepper to new goal
    stepper.moveToPositionInMillimeters(smoothedNewStepperPositionGoal); 
  }
  else{
    Serial.println(F("Required stepsize too small, none taken"));
  }
  
  delay(10); // Delay to postpone next iteration
  
}


void pinChangeIntISR()
{
  //local variables
  static boolean pin_state_new = LOW; //initialize
  static boolean pin_state_old = LOW; //initialize
  static unsigned long t_start = 0; //units of 0.5us
  static unsigned long t_start_old = 0; //units of 0.5us
  
  pin_state_new = fastDigitalRead(p_input_pin_register,input_pin_bitMask);
  if (pin_state_old != pin_state_new)
  {
    //if the pin state actualy changed, & it was not just noise lasting < ~2~4us
    pin_state_old = pin_state_new; //update the state
    if (pin_state_new == HIGH)
    {
      t_start = timer2.get_count(); //0.5us units
      pd = t_start - t_start_old; //0.5us units, the incoming pulse period
      t_start_old = t_start; //0.5us units; update
    }
    else //pin_state_new == LOW
    {
      unsigned long t_end = timer2.get_count(); //0.5us units
      pulseCounts = t_end - t_start; //0.5us units
      output_data = true;
    }
  }
}

//Interrupt Service Routines (ISRs) for Pin Change Interrupts
//see here: http://www.gammon.com.au/interrupts

//PCINT0_vect is for pins D8 to D13
ISR(PCINT0_vect)
{
  pinChangeIntISR();
}

//PCINT1_vect is for pins A0 to A5
ISR(PCINT1_vect)
{
  pinChangeIntISR();
}

//PCINT2_vect is for pins D0 to D7
ISR(PCINT2_vect)
{
  pinChangeIntISR();
}

void configurePinChangeInterrupts()
{
  //Pin Change Interrupt Configuration
  //see ATmega328 datasheet, ex: pgs. 73-75
  //also see: C:\Program Files (x86)\Arduino\hardware\arduino\avr\variants\standard\pins_arduino.h for the macros used here
  //1st, set flags on the proper Pin Change Mask Register (PCMSK)
  volatile byte* p_PCMSK = (volatile byte*)digitalPinToPCMSK(PWM_INPUT_PIN); //pointer to the proper PCMSK register
  *p_PCMSK = _BV(digitalPinToPCMSKbit(PWM_INPUT_PIN));
  //2nd, set flags in the Pin Change Interrupt Control Register (PCICR)
  volatile byte* p_PCICR = (volatile byte*)digitalPinToPCICR(PWM_INPUT_PIN); //pointer to PCICR
  *p_PCICR |= _BV(digitalPinToPCICRbit(PWM_INPUT_PIN));
  
//  //ex: to use digital pin 8 as the INPUT_PIN:
//  //turn on PCINT0_vect Pin Change Interrupts (for pins D8 to D13); see datasheet pg. 73-75.
//  //1st, set flags on the proper Pin Change Mask Register
//  PCMSK0 = 0b00000001; //here I am setting Bit0 to a 1, to mark pin D8's pin change register as on; for pin mapping see here: http://arduino.cc/en/Hacking/PinMapping168
//  //2nd, set flags in the Pin Change Interrupt Control Register
//  PCICR |= 0b00000001; //here I am turning on the pin change vector 0 interrupt, for PCINT0_vect, by setting the right-most bit to a 1
}
