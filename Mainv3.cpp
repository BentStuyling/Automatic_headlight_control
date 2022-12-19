#include <Arduino.h>
#include <SpeedyStepper.h>
#include <EEPROM.h>
#include <eRCaGuy_Timer2_Counter.h>

//
// PWM READING
// macros
#define fastDigitalRead(p_inputRegister, bitMask) ((*p_inputRegister & bitMask) ? HIGH : LOW)
// Global Variables & defines for PWMREADING
const byte PWM_INPUT_PIN = 12;                                                                                      // You can change this to ANY digital or analog pin, ex: 10, 8, A0, A5, etc, 
byte input_pin_bitMask;                                                                                             // EXCEPT A6 and A7 (which exists on the Nano and Pro Mini, for example, and are NOT capable of digital operations)
volatile byte* p_input_pin_register;
// volatile variables for use in the ISR (Interrupt Service Routine)
volatile boolean output_data = false;                                                                               // the main loop will try to output data each time a new pulse comes in, which is when this gets set true
volatile unsigned long pulseCounts = 0;                                                                             // units of 0.5us; the input signal high pulse time
volatile unsigned int pd = 0;                                                                                      // units of 0.5us; the pulse period (ie: time from start of high pulse to start of next high pulse)

//
// Define constants
const int Calibrate_pin = 2;                                                                                        // Pin activate calibration mode (active when pin grouned) [-]
const int MOTOR_ENABLE_PIN = 5;                                                                                     // Pin to set low to activate stepper driver
const int MOTOR_STEP_PIN = 3;                                                                                       // Stepper motor step pin [-]
const int MOTOR_DIRECTION_PIN = 4;                                                                                  // Stepper motor direction pin [-]
const float stroke = 11.5;                                                                                          // Max useable stroke of steppermotor [mm]
const float holeToHoleDistance = 97.3;                                                                              // Distance between vertical mounting points headlight [mm]
const float defaultStepperposition = 9.0;                                                                           // Default stepper motor position [mm]
const float minimumStepsize = 0.05;                                                                                 // Minimum step size (0.05mm corresponds to a Headlight tilt angle of 0.03 deg) [mm]
float baseRearAngleSensorAngle;                                                                                     // Base value for rear axle level sensor, adjusted in calibratoin mode [deg]
const int minimumRearCarAngle = 83;                                                                                 // Minimum value the rear axle sensor can attain [deg]
const int maximumRearCarAngle = 97;                                                                                 // Maximum value the rear axle sensor can attain [deg][deg]
const float ewmaAlpha = 0.90;                                                                                       // The EWMA alpha value (α) for smoothing (smaller is more smoothing) [-]
float currentRearCarAngle = 0;                                                                                      // Initialize rear axle car angle variable [deg]
float smoothedRearCarAngle = 88;                                                                                    // Initialize smoothed rear axle car angle variable [deg]
int lastPWMSignalTime = 0;                                                                                          // Timersignal for read PWM function
int lastAdjustmentrun = 0;                                                                                          // Timersignal for headlight adjustment function


// create the stepper motor object
SpeedyStepper stepper;

// Functions:
void configurePinChangeInterrupts() {
  //Pin Change Interrupt Configuration
    //1st, set flags on the proper Pin Change Mask Register (PCMSK)
  volatile byte* p_PCMSK = (volatile byte*)digitalPinToPCMSK(PWM_INPUT_PIN); //pointer to the proper PCMSK register
  *p_PCMSK = _BV(digitalPinToPCMSKbit(PWM_INPUT_PIN));
  //2nd, set flags in the Pin Change Interrupt Control Register (PCICR)
  volatile byte* p_PCICR = (volatile byte*)digitalPinToPCICR(PWM_INPUT_PIN); //pointer to PCICR
  *p_PCICR |= _BV(digitalPinToPCICRbit(PWM_INPUT_PIN));    
}                                                                       

float headlightAngleToStroke(float alpha) {
  return tan(alpha/57.2958)*holeToHoleDistance;                                                                     // Convert stepper stroke into headlight tilt angle
}

float rearPWMSenorCarAngle() {
  //local variables
  static float pulseTime = 0;                                                                                       // us; the most recent input signal high pulse time
  static float pd_us = 0;                                                                                           // us; the most recent input signal period between pulses
  static float dutyCycle = 0;
  int timeCurrentPWMread = millis();
  

  if (output_data==true)                                                                                            // if a pulse just came in
  {
    //turn off interrupts, grab copies of volatile data, and re-enable interrupts
    noInterrupts();
    output_data = false;                                                                                            // reset
    unsigned long pulseCountsCopy = pulseCounts;                                                                    // 0.5us units
    unsigned long pdCopy = pd;                                                                                      // 0.5us units
    interrupts();
    
    // Do calculations
    pulseTime = pulseCountsCopy/2.0;                                                                                // Pulse HIGH time [us]
    pd_us = pdCopy/2.0;                                                                                             // Period of signal (HIGH+LOW) [us]
    dutyCycle = 100*(pulseTime/pd_us);                                                                              // Duty cycle in [%]
    currentRearCarAngle = 0.06*dutyCycle + 86;                                                                      // Convert from dutycycle to angles
    lastPWMSignalTime = millis();
  }

  // Set limits to output values in case of faulty measurement
  if(currentRearCarAngle < minimumRearCarAngle ){
    currentRearCarAngle = maximumRearCarAngle;                                                                      // Default to most conservative setting
    Serial.println(F("Car angle below minimum setpoint, default to max")); 
  }
  else if(currentRearCarAngle > maximumRearCarAngle){
    currentRearCarAngle = maximumRearCarAngle;                                                                      // Default to most conservative realistic setting
    Serial.println(F("Car angle above maximum setpoint, default to max")); 
  }
  else if(isnan(currentRearCarAngle)){                                                                              // Check if the calculated value is a NaN
    currentRearCarAngle = maximumRearCarAngle;                                                                      // Default to most conservative realistic setting
    Serial.println(F("No valid car angle value, default to max"));
  }  
  else if((timeCurrentPWMread - lastPWMSignalTime)> 3000){                                                          // If too much time (more than 3s) since last new PWM signal:
    currentRearCarAngle = maximumRearCarAngle;                                                                      // Default to most conservative realistic setting 
    Serial.println(F("No new PWM signal received for 3s, default to max "));
  }

  // Smooth Rear car angle signal 
  smoothedRearCarAngle = (ewmaAlpha * currentRearCarAngle) + (1 - ewmaAlpha) * smoothedRearCarAngle;
  return smoothedRearCarAngle;
}

void adjustmentrun() {
                                    
  // Adjustment run
  float currentCarRearAngleOffset = smoothedRearCarAngle - baseRearAngleSensorAngle;                                // Calculate offset between baseline car angle and current angle. Positive is nose further up than baseline. 
  float requiredHeadlightCorrectionInmm = headlightAngleToStroke(currentCarRearAngleOffset);                        //Calculate the required stepper stroke to correct for the current car angle offset, positive means the headlight needs to move down.
  float newStepperPositionGoal = defaultStepperposition - requiredHeadlightCorrectionInmm;
  Serial.print(F("Smoothed rear car angle [deg] = ")); Serial.print(smoothedRearCarAngle);  
  Serial.print(F(", offset [deg] = "));  Serial.print(currentCarRearAngleOffset);
    
  // Check of the stepper goal is not out of bounds
  if(newStepperPositionGoal < 0.0) {
    newStepperPositionGoal = 0.0;
    Serial.println(F("NewStepperPositionGoal out of bounds (too low), move to fully retracted"));
  }
  else if (newStepperPositionGoal > stroke)
  {
    newStepperPositionGoal = stroke;
    Serial.println(F("NewStepperPositionGoal out of bounds (too high), move to full extension"));
  }  
  
  // Check if required stepsize is not below minimum
  float requiredStepSize = abs(stepper.getCurrentPositionInMillimeters() - newStepperPositionGoal); 
  if(requiredStepSize >= minimumStepsize){
    digitalWrite(MOTOR_ENABLE_PIN, LOW);                                                                              // Enable stepper driver
    // Move stepper to new goal
    stepper.moveToPositionInMillimeters(newStepperPositionGoal); 
    // digitalWrite(MOTOR_ENABLE_PIN, HIGH);                                                                             // Disable stepper driver 
    Serial.print(F(", stepper position [mm] = ")); Serial.println(stepper.getCurrentPositionInMillimeters());
  }
  else{
    Serial.print(F(", stepper position [mm] = ")); Serial.println(stepper.getCurrentPositionInMillimeters());
    Serial.println(F("Required stepsize too small, none taken"));
  }
  
}

void setup() {
  Serial.begin(115200);
  Serial.println(F("Start setup void"));
  
  // Setup PWM Reading
  pinMode(PWM_INPUT_PIN,INPUT_PULLUP);                                                                              // Use INPUT_PULLUP to keep the pin from floating and jumping around when nothing is connected
  timer2.setup();                                                                                                   // Configure timer2
  // Prepare for FAST digital reads on INPUT_PIN, by mapping to the input register (ex: PINB, PINC, or PIND), and creating a bitMask
  input_pin_bitMask = digitalPinToBitMask(PWM_INPUT_PIN);
  p_input_pin_register = portInputRegister(digitalPinToPort(PWM_INPUT_PIN));
  configurePinChangeInterrupts();
  
  // connect and configure the stepper motor to its IO pins
  pinMode(MOTOR_ENABLE_PIN,OUTPUT);     
  digitalWrite(MOTOR_ENABLE_PIN, LOW);                                                                              // Enable stepper driver                                                                            // Set motor enable pin as digital output
  stepper.connectToPins(MOTOR_STEP_PIN, MOTOR_DIRECTION_PIN);
  stepper.setStepsPerMillimeter(-30.3030);                                                                          // 1x microstepping, 0.033mm per step
  // set the speed and acceleration rates for the stepper motor
  stepper.setSpeedInStepsPerSecond(1000);                                                                           // PPS Pulses Per Second [P/s]
  stepper.setAccelerationInStepsPerSecondPerSecond(15000);                                                          // Pulses per second per second [P/s^2]
  
  // Do full headlight vertical sweep to zero steppermotor. Then sweep up and to default startpoint.
  stepper.moveRelativeInMillimeters(-stroke -1.5);                                                                  // Move full stroke up (out) plust 1.5 mmm extra for good measure
  stepper.setSpeedInStepsPerSecond(500);   
  stepper.moveRelativeInMillimeters(+1.5);                                                                          // retract 1.5mm for hysterisis
  stepper.moveRelativeInMillimeters(-1.5);                                                                          // move to full out without overrrun
  stepper.moveRelativeInMillimeters(0.5);                                                                           // Retract 0.5mm to stay away from endstop
  delay(10);                                                                           
  stepper.setCurrentPositionInMillimeters(0.0);                                                                     // Set current position (end of stroke) position to max stroke value
  //stepper.moveToPositionInMillimeters(0);                                                                         // Move all the way down (in)
  stepper.moveToPositionInMillimeters(defaultStepperposition);                                                      // Move to default stepper position (out)
  digitalWrite(MOTOR_ENABLE_PIN, HIGH);                                                                             // Disable stepper driver 
  delay(10);

  // Read baseline rearaxleanglesensor value from eeprom. If calibration mode = Active, read value from sensor and store.
  int eeAddress = 0;                                                                                                // EEPROM address to read and put the default rear sensor value
  pinMode(Calibrate_pin, INPUT_PULLUP);                                                                             // Set calibration pin to input and pull High
  if(digitalRead(Calibrate_pin) == HIGH) {                                                                          // If the Calibration button is NOT pushed (not grouned), no calibration requested
    EEPROM.get(eeAddress, baseRearAngleSensorAngle);                                                                // Read from EEPROM.
  }
  else {                                                                                                            // If the calibration buttin IS pushed (grouned), calibration mode active                                      
    while (digitalRead(Calibrate_pin) == LOW) {
      Serial.println(F("Calibration mode Active"));
      stepper.moveToPositionInMillimeters(defaultStepperposition);                                                  // Move stepper to default value for calibration mode
      Serial.println(F("Stepper moved to default position"));
      rearPWMSenorCarAngle();                                                                                       // Read rear axle car angle  for new baseline value
      EEPROM.put(eeAddress, currentRearCarAngle);                                                                   // Write new rear axle car angle value to EEPROM (if different)
      baseRearAngleSensorAngle = currentRearCarAngle;                                                               // Write new rear axle car angle value variable for this script
      Serial.print(F("Calibrated baseRearAngleSensorAngle= ")); Serial.println(baseRearAngleSensorAngle);
      delay(2000);
    }
  }
  Serial.print(F("baseRearAngleSensorAngle= ")); Serial.println(baseRearAngleSensorAngle);
}

void loop() {
  // Poll rear PWM anglesensor
  rearPWMSenorCarAngle();
  
  int currentLoopTime = millis();                                                                                     // Current loop time for adjustmentruntimer
  int adjustmentInterval = 100;                                                                                       // Time between adjustments in [ms]
  if((currentLoopTime - lastAdjustmentrun) >adjustmentInterval){
    adjustmentrun();                                                                                                  // Run 1 adjustment 
    lastAdjustmentrun = millis();                                                                                     // Save last adjustment time
  }
  delay(10);                                                                                                          // Delay to postpone next iteration (basically polling interval for PWM sensor)
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


  
