#include <Arduino.h>
#include <SpeedyStepper.h>
#include <EEPROM.h>
#include <eRCaGuy_Timer2_Counter.h>
// gryo:
#include <I2Cdev.h>
#include "Wire.h"
#include <MPU6050_6Axis_MotionApps20.h>

//
// PWM READING
// macros
#define fastDigitalRead(p_inputRegister, bitMask) ((*p_inputRegister & bitMask) ? HIGH : LOW)
// Global Variables & defines for PWMREADING pin 1
const byte PWM_INPUT_PIN1 = 8;                                                                                      // You can change this to ANY digital or analog pin, ex: 10, 8, A0, A5, etc, 
byte input_pin_bitMask1;                                                                                             // EXCEPT A6 and A7 (which exists on the Nano and Pro Mini, for example, and are NOT capable of digital operations)
volatile byte* p_input_pin_register1;
// volatile variables for use in the ISR (Interrupt Service Routine)
volatile boolean output_data1 = false;                                                                               // the main loop will try to output data each time a new pulse comes in, which is when this gets set true
volatile unsigned long pulseCounts1 = 0;                                                                             // units of 0.5us; the input signal high pulse time
volatile unsigned int pd1 = 0;                                                                                      // units of 0.5us; the pulse period (ie: time from start of high pulse to start of next high pulse)

// Global Variables & defines for PWMREADING pin 2
const byte PWM_INPUT_PIN2= 2;                                                                                      // You can change this to ANY digital or analog pin, ex: 10, 8, A0, A5, etc, 
byte input_pin_bitMask2;                                                                                             // EXCEPT A6 and A7 (which exists on the Nano and Pro Mini, for example, and are NOT capable of digital operations)
volatile byte* p_input_pin_register2;
// volatile variables for use in the ISR (Interrupt Service Routine)
volatile boolean output_data2 = false;                                                                               // the main loop will try to output data each time a new pulse comes in, which is when this gets set true
volatile unsigned long pulseCounts2 = 0;                                                                             // units of 0.5us; the input signal high pulse time
volatile unsigned int pd2 = 0;                                                                                      // units of 0.5us; the pulse period (ie: time from start of high pulse to start of next high pulse)

//MPU-6050
MPU6050 mpu;
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


// volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
// void dmpDataReady() {
//     mpuInterrupt = true;
// }

//
// Define constants
const int CALIBRATE_HEIGHT = A1;                                                                                    // Pin activate calibration mode (active when pin grouned) [-]
const int CALIBRATE_STEERING = A2;                                                                                  // Pin activate calibration mode (active when pin grouned) [-]
const int MOTOR_STEP_PIN = 3;                                                                                       // Stepper motor step pin [-]
const int MOTOR_DIRECTION_PIN = 4;                                                                                  // Stepper motor direction pin [-]
const float stroke = 11.0;                                                                                          // Max useable stroke of steppermotor [mm] (hella 11.5)
const float holeToHoleDistance = 97.3;                                                                              // Distance between vertical mounting points headlight [mm]
const float defaultStepperposition = 9.0;                                                                           // Default stepper motor position [mm]
const float minimumStepsize = 0.05;                                                                                 // Minimum step size (0.05mm corresponds to a Headlight tilt angle of 0.03 deg) [mm]
// Rear axle angle sensor
float baseRearCarAngle;                                                                                             // Base value for rear axle level sensor, adjusted in calibratoin mode [deg]
const int minimumRearCarAngle = 83;                                                                                 // Minimum value the rear axle sensor can attain [deg]
const int maximumRearCarAngle = 97;                                                                                 // Maximum value the rear axle sensor can attain [deg][deg]
const float ewmaAlphaRearCarAngle = 0.01;                                                                           // The EWMA alpha value (α) for smoothing (smaller is more smoothing) [-]
float currentRearCarAngle = 0;                                                                                      // Initialize rear axle car angle variable [deg]
float smoothedRearCarAngle = 88;                                                                                    // Initialize smoothed rear axle car angle variable [deg]
int lastRearCarAnglePWMSignalTime = 0;                                                                              // Timersignal for read PWM function
// Gyro car angle
const int baseGyroCarAngle = 0;
const int minimumGyroCarAngle = -30;
const int maximumGyroCarAngle = 30;
const float ewmaAlphaGyroCarAngle = 0.10;                                                                           // The EWMA alpha value (α) for smoothing (smaller is more smoothing) [-]
float currentGyroCarAngle = 0;
float smoothedGyroCarAngleSignal = 0;
float smoothedGyroCarAngle = 0;
float const ewmaAlphafloatingOffsetGyroCarAngle= 0.005;                                                              // The EWMA alpha value (α) for smoothing (smaller is more smoothing) [-]
float floatingOffsetGyroCarAngle = 0;                                                                                // Initialize variable [deg]
int lastGyroSignalTime = 0;

int lastAdjustmentrun = 0;                                                                                          // Timersignal for headlight adjustment function


// create the stepper motor object
SpeedyStepper stepper;

// Functions:
void configurePinChangeInterrupts() {
  //Pin Change Interrupt Configuration for PWM pin 1
  //1st, set flags on the proper Pin Change Mask Register (PCMSK)
  volatile byte* p_PCMSK1 = (volatile byte*)digitalPinToPCMSK(PWM_INPUT_PIN1); //pointer to the proper PCMSK register
  *p_PCMSK1 = _BV(digitalPinToPCMSKbit(PWM_INPUT_PIN1));
  //2nd, set flags in the Pin Change Interrupt Control Register (PCICR)
  volatile byte* p_PCICR1 = (volatile byte*)digitalPinToPCICR(PWM_INPUT_PIN1); //pointer to PCICR
  *p_PCICR1 |= _BV(digitalPinToPCICRbit(PWM_INPUT_PIN1));    

  //Pin Change Interrupt Configuration for PWM pin 1
  //1st, set flags on the proper Pin Change Mask Register (PCMSK)
  volatile byte* p_PCMSK2 = (volatile byte*)digitalPinToPCMSK(PWM_INPUT_PIN2); //pointer to the proper PCMSK register
  *p_PCMSK2 = _BV(digitalPinToPCMSKbit(PWM_INPUT_PIN2));
  //2nd, set flags in the Pin Change Interrupt Control Register (PCICR)
  volatile byte* p_PCICR2 = (volatile byte*)digitalPinToPCICR(PWM_INPUT_PIN2); //pointer to PCICR
  *p_PCICR2 |= _BV(digitalPinToPCICRbit(PWM_INPUT_PIN2)); 
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
 
  if (output_data1==true)                                                                                            // if a pulse just came in
  {
    //turn off interrupts, grab copies of volatile data, and re-enable interrupts
    noInterrupts();
    output_data1 = false;                                                                                            // reset
    unsigned long pulseCountsCopy = pulseCounts1;                                                                    // 0.5us units
    unsigned long pdCopy = pd1;                                                                                      // 0.5us units
    interrupts();
    
    // Do calculations
    pulseTime = pulseCountsCopy/2.0;                                                                                // Pulse HIGH time [us]
    pd_us = pdCopy/2.0;                                                                                             // Period of signal (HIGH+LOW) [us]
    dutyCycle = 100*(pulseTime/pd_us);                                                                              // Duty cycle in [%]
    currentRearCarAngle = 0.06*dutyCycle + 86;                                                                      // Convert from dutycycle to angles
    lastRearCarAnglePWMSignalTime = millis();
  }
  // Set limits to output values in case of faulty measurement
  if(currentRearCarAngle < minimumRearCarAngle ){
    currentRearCarAngle = maximumRearCarAngle;                                                                      // Default to most conservative setting
    Serial.println(F("Rear Car angle below minimum setpoint, default to max")); 
  }
  else if(currentRearCarAngle > maximumRearCarAngle){
    currentRearCarAngle = maximumRearCarAngle;                                                                      // Default to most conservative realistic setting
    Serial.println(F("Rear Car angle above maximum setpoint, default to max")); 
  }
  else if(isnan(currentRearCarAngle)){                                                                              // Check if the calculated value is a NaN
    currentRearCarAngle = maximumRearCarAngle;                                                                      // Default to most conservative realistic setting
    Serial.println(F("No valid rear car angle value, default to max"));
  }  
  else if((timeCurrentPWMread - lastRearCarAnglePWMSignalTime)>3000){                                                          // If too much time (more than 3s) since last new PWM signal:
    currentRearCarAngle = maximumRearCarAngle;                                                                      // Default to most conservative realistic setting 
    Serial.println(F("No new PWM signal for rear car angle received for 3s, default to max "));
  }

  // Smooth Rear car angle signal 
  smoothedRearCarAngle = (ewmaAlphaRearCarAngle * currentRearCarAngle) + (1 - ewmaAlphaRearCarAngle) * smoothedRearCarAngle;
  
}

void gyroCarAngle() {
  int timeCurrentGyroRead = millis();
    // if programming failed, don't try to do anything
  if (!dmpReady) return;
  mpuIntStatus = mpu.getIntStatus();
  // get current FIFO count
  fifoCount = mpu.getFIFOCount();
  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
      // reset so we can continue cleanly
      mpu.resetFIFO();
      Serial.println(F("FIFO overflow!"));
  // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if ((mpuIntStatus & 0x02) > 0) {
      // wait for correct available data length, should be a VERY short wait
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

      // read a packet from FIFO
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      
      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifoCount -= packetSize;
  
      // display Euler angles in degrees
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      currentGyroCarAngle = ypr[2] * 180/M_PI;
      lastGyroSignalTime = millis();
  }
  
  // Set limits to output values in case of faulty measurement
  if(currentGyroCarAngle < minimumGyroCarAngle ){
    currentGyroCarAngle = baseGyroCarAngle;                                                                      // Default to most conservative setting
    Serial.println(F("Gyro  angle below minimum setpoint, default to base value")); 
  }
  else if(currentGyroCarAngle > maximumGyroCarAngle){
    currentGyroCarAngle = baseGyroCarAngle;                                                                      // Default to most conservative realistic setting
    Serial.println(F("Gyro angle above maximum setpoint, default to base value")); 
  }
  else if(isnan(currentGyroCarAngle)){                                                                              // Check if the calculated value is a NaN
    currentGyroCarAngle = baseGyroCarAngle;                                                                      // Default to most conservative realistic setting
    Serial.println(F("No valid gyro angle value, default to base value"));
  }  
  else if((timeCurrentGyroRead - lastGyroSignalTime)> 3000){                                                          // If too much time (more than 3s) since last new PWM signal:
    currentGyroCarAngle = baseGyroCarAngle;                                                                      // Default to most conservative realistic setting 
    Serial.println(F("No new gyro signal received for 3s, default base value "));
  }


  // Smooth gyro car angle signal 
  smoothedGyroCarAngleSignal = (ewmaAlphaGyroCarAngle * currentGyroCarAngle) + (1 - ewmaAlphaGyroCarAngle) * smoothedGyroCarAngleSignal; //Smoothing calculation to remove high frequency ripple on the gyro signal measurement
  floatingOffsetGyroCarAngle = (ewmaAlphafloatingOffsetGyroCarAngle * currentGyroCarAngle) + (1 - ewmaAlphafloatingOffsetGyroCarAngle) * floatingOffsetGyroCarAngle; //Smoothing calculation (very low pass filter) to calculate a floating 'base' reading for the gyro.
  smoothedGyroCarAngle = smoothedGyroCarAngleSignal - floatingOffsetGyroCarAngle;                                                         // Substract 'base reading' from the smoothed gyro car angle to correct for a constant bias (i.e. driving uphil)
}


void adjustmentrun() {
                                    
  // Adjustment run
  float currentCarRearAngleOffset = smoothedRearCarAngle - baseRearCarAngle;                                // Calculate offset between baseline car angle and current angle. Positive is nose further up than baseline. 
  float requiredHeadlightCorrectionInmm = headlightAngleToStroke(currentCarRearAngleOffset-smoothedGyroCarAngle);                        //Calculate the required stepper stroke to correct for the current car angle offset, positive means the headlight needs to move down.
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
    // Move stepper to new goal
    stepper.moveToPositionInMillimeters(newStepperPositionGoal); 
    Serial.print(F(", stepper position [mm] = ")); Serial.println(stepper.getCurrentPositionInMillimeters());
  }
  else{
    Serial.print(F(", stepper position [mm] = ")); Serial.println(stepper.getCurrentPositionInMillimeters());
    Serial.println(F("Required stepsize too small, none taken"));
  }
  Serial.print(F("Smoothed gyro car angle [deg] = ")); Serial.println(smoothedGyroCarAngle);  
}

void setup() {
  Serial.begin(115200);
  Serial.println(F("Start setup void"));
  
  // PWM Setup
  timer2.setup();                                                                                                  // Configure timer2    
  // Setup PWM Reading of pin1
  pinMode(PWM_INPUT_PIN1,INPUT_PULLUP);                                                                              // Use INPUT_PULLUP to keep the pin from floating and jumping around when nothing is connected
  // Prepare for FAST digital reads on INPUT_PIN, by mapping to the input register (ex: PINB, PINC, or PIND), and creating a bitMask
  input_pin_bitMask1 = digitalPinToBitMask(PWM_INPUT_PIN1);
  p_input_pin_register1 = portInputRegister(digitalPinToPort(PWM_INPUT_PIN1));

  // Setup PWM Reading of pin2
  pinMode(PWM_INPUT_PIN2,INPUT_PULLUP);                                                                              // Use INPUT_PULLUP to keep the pin from floating and jumping around when nothing is connected
  // Prepare for FAST digital reads on INPUT_PIN, by mapping to the input register (ex: PINB, PINC, or PIND), and creating a bitMask
  input_pin_bitMask2 = digitalPinToBitMask(PWM_INPUT_PIN2);
  p_input_pin_register2 = portInputRegister(digitalPinToPort(PWM_INPUT_PIN2));
  configurePinChangeInterrupts();
  
  pinMode(MOTOR_STEP_PIN,OUTPUT);
  // connect and configure the stepper motor to its IO pins
  stepper.connectToPins(MOTOR_STEP_PIN, MOTOR_DIRECTION_PIN);
  stepper.setStepsPerMillimeter(-220.8);                                                                            // 8x microstepping, 0.033mm per step hella unit = 242.424, asmo = 224.5
  // set the speed and acceleration rates for the stepper motor
  stepper.setSpeedInStepsPerSecond(4000);                                                                           // PPS Pulses Per Second [P/s] hella unit = 8000
  stepper.setAccelerationInStepsPerSecondPerSecond(120000);                                                         // Pulses per second per second [P/s^2] hella unit = 120000
  
  // Do full headlight vertical sweep to zero steppermotor. Then sweep up and to default startpoint.
  stepper.moveRelativeInMillimeters(-stroke -1.5);                                                                  // Move full stroke in plust 1.5 mmm extra for good measure
  stepper.setSpeedInStepsPerSecond(3000);                                                                           // Hella unit = 4000
  stepper.moveRelativeInMillimeters(+1.5);                                                                          // move out 1.5mm for hysterisis
  stepper.moveRelativeInMillimeters(-1.5);                                                                          // move to fully retracted without overrrun
  stepper.moveRelativeInMillimeters(0.25);                                                                          // Move out 0.25mm to stay away from endstop
  delay(10);                                                                           
  stepper.setCurrentPositionInMillimeters(0.0);                                                                     // Set current position (beginning of stroke) position to zero stroke value
  stepper.moveToPositionInMillimeters(defaultStepperposition);                                                      // Move to default stepper position (out)
  delay(10);

  // Read baseline rearaxleanglesensor value from eeprom. If calibration mode = Active, read value from sensor and store.
  int eeAddress = 0;                                                                                                // EEPROM address to read and put the default rear sensor value
  pinMode(CALIBRATE_HEIGHT, INPUT_PULLUP);                                                                             // Set calibration pin to input and pull High
  if(digitalRead(CALIBRATE_HEIGHT) == HIGH) {                                                                          // If the Calibration button is NOT pushed (not grouned), no calibration requested
    EEPROM.get(eeAddress, baseRearCarAngle);                                                                          // Read from EEPROM.
  }
  else {                                                                                                            // If the calibration buttin IS pushed (grouned), calibration mode active                                      
    while (digitalRead(CALIBRATE_HEIGHT) == LOW) {
      Serial.println(F("Calibration mode Active"));
      stepper.moveToPositionInMillimeters(defaultStepperposition);                                                  // Move stepper to default value for calibration mode
      Serial.println(F("Stepper moved to default position"));
      rearPWMSenorCarAngle();                                                                                       // Read rear axle car angle  for new baseline value
      EEPROM.put(eeAddress, currentRearCarAngle);                                                                   // Write new rear axle car angle value to EEPROM (if different)
      baseRearCarAngle = currentRearCarAngle;                                                                       // Write new rear axle car angle value variable for this script
      Serial.print(F("Calibrated base car angle = ")); Serial.println(baseRearCarAngle);
      delay(2000);
    }
  }
  Serial.print(F("Base car angle = ")); Serial.println(baseRearCarAngle);
  //MPU SETUP 
  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();
	TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  delay(100);
  mpu.initialize();
  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
      // turn on the DMP, now that it's ready
      Serial.println(F("Enabling DMP..."));
      mpu.setDMPEnabled(true);
      dmpReady = true;
      // get expected DMP packet size for later comparison
      packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
      // ERROR!
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      // (if it's going to break, usually the code will be 1)
      Serial.print(F("DMP Initialization failed (code "));
      Serial.print(devStatus);
      Serial.println(F(")"));
  }


}

void loop() {
  // Poll sensors
  rearPWMSenorCarAngle();
  gyroCarAngle();
  
  int currentLoopTime = millis();                                                                                     // Current loop time for adjustmentruntimer
  int adjustmentInterval = 50;                                                                                        // Time between adjustments in [ms]
  if((currentLoopTime - lastAdjustmentrun) >adjustmentInterval){
    adjustmentrun();                                                                                                  // Run 1 adjustment 
    lastAdjustmentrun = millis();                                                                                     // Save last adjustment time
  }
  delay(1);                                                                                                           // Delay to postpone next iteration (basically polling interval for PWM sensor)
}


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


  