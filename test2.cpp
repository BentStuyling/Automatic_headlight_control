#include <Wire.h>

int gyroResult[3], accelResult[3];
float timeStep = 0.02;
float biasGyroX, biasGyroY, biasGyroZ, biasAccelX, biasAccelY, biasAccelZ;
float pitchGyro = 0;
float pitchAccel = 0;
float pitchPrediction = 0; //Output of Kalman filter
float rollGyro = 0;
float rollAccel = 0;
float rollPrediction = 0;  //Output of Kalman filter
float giroVar = 0.1;
float deltaGiroVar = 0.1;
float accelVar = 5;
float Pxx = 0.1; // angle variance
float Pvv = 0.1; // angle change rate variance
float Pxv = 0.1; // angle and angle change rate covariance
float kx, kv;
unsigned long timer;

void writeTo(byte device, byte toAddress, byte val) {
  Wire.beginTransmission(device);  
  Wire.send(toAddress);        
  Wire.send(val);        
  Wire.endTransmission();
}

void readFrom(byte device, byte fromAddress, int num, byte result[]) {
  Wire.beginTransmission(device);
  Wire.send(fromAddress);
  Wire.endTransmission();
  Wire.requestFrom((int)device, num);
  int i = 0;
  while(Wire.available()) {
    result[i] = Wire.receive();
    i++;
  }
}

void getGyroscopeReadings(int gyroResult[]) {
  byte buffer[6];
  readFrom(0x68,0x1D,6,buffer);
  gyroResult[0] = (((int)buffer[0]) << 8 ) | buffer[1];
  gyroResult[1] = (((int)buffer[2]) << 8 ) | buffer[3];
  gyroResult[2] = (((int)buffer[4]) << 8 ) | buffer[5];
} 

void getAccelerometerReadings(int accelResult[]) {
  byte buffer[6];
  readFrom(0x53,0x32,6,buffer);
  accelResult[0] = (((int)buffer[1]) << 8 ) | buffer[0];
  accelResult[1] = (((int)buffer[3]) << 8 ) | buffer[2];
  accelResult[2] = (((int)buffer[5]) << 8 ) | buffer[4];
}

void setup() {
  int totalGyroXValues = 0;
  int totalGyroYValues = 0;
  int totalGyroZValues = 0;
  int totalAccelXValues = 0;
  int totalAccelYValues = 0;
  int totalAccelZValues = 0;
  int i;
  
  Wire.begin(); 
  Serial.begin(115200);
  
  writeTo(0x53,0x31,0x09); //Set accelerometer to 11bit, +/-4g
  writeTo(0x53,0x2D,0x08); //Set accelerometer to measure mode
  writeTo(0x68,0x16,0x1A); //Set gyro to +/-2000deg/sec and 98Hz low pass filter
  writeTo(0x68,0x15,0x09); //Set gyro to 100Hz sample rate
  
  delay(100); //wait for gyro
  for (i = 0; i < 50; i += 1) {
    getGyroscopeReadings(gyroResult);
    getAccelerometerReadings(accelResult);
    totalGyroXValues += gyroResult[0];
    totalGyroYValues += gyroResult[1];
    totalGyroZValues += gyroResult[2];
    totalAccelXValues += accelResult[0];
    totalAccelYValues += accelResult[1];
    totalAccelZValues += accelResult[2];
    delay(50);
  }
  biasGyroX = totalGyroXValues / 50;
  biasGyroY = totalGyroYValues / 50;
  biasGyroZ = totalGyroZValues / 50;
  biasAccelX = totalAccelXValues / 50;
  biasAccelY = totalAccelYValues / 50;
  biasAccelZ = (totalAccelZValues / 50) - 256;
  
  Serial.print("Pitch gyro\tPitch accel\tPitch Kalman\t");
  Serial.print("Roll gyro\tRoll accel\tRoll Kalman\n");
}

void loop() {
  timer = millis();
  getGyroscopeReadings(gyroResult);
  getAccelerometerReadings(accelResult);
  
  pitchAccel = atan2((accelResult[1] - biasAccelY) / 256, (accelResult[2] - biasAccelZ) / 256) * 360.0 / (2*PI);
  pitchGyro = pitchGyro + ((gyroResult[0] - biasGyroX) / 14.375) * timeStep;
  pitchPrediction = pitchPrediction + ((gyroResult[0] - biasGyroX) / 14.375) * timeStep;
  
  rollAccel = atan2((accelResult[0] - biasAccelX) / 256, (accelResult[2] - biasAccelZ) / 256) * 360.0 / (2*PI);
  rollGyro = rollGyro - ((gyroResult[1] - biasGyroY) / 14.375) * timeStep; 
  rollPrediction = rollPrediction - ((gyroResult[1] - biasGyroY) / 14.375) * timeStep;
  
  Pxx += timeStep * (2 * Pxv + timeStep * Pvv);
  Pxv += timeStep * Pvv;
  Pxx += timeStep * giroVar;
  Pvv += timeStep * deltaGiroVar;
  kx = Pxx * (1 / (Pxx + accelVar));
  kv = Pxv * (1 / (Pxx + accelVar));
  
  pitchPrediction += (pitchAccel - pitchPrediction) * kx;
  rollPrediction += (rollAccel - rollPrediction) * kx;
  
  Pxx *= (1 - kx);
  Pxv *= (1 - kx);
  Pvv -= kv * Pxv;
  
  Serial.print(pitchGyro);
  Serial.print("\t");
  Serial.print(pitchAccel);
  Serial.print("\t");
  Serial.print(pitchPrediction);
  Serial.print("\t"); 
  Serial.print(rollGyro);
  Serial.print("\t");
  Serial.print(rollAccel);
  Serial.print("\t");
  Serial.print(rollPrediction);
  Serial.print("\n");
  
  timer = millis() - timer;
  timer = (timeStep * 1000) - timer; 
  delay(timer);
}