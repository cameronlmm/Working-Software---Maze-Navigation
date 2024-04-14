// 14 April 2024 1 (Current Final Sketch)

#include <Wire.h>
#include <ESP32Servo.h>
#include <ESP32Encoder.h>



// ******* For the Encoders ******* //

#define CLK 36 
#define DT 39  
#define CLK2 35
#define DT2 34 

ESP32Encoder encoder;
ESP32Encoder encoder2; 

const float wheelCircumference = 21.6;                        //----------------------------------------|
const int encoderCPR = 47;                                    // For calculating distance from encoders |
const float pulsePerCM = wheelCircumference / encoderCPR;     //----------------------------------------|




  // ******* For the Servo ******* //

Servo myservo;           
int servoPin = 13;      
int pos = 0; 



  // ******* For the Motors ******* //

const int motorPin1 = 26;  
const int motorPin2 = 27;  
const int motorPin3 = 14;  
const int motorPin4 = 12;  
const int enablePinA = 33; 
const int enablePinB = 25; 

const int freq = 2000;              //-----------------------------------|
const int pwmChannela = 0;          //                                   |
const int pwmChannelb = 1;          // Setting PWM properties for motors |
const int resolution = 8;           //                                   |
int dutyCycle = 140;                //-----------------------------------|



  // ******* For Maze Navigation - Pitch, Key, and PID ******* //

int pitch;
char Key;
char commands[20];
int commandIndex = 0;

int P, I, D, previousError, PID;
int setpoint = 0;
int kP = 1;
int kI = 0;
int kD = 0;
int steeringAngle;

bool newDataAvailable = false;
bool pitchUpdated = false;
bool pidEnabled = true; // Global flag to enable/disable PID control






void setup() {
  Wire.begin(8);             
  Wire.onReceive(receiveData);
  Serial.begin(9600);

  // Setting up motor control pins as outputs to control direction of EEEbot (motorPin1-4) and speed (enablePin1-2)
  pinMode(motorPin1, OUTPUT); 
  pinMode(motorPin2, OUTPUT); 
  pinMode(motorPin3, OUTPUT);
  pinMode(motorPin4, OUTPUT);
  pinMode(enablePinA, OUTPUT);
  pinMode(enablePinB, OUTPUT);

  // Setting up PWM channels for motor speed control
  ledcSetup(pwmChannela, freq, resolution);
  ledcSetup(pwmChannelb, freq, resolution);
  
  // Attaching enable pins to PWM channels for motor speed control
  ledcAttachPin(enablePinA, pwmChannela); 
  ledcAttachPin(enablePinB, pwmChannelb); 

  // PWM for servo
  ESP32PWM::allocateTimer(2);   
  myservo.setPeriodHertz(50);    
  myservo.attach(servoPin, 1000, 2000); 

  encoder.attachHalfQuad(DT, CLK);    // Attach encoder 1 to DT and CLK pins on ESP32
  encoder2.attachHalfQuad(DT2, CLK2); // Attach encoder 2 to DT and CLK pins on ESP32
  encoder.setCount(0);  // Initialise encoder 1 count to 0
  encoder2.setCount(0); // Initialise encoder 2 count to 0  
}




void loop() {
  
  if (pitchUpdated) {
    updateSteering();
    pitchUpdated = false;
  }

  if (newDataAvailable) {
    for (int i = 0; i < commandIndex; i++) {
    processCommand(commands[i]);
    }
    
    commandIndex = 0;
    memset(commands, 0, sizeof(commands)); // Reset commands array
    newDataAvailable = false;
  }
  
}




void temporaryDisablePID() {
    pidEnabled = false;
}


void restorePID() {
    pidEnabled = true;
}



void updateSteering() {
  
  if (!pidEnabled) return;
    
  else {
  int error = setpoint - pitch;
  P = error;
  I = I + error;
  D = error - previousError;

  PID = (kP * P) + (kI * I) + (kD * D);
  previousError = error;

  Serial.println(PID);

  steeringAngle = map(PID, -50, 50, 180, 0);
  myservo.write(steeringAngle);
  }
  
}



void processCommand(char currentCommand)
{
    switch (currentCommand) 
    {
      case '0':
        myservo.write(90);
        delay(1000);
        break;
      
      case '1':
        go10();      
        break;
      
      case '2':
        go20();
        break;
      
      case '3':
        go30();
        break;
      
      case '4':
        go40();
        break;
      
      case '5':
        go50();
        break;
      
      case '6':
        go60();
        break;
      
      case '7':
        goForward();
        delay(1000);
        break;
      
      case '8':
        temporaryDisablePID();
        turnRight();
        restorePID();
        delay(1000);
        break;
      
      case '9':
        temporaryDisablePID();
        turnLeft();
        restorePID();
        break;
      
      case '*':
        goBackwards();
        delay(1000);
        break;
      
      case '#':
        stopMotors();
        delay(1000);
        break;
    }
}




void receiveData(int howMany) 
{
  
  if (Wire.available() >= 3) {                   // Expecting three bytes
    
    int highByte = Wire.read();                  //-------------------------------------------
    int lowByte = Wire.read();                   //
    pitch = (highByte << 8) | lowByte;           //
    if (highByte & 0x80) {                       // For Handling Pitch Received (as two bytes)
      pitch = pitch | 0xFFFF0000;                //
    }                                            //
    pitchUpdated = true;                         //-------------------------------------------

    
    Key = Wire.read();                           //-------------------------------------------
    if (Key != 0xFF && commandIndex < 20) {      //
      commands[commandIndex++] = Key;            //
    }                                            // For Handling Key Received (as one byte)
    if (commandIndex >= 20 || Key == '#') {      //
      newDataAvailable = true;                   //
    }                                            //-------------------------------------------
    
  }
  
}




void goDistance(int targetDistance) {
  encoder.setCount(0);
  encoder2.setCount(0);
  long targetCounts = targetDistance / pulsePerCM;

  goForward();

  while(true) {
    long count1 = encoder.getCount();
    long count2 = encoder2.getCount();
    float averageDistance = ((count1 + count2) / 2.0) * pulsePerCM;
    if (pidEnabled) {
      updateSteering();
    }
    if (averageDistance >= targetDistance) {
      stopMotors(); 
      break; 
    }
    delay(10);
  }

  if (pidEnabled) {
    myservo.write(90);
  }
  
}



void go10() {
  encoder.setCount(0);
  encoder2.setCount(0); 
  goDistance(7);
}



void go20() {
  encoder.setCount(0);  
  encoder2.setCount(0);
  goDistance(18);
}



void go30() {
  encoder.setCount(0); 
  encoder2.setCount(0); 
  goDistance(28);
}



void go40() {
  encoder.setCount(0); 
  encoder2.setCount(0);
  goDistance(36);
}



void go50() {
  encoder.setCount(0);  // Reset encoder 1 count to 0
  encoder2.setCount(0); // Reset encoder 2 count to 0
  goDistance(45);
}



void go60() {
  encoder.setCount(0);  // Reset encoder 1 count to 0
  encoder2.setCount(0); // Reset encoder 2 count to 0
  goDistance(55);
}



void goForward() {
  digitalWrite(motorPin1, HIGH);
  digitalWrite(motorPin2, LOW); 
  digitalWrite(motorPin3, HIGH);
  digitalWrite(motorPin4, LOW); 

  ledcWrite(pwmChannela, 140);
  ledcWrite(pwmChannelb, 140);
}

int beginTurn() {
  return pitch;
}

/*
void updateSetpointToCurrentPitch() {
  // Set the current pitch as the new setpoint
  setpoint = pitch;
}
*/

void turnLeft() {
  int startValue = beginTurn();
  int endValue = startValue + 139;
  int currentValue = pitch;
  while (pitch < endValue) {
    myservo.write(0);
    goForward();
    motorSpeed(140, 140);
    delay(10);
  }
  //updateSetpointToCurrentPitch();
  stopMotors();
  delay(27000);
}



void turnRight() {
  int startValue = beginTurn();
  int endValue = startValue - 139;
  int currentValue = pitch;
  while (pitch > endValue) {
    myservo.write(180);
    goForward();
    motorSpeed(140, 140);
    delay(10);
  }
  //updateSetpointToCurrentPitch();
  stopMotors();
  delay(27000);
}



void goBackwards() {
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, HIGH); 
  digitalWrite(motorPin3, LOW);
  digitalWrite(motorPin4, HIGH); 
}


void stopMotors() {
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, LOW);
  digitalWrite(motorPin3, LOW);
  digitalWrite(motorPin4, LOW);
  myservo.write(90);
}


void motorSpeed(int leftSpeed, int rightSpeed) {
  ledcWrite(pwmChannela, leftSpeed);
  ledcWrite(pwmChannelb, rightSpeed);
}