// downstairs using encoders for turns and PID

//downstairs code to receive pitch and key
#include <Wire.h>
#include <ESP32Servo.h>
#include <ESP32Encoder.h>

#define CLK 36  // CLK for Encoder 1
#define DT 39   // DT for Encoder 1
#define CLK2 35 // CLK for Encoder 2
#define DT2 34  // DT for Encoder 2

ESP32Encoder encoder;   // Create rotary encoder object encoder (1st encoder)
ESP32Encoder encoder2;  // Create rotary encoder object encoder2 (2nd encoder)

Servo myservo;            // Create servo object myservo
int servoPin = 13;        // Servo pin on downstairs ESP32
int pos = 0;              // Initialise servo position

// L298N Motor Driver pins
const int motorPin1 = 26;  // IN_A pin on downstairs ESP32 (according to schematic); Direction for Motor 1
const int motorPin2 = 27;  // IN_B pin on downstairs ESP32 (according to schematic); Direction for Motor 1
const int motorPin3 = 14;  // IN_C pin on downstairs ESP32 (according to schematic); Direction for Motor 2
const int motorPin4 = 12;  // IN_D pin on downstairs ESP32 (according to schematic); Direction for Motor 2
const int enablePinA = 33; // EN_A pin on downstairs ESP32 (according to schematic); PWM inputs for speed control of Motor 1
const int enablePinB = 25; // EN_B pin on downstairs ESP32 (according to schematic); PWM inputs for speed control of Motor 2

// Setting PWM properties for motors
const int freq = 2000;
const int pwmChannela = 0;  // PWM channel allocation for speed control of Motor 1
const int pwmChannelb = 1;  // PWM channel allocation for speed control of Motor 2
const int resolution = 8;
int dutyCycle = 140;

int pitch;

// For KeyPad
char Key;
char commands[20];
int commandIndex = 0;

// For distance from encoders
const float wheelCircumference = 21.6; // EEEBot rear wheel circumference in cm
const int encoderCPR = 47;             // Counts per revolution; determined empirically (rather than from datasheet)
const float pulsePerCM = wheelCircumference / encoderCPR; // Pulse per centimeter; multiply by counts to get distance in cm

int P, I, D, previousError, PID;
int setpoint = 0;
int kP = 1;
int kI = 0;
int kD = 0;

bool newDataAvailable = false;
bool pitchUpdated = false;
bool pidEnabled = true; // Global flag to enable/disable PID control

int steeringAngle;

void setup() 
{
  Wire.begin(8);                // I2C slave address
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
  ledcAttachPin(enablePinA, pwmChannela); // pwmChannela will define motor speed for motor 1
  ledcAttachPin(enablePinB, pwmChannelb); // pwmChannelb will define motor speed for motor 2

  ESP32PWM::allocateTimer(2);    // Allocate timer for PWM control of servo (timers 0 and 1 of ESP32 already occupied by pwmChannela and pwmChannelb, respectively)
  myservo.setPeriodHertz(50);    // Standard 50 hz servo
  myservo.attach(servoPin, 1000, 2000); // Attach servo (myservo) to control pin (13) according to schematic}

  encoder.attachHalfQuad(DT, CLK);    // Attach encoder 1 to DT and CLK pins on ESP32
  encoder2.attachHalfQuad(DT2, CLK2); // Attach encoder 2 to DT and CLK pins on ESP32
  encoder.setCount(0);  // Initialise encoder 1 count to 0
  encoder2.setCount(0); // Initialise encoder 2 count to 0  
}

void loop() 
{
  if (pitchUpdated)
  {
    // Debugging output
    Serial.print("Pitch: ");
    Serial.println(pitch);
    updateSteering();
    pitchUpdated = false;
  }

  if (newDataAvailable)
  {
    for (int i = 0; i < commandIndex; i++) 
    {
    //char currentCommand = commands[i];
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

void updateSteering()
{
  if (!pidEnabled) return;
  else
  {
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
  // Perform unique actions based on each command
    switch (currentCommand) 
    {
      case '0':
        // Action for command '0'
        Serial.println("Command 0: Straight Steering");
        myservo.write(90);
        delay(1000);
        break;
      case '1':
        // Action for command '1'
        Serial.println("Command 1: Move 10cm");  
        go10();      
        break;
      case '2':
        // Action for command '2'
        Serial.println("Command 2: Move 20cm");
        go20();
        break;
      case '3':
        // Action for command '3'
        Serial.println("Command 3: Move 30cm");
        go30();
        //delay(7000);
        break;
      case '4':
        // Action for command '4'
        Serial.println("Command 4: Move 40cm");
        go40();
        break;
      case '5':
        // Action for command '5'
        Serial.println("Command 5: Move 50cm");
        go50();
        break;
      case '6':
        // Action for command '6'
        Serial.println("Command 6: Move 60cm");
        go60();
        break;
      case '7':
        // Action for command '7'
        Serial.println("Command 7: Move 70cm");
        goForward();
        delay(1000);
        break;
      case '8':
        // Action for command '8'
        Serial.println("Command 8: Turn Right");
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
        // Action for command '*'
        Serial.println("Command *: Going Backwards");
        goBackwards();
        delay(1000);
        break;
      case '#':
        // Action for command '#'
        Serial.println("Command #: Ending Sequence/Stopping Motors");
        stopMotors();
        delay(1000);
        break;
    }
}

void receiveData(int howMany) 
{
  if (Wire.available() >= 3) 
  { // Expecting 3 bytes
    
    int highByte = Wire.read(); // High byte of pitch
    int lowByte = Wire.read();  // Low byte of pitch
    pitch = (highByte << 8) | lowByte; 
    if (highByte & 0x80) 
    {
      pitch = pitch | 0xFFFF0000; // Sign extension for negative values
    }
    pitchUpdated = true; // Set pitch update flag

    Key = Wire.read(); // Read the key or placeholder
    if (Key != 0xFF && commandIndex < 20) 
    { // Ensure there's space for new commands
      commands[commandIndex++] = Key;
    }
    // Check if conditions are met to process commands
    if (commandIndex >= 20 || Key == '#') 
    {
      newDataAvailable = true; // Set flag to true only when conditions are met
    }
  }
}

void goDistance(int targetDistance)
{
  encoder.setCount(0);  // Reset encoder counts to 0
  encoder2.setCount(0);

  // Convert target distance to target encoder counts
  long targetCounts = targetDistance / pulsePerCM;

  goForward(); // Start moving forward
  //motorSpeed(140, 140);

  while(true)
  {
    long count1 = encoder.getCount();   // Current count for encoder 1
    long count2 = encoder2.getCount();  // Current count for encoder 2

    // Calculate the average distance based on encoder counts
    float averageDistance = ((count1 + count2) / 2.0) * pulsePerCM;

    if (pidEnabled)
    {
      updateSteering(); // Continuously adjust steering based on current pitch
    }
  
    // Check if the vehicle has reached or exceeded the target distance
    if (averageDistance >= targetDistance) 
    {
      stopMotors(); // Stop motors
      break; // Exit the loop
    }

    // Small delay to prevent spamming the CPU, adjust as necessary for responsiveness vs CPU usage
    delay(10);
  }
  if (pidEnabled) {
    myservo.write(90); // Center the servo if PID is still considered active
  }
}

void go10()
{
  myservo.write(steeringAngle);
  encoder.setCount(0);  // Reset encoder 1 count to 0
  encoder2.setCount(0); // Reset encoder 2 count to 0
  goDistance(7);
}

void go20()
{
  //myservo.write(90);
  encoder.setCount(0);  // Reset encoder 1 count to 0
  encoder2.setCount(0); // Reset encoder 2 count to 0
  goDistance(18);
}

void go30()
{
  myservo.write(steeringAngle);
  encoder.setCount(0);  // Reset encoder 1 count to 0
  encoder2.setCount(0); // Reset encoder 2 count to 0
  goDistance(28);
}

void go40()
{
  //myservo.write(90);
  encoder.setCount(0);  // Reset encoder 1 count to 0
  encoder2.setCount(0); // Reset encoder 2 count to 0
  goDistance(36);
}

void go50()
{
  myservo.write(steeringAngle);
  encoder.setCount(0);  // Reset encoder 1 count to 0
  encoder2.setCount(0); // Reset encoder 2 count to 0
  goDistance(45);
}

void go60()
{
  //myservo.write(90);
  encoder.setCount(0);  // Reset encoder 1 count to 0
  encoder2.setCount(0); // Reset encoder 2 count to 0
  goDistance(55);
}

void goForward()
{
  digitalWrite(motorPin1, HIGH);
  digitalWrite(motorPin2, LOW); 
  digitalWrite(motorPin3, HIGH);
  digitalWrite(motorPin4, LOW); 

  ledcWrite(pwmChannela, 140);
  ledcWrite(pwmChannelb, 140);
}

void turnLeft() 
{
  encoder.setCount(0);  // Reset encoder counts to 0
  encoder2.setCount(0);
  myservo.write(0); // Ensure straight movement

  // Convert target distance to target encoder counts
  long targetCounts = 44 / pulsePerCM;

  goForward(); // Start moving forward
  motorSpeed(140, 140);

  while(true)
  {
    long count1 = encoder.getCount();   // Current count for encoder 1
    long count2 = encoder2.getCount();  // Current count for encoder 2

    // Calculate the average distance based on encoder counts
    float averageDistance = ((count1 + count2) / 2.0) * pulsePerCM;
    // Check if the vehicle has reached or exceeded the target distance
    if (averageDistance >= 44) 
    {
      stopMotors(); // Stop motors
      encoder.setCount(0);  // Reset encoder counts to 0
      encoder2.setCount(0);
      break; // Exit the loop
    }

    // Small delay to prevent spamming the CPU, adjust as necessary for responsiveness vs CPU usage
    delay(10);
  }
  delay(27000);
}

void turnRight()
{
  encoder.setCount(0);  // Reset encoder counts to 0
  encoder2.setCount(0);
  myservo.write(180); // Ensure straight movement

  // Convert target distance to target encoder counts
  long targetCounts = 32 / pulsePerCM;

  goForward(); // Start moving forward
  motorSpeed(140, 140);

  while(true)
  {
    long count1 = encoder.getCount();   // Current count for encoder 1
    long count2 = encoder2.getCount();  // Current count for encoder 2

    // Calculate the average distance based on encoder counts
    float averageDistance = ((count1 + count2) / 2.0) * pulsePerCM;
    // Check if the vehicle has reached or exceeded the target distance
    if (averageDistance >= 32) 
    {
      stopMotors(); // Stop motors
      encoder.setCount(0);  // Reset encoder counts to 0
      encoder2.setCount(0);
      break; // Exit the loop
    }

    // Small delay to prevent spamming the CPU, adjust as necessary for responsiveness vs CPU usage
    delay(10);
  }
  delay(27000);
}
void goBackwards()
{
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, HIGH); 
  digitalWrite(motorPin3, LOW);
  digitalWrite(motorPin4, HIGH); 
}
void stopMotors()
{
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, LOW);
  digitalWrite(motorPin3, LOW);
  digitalWrite(motorPin4, LOW);
  myservo.write(90);
}
void motorSpeed(int leftSpeed, int rightSpeed)
{
  ledcWrite(pwmChannela, leftSpeed);
  ledcWrite(pwmChannelb, rightSpeed);
}