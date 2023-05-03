/*
   Course:  MECHENG 890
   Ankle Rehab Arduino Control Code
   
   This program applies position control to 3 DC motors 

   Pins
   - The details of each pin are within the code.
   
   The circuit
   -The sketch of the circuit is attach to this file. 
 */

// Definition of constant values. 
#define convM1 1//1.28 //360/280
#define convM2 1//3.33 //360/108
#define convM3 1//3.33 //360/108
#define velM1 20   // Intensity of the pwm signal
#define velM2 20   // Intensity of the pwm signal
#define velM3 20   // Intensity of the pwm signal
const double tolM1 = 10;
const double tolM2 = 10;
const double tolM3 = 0.5;

// emergency and limit switch pins
const byte emergency = 50;

// limit Switch setup
const byte m1LowLim = 35;
const byte m2LowLim = 37;
const byte m3LowLim = 39;

const byte m1UpLim = 34;
const byte m2UpLim = 36;
const byte m3UpLim = 38;

// motor pwm pins (Used to set the maximum speed of each motor)
const byte motor1 = 4;
const byte motor2 = 3;
const byte motor3 = 2;

// Interrupt pins (Used to trigger the counting of position of each motor)
const byte interruptPin1 = 19;
const byte interruptPin2 = 20;
const byte interruptPin3 = 21;

// Direction pins (Used to obtain information about the direction at which the motors are rotating)
const byte dirPin1 = 28;
const byte dirPin2 = 30;
const byte dirPin3 = 32;

// Actions pins Motor 1 (d stands for down, while u stands for up)
const byte d1 = 22;
const byte u1 = 23;

// Actions pins Motor 2 (d stands for down, while u stands for up)
const byte d2 = 24;
const byte u2 = 25;

// Actions pins Motor 3 (d stands for down, while u stands for up)
const byte d3 = 26;
const byte u3 = 27;

// Switch variables
int emAlarm = 1;
int m1LowLimR = 0;
int m2LowLimR = 0;
int m3LowLimR = 0;
int m1UpLimR = 0;
int m2UpLimR = 0;
int m3UpLimR = 0;

// Initial Position
volatile float iPos1 = 0;
volatile float iPos2 = 0;
volatile float iPos3 = 0;

// Actual Position
volatile float aPos1 = 0;
volatile float aPos2 = 0;
volatile float aPos3 = 0;

// Target Position
float tPos1 = 0;
float tPos2 = 0;
float tPos3 = 0;

// counters
int count1 = 0;
int count2 = 0;
int count3 = 0;
int countSent = 0;

void setup() {
  // start serial communcations
  Serial.begin(9600);
  // Set pins for emergency and limit swicth
  pinMode(emergency, INPUT_PULLUP);
  
  pinMode(m1LowLim, INPUT_PULLUP);
  pinMode(m2LowLim, INPUT_PULLUP);
  pinMode(m3LowLim, INPUT_PULLUP);

  pinMode(m1UpLim, INPUT_PULLUP);
  pinMode(m2UpLim, INPUT_PULLUP);
  pinMode(m3UpLim, INPUT_PULLUP);
  
  // Set pins for the motors
  pinMode(motor1, OUTPUT);
  pinMode(motor2, OUTPUT);
  pinMode(motor3, OUTPUT);
  
  // Set action pins
  pinMode(d1, OUTPUT);
  pinMode(u1, OUTPUT);
  pinMode(d2, OUTPUT);
  pinMode(u2, OUTPUT);
  pinMode(d3, OUTPUT);
  pinMode(u3, OUTPUT);

  // Set direction pins
  pinMode(dirPin1, INPUT);
  pinMode(dirPin2, INPUT);
  pinMode(dirPin3, INPUT);

  // Set Interrupts
  attachInterrupt(digitalPinToInterrupt(interruptPin1), c1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(interruptPin2), c2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(interruptPin3), c3, CHANGE);

//  Serial.println("<Arduino is ready>");
}

void loop() {
  // check for emergency switch value
  emAlarm = digitalRead(emergency);
  
  // Input of the desire angle of the platform

  // Implementation of the inverse kinematics to compute the desire length

  if(emAlarm == 0)
  {
    // Lock the motors
    digitalWrite(d1, LOW);
    digitalWrite(u1, LOW);
    digitalWrite(d2, LOW);
    digitalWrite(u2, LOW);
    digitalWrite(d3, LOW);
    digitalWrite(u3, LOW);
  }
  else
  {
    recvWithStartEndMarkers();
    parseData(); 
    runMotors();
    if(countSent == 100){
      serialSendData();
      countSent = 0;
    }
    countSent++;
  }
  
}

// Function to run motors when the emergency switch is not active
void runMotors()
{
  // Motor 1 control
  if (tPos1 - 0.5 > aPos1)
  {
    digitalWrite(d1, HIGH);
    digitalWrite(u1, LOW);
  }
  else if(tPos1 + 0.5 < aPos1)
  {
    digitalWrite(d1, LOW);
    digitalWrite(u1, HIGH);
  }
  else
  {
    digitalWrite(u1, LOW);
    digitalWrite(d1, LOW);
  }
  // Motor 2 control
  if (tPos2 - 0.5 > aPos2)
  {
    digitalWrite(u2, HIGH);
    digitalWrite(d2, LOW);
  }
  else if(tPos2 + 0.5 < aPos2)
  {
    digitalWrite(u2, LOW);
    digitalWrite(d2, HIGH);
  }
  else
  {
    digitalWrite(u2, LOW);
    digitalWrite(d2, LOW);
  }
  // Motor 3 control
  if (tPos3 - 0.5 > aPos3)
  {
    digitalWrite(u3, HIGH);
    digitalWrite(d3, LOW);
  }
  else if(tPos3 + 0.5 < aPos3)
  {
    digitalWrite(u3, LOW);
    digitalWrite(d3, HIGH);
  }
  else
  {
    digitalWrite(u3, LOW);
    digitalWrite(d3, LOW);
  }
  // Set the velocity of the motors
  analogWrite(motor1, velM1);
  analogWrite(motor2, velM2);
  analogWrite(motor3, velM3);
}

// c1 is activated with the first interrupt, the function counts the number of signals of 
//the encoder and the obtain the value of the actual length of the cable. 
void c1() {
  if (digitalRead(dirPin1) != digitalRead(interruptPin1))
  {
    count1 ++;
    aPos1 = iPos1 - convM1*count1;
  }
  else
  {
    count1 --;
    aPos1 = iPos1 - convM1*count1;
  }
}

//Function count 2
void c2() {
  if (digitalRead(dirPin2) != digitalRead(interruptPin2))
  {
    count2 ++;
    aPos2 = iPos2 + convM2*count2;
  }
  else
  {
    count2 --;
    aPos2 = iPos2 + convM2*count2;
  }
}

//Function count 3
void c3() {
  if (digitalRead(dirPin3) == digitalRead(interruptPin3))
  {
    count3 ++;
    aPos3 = iPos3 - convM3*count3;
  }
  else
  {
    count3 --;
    aPos3 = iPos3 - convM3*count3;
  }
}

// Function to send data to PC
void serialSendData(){
    m1LowLimR = digitalRead(m1LowLim);
    m2LowLimR = digitalRead(m2LowLim);
    m3LowLimR = digitalRead(m3LowLim);

    m1UpLimR = digitalRead(m1UpLim);
    m2UpLimR = digitalRead(m2UpLim);
    m3UpLimR = digitalRead(m3UpLim);
//    Serial.print('<');
//    Serial.print(aPos1);
//    Serial.print(",");
//    Serial.print(aPos2);
//    Serial.print(",");
//    Serial.println(aPos3);
//    Serial.println('>');
    Serial.print(m1LowLimR);
    Serial.print(",");
    Serial.print(m1UpLimR);
    Serial.print(" | ");
    Serial.print(m2LowLimR);
    Serial.print(",");
    Serial.print(m2UpLimR);
    Serial.print(" | ");
    Serial.print(m3LowLimR);
    Serial.print(",");
    Serial.println(m3UpLimR);
    
}
