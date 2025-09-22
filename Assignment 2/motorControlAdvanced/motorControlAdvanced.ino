/*Author: Jesus Ledezma
 *Date of creation: 9/9/2025
 *Assignment 2: Exercise 2a
 *Purpose: Motor Velocity Control (P-controller)
 *Note: Current setup has Bravo using motor1 connections and Alpha using motor2 
 *ie Brave red connected to M1A, black connected to M1B, Encoder yellow connected to digital pin 5, white connected to digital pin 2
 *Alpha red connected to M2A, black connected to M2B, Encoder yellow connected to digital pin 3, white connected to digital pin 6
 */

float batteryVoltage = 7.5; 

// Motorshield interface Variables
int enablePin = 4; 
int m1VoltageSign = 7; //control direction 
int m2VoltageSign = 8;
int m1Voltage = 9; //control speed
int m2Voltage = 10;

//Encoder variables
volatile long encoderCounts1 = 0;
int APIN = 2;
int BPIN = 5; //pin 5 should be free for digital reading, pin 4 is need for enable, DON'T USE
int thisA; 
int thisB;
int lastA;
int lastB;

volatile long encoderCounts2 = 0;
int APIN2 = 3;
int BPIN2 = 6;
int thisA2;
int thisB2;
int lastA2;
int lastB2;

// Time variables
unsigned long desired_Ts_ms = 5; // desired sample time in milliseconds
unsigned long last_time_ms;
unsigned long start_time_ms;
float deltaTime = (float)desired_Ts_ms / 1000;//((float)(millis()-last_time_ms))/1000; -> PROBLEMATIC!!!!, time passed between millis and last_time_ms is not long enough for integer precision
float current_time;

// Variables for calculating position
long pos1_counts; //current position
long pos2_counts;
float pos1_rad; //in radians
float pos2_rad;
float last_pos1; //last position in radians
float last_pos2;

// Variables for calculating velocity & error
float deltaDist1; //change in distance(radians)
float deltaDist2;
float angularVelocity1; //radians/sec -> do we have to convert ms to s?
float angularVelocity2;
float angularVelocityDesired = -2 * PI;

//Variables for P controller
float directionConstant = 1;
float e1; //error
float e2;
float c1; //control
float c2;
int Kp1 = 10; //found optimal values in simulation
int Kp2 = 10;
int m1Pwm = 0; 
int m2Pwm = 0;

void setup() {
  pinMode(enablePin, OUTPUT);
  pinMode(m1VoltageSign, OUTPUT);
  pinMode(m1Voltage, OUTPUT);
  pinMode(m2VoltageSign, OUTPUT);
  pinMode(m2Voltage, OUTPUT);

  digitalWrite(enablePin, HIGH);

  digitalWrite(m1VoltageSign, LOW); //switching these between high and low will switch the voltage hooked up to the motors, changing their direction
  digitalWrite(m2VoltageSign, HIGH); 

  
  if (angularVelocityDesired < 0) { //used if we want negative angular velocity
    directionConstant = -1;
    digitalWrite(m1VoltageSign, HIGH); 
    digitalWrite(m2VoltageSign, LOW);
  }

  pinMode(APIN, INPUT);
  pinMode(BPIN, INPUT);
  
  pinMode(APIN2, INPUT);
  pinMode(BPIN2, INPUT);

  attachInterrupt(digitalPinToInterrupt(APIN), APIN_Interrupt,CHANGE);
  attachInterrupt(digitalPinToInterrupt(APIN2), APIN2_Interrupt, CHANGE);
  
  Serial.begin(115200);
  last_time_ms = millis();
  start_time_ms = last_time_ms;

  last_pos1 = 0; 
  last_pos2 = 0;
}

void APIN_Interrupt(){
  thisA = digitalRead(APIN);
  thisB = digitalRead(BPIN);

  if (thisA == thisB) {
    encoderCounts1 += 2;
    //rotated 2 counts in one direction
  } else {
    encoderCounts1 -= 2;
    // 2 counts in other direction
  }

  lastA = thisA;
  lastB = thisB;
}

void APIN2_Interrupt(){ 
  thisA2 = digitalRead(APIN2);
  thisB2 = digitalRead(BPIN2);

  if (thisA2 == thisB2) {
    encoderCounts2 += 2; 
    //rotated 2 counts in one direction
  } else {
    encoderCounts2 -=2;
    // 2 counts in other direction
  }

  lastA2 = thisA2;
  lastB2 = thisB2;
}


long MyEnc(int motorSelect) { //encoderCount correction function, can be used for m1 or m2
  
  int prevA, prevB, currA, currB, readPinA, readPinB;
  long counts;

  if (motorSelect == 1) {
    prevA = lastA;
    prevB = lastB;
    currA = thisA;
    currB = thisB;
    readPinA = APIN;
    readPinB = BPIN;
    counts = encoderCounts1;
  } else if (motorSelect == 2) {
    prevA = lastA2;
    prevB = lastB2;
    currA = thisA2;
    currB = thisB2;
    readPinA = APIN2;
    readPinB = BPIN2;
    counts = encoderCounts2;
  }

  currA = digitalRead(readPinA);
  currB = digitalRead(readPinB);

  int prev = (prevA << 1) | prevB;
  int curr = (currA << 1) | currB;

  switch(prev) {
    case 0b00:
      if(curr == 0b01) counts++; // CW
      else if(curr == 0b10) counts--; // CCW
      break;
    case 0b01:
      if(curr == 0b11) counts++; // CW
      else if(curr == 0b00) counts--; // CCW
      break;
    case 0b11:
      if(curr == 0b10) counts++; // CW
      else if(curr == 0b01) counts--; // CCW
      break;
    case 0b10:
      if(curr == 0b00) counts++; // CW
      else if(curr == 0b11) counts--; // CCW
      break;
}
  prevA = currA;
  prevB = currB; 

  return counts;
}

void loop() {

  pos1_counts = MyEnc(1); 
  pos1_rad= 2*PI*(float)encoderCounts1/3200; //have current position, not change in distance
  pos2_counts = MyEnc(2);
  pos2_rad= 2*PI*(float)encoderCounts2/3200; //changed from pos2_counts to encoderCounts2, 

  deltaDist1 = pos1_rad - last_pos1;//change in distance
  deltaDist2 = pos2_rad - last_pos2;//change in distance

  angularVelocity1 = deltaDist1 / deltaTime; //rad / s
  angularVelocity2 = deltaDist2 / deltaTime;

  e1 = (angularVelocityDesired - angularVelocity1) * directionConstant;
  e2 = (angularVelocityDesired - angularVelocity2) * directionConstant;

  c1 = Kp1 * e1;
  c2 = Kp2 * e2;

  m1Pwm = constrain((c1 * 255) / batteryVoltage, 0, 255); 
  m2Pwm = constrain((c2 * 255) / batteryVoltage, 0, 255);
  
  analogWrite(m1Voltage, m1Pwm);
  analogWrite(m2Voltage, m2Pwm);

  current_time = (float)(last_time_ms-start_time_ms)/1000;
  if (current_time <= 3) { //output current time, m1 voltage, and m1 angular velocity
    Serial.print(current_time);
    Serial.print("\t");
    Serial.print(((batteryVoltage * (float)m1Pwm) / 255) * directionConstant);  
    Serial.print("\t");
    Serial.print(angularVelocity1);

    //Serial.print("\t"); //optionally output m2 angular velocity as well
    //Serial.print(angularVelocity2);
  
    Serial.println("");
  }
  
  while (millis()<last_time_ms + desired_Ts_ms) {}  //delay
  last_time_ms = millis();
  last_pos1 = pos1_rad;
  last_pos2 = pos2_rad;
}
