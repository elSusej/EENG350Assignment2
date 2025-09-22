/*Author: Jesus Ledezma
 *Date of creation: 9/9/2025
 *Assignment 2: Exercise 1b
 *Purpose: Spinnning DC motor w/Arduino
 */

int enablePin = 4;
int m1VoltagePin = 9; //controls speed for M1
int m1VoltageSign = 7; //controls direction for M1

void setup() {
  pinMode(enablePin, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(9, OUTPUT);

  digitalWrite(enablePin, HIGH);
  analogWrite(9, 50);
  digitalWrite(7, HIGH); //switching this between high and low will switch the voltage hooked up to the motor, changing it's direction
}

void loop() {
}
