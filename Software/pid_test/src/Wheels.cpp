#include <Arduino.h>
#include "Wheels.h"
#include "pinout.h"

// Initial setup
volatile int pos = 0; // specify posi as volatile: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
volatile int target = 0;
long prevT = 0;
float eprev = 0;
float eintegral = 0;

// PID parameter
float kp = 100;
float kd = 0.15;
float ki = 0.0;

void wheel_setup() {
  pinMode(E1A,INPUT);
  pinMode(E1B,INPUT);
  attachInterrupt(E1A,readLeftEncoder,RISING);

  pinMode(E2A,INPUT);
  pinMode(E2B,INPUT);
  attachInterrupt(E2A,readRightEncoder,RISING);
  
  pinMode(ME1A,OUTPUT);
  pinMode(ME1B,OUTPUT);
  pinMode(ME2A,OUTPUT);
  pinMode(ME2B,OUTPUT);
  
  Serial.println("Wheels Initialized.");
}

void wheel_run() {

  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;

  // Read the position in an atomic block to avoid a potential
  // misread if the interrupt coincides with this code running
  // see: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
  
  // error
  int e = pos - target;

  // derivative
  float dedt = (e-eprev)/(deltaT);

  // integral
  eintegral = eintegral + e*deltaT;

  // control signal
  float u = kp*e + kd*dedt + ki*eintegral;

  // motor power
  float pwr = fabs(u);
  if( pwr > 255 ){
    pwr = 255;
  }

  // motor direction
  int dir = 1;
  if(u<0){
    dir = -1;
  }

  // signal the motor
  setMotor(dir,pwr);


  // store previous error
  eprev = e;

  Serial.print(target);
  Serial.print(" ");
  Serial.print(pos);
  Serial.println();
}

void setMotor(int dir, int pwmVal){
  if(dir == -1){
    digitalWrite(ME1A,LOW);
    analogWrite(ME1B,pwmVal);
  }
  else if(dir == 1){
    digitalWrite(ME1A, HIGH);
    analogWrite(ME1B,255-pwmVal);
  }
  else{
    digitalWrite(ME1A,LOW);
    digitalWrite(ME1B,0);
  }  
}

void readLeftEncoder(){
  int b = digitalRead(E1B);
  if(b > 0){
    pos++;
  }
  else{
    pos--;
  }
}

void readRightEncoder(){
  int b = digitalRead(E2B);
  if(b > 0){
    pos++;
  }
  else{
    pos--;
  }
}

void setTarget(int targetVal){
    target = targetVal;
}