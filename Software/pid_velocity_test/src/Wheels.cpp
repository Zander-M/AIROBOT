#include <Arduino.h>
#include "Wheels.h"
#include "pinout.h"

// Initial setup
volatile int l_pos = 0; // specify posi as volatile: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
volatile int r_pos = 0; // specify posi as volatile: https://www.arduino.cc/reference/en/language/variables/variable-scope-qualifiers/volatile/
volatile int l_target = 0;
volatile int r_target = 0;
long prevT = 0;
float l_eprev = 0;
float l_eintegral = 0;
float r_eprev = 0;
float r_eintegral = 0;

// PID parameter
float kp = 100;
float kd = 0.15;
float ki = 0.0;

int min_power = 50;
int max_power = 255;

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
  int l_e = l_pos - l_target;
  int r_e = r_pos - r_target;

  // derivative
  float l_dedt= (l_e-l_eprev)/(deltaT);
  float r_dedt = (r_e-r_eprev)/(deltaT);

  // integral
  l_eintegral = l_eintegral + l_e*deltaT;
  r_eintegral = r_eintegral + r_e*deltaT;

  // control signal
  float l_u = kp*l_e + kd*l_dedt + ki*l_eintegral;
  float r_u = kp*r_e + kd*r_dedt + ki*r_eintegral;

  // motor power
  float l_pwr = fabs(l_u);
  if( l_pwr > 255 ){
    l_pwr = 255;
  }

  float r_pwr = fabs(r_u);
  if( r_pwr > 255 ){
    r_pwr = 255;
  }

  // motor direction
  int l_dir = 1;
  if(l_u<0){
    l_dir = -1;
  }

  int r_dir = -1;
  if(r_u<0){
    r_dir = 1;
  }

  // store previous error
  l_eprev = l_e;
  r_eprev = r_e;

  // signal the motor
  setLeftMotor(l_dir,l_pwr);
  setRightMotor(r_dir,r_pwr);

  Serial.print(l_target);
  Serial.print(" ");
  Serial.print(l_pos);
  Serial.print(" ");
  Serial.print(r_target);
  Serial.print(" ");
  Serial.print(r_pos);
  Serial.println();
}

void setLeftMotor(int dir, int pwmVal){
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

void setRightMotor(int dir, int pwmVal){
  if(dir == -1){
    digitalWrite(ME2A,LOW);
    analogWrite(ME2B,pwmVal);
  }
  else if(dir == 1){
    digitalWrite(ME2A, HIGH);
    analogWrite(ME2B,255-pwmVal);
  }
  else{
    digitalWrite(ME2A,LOW);
    analogWrite(ME2B,0);
  }  
}

void readLeftEncoder(){
  int b = digitalRead(E1B);
  if(b > 0){
    l_pos++;
  }
  else{
    l_pos--;
  }
}

void readRightEncoder(){
  int b = digitalRead(E2B);
  if(b > 0){
    r_pos--;
  }
  else{
    r_pos++;
  }
}

void setLeftTarget(int targetVal){
    l_target = targetVal;
}

void setRightTarget(int targetVal){
    r_target = targetVal;
}