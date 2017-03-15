#include <Arduino.h>
#include "DCMotor.h"

DCMotor::DCMotor(int id,int pin2, int pin1, int pinPWM,int pinSTBY, Encoder& enc): _encoder(enc)
{
  pinMode(pin1,OUTPUT);
  pinMode(pin2,OUTPUT);
  pinMode(pinPWM,OUTPUT);//Speed control
  pinMode(pinSTBY,OUTPUT);

  this->in1_pin = pin1;
  this->in2_pin = pin2;
  this->pwm_pin = pinPWM;
  this->stby_pin = pinSTBY;

  this->id = id;
  this->direction = STOP;
  this->pwm_min = 0;
  this->pwm_max = 255;

  this->speed_offset = 0;
  this->goal_speed = 0;
  this->avg_speed = 0;

  this->acc_error = 0;
  this->actual_encoder_pos = 0;
  this->old_encoder_pos = 0;
}

void DCMotor::receiveData(int goal_speed, int direction)
{
  this->direction = direction;

  if(goal_speed>10){
    this->goal_speed = 10;
  }else{
    if(goal_speed<0){
      this->goal_speed = 0;
    }else{
      this->goal_speed = goal_speed;
    }
  }
}

int DCMotor::speedToPwm(int speed){
  //return (((this->pwm_max-this->pwm_min)/10)*speed + this->pwm_min);
  return (((this->pwm_max-this->pwm_min)/MAX_SPEED)*speed + this->pwm_min);
}
void DCMotor::move(float meters, int direction)
{
  /*if(motor_L.getEncoder() % WHEEL_FULLBACK <= 10){
      //A complete WHEEL_FULLBACK;
  }*/
}
void DCMotor::move()
{
  /********************************
  *   DIRECTION:  0  - reverse    *
  *               1  - forward    *
  *********************************/

  //int pwm_value = speedToPwm(this->goal_speed);
  int pwm_value = speedToPwm(this->goal_speed) + speedUpdate();


  digitalWrite(this->stby_pin, HIGH);//disable standby

  if(this->id == LEFT_MOTOR){
    if(this->direction == STOP){
      stbyEnable();
      digitalWrite(in1_pin,LOW);
      digitalWrite(in2_pin,LOW);
    }else{
      if(this->direction == FORWARD){
        digitalWrite(in1_pin,HIGH);
        digitalWrite(in2_pin,LOW);
      }else{
        digitalWrite(in1_pin,LOW);
        digitalWrite(in2_pin,HIGH);
      }
    }
  }else{//RIGHT_MOTOR
    if(this->direction == STOP){
      stbyEnable();
      digitalWrite(in1_pin,LOW);
      digitalWrite(in2_pin,LOW);
    }else{
      if(this->direction == FORWARD){
        digitalWrite(in1_pin,HIGH);
        digitalWrite(in2_pin,LOW);
      }else{
        digitalWrite(in1_pin,LOW);
        digitalWrite(in2_pin,HIGH);
      }
    }
  }
  analogWrite(this->pwm_pin,pwm_value);
}

int DCMotor::speedUpdate()
{
  int actual_speed = 0;
  unsigned int pwm_correction = 0;
  int error = 0;

  encoderPositionUpdate();
  /*if(this->actual_encoder_pos%WHEEL_FULLBACK <= 10){
    //1 Full back
  }*/
  actual_speed = this->actual_encoder_pos - this->old_encoder_pos + speed_offset;
  this->avg_speed = 0.9*this->avg_speed + 0.1*actual_speed;

  error = this->goal_speed - abs(actual_speed);

  this->acc_error += error;
  pwm_correction = 25*error/this->goal_speed + this->acc_error/8;
  return pwm_correction;
}

int DCMotor::calcPID(float desired, float actual)
{
/*  float max_acc;
  float max_pid;
  float acc = 0.0;
  float err = 0.0;
  float p;
  float i;
  float d;
  float error = desired - actual;

  acc += error;

  float p_term = p * error;
  float i_term = i * constrain(acc,-max_acc,max_acc);
  float d_term = d * (error - err);

  err = error;

  float pid = p_term + i_term + d_term;
  return constrain(pid,-max_pid,max_pid);*/
}

void DCMotor::encoderPositionUpdate()
{
  this->old_encoder_pos = this->actual_encoder_pos;
  this->actual_encoder_pos = getEncoder();
  delay(2);//In order to decrease update frequency
}

void DCMotor::sendData()
{
  //TODO
}

void DCMotor::stbyEnable()                          {digitalWrite(this->stby_pin,LOW);}
void DCMotor::stbyDisable()                         {digitalWrite(this->stby_pin,HIGH);}
long DCMotor::getEncoder()                          {return _encoder.read();}//I believe that 44 is equivalent a full back (360º)
void DCMotor::setEncoder(long pos)                  {_encoder.write(pos);}
int DCMotor::getID()                                {return this->id;}
void DCMotor::setID(int new_ID)                     {this->id = new_ID;}
int DCMotor::getDirection()                         {return this->direction;}
void DCMotor::setDirection(int new_direction)       {this->direction = new_direction;}
int DCMotor::getPwmMin()                            {return this->pwm_min;}
void DCMotor::setPwmMin(int new_pwm_min)            {this->pwm_min = new_pwm_min;}
int DCMotor::getPwmMax()                            {return this->pwm_max;}
void DCMotor::setPwmMax(int new_pwm_max)            {this->pwm_max = new_pwm_max;}
int DCMotor::getSpeedOffset()                       {return this->speed_offset;}
void DCMotor::setSpeedOffset(int new_speed_offset)  {this->speed_offset = new_speed_offset;}
int DCMotor::getGoalSpeed()                         {return this->goal_speed;}
void DCMotor::setGoalSpeed(int new_speed)           {this->goal_speed = new_speed;}
int DCMotor::getAvgSpeed()                          {return this->avg_speed;}
