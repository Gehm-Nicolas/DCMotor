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
  this->direction = FORWARD;
  this->actual_encoder_pos = 0;
  this->old_encoder_pos = 0;

  this->goal_speed = 0;
  this->avg_speed = 0;
  this->acc_error=0;
}

void DCMotor::move(int speed, int direction)
{
  /********************************
  *   SPEED:      0 - off         *
  *             255 - full speed  *
  *   DIRECTION:  0 - reverse     *
  *               1 - forward    *
  *********************************/
  digitalWrite(this->stby_pin, HIGH);//disable standby

  if(this->id == LEFT_MOTOR){
    if(direction == STOP){
      digitalWrite(in1_pin,LOW);
      digitalWrite(in2_pin,LOW);
    }else{
      if(direction == FORWARD){
        digitalWrite(in1_pin,HIGH);
        digitalWrite(in2_pin,LOW);
      }else{
        digitalWrite(in1_pin,LOW);
        digitalWrite(in2_pin,HIGH);
      }
    }
  }else{//RIGHT_MOTOR
    if(direction == STOP){
      digitalWrite(in1_pin,LOW);
      digitalWrite(in2_pin,LOW);
    }else{
      if(direction == FORWARD){
        digitalWrite(in1_pin,LOW);
        digitalWrite(in2_pin,HIGH);
      }else{
        digitalWrite(in1_pin,HIGH);
        digitalWrite(in2_pin,LOW);
      }
    }
  }
  analogWrite(pwm_pin,speed);
}

void DCMotor::stop()
{
  //Enable STANDBY
  digitalWrite(this->stby_pin,LOW);
}

void DCMotor::start()
{
  //Disable STANDBY
  digitalWrite(this->stby_pin,HIGH);
}

int DCMotor::speedUpdate()
{
  int actual_speed = 0;
  int new_speed = 0;
  int error = 0;

  encoderPositionUpdate();
  actual_speed = this->actual_encoder_pos - this->old_encoder_pos;

  this->avg_speed = 0.9*this->avg_speed + 0.1*actual_speed;

  error = this->goal_speed - actual_speed;

  this->acc_error += error;
  new_speed = 100*error + this->acc_error/8;
  return actual_speed;
  //}
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

long DCMotor::encoderRead()
{
  //I believe that 44 is equivalent a full back (360º)
  return _encoder.read();
}

void DCMotor::encoderWrite(long pos)
{
  _encoder.write(pos);
}

void DCMotor::encoderPositionUpdate()
{
  this->old_encoder_pos = this->actual_encoder_pos;
  this->actual_encoder_pos = encoderRead();
  delay(2);//In order to decrease update frequency
}

void DCMotor::receiveData(int goal_speed, int direction)
{
  this->goal_speed = goal_speed;
  this->direction = direction;
}

void DCMotor::sendData()
{
  //TODO
}

int DCMotor::getDirection()
{
  return this->direction;
}

int DCMotor::getAvgSpeed()
{
  return this->avg_speed;
}

void DCMotor::setGoalSpeed(int new_speed)
{
  this->goal_speed = new_speed;
}

int DCMotor::getGoalSpeed()
{
  return this->goal_speed;
}
