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
  this->actual_encoder_pos = 0;
  this->old_encoder_pos = 0;

  this->pwm_min = 64;
  this->pwm_max = 255;

  this->speed_offset = 3;
  this->goal_speed = 0; //0 to 10
  this->avg_speed = 0;  //0 to 10
  this->acc_error = 0;
}

int DCMotor::speedToPwm(int speed)
{
  /*SPEED   PWM
      10    PWM_MAX
      0     PWM_MIN*/
  if(speed>10){
    speed = 10;
  }else{
    if(speed<0){
      speed = 0;
    }
  }
  return (((this->pwm_max-this->pwm_min)/10)*speed + this->pwm_min);
}

void DCMotor::move(float meters, int direction)
{
  /*if(motor_L.encoderRead() % WHEEL_FULLBACK <= 10){
      //A complete WHEEL_FULLBACK;
  }*/
}

void DCMotor::move(int speed, int direction)
{
  /********************************
  *   SPEED:      0 - 64          *
  *              10 - full speed  *
  *   DIRECTION:  0 - reverse     *
  *               1 - forward     *
  *********************************/
  int pwm_value = speedToPwm(speed);

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
  analogWrite(this->pwm_pin,pwm_value);
}

void DCMotor::stbyEnable()
{
  //Enable STANDBY
  digitalWrite(this->stby_pin,LOW);
}

void DCMotor::stbyDisable()
{
  //Disable STANDBY
  digitalWrite(this->stby_pin,HIGH);
}

int DCMotor::speedLimitControl(int desired_speed)
{
  if(desired_speed < 0){
    return 0;
  }else{
    if(desired_speed > 10){
      return 10;
    }else{
      return desired_speed;
    }
  }
}

void DCMotor::speedUpdate()
{
  int actual_speed = 0;
  int new_pwm = 0;
  int error = 0;

  encoderPositionUpdate();
  /*if(this->actual_encoder_pos%WHEEL_FULLBACK <= 10){
    //1 Full back
  }*/
  actual_speed = this->actual_encoder_pos - this->old_encoder_pos - this->speed_offset;

  this->avg_speed = 0.9*this->avg_speed + 0.1*actual_speed;

  error = this->goal_speed - actual_speed;
  this->acc_error += error;
  new_pwm = 100*error + this->acc_error/8;
  digitalWrite(this->pwm_pin,new_pwm);
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

int DCMotor::getPwmMin()
{
  return this->pwm_min;
}

void DCMotor::setPwmMin(int new_pwm_min)
{
  this->pwm_min = new_pwm_min;
}

int DCMotor::getPwmMax()
{
  return this->pwm_max;
}

void DCMotor::setPwmMax(int new_pwm_max)
{
  this->pwm_max = new_pwm_max;
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
