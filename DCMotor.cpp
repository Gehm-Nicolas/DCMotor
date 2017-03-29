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

  this->goal_speed = 0;
  this->avg_speed = 0;

  this->present_encoder_pos = 0;
  this->last_encoder_pos = 0;

  this->goal_position = -1;
}

void DCMotor::receiveData(int memAddress, int data)
{
  //TODO
}
void DCMotor::sendData()
{
  //TODO
}

void DCMotor::move(int pwm_value)
{
  /********************************
  *   DIRECTION:  0  - REVERSE    *
  *               1  - FORWARD    *
  *               2  - STOP       *
  *********************************/
  if(this->id == LEFT_MOTOR){
    if(this->direction == STOP){
      stbyEnable();
      digitalWrite(in1_pin,LOW);
      digitalWrite(in2_pin,LOW);
    }else{
      if(this->direction == FORWARD){
        stbyDisable();
        digitalWrite(in1_pin,HIGH);
        digitalWrite(in2_pin,LOW);
      }else{
        stbyDisable();
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
        stbyDisable();
        digitalWrite(in1_pin,LOW);
        digitalWrite(in2_pin,HIGH);
      }else{
        stbyDisable();
        digitalWrite(in1_pin,HIGH);
        digitalWrite(in2_pin,LOW);
      }
    }
  }
  analogWrite(this->pwm_pin,pwm_value);
}

int DCMotor::update()
{
  static unsigned long lastMilli = 0;//loop timer

  int data = 0;
  int new_pwm = 0;

  if((millis() - lastMilli) >= ENCODER_UPDATE_TIME)
  {
    lastMilli = millis();
    encoderUpdate();
    new_pwm = speedUpdate();
    move(new_pwm);
    return new_pwm;
  }
}

int DCMotor::speedToPwm()
{
  /*SPEED:0   1   2   3   4   5   6   7   8   9   10
    PWM  :0   25  50  75  100 125 150 175 200 225 250
  */
  return 25*this->goal_speed;
}

void DCMotor::encoderUpdate()
{
    this->last_encoder_pos = this->present_encoder_pos;
    this->present_encoder_pos = getEncoder();
}

int DCMotor::speedUpdate()
{
  /*PWM correction using PID*/
  static int error_acc = 0;
  //static int last_error = 0;

  int present_encoder_speed = 0;
  int goal_encoder_speed = 0;
  int error = 0;
  int pwm_correction = 0;
  int new_pwm = 0;

  float Kp = 2;
  float Ki = 1/8;
  //float Kd = 0;

  present_encoder_speed = abs(this->present_encoder_pos - this->last_encoder_pos);
  goal_encoder_speed = 25*this->goal_speed;

  this->avg_speed = 0.9*this->avg_speed + 0.1*present_encoder_speed;

  error = goal_encoder_speed - present_encoder_speed;
  error_acc += error;

  //pwm_correction = Kp*error + Ki*error_acc + Kd*(last_error - error);
  pwm_correction = Kp*error + Ki*error_acc;
  new_pwm = speedToPwm() + pwm_correction;

  //last_error = error;

  return constrain(new_pwm,MIN_PWM,MAX_PWM);
}


/*int DCMotor::speedUpdate()
{

  present_speed = this->present_encoder_pos - this->old_encoder_pos;
  this->avg_speed = 0.9*this->avg_speed + 0.1*present_speed;

  //error = abs(this->goal_speed - present_speed);
  error =  abs(this->avg_speed - present_speed);

  this->acc_error += error;
  pwm_correction = 100*error/this->goal_speed + this->acc_error/8;
  return present_speed;
}*/





int DCMotor::updatePID(float desired, float present)
{
/*int maxACC;
  int maxPID = this->pwm_max;
  inr minPID = this->pwm_min;
  int acc = 0;
  int err = 0;
  float p = 100;
  float i = 1/8;
  float d = ;
  float error = desired - present;

  acc += error;

  int p_term = p * error;
  int i_term = i * constrain(acc,-max_acc,max_acc);
  int d_term = d * (error - err);

  int pid = p_term + i_term + d_term;
  return constrain(pid,this->pwm_min,this->pwm_max);*/
}


void DCMotor::stbyEnable()                            {digitalWrite(this->stby_pin,LOW);}
void DCMotor::stbyDisable()                           {digitalWrite(this->stby_pin,HIGH);}

long DCMotor::getEncoder()                            {return _encoder.read();}//I believe that 44 is equivalent a full back (360º)
void DCMotor::setEncoder(long pos)                    {_encoder.write(pos);}

int DCMotor::getID()                                  {return this->id;}
void DCMotor::setID(int new_ID)                       {this->id = new_ID;}

int DCMotor::getDirection()                           {return this->direction;}
void DCMotor::setDirection(int new_direction)         {this->direction = new_direction;}

//int DCMotor::getPwmMin()                              {return this->pwm_min;}
//void DCMotor::setPwmMin(int new_pwm_min)              {this->pwm_min = new_pwm_min;}

//int DCMotor::getPwmMax()                              {return this->pwm_max;}
//void DCMotor::setPwmMax(int new_pwm_max)              {this->pwm_max = new_pwm_max;}

//int DCMotor::getSpeedOffset()                         {return this->speed_offset;}
//void DCMotor::setSpeedOffset(int new_speed_offset)    {this->speed_offset = new_speed_offset;}

int DCMotor::getGoalSpeed()                           {return this->goal_speed;}
void DCMotor::setGoalSpeed(int new_speed)             {this->goal_speed = new_speed;}

float DCMotor::getGoalPosition()                      {return this->goal_position;}
void DCMotor::setGoalPosition(float new_goal_position){this->goal_position = new_goal_position;}

int DCMotor::getAvgSpeed()                            {return this->avg_speed;}
