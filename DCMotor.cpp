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
  this->last_encoder_pos    = 0;
  this->start_encoder_pos   = 0;

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

int DCMotor::update()
{
  /*Main and only function to control the motor*/
  static unsigned long lastMilli = 0;//loop timer

  if((millis() - lastMilli) >= ENCODER_UPDATE_TIME)
  {
    int new_pwm = 0;
    int encoder_dif = 0;

    lastMilli = millis();
    encoderUpdate();
    encoder_dif = (int)positionUpdate();//encoder_difference to achieve goal
    new_pwm = speedUpdate();
    move(new_pwm);
    return encoder_dif;
  }
}

void DCMotor::move(int pwm_value)
{
  /********************************
  *   DIRECTION:  0  - REVERSE    *
  *               1  - FORWARD    *
  *               2  - STOP       *
  *********************************/
  stbyDisable();

  if(this->id == LEFT_MOTOR){
    if(this->direction == STOP){
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
      digitalWrite(in1_pin,LOW);
      digitalWrite(in2_pin,LOW);
    }else{
      if(this->direction == FORWARD){
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
  //PWM correction using PID//
  static int error_acc = 0;
  static int last_error = 0;

  int present_encoder_speed = 0;
  int goal_encoder_speed = 0;
  int error = 0;
  int pwm_correction = 0;
  int new_pwm = 0;

  float Kp = 100;
  float Ki = 1/8;
  float Kd = 0;

  present_encoder_speed = abs(this->present_encoder_pos) - abs(this->last_encoder_pos);
  goal_encoder_speed = this->goal_speed;

  this->avg_speed = 0.9*this->avg_speed + 0.1*present_encoder_speed;

  error = goal_encoder_speed - present_encoder_speed;
  error_acc += error;

  pwm_correction = Kp*error + Ki*error_acc + Kd*(last_error - error)/ENCODER_UPDATE_TIME;

  last_error = error;

  return constrain(pwm_correction,MIN_PWM,MAX_PWM);
}

long DCMotor::positionUpdate()
{
  /*Only runs when there is a goal_position (Centimeters).
    goal_position converted to MM multiplying by 10
    This code doesn't take into account the inertia*/
  if(this->goal_position != -1)
  {
      if(this->start_encoder_pos == 0)
      {
        this->start_encoder_pos = abs(getEncoder());
      }
      float goal_encoder_distance = 10*this->goal_position * (WHEEL_FULLBACK/WHEEL_CIRCUMFERENCE);
      float goal_encoder_pos = this->start_encoder_pos + goal_encoder_distance;

      float encoder_dif = goal_encoder_pos - abs((float)getEncoder());

      if(encoder_dif <= 0.0)
      {
        setDirection(STOP);
        setGoalSpeed(0);
        setGoalPosition(-1);
        this->start_encoder_pos = 0.0;
      }

      return encoder_dif;
  }
  return 0;
}

/*int DCMotor::softSpeedUpdate()
{
  float SPEED_UP_SLOPE = 0.25;
  float speedControl = 1;
  int pwm_value = 0;

  if(this->goal_position != -1)
  {
    //traveled distance control
    int traveledDistance = WHEEL_CIRCUMFERENCE * abs(this->present_encoder_pos - this->start_encoder_pos)/WHEEL_FULLBACK;//distance in mm
    float progress = this->goal_position / traveledDistance;

    if((progress<SPEED_UP_SLOPE) || ((1-progress)<SPEED_UP_SLOPE))
    {
      speedControl = progress / SPEED_UP_SLOPE;
    }else{
      if(progress > 0.95)
      {
        setGoalSpeed(0);
        setDirection(STOP);
        setGoalPosition(-1);
        this->start_encoder_pos = 0;
      }else{
          speedControl = 1;
      }
    }
  }
  pwm_value = round(speedControl * speedToPwm());
  return pwm_value;
}*/

void DCMotor::stbyEnable()                            {digitalWrite(this->stby_pin,LOW);}
void DCMotor::stbyDisable()                           {digitalWrite(this->stby_pin,HIGH);}

long DCMotor::getEncoder()                            {return _encoder.read();}//I believe that 44 is equivalent a full back (360º)
void DCMotor::setEncoder(long pos)                    {_encoder.write(pos);}

int DCMotor::getID()                                  {return this->id;}
void DCMotor::setID(int new_ID)                       {this->id = new_ID;}

int DCMotor::getDirection()                           {return this->direction;}
void DCMotor::setDirection(int new_direction)         {this->direction = new_direction;}

int DCMotor::getGoalSpeed()                           {return this->goal_speed;}
void DCMotor::setGoalSpeed(int new_speed)             {this->goal_speed = new_speed;}

float DCMotor::getGoalPosition()                      {return this->goal_position;}
void DCMotor::setGoalPosition(float new_goal_position){this->goal_position = new_goal_position;}

int DCMotor::getAvgSpeed()                            {return this->avg_speed;}
