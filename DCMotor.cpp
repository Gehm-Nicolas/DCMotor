#include <Arduino.h>
#include "DCMotor.h"

DCMotor::DCMotor(int id,int pin2, int pin1, int pinPWM,int pinSTBY, Encoder& enc): _encoder(enc)
{
  pinMode(pin1,OUTPUT);
  pinMode(pin2,OUTPUT);
  pinMode(pinPWM,OUTPUT);
  pinMode(pinSTBY,OUTPUT);

  this->in1_pin 	= pin1;
  this->in2_pin 	= pin2;
  this->pwm_pin 	= pinPWM;
  this->stby_pin 	= pinSTBY;

  this->id 			            = id;
  this->control_goal_speed  = 0;
  this->goal_speed 	        = 0;
  this->avg_speed           = 0;
  this->pwm_value           = 0;

  this->present_encoder_pos = 0;
  this->last_encoder_pos    = 0;
  this->error 				      = 0;
  this->error_acc 			    = 0;
  this->direction           = STOP;

  //this->goal_encoder_pos    = 0;

  stbyDisable();
}

void DCMotor::move()
{
  if(this->id == LEFT_MOTOR){
    if(this->direction == STOP){    digitalWrite(in1_pin,LOW);
								                    digitalWrite(in2_pin,LOW);}
    if(this->direction == FORWARD){ digitalWrite(in1_pin,HIGH);
								                    digitalWrite(in2_pin,LOW);}
		if(this->direction == REVERSE){ digitalWrite(in1_pin,LOW);
								                    digitalWrite(in2_pin,HIGH);}
	}else{//RIGHT_MOTOR
		if(this->direction == STOP){    digitalWrite(in1_pin,LOW);
								                    digitalWrite(in2_pin,LOW);}
		if(this->direction == FORWARD){ digitalWrite(in1_pin,LOW);
								                    digitalWrite(in2_pin,HIGH);}
		if(this->direction == REVERSE){ digitalWrite(in1_pin,HIGH);
								                    digitalWrite(in2_pin,LOW);}
	}
	analogWrite(this->pwm_pin,this->pwm_value);
}

void DCMotor::encoderUpdate()
{
  this->last_encoder_pos = this->present_encoder_pos;
  if(this->id == LEFT_MOTOR){
    this->present_encoder_pos	= -getEncoder();
  }else{//RIGHT_MOTOR
    this->present_encoder_pos	= getEncoder();
  }
}

void DCMotor::speedUpdate()
{
  /*PWM correction using PID*/
  long pwm_correction 			= 0;
  long control_value        = 0;
  static long last_error 	  = 0;
  long present_encoder_speed= 0;
  long goal_encoder_speed 	= 0;

  float Kp = 0.275;//0.4 - 0.275 - 0.55
  float Ki = 0.125;//0   - 0.125 - 0.0275
  float Kd = 0;//0,

  goal_encoder_speed 	  = 14*this->goal_speed;
  present_encoder_speed = this->present_encoder_pos - this->last_encoder_pos;
  this->avg_speed 		  = 0.8*this->avg_speed + 0.2*present_encoder_speed;
  //this->error			    = goal_encoder_speed - present_encoder_speed;
  this->error           = goal_encoder_speed - this->avg_speed;
  this->error_acc 		  += this->error;

  //pwm_correction = Kp*this->error + Ki*this->error_acc + Kd*(this->error - last_error) + Kdiff*wheels_diff;
  control_value = Kp*this->error + Ki*this->error_acc + Kd*(this->error - last_error);
  last_error = this->error;
  if(this->avg_speed == 0 && this->goal_speed == 0){
    this->direction = STOP;
    this->pwm_value = 0;
    this->error_acc = 0;
  }else{
    if (control_value >= 0)
    {
      this->direction = FORWARD;
      pwm_correction = control_value;
    } else {
      this->direction = REVERSE;
      pwm_correction = -control_value;
    }
    this->pwm_value = constrain(pwm_correction,MIN_PWM,MAX_PWM);
  }
}

/*void DCMotor::positionUpdate()
//WARNING: limit of 2bytes = 65536
{
  if(this->present_encoder_pos!=this->goal_encoder_pos){
    pass;
  }
}*/

void DCMotor::gearBox()
{
  //Control "goal_speed" in "direction/goal_speed" changes - SPEED_UP_RATE
  if(this->control_goal_speed != this->goal_speed)
  {
    if(this->direction == FORWARD && this->control_goal_speed<0){
      this->goal_speed = this->goal_speed - BREAK_RATE;
    }else{
      if(this->direction == REVERSE && this->control_goal_speed>0){
        this->goal_speed = this->goal_speed + BREAK_RATE;
      }else{//Same direction
        this->goal_speed = this->control_goal_speed;
      }
    }
  }
  speedUpdate();
}

void DCMotor::stbyEnable()                {digitalWrite(this->stby_pin,LOW);}
void DCMotor::stbyDisable()         		  {digitalWrite(this->stby_pin,HIGH);}
long DCMotor::getEncoder()              	{return _encoder.read();}
void DCMotor::setEncoder(long pos)      	{_encoder.write(pos);
											                     this->present_encoder_pos = pos;
                                           this->last_encoder_pos = pos;}
int	 DCMotor::getID()                   	{return this->id;}
void DCMotor::setID(int new_ID)         	{this->id = new_ID;}
int DCMotor::getGoalSpeed()            		{return abs(this->goal_speed);}
void DCMotor::setGoalSpeed(int new_speed)	{if(new_speed>=0 || new_speed<=20){ //new_speed:0 to 20
                                          this->control_goal_speed = float(new_speed - 10);}}//this->goal_speed:-10 to 10
int DCMotor::getAvgSpeed()               	{return this->avg_speed;}
int DCMotor::getPWM()                     {return this->pwm_value;}
//void DCMotor::setGoalPosition(unsigned short int goal_position) {this->goal_encoder_pos=goal_position}
//int DCMotor::getGoalPosition()                                  {return this->goal_encoder_pos}
