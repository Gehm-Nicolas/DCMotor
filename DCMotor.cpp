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

  this->id 			    = id;
  this->goal_speed 	= 0;
  this->avg_speed 	= 0;
  this->pwm_value 	= 0;

  this->present_encoder_pos = 0;
  this->last_encoder_pos    = 0;
  this->error 				      = 0;
  this->error_acc 			    = 0;

  stbyDisable();
}

void DCMotor::move()
{
  if(this->id == LEFT_MOTOR){
    if(this->goal_speed==0 && this->avg_speed==0){
								            digitalWrite(in1_pin,LOW);
								            digitalWrite(in2_pin,LOW);}
    if(this->goal_speed>0){	digitalWrite(in1_pin,HIGH);
								            digitalWrite(in2_pin,LOW);}
		if(this->goal_speed<0){	digitalWrite(in1_pin,LOW);
								            digitalWrite(in2_pin,HIGH);}
	}else{//RIGHT_MOTOR
		if(this->goal_speed==0 && this->avg_speed==0){
								            digitalWrite(in1_pin,LOW);
								            digitalWrite(in2_pin,LOW);}
		if(this->goal_speed>0){	digitalWrite(in1_pin,LOW);
								            digitalWrite(in2_pin,HIGH);}
		if(this->goal_speed<0){	digitalWrite(in1_pin,HIGH);
								            digitalWrite(in2_pin,LOW);}
	}
	analogWrite(this->pwm_pin,this->pwm_value);
}

void DCMotor::encoderUpdate()
{
  this->last_encoder_pos 		= this->present_encoder_pos;
  this->present_encoder_pos	= abs(getEncoder());
}

void DCMotor::speedUpdate()
{
  /*PWM correction using PID*/
  long pwm_correction 			= 0;
  static long last_error 	  = 0;
  long present_encoder_speed= 0;
  long goal_encoder_speed 	= 0;

  float Kp = 0.275;//0.4
  float Ki = 0.125;//0.1,0.2,0.125,0.05,0.1,0.125,0.2
  float Kd = 0;//0,0.5,0.2
  //float Kdiff = 0.2; //0.35,0.5,1,0.8,1,1.5,(1.55),0.155
  //float Kdiff = 0.025*this->goal_speed;//0.2,0.22,0.23,0.25
  //float Kdiff = 0.25;//0.25,

  goal_encoder_speed 	  = 14*abs(this->goal_speed);
  present_encoder_speed = this->present_encoder_pos - this->last_encoder_pos;
  this->avg_speed 		  = 0.8*this->avg_speed + 0.2*present_encoder_speed;
  //this->error			    = goal_encoder_speed - present_encoder_speed;
  this->error           = goal_encoder_speed - this->avg_speed;
  this->error_acc 		  += this->error;

  //pwm_correction = Kp*this->error + Ki*this->error_acc + Kd*(this->error - last_error) + Kdiff*wheels_diff;
  pwm_correction = Kp*this->error + Ki*this->error_acc + Kd*(this->error - last_error);
  last_error = this->error;
  if(this->avg_speed <= 0 && this->goal_speed == 0){
    this->pwm_value = 0;
	  this->error 	= 0;
	  this->error_acc = 0;
  }else{
    this->pwm_value = constrain(pwm_correction,MIN_PWM,MAX_PWM);
  }
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
void DCMotor::setGoalSpeed(int new_speed)	{if(new_speed<0 || new_speed>20){
												                    this->goal_speed = 0;
											                    }else{
												                    this->goal_speed = new_speed - 10;}}
int DCMotor::getAvgSpeed()               	{return this->avg_speed;}
int DCMotor::getPWM()                     {return this->pwm_value;}
