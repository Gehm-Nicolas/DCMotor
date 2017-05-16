#ifndef DCMOTOR_h
#define DCMOTOR_h

#define LIBRARY_VERSION 1.0.0

/*DC_MOTOR
  Reduction:  1:75 (1:74.83)*/

#include <Encoder.h>
#include <math.h>

#define WHEEL_DIAMETER      100 //mm
#define WHEEL_CIRCUMFERENCE 314 //mm
#define WHEEL_FULLBACK      3292
#define ENCODER_FULLBACK    44

#define FORWARD 2
#define REVERSE 1
#define STOP    0

#define BREAK_RATE  1

#define LEFT_MOTOR  9
#define RIGHT_MOTOR 10
#define MAX_SPEED   10
#define MIN_SPEED  -10
#define MAX_PWM     250 //Actually 255
#define MIN_PWM     0  	//40(25+15)=In order to have strong enough to move the motor itslef and to move robot weight.

class DCMotor {
//private:
public:
  Encoder& _encoder;
  int in1_pin;
  int in2_pin;
  int pwm_pin;
  int stby_pin;

  float control_goal_speed;
  float goal_speed; //-10.0 to 10.0
  int id;         //"RIGHT_MOTOR" or "LEFT_MOTOR"
  int avg_speed;  //0 to 10
  int pwm_value;  //0 to 255
  int direction;  //STOP,REVERSE,FORWARD

  long present_encoder_pos;
  long last_encoder_pos;
  long error;
  long error_acc;

  //long goal_encoder_pos;

public:
  DCMotor(int id,int pin2, int pin1, int pinPWM, int pinSTBY,Encoder& enc);
  void 	encoderUpdate();
  //void positionUpdate();
  void  gearBox();
  void 	speedUpdate();
  void 	move();
  void 	stbyEnable();
  void 	stbyDisable();

  int	  getID();
  void 	setID(int new_ID);
  int 	getGoalSpeed();
  void 	setGoalSpeed(int new_speed);
  int 	getAvgSpeed();
  int 	getPWM();
  long 	getEncoder();
  void 	setEncoder(long pos);
  //void  setGoalPosition(unsigned short int goal_position);
  //int   getGoalPosition();
};
#endif
