#ifndef DCMOTOR_h
#define DCMOTOR_h

#define LIBRARY_VERSION 1.0.0

/*DC_MOTOR
  Reduction:  1:75 (1:74.83)*/

#define LEFT_MOTOR  9
#define RIGHT_MOTOR 10

#define MAX_SPEED     10  //PWM = 250
#define MIN_SPEED     0   //PWM = 0

#define MAX_PWM     255
#define MIN_PWM     0

#define STOP        2 //WHEEL_MODE
#define FORWARD     1 //WHEEL_MODE
#define REVERSE     0 //WHEEL_MODE

#define WHEEL_DIAMETER      100 //mm
#define WHEEL_CIRCUMFERENCE 314 //mm
#define WHEEL_FULLBACK      3292
#define ENCODER_FULLBACK    44

#define ENCODER_UPDATE_TIME 50 //ms

#include <Encoder.h>
#include <math.h>

class DCMotor {
private:
  Encoder& _encoder;

  int in1_pin;
  int in2_pin;
  int pwm_pin;
  int stby_pin;

  int id;         //"RIGHT_MOTOR" or "LEFT_MOTOR"
  int direction;  //FORWARD,REVERSE or PAUSE

  int goal_speed; //0 to 10
  int avg_speed;

  int present_encoder_pos;
  int last_encoder_pos;

  int goal_position;//-1(WHEEL_MODE)
                      //!=-1(JOINT_MODE)
                      //value in "milimeters"

public:
  DCMotor(int id,int pin2, int pin1, int pinPWM, int pinSTBY,Encoder& enc);

  void receiveData(int memAddress , int data);
  void sendData();

  void move(int pwm_value);

  int speedToPwm();

  int update();
  void encoderUpdate();
  int speedUpdate();
  int updatePID(float desired, float present);

  void stbyEnable();
  void stbyDisable();

  long getEncoder();
  void setEncoder(long pos);

  int getID();
  void setID(int new_ID);

  int getDirection();
  void setDirection(int new_direction);

  int getPwmMin();
  void setPwmMin(int new_pwm_min);

  int getPwmMax();
  void setPwmMax(int new_pwm_max);

  int getSpeedOffset();
  void setSpeedOffset(int new_speed_offset);

  int getGoalSpeed();
  void setGoalSpeed(int new_speed);

  float getGoalPosition();//return value in "milimeters"
  void setGoalPosition(float new_goal_position);

  int getAvgSpeed();
};

#endif
