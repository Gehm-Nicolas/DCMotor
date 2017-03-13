#ifndef DCMOTOR_h
#define DCMOTOR_h

#define LIBRARY_VERSION 1.0.0

/*DC_MOTOR
  Reduction:  1:75 (1:74.83)*/

#define STOP    2
#define FORWARD 1
#define REVERSE 0
#define RIGHT_MOTOR   10
#define LEFT_MOTOR    11

#define MAX_SPEED     10
#define MIN_SPEED     0

#define WHEEL_DIAMETER      100 //mm
#define WHEEL_CIRCUMFERENCE 314 //mm
#define WHEEL_FULLBACK      3292
#define ENCODER_FULLBACK    44

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
  int direction;  //FORWARD or REVERSE
  int pwm_min;    //Minimum DC value to move motor
  int pwm_max;

  int speed_offset;
  int goal_speed;
  float avg_speed;

  int acc_error;
  long actual_encoder_pos;
  long old_encoder_pos;

public:
  DCMotor(int id,int pin2, int pin1, int pinPWM, int pinSTBY,Encoder& enc);

  void receiveData(int goal_speed, int direction);
  int speedToPwm(int speed);
  void move(float meters, int direction);
  void move();
  int speedUpdate();
  int calcPID(float desired, float actual);
private:
  void encoderPositionUpdate();
public:
  void sendData();
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
  int getAvgSpeed();
};

#endif
