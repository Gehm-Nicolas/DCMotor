#ifndef DCMOTOR_h
#define DCMOTOR_h

#define LIBRARY_VERSION 1.0.0

/*DC_MOTOR
  Reduction:  1:75 (1:74.83)*/

#define STOP        2 //WHEEL_MODE
#define FORWARD     1 //WHEEL_MODE
#define REVERSE     0 //WHEEL_MODE
#define RIGHT_MOTOR   10
#define LEFT_MOTOR    11

#define MAX_SPEED     10
#define MIN_SPEED     0

#define WHEEL_DIAMETER      100 //mm
#define WHEEL_CIRCUMFERENCE 314 //mm
#define WHEEL_FULLBACK      3292
#define ENCODER_FULLBACK    44
#define WM_PROP             10484 // WHEEL_FULLBACK(pulses)*1000(mm)/ WHEEL_CIRCUMFERENCE(mm)

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

  float goal_position;//-1(WHEEL_MODE)
                      //!=-1(JOINT_MODE)
                      //value in "meters"
  int acc_error;

  long start_encoder_pos;
  long present_encoder_pos;
  long old_encoder_pos;
public:
  DCMotor(int id,int pin2, int pin1, int pinPWM, int pinSTBY,Encoder& enc);

  int speedUpdate();
  int speedToPwm(int speed);
  int softSpeedToPwm();
  int pwmControl();

  void move();

  int calcPID(float desired, float present);

private:
  void encoderPositionUpdate();

public:
  void receiveData(int memAddress , int data);
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

  void setGoalPosition(float new_goal_position);
  float getGoalPosition();//return value in "meters"

  int getAvgSpeed();
};

#endif
