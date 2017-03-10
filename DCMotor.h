#ifndef DCMOTOR_h
#define DCMOTOR_h

#define LIBRARY_VERSION 1.0.0

/*DC_MOTOR
  Reduction:  1:75 (1:74.83)
  /*WHEEL
    Diameter = 100mm
    Circumference = 314mm*/
/*ENCODER
  Encoder_FullBack: |44|
  Wheel_FullBack: |3292|*/

#define STOP    2
#define FORWARD 1
#define REVERSE 0
#define RIGHT_MOTOR   10
#define LEFT_MOTOR    11

#define WHEEL_DIAMETER      100 //mm
#define WHEEL_CIRCUMFERENCE 314 //mm
#define WHEEL_FULLBACK      3292
#define ENCODER_FULLBACK    44

#include <Encoder.h>

class DCMotor {
private:
  Encoder& _encoder;

  int in1_pin;
  int in2_pin;
  int pwm_pin;
  int stby_pin;

  int id;//"RIGHT_MOTOR" or "LEFT_MOTOR"
  int direction;//FORWARD or REVERSE
  long actual_encoder_pos;
  long old_encoder_pos;

  int pwm_min;//Minimum value to move robot weight
  int pwm_max;

  int speed_offset;
  int goal_speed;
  float avg_speed;
  int acc_error;

public:
  DCMotor(int id,int pin2, int pin1, int pinPWM, int pinSTBY,Encoder& enc);
  void move(int speed, int direction);
  int speedToPwm(int speed);
  void stbyEnable();
  void stbyDisable();

  int speedUpdate();
  int calcPID(float desired, float actual);

  long encoderRead();
  void encoderWrite(long pos);
private:
  void encoderPositionUpdate();

public:
  int getPwmMin();
  void setPwmMin(int new_pwm_min);

  int getPwmMax();
  void setPwmMax(int new_pwm_max);

  void receiveData(int goal_speed, int direction);
  void sendData();

  int getDirection();

  int getAvgSpeed();

  void setGoalSpeed(int new_speed);
  int getGoalSpeed();
};

#endif
