#ifndef DCMOTOR_h
#define DCMOTOR_h

#define LIBRARY_VERSION 1.0.0

/*DC_MOTOR
  Reduction(teorical):  1:75 (1:74.83)
  Reduction(pratictal): 1:72 (1:?)*/
  /*WHEEL
    Diameter = 100mm
    Circumference = 314mm*/
/*ENCODER
  Encoder_FullBack: |44|
  Wheel_FullBack: |3168|*/

#define STOP    2
#define FORWARD 1
#define REVERSE 0
#define RIGHT_MOTOR   10
#define LEFT_MOTOR    11

#define VEL_OFFSET          10
#define WHEEL_DIAMETER      100 //mm
#define WHEEL_CIRCUMFERENCE 314 //mm
#define Wheel_FullBack      3168
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
  long encoder_position;

  int goal_speed;
  int avg_speed;
  int acc_error;

public:
  DCMotor(int id,int pin2, int pin1, int pinPWM, int pinSTBY,Encoder& enc);
  void move(int speed, int direction);
  void stop();
  void start();
  int speed_update();
  int calc_PID(float desired, float actual);
  long encoderRead();
  void encoderWrite(long pos);
  void receiveData(int goal_speed, int direction);
  void sendData();
  int getDirection();
  int getAvgSpeed();
  int getGoalSpeed();
};

#endif
