#include "XL320.h"
#include "DCMotor.h"
#include "Encoder.h"

//GLOBAL DEFINITIONS
#define LEFT_MOTOR_ID   9 
#define RIGHT_MOTOR_ID  10
#define UPDATE_TIME     50//50ms
#define DXL_ERROR_OVERLOAD              1
#define DXL_ERROR_OVER_HEATING          2
#define DXL_ERROR_INPUT_VOLTAGE         3
#define DXL_ERROR_INVALID_INSTRUCTION   4

//******************//
//* ARDUINO PINOUT *//
//******************//
#define RX              1
#define TX              0
#define STBY            9
#define WRITE_CONTROL   13
//LEFT MOTOR
#define LEFT_ENCODER_1  3
#define LEFT_ENCODER_2  5
#define PWMA            6
#define AIN2            7 //A02 -> M2
#define AIN1            8 //A01 -> M1
//RIGHT MOTOR
#define RIGHT_ENCODER_1 2
#define RIGHT_ENCODER_2 4
#define PWMB            10
#define BIN2            11 //B02 -> M2
#define BIN1            12 //B01 -> M1

//XL320 dxl(1000000,LEFT_MOTOR_ID,RIGHT_MOTOR_ID,Serial);

Encoder encoder_L(LEFT_ENCODER_1,LEFT_ENCODER_2);
DCMotor motor_L(LEFT_MOTOR_ID,AIN2,AIN1,PWMA,STBY,encoder_L);

Encoder encoder_R(RIGHT_ENCODER_1,RIGHT_ENCODER_2);
DCMotor motor_R(RIGHT_MOTOR_ID,BIN2,BIN1,PWMB,STBY,encoder_R);


void setup()
{
  pinMode(WRITE_CONTROL,OUTPUT);
  digitalWrite(WRITE_CONTROL,HIGH);//HIGH => disable Arduino write/send.
  //dxl.init();
  Serial.begin(112500);
}

int vel = 20;//-10 to 10
bool flag_move = true;
unsigned long lastMilli = 0;//loop timer
long wheels_diff = 0;

void loop(){
  if((millis() - lastMilli) >= UPDATE_TIME)
  {
    lastMilli = millis();

    motor_R.encoderUpdate();
    motor_L.encoderUpdate();
    motor_R.speedUpdate();
    motor_L.speedUpdate();
  }
  motor_R.move();
  motor_L.move();
  
  if(flag_move){
    motor_L.setGoalSpeed(vel);
    motor_R.setGoalSpeed(vel);   
    //motor_L.pwm_value = pwm;
    //motor_L.move();
    //motor_L.encoderUpdate();
    //motor_L.speedUpdate();    
    //motor_R.pwm_value = pwm;
    //motor_R.move();
    //motor_R.encoderUpdate();
    //motor_R.speedUpdate();
    flag_move = false;
  }
  if(motor_R.present_encoder_pos >= 3292){
    motor_R.setGoalSpeed(10);
    motor_L.setGoalSpeed(10);
  }

  if(millis() <= 5000){
    //Data to MATLAB
    Serial.print(millis());
    //Serial.print(14*motor_L.goal_speed);
    Serial.print("\t");
    Serial.print(motor_L.present_encoder_pos);
    Serial.print("_Avg");
    //Serial.print(motor_L.present_encoder_pos-motor_L.last_encoder_pos);
    Serial.print(motor_L.avg_speed);
    Serial.print("_(");
    Serial.print(motor_L.error);
    Serial.print("::");
    Serial.print(motor_L.error_acc);
    Serial.print(")_");    
    Serial.print(motor_L.pwm_value);  
    
    Serial.print("\t\t");

    Serial.print(motor_R.present_encoder_pos);
    Serial.print("_Avg");
    //Serial.print(motor_R.present_encoder_pos-motor_R.last_encoder_pos);
    Serial.print(motor_R.avg_speed);
    Serial.print("_(");
    Serial.print(motor_R.error);
    Serial.print("::");
    Serial.print(motor_R.error_acc);
    Serial.print(")_");
    Serial.println(motor_R.pwm_value);
  }
}
