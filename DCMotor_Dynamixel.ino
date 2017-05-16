#include "XL320.h"
#include "DCMotor.h"
#include "Encoder.h"

//GLOBAL DEFINITIONS
#define LEFT_MOTOR_ID   9 
#define RIGHT_MOTOR_ID  10
#define UPDATE_TIME     50//50ms
/*ERRORS
DXL_ERROR_OVERLOAD              1
DXL_ERROR_OVER_HEATING          2
DXL_ERROR_INPUT_VOLTAGE         3
DXL_ERROR_INVALID_INSTRUCTION   4*/

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

XL320 dxl(1000000,LEFT_MOTOR_ID,RIGHT_MOTOR_ID,Serial);
Encoder encoder_L(LEFT_ENCODER_1,LEFT_ENCODER_2);
DCMotor motor_L(LEFT_MOTOR_ID,AIN2,AIN1,PWMA,STBY,encoder_L);
Encoder encoder_R(RIGHT_ENCODER_1,RIGHT_ENCODER_2);
DCMotor motor_R(RIGHT_MOTOR_ID,BIN2,BIN1,PWMB,STBY,encoder_R);

unsigned long lastMilli = 0;//loop timer

void setup()
{
  pinMode(WRITE_CONTROL,OUTPUT);
  digitalWrite(WRITE_CONTROL,HIGH);//HIGH => disable Arduino write/send.
  dxl.init();
}

void loop()
{
  dxl.checkMessages();
  if(dxl.instruction != DXL_NO_DATA)
  {
    switch(dxl.instruction)
    {
      case DXL_PING:break;
      case DXL_READ_DATA: dxlReadData();
                          break;
      case DXL_WRITE_DATA:dxlWriteData();
                          break;
      case DXL_RESET:break;
      case DXL_SYNC_WRITE:break;
      default:break;
    }
  }else{
    //Do nothing
  }
  
  if((millis() - lastMilli) >= UPDATE_TIME)
  {
    lastMilli = millis();
    motor_R.encoderUpdate();
    motor_L.encoderUpdate();
    motor_R.gearBox();
    motor_L.gearBox();
  }
  motor_R.move();
  motor_L.move();
}

void dxlReadData(){
  int memAddress = 0;
  int dataLen = 0;
  unsigned int data = 0;
  unsigned char error = 0;
  memAddress = dxl.parameters[0];
  memAddress += ((dxl.parameters[1] & 0x00FF) << 8);
  dataLen = dxl.parameters[2];
  dataLen += ((dxl.parameters[3] & 0x00FF) << 8);

  if(dxl.received_id == motor_L.getID())//LEFT_MOTOR_ID
  {
    switch(memAddress)
    {
      case DXL_ID:              data = motor_L.getID();
                                break;
      case DXL_PRESENT_POSITION:data = abs(motor_L.getEncoder());
                                break;    
      case DXL_GOAL_SPEED:      data = motor_L.getGoalSpeed();
                                break; 
      case DXL_PRESENT_SPEED:   data = motor_L.getAvgSpeed();
                                break;
      case DXL_PRESENT_VOLTAGE: data = motor_L.getPWM();  
                                break;
      default:                  error = DXL_ERROR_INVALID_INSTRUCTION;
                                break;
    }  
  }else{//RIGHT_MOTOR_ID
    switch(memAddress)
    {
      case DXL_ID:              data = motor_R.getID();
                                break;
      case DXL_PRESENT_POSITION:data = abs(motor_R.getEncoder());
                                break;    
      case DXL_GOAL_SPEED:      data = motor_R.getGoalSpeed();
                                break; 
      case DXL_PRESENT_SPEED:   data = motor_R.getAvgSpeed();
                                break;
      case DXL_PRESENT_VOLTAGE: data= motor_R.getPWM();
                                break;
      default:                  error = DXL_ERROR_INVALID_INSTRUCTION;
                                break;
    }    
  }
  digitalWrite(WRITE_CONTROL,LOW);//enable Arduino write
  dxl.sendStatusPacket(dxl.received_id,dxl.instruction,memAddress,error,data);
  digitalWrite(WRITE_CONTROL,HIGH);//disable Arduino write
}










void dxlWriteData(){
  unsigned char error = 0;
  unsigned int data = 0;
  int memAddress = 0;
    
  memAddress = dxl.parameters[0];
  memAddress += ((dxl.parameters[1] & 0x00FF)<< 8);
  if(dxl.total_parameters == 4){//2 length_bytes + 2 data_bytes
    data = dxl.parameters[2];
    data += ((dxl.parameters[3] & 0x00FF)<< 8);
  }else{//2 length_bytes + 1 data_byte
    data = dxl.parameters[2];
  }

  if(dxl.received_id == motor_L.getID()){//LEFT_MOTOR_ID
    switch(memAddress)
    {
      case DXL_ID:              dxl.left_id = data;
                                motor_L.setID(data);
                                break;
      case DXL_GOAL_SPEED:      motor_L.setGoalSpeed(data);//GOAL_SPEED(0 to 10)
                                break;
      case DXL_PRESENT_POSITION:motor_L.setEncoder(data);
                                break;
      default:                  error = DXL_ERROR_INVALID_INSTRUCTION;
                                break;
    }  
  }else{//RIGHT_MOTOR_ID
    switch(memAddress)
    {
      case DXL_ID:              dxl.right_id = data;
                                motor_R.setID(data);
                                break;
      case DXL_GOAL_SPEED:      motor_R.setGoalSpeed(data);//GOAL_SPEED(0 to 10)
                                break;
      case DXL_PRESENT_POSITION:motor_R.setEncoder(data);
                                break;                              
      default:                  error = DXL_ERROR_INVALID_INSTRUCTION;
                                break;
    }  
  }
  digitalWrite(WRITE_CONTROL,LOW);//enable Arduino write
  dxl.sendStatusPacket(dxl.received_id,dxl.instruction,memAddress,error,data);
  digitalWrite(WRITE_CONTROL,HIGH);//disable Arduino write
}
