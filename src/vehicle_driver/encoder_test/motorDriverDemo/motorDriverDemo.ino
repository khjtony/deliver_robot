// By Hengjiu Kang
// Last edited on Dec.15 2016 by Hengjiu Kang
// MIT License
// [Dec.15 2016]HAVE NOT SOLVEN THE MILLIS OVER FLOW!

#include "motordriver_4wd.h"
#include "PID_v1.h"
#include <StandardCplusplus.h>
#include <string>
#include <vector>
#include <stack>
#include <iterator>
#include <PinChangeInt.h>
#include <seeed_pwm.h>

using namespace std;

#define DEBUG

#define BYTEAT(x) (byte)x
#define HALF_16 32768

#define LEFT_ENCODER 3
#define RIGHT_ENCODER 2
// remember: this multiplicator is: disatance = encoder * multi
#define LEFT_ENCODER_MULTI 6
#define RIGHT_ENCODER_MULTI 6
// time division is encoder sample time to the 1s. for example, when
// we record encoder for 200ms, division will be 5
#define LEFT_ENCODER_DIVISION 5
#define RIGHT_ENCODER_DIVISION 5


// Don't change these.
#define FIRST_ANALOG_PIN 14
#define TOTAL_PINS 19
// Notice that anything that gets modified inside an interrupt, that I wish to access
// outside the interrupt, is marked "volatile". That tells the compiler not to optimize
// them.
volatile uint8_t latest_interrupted_pin;
volatile uint8_t interrupt_count[TOTAL_PINS]={0}; // possible arduino pins

// Do not use any Serial.print() in interrupt subroutines. Serial.print() uses interrupts,
// and by default interrupts are off in interrupt subroutines.
// Here we update a counter corresponding to whichever pin interrupted.
void quicfunc0() {
  interrupt_count[LEFT_ENCODER]++;
};

void quicfunc1() {
  interrupt_count[RIGHT_ENCODER]++;
};

// serial parser related
vector<char> in_buffer;

// timing related
unsigned long encoder_timer=0;
unsigned long motor_timer=0;
unsigned long serial_timer=0;
uint8_t serial_failure=0;
int left_dir=1;
int right_dir=1;
double left_encoder=0;
double right_encoder=0;
double left_setencoder=0;
double right_setencoder=0;
double left_output=0;
double right_output=0;

// PID setup
double aggKp=4, aggKi=0.2, aggKd=1;
double consKp=2, consKi=0.00, consKd=0.25;

PID left_PID(&left_encoder, &left_output, &left_setencoder, consKp, consKi, consKd, DIRECT);
PID right_PID(&right_encoder, &right_output, &right_setencoder, consKp, consKi, consKd, DIRECT);



void serialize(uint16_t left_counter, uint16_t right_counter, int _left_dir, int _right_dir){
  //format
  //[0x7f][HI_len][LO_len][millis, left_speed, right_speed][checksum]
  vector<char> out_buffer;
  char checksum=0;
  //convert from encoder/s to mm/s
  left_counter=left_counter * LEFT_ENCODER_MULTI * LEFT_ENCODER_DIVISION;
  right_counter=right_counter * RIGHT_ENCODER_MULTI * RIGHT_ENCODER_DIVISION;
  
  stack<char> _inverse_container;
  
  //timestamp
  unsigned long now_time=serial_timer;
  while(now_time){
    _inverse_container.push(now_time%10+'0');
    now_time/=10;
  }
  while(!_inverse_container.empty()){
    out_buffer.push_back(_inverse_container.top());
    _inverse_container.pop();
  }
  out_buffer.push_back(',');
  
  // left
  if(_left_dir==-1){
    out_buffer.push_back('-');
  }
  while(left_counter){
    _inverse_container.push(left_counter%10+'0');
    left_counter/=10;
  }
  out_buffer.push_back('0');
  while(!_inverse_container.empty()){
    out_buffer.push_back(_inverse_container.top());
    _inverse_container.pop();
  }

  out_buffer.push_back(',');
  // right
  if(_right_dir==-1){
    out_buffer.push_back('-');
  }
  while(right_counter){
    _inverse_container.push(right_counter%10+'0');
    right_counter/=10;
  }
  out_buffer.push_back('0');
  while(!_inverse_container.empty()){
    out_buffer.push_back(_inverse_container.top());
    _inverse_container.pop();
  }

  //checksum & send
  Serial.print(0x7f);
  Serial.print(' ');
  Serial.print(out_buffer.size()>>8);
  Serial.print(' ');
  Serial.print(0xff&out_buffer.size());
  Serial.print(' ');
  for(int i=0;i<out_buffer.size();i++){
    checksum+=out_buffer[i];
    Serial.print(out_buffer[i]);
  }
  checksum = 0xff-(0xff&(checksum));
  Serial.print(checksum);

  #ifdef DEBUG
  Serial.println("");
  #endif
  
}

void simple_serial_parser(double* left_target, double* right_target, int* left_setdir, int* right_setdir){
  if(Serial.available()==0 && in_buffer.size()<=0){
    serial_failure+=1;
    return;
  }

  // remember that left speed and right speed are setting for the encoder 
  // with bias 127
  // format:
  // [0x7f][HI_len][LO_len][data....][checksum] 

  //left_speed and right_speed are at mm/s
  int left_speed=0;
  int right_speed=0;
  int left_dir=1;
  int right_dir=1;
  uint8_t checksum=0;
  char _msg_len=0;
  int left_index=0;
  int right_index=0;
  int read_flag=0;
  
  char inChar;

  while(Serial.available()){
    inChar = (char)Serial.read();
    in_buffer.push_back(inChar);
  }

  // trim 
  if(in_buffer.size()>0 && BYTEAT(in_buffer[0])!='H'){
    in_buffer.erase(in_buffer.begin());
  }

  if(in_buffer[in_buffer.size()-1]!='E'){
    // reading not finished
    return;
  }

      // if the message is valid
      left_dir=1;
      right_dir=1;
      left_speed=0;
      right_speed=0;
      for(int i=1;i<in_buffer.size();i++){
        if(in_buffer[i]==','){
          in_buffer.erase(in_buffer.begin(), in_buffer.begin()+i);  
          break;
        }
        if(in_buffer[i]=='-'){
          left_dir=-1;
        }
        if(in_buffer[i]==' '){
          continue;
        }
        left_speed=left_speed*10+(in_buffer[i]-'0');
      }
      
      for(int i=1;i<in_buffer.size();i++){
        if(in_buffer[i]=='-'){
          right_dir=-1;
        }
        if(in_buffer[i]==' '){
          continue;
        }
        if(in_buffer[i]=='E'){
          break;
        }
        right_speed=right_speed*10+(in_buffer[i]-'0');
      }

    // convert mm/s to encoder/s
    *left_setdir = left_dir;
    *right_setdir = right_dir;
    *left_target = (double)(1.0*left_speed/LEFT_ENCODER_MULTI/LEFT_ENCODER_DIVISION);
    *right_target = (double)(1.0*right_speed/RIGHT_ENCODER_MULTI/RIGHT_ENCODER_DIVISION);

    Serial.print("Setpoint: ");
    Serial.print(*left_target);
    Serial.print(", ");
    Serial.print(*right_target);
    Serial.println("");
    
    in_buffer.erase(in_buffer.begin(), in_buffer.end());     
}


// have problem.
//
void serial_parser(double* left_target, double* right_target, int* left_setdir, int* right_setdir){
  if(Serial.available()==0 && in_buffer.size()<=0){
    serial_failure+=1;
    return;
  }

  // remember that left speed and right speed are setting for the encoder 
  // with bias 127
  // format:
  // [0x7f][HI_len][LO_len][data....][checksum] 

  //left_speed and right_speed are at mm/s
  int left_speed=0;
  int right_speed=0;
  int left_dir=1;
  int right_dir=1;
  uint8_t checksum=0;
  char _msg_len=0;
  int left_index=0;
  int right_index=0;
  
  char inChar;

  while(Serial.available()){
    inChar = (char)Serial.read();
    in_buffer.push_back(inChar);
  }

  // trim 
  if(in_buffer.size()>0 && BYTEAT(in_buffer[0])!=0x7f){
    in_buffer.erase(in_buffer.begin());
  }

  while(in_buffer.size()>=3){
    _msg_len=BYTEAT(in_buffer[2]);
    if(in_buffer.size()<(_msg_len+4)){
      return;
    }
    // check checksum
    for(int i=3;i<3+_msg_len;i++){
      checksum+=in_buffer[i];
    }
    checksum = 0xff -(0xff&checksum);
    if (BYTEAT(checksum)!=BYTEAT(in_buffer[3+_msg_len])){
      // if checksum is invalid, erase the whole msg
      in_buffer.erase(in_buffer.begin(), in_buffer.begin()+4+_msg_len);  
    }else{
      // if the message is valid
      left_dir=1;
      right_dir=1;
      left_speed=0;
      right_speed=0;
      for(int i=3;i<3+_msg_len;i++){
        if(in_buffer[i]==','){
          right_index=i+1;
          break;
        }
        if(in_buffer[i]=='-'){
          left_dir=-1;
        }
        if(in_buffer[i]==' '){
          continue;
        }
        left_speed=left_speed*10+(in_buffer[i]-'0');
      }
      
      for(int i=right_index;i<3+_msg_len;i++){
        if(in_buffer[i]=='-'){
          right_dir=-1;
        }
        if(in_buffer[i]==' '){
          continue;
        }
        right_speed=right_speed*10+(in_buffer[i]-'0');
      }
    }

    // convert mm/s to encoder/s
    *left_setdir = left_dir;
    *right_setdir = right_dir;
    *left_target = (double)(1.0*left_speed/LEFT_ENCODER_MULTI/LEFT_ENCODER_DIVISION);
    *right_target = (double)(1.0*right_speed/RIGHT_ENCODER_MULTI/RIGHT_ENCODER_DIVISION);
    
    in_buffer.erase(in_buffer.begin(), in_buffer.begin()+4+_msg_len);     
  }
}


void setup()
{
    MOTOR.init();
    pinMode(LEFT_ENCODER, INPUT_PULLUP);
    attachPinChangeInterrupt(LEFT_ENCODER, quicfunc0, RISING);
    pinMode(RIGHT_ENCODER, INPUT_PULLUP);
    attachPinChangeInterrupt(RIGHT_ENCODER, quicfunc1, RISING);
    Serial.begin(115200);
    Serial.println("Hello world!");
    MOTOR.setStop1();
    MOTOR.setStop2();
    left_PID.SetMode(AUTOMATIC);
    right_PID.SetMode(AUTOMATIC);
}

void loop()
{
    // every 200ms(5hz) record encoder and do PID
    if(millis()-encoder_timer>200){
      encoder_timer = millis();
      left_encoder = double(1.0*interrupt_count[LEFT_ENCODER]);
      right_encoder = double(1.0*interrupt_count[RIGHT_ENCODER]);
      interrupt_count[LEFT_ENCODER]=0;
      interrupt_count[RIGHT_ENCODER]=0;

      #ifdef DEBUG
      Serial.print("encoder speed: ( ");
      Serial.print(left_encoder);
      Serial.print("\t,");
      Serial.print(right_encoder);
      Serial.println("\t)");
      #endif
      
      //PID controller
      left_PID.Compute();
      right_PID.Compute();
      //set motor
      if(left_dir==1){
        MOTOR.setSpeedDir1((int)floor(left_output>100?100:left_output), DIRR);
      }else{
        MOTOR.setSpeedDir1((int)floor(left_output>100?100:left_output), DIRF);
      }
      if(right_dir==1){
        MOTOR.setSpeedDir2((int)floor(right_output>100?100:right_output), DIRF);
      }else{
        MOTOR.setSpeedDir2((int)floor(right_output>100?100:right_output), DIRR);
      }
    }

    // subscribe motor set at 20hz
    if(millis()-motor_timer>50){
      motor_timer=millis();
      //serial_parser(&left_setencoder,&left_setencoder, &left_dir, &right_dir);  
      simple_serial_parser(&left_setencoder,&left_setencoder, &left_dir, &right_dir);  
    }

    // publisher motor speed at 20hz
    if(millis()-serial_timer>50){
      serial_timer=millis();
     // serialize(floor(left_encoder), floor(right_encoder), left_dir, right_dir);
    }

}
/*********************************************************************************************************
 * END FILE
 *********************************************************************************************************/


