#include <avr/pgmspace.h>
#include <StandardCplusplus.h>
#include <serstream>
#include <avr/pgmspace.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>
#include <LiquidCrystal.h>
#include <avr/sleep.h>


SoftwareSerial mySerial(8,9);

using namespace std;

#define BYTEAT(x) (byte)x
unsigned int DEFAULT_DECK_SIZE=5;
#define MY_ID 3
#define DEBOUNCE_DELAY 200
// 1.5s timeout
#define CHECK_TIMEOUT 1500
// 500ms delete tolerence
#define DELETE_TIMEOUT 500
// hibernate timeout
#define HIBERNATE_TIMEOUT 60000    //30s


char msg_report[]={0x7E,0x00,0x0F,0x00,0x01,0x00,0x13,0xA2,0x00,0x41,0x47,0x86,0xAF,0x00,0x4D,0x31,0x3A,0x02,0xd3};
volatile bool button_pressed = false;
volatile unsigned long button_time=0;


vector<unsigned long> time_series;
vector<int>the_deck;
unsigned long last_input=0;
unsigned long system_time=0;
unsigned long delete_time=0;
uint8_t is_deleted=0;



int btn=2;
int motor=3;
int sleep=6;

vector<char> in_buffer;

unsigned long _abs_diff(unsigned long A, unsigned long B){
	if(A>B){
		return A-B;
	}else{
		return B-A;
	}
}

void motor_enable(unsigned long duration){
	digitalWrite(motor,HIGH);
	delay(duration);
	digitalWrite(motor, LOW);
}


void isr_button() {
  unsigned long now_time = millis();
  if (now_time- button_time > DEBOUNCE_DELAY) {
    button_time = now_time;
    button_pressed = true;
    detachInterrupt(0);
  }
}


void wakeUpNow()        // here the interrupt is handled after wakeup
{
  // execute code here after wake-up before returning to the loop() function
  // timers and code using timers (serial.print and more...) will not work here.
  // we don't really need to execute any special functions here, since we
  // just want the thing to wake up

  button_pressed=1;
  button_time = millis();
}


void sleepNow()         // here we put the arduino to sleep
{
    /* Now is the time to set the sleep mode. In the Atmega8 datasheet
     * http://www.atmel.com/dyn/resources/prod_documents/doc2486.pdf on page 35
     * there is a list of sleep modes which explains which clocks and
     * wake up sources are available in which sleep mode.
     *
     * In the avr/sleep.h file, the call names of these sleep modes are to be found:
     *
     * The 5 different modes are:
     *     SLEEP_MODE_IDLE         -the least power savings
     *     SLEEP_MODE_ADC
     *     SLEEP_MODE_PWR_SAVE
     *     SLEEP_MODE_STANDBY
     *     SLEEP_MODE_PWR_DOWN     -the most power savings
     *
     * For now, we want as much power savings as possible, so we
     * choose the according
     * sleep mode: SLEEP_MODE_PWR_DOWN
     *
     */  
    set_sleep_mode(SLEEP_MODE_STANDBY);   // sleep mode is set here
 
    sleep_enable();          // enables the sleep bit in the mcucr register
                             // so sleep is possible. just a safety pin
 
    /* Now it is time to enable an interrupt. We do it here so an
     * accidentally pushed interrupt button doesn't interrupt
     * our running program. if you want to be able to run
     * interrupt code besides the sleep function, place it in
     * setup() for example.
     *
     * In the function call attachInterrupt(A, B, C)
     * A   can be either 0 or 1 for interrupts on pin 2 or 3.  
     *
     * B   Name of a function you want to execute at interrupt for A.
     *
     * C   Trigger mode of the interrupt pin. can be:
     *             LOW        a low level triggers
     *             CHANGE     a change in level triggers
     *             RISING     a rising edge of a level triggers
     *             FALLING    a falling edge of a level triggers
     *
     * In all but the IDLE sleep modes only LOW can be used.
     */
 
    attachInterrupt(0,isr_button, LOW); // use interrupt 0 (pin 2) and run function
                                       // wakeUpNow when pin 2 gets LOW
 
    sleep_mode();            // here the device is actually put to sleep!!
                             // THE PROGRAM CONTINUES FROM HERE AFTER WAKING UP
 
    sleep_disable();         // first thing after waking from sleep:
                             // disable sleep...
    //detachInterrupt(0);      // disables interrupt 0 on pin 2 so the
                             // wakeUpNow code will not be executed
                             // during normal running time.
 
}




void delete_notify(){
  Serial.println("Delete!");
  msg_report[17]=0;
  //update checksum
  char checksum=0;
  msg_report[15]='0'+MY_ID;
  for(int i=3;i<18;i++){
    checksum += msg_report[i];
  }
  checksum = 0xff-(0xff&(checksum));
  msg_report[18] = checksum;
  for(int i=0;i<19;i++){
    mySerial.print(msg_report[i]);
  }
}

void card_notify(int card){
  Serial.print("New card: ");
  Serial.println(card);

  
  char checksum=0;
  msg_report[17]=(uint8_t)card;
  //update checksum
  msg_report[15]='0'+MY_ID;
  for(int i=3;i<18;i++){
    checksum += msg_report[i];
  }
  checksum = 0xff-(0xff&(checksum));
  msg_report[18] = checksum;

  
  for(int i=0;i<19;i++){
    mySerial.print(msg_report[i]);
    Serial.print((byte)msg_report[i],HEX);
    Serial.print(" ");
  }
  Serial.println("");
}

int update_myserial(){
if (mySerial.available() == 0 && in_buffer.size()<=0) {
    return -1;
  }

  int result = -1;
  char inChar;
  int v_index=14;
  uint16_t _msg_len=0;

  while(mySerial.available()){
    inChar = (char)mySerial.read();
    in_buffer.push_back(inChar);
  }

 if(in_buffer.size()>0 && BYTEAT(in_buffer[0])!=0x7e){
    Serial.println("Trimming");
    in_buffer.erase(in_buffer.begin());
  }
  //interprate (Because the input msg will not exceed 20 chars
  while(in_buffer.size()>=3){
    //test length
    _msg_len= BYTEAT(in_buffer[2]);
    if(in_buffer.size()<(_msg_len+4)){
      // not received all
      return;
    }
    // check if it is other frame
    if (BYTEAT(in_buffer[3])!=0x80){
      in_buffer.erase(in_buffer.begin(),in_buffer.begin()+4+_msg_len);
      return;
    }
    // interprate content
    if(in_buffer[v_index]=='S'){
      // slave
      Serial.println("Slave");
      delete_time=millis();
      last_input=millis();
    }else if(in_buffer[v_index]=='V' && in_buffer[v_index+1]=='D'){
      Serial.println("DECK");
      DEFAULT_DECK_SIZE=(unsigned int)in_buffer[v_index+2];
      for(int i=0;i<DEFAULT_DECK_SIZE;i++){
        the_deck.push_back(in_buffer[i+v_index+3]);
      }
    }
    in_buffer.erase(in_buffer.begin(), in_buffer.begin()+4+_msg_len);
  }
  return 0;
}

int update_time_series(){
	if(time_series.empty()){
		return 0;
	}
	if(millis()-last_input<CHECK_TIMEOUT){
		return 0;
	}
 
	//check time_series
	card_notify(time_series.size());
  is_deleted=0;
	motor_enable(1000);
	// clear the time_series
	time_series.clear();
	delete_time=0;
	return 0;
}

void init_setup(){
	//init variables
	time_series.clear();
	the_deck.clear();
	last_input=millis();
	system_time=millis();
	in_buffer.clear();
  is_deleted=1;
}

void setup(){
	//init serial
	Serial.begin(115200);
	mySerial.begin(19200);
	init_setup();
	in_buffer.reserve(150);
  in_buffer.clear();
  pinMode(btn, INPUT);
  digitalWrite(btn, HIGH);
  pinMode(motor, OUTPUT);
  digitalWrite(motor, LOW);
  pinMode(sleep, OUTPUT);
  digitalWrite(sleep, LOW);
  attachInterrupt(digitalPinToInterrupt(btn), isr_button, LOW);
  Serial.println("Hello");
}

void loop(){
   if(millis()>system_time && millis()-system_time>HIBERNATE_TIMEOUT){
    digitalWrite(sleep,HIGH);
    sleepNow();
    digitalWrite(sleep,LOW);
  }
	if(button_pressed){
    button_pressed=0;
    //delay(30);
    //if(digitalRead(btn)==0){
      //not radio frequency
      
      Serial.println("BTN pressed!");
		  motor_enable(200);
		  time_series.push_back(millis());
      last_input=millis();
      system_time = millis();
      attachInterrupt(digitalPinToInterrupt(btn), isr_button, LOW);
	  //}
	}
	update_myserial();
	update_time_series();
  if(the_deck.size()>0){
    digitalWrite(sleep, HIGH);
    for(int i=0;i<the_deck.size();i++){
      for(int j=0;j<the_deck[i];j++){
          for(int k=0;k<i+1;k++){
            motor_enable(200);
            delay(200);
          }
          delay(1000);
       }
   }
   init_setup();
   digitalWrite(sleep, HIGH);
   sleepNow();
   attachInterrupt(digitalPinToInterrupt(btn), isr_button, FALLING);
   digitalWrite(sleep, LOW);
   delay(50);
 }
}
