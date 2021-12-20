#include <Servo.h>


#define PIN_IR A0
#define PIN_LED 9
#define PIN_SERVO 10

//#include "medianfilter.h"

#define Max 1703.0
#define Min 1202.0
#define Neu 1568.0

#define target 255.0 //close:80 far:350
#define Start 80.0
#define End 290.0

#define _angle 10.0
#define _speed 1000.0

#define interval_dist 20.0
#define interval_servo 20.0
#define interval_serial 100.0

#define KP 0.61 // proportional gain
#define KD 11.5 // d gain
#define KI 0.026 // i gain

Servo myservo;

float dist_target;
float dist_raw;

unsigned long last_time_dist, last_time_servo, last_time_serial;
bool event_dist, event_servo, event_serial;

float duty_chg;
float duty_target, duty_curr;

float error_curr, error_prev, control, pterm, dterm, iterm;

//MedianFilter<> filter;

void setup() {
  pinMode(PIN_LED,OUTPUT);
  digitalWrite(PIN_LED, 1);
  
  myservo.attach(PIN_SERVO); 
  myservo.writeMicroseconds(Neu);
  duty_curr = Neu;
  duty_target = Neu;
  duty_chg = (float)(Max - Min) * _speed / 180.0 * interval_servo / 1000.0;

  event_dist = false; 
  event_servo = false;
  event_serial = false;

  dist_target = target;

  last_time_dist = 0; 
  last_time_servo = 0; 
  last_time_serial = 0;

  error_prev = 0;

  //filter.init();

  Serial.begin(57600);
}

void loop() {
  if(millis() >= last_time_dist + interval_dist){
    last_time_dist += interval_dist; 
    event_dist = true;
  }
  if(millis() >= last_time_servo + interval_servo){
    last_time_servo += interval_servo;
    event_servo = true;
  }
  if(millis() >= last_time_serial + interval_serial){
    last_time_serial += interval_serial;
    event_serial = true;
  }


  if(event_dist){
    event_dist = false;

    dist_raw = ir_filter();

    error_curr = dist_target - dist_raw;
    pterm = KP * error_curr;
    dterm = KD * (error_curr - error_prev);
    iterm += KI * error_curr;
    control = pterm + dterm + iterm; 
    error_prev = error_curr;

    duty_target = Neu + control;
    if (duty_target > Max){
      duty_target = Max;
    }
    else if (duty_target < Min){
      duty_target = Min;
    }
  }

  if(event_servo){
    event_servo = false;
    
    if(duty_target > duty_curr){
      //Serial.print("duty increase    ");
      duty_curr += duty_chg;
      if(duty_curr > duty_target) duty_curr = duty_target;
    }
    else{
      //Serial.print("DDDuty Decrease    ");
      duty_curr -= duty_chg;
      if(duty_curr < duty_target) duty_curr = duty_target;
    }
    
    myservo.writeMicroseconds(duty_curr);
  }

  if(event_serial){
    event_serial = false;

    Serial.print("IR:");
    Serial.print(dist_raw);
    Serial.print(",T:");
    Serial.print(dist_target);
    Serial.print(",P:");
    Serial.print(map(pterm,-1000,1000,510,610));
    Serial.print(",D:");
    Serial.print(map(dterm,-1000,1000,510,610));
    Serial.print(",I:");
    Serial.print(map(iterm,-1000,1000,510,610));
    Serial.print(",DTT:");
    Serial.print(map(duty_target,1000,2000,410,510));
    Serial.print(",DTC:");
    Serial.print(map(duty_curr,1000,2000,410,510));
    Serial.println(",-G:245,+G:265,m:0,M:800");
  }
   
}

float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0 + 85;
  return val;
}

float ir_filter(void){
  float val = 0;
  for(int i = 0;i<1000;i++){
    val += ir_distance();
  }
  return val/1000;
}
