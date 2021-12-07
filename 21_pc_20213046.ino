#include <Servo.h>

#define PIN_IR A0
#define PIN_LED 9
#define PIN_SERVO 10

#define Max 1693.0
#define Min 1354.0
#define Neu 1567.0

#define target 171.5 //close:80 far:350
#define Start 80.0
#define End 290.0

#define _angle 10.0
#define _speed 143.0

#define interval_dist 20.0
#define interval_servo 20.0
#define interval_serial 100.0

#define KP 1.07 // proportional gain

Servo myservo;

float dist_target;
float dist_raw;

unsigned long last_time_dist, last_time_servo, last_time_serial;
bool event_dist, event_servo, event_serial;

float duty_chg;
float duty_target, duty_curr;

float error_curr, error_prev, control, pterm;

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

    dist_raw = ir_distance();

    error_curr = dist_target - dist_raw;
    pterm = error_curr;
    control = KP * pterm; 

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

    Serial.print("dist_ir:");
    Serial.print(dist_raw);
    Serial.print(",pterm:");
    Serial.print(pterm);
    Serial.print(",duty_target:");
    Serial.print(duty_target);
    Serial.print(",duty_curr:");
    Serial.print(duty_curr);
    Serial.println(",Min:100,Low:200,dist_target:255,High:310,Max:410");
  }
   
}

float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}
