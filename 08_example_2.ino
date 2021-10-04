#define PIN_LED 9
#define PIN_TRIG 12
#define PIN_ECHO 13

#define SND_VEL 346.0 
#define INTERVAL 25 
#define _DIST_MIN 100 
#define _DIST_MAX 300 

int len = _DIST_MAX - _DIST_MIN;
float timeout; 
float dist_min, dist_max, dist_raw; 
unsigned long last_sampling_time; 
float scale; 

void setup() {
  pinMode(PIN_LED,OUTPUT);
  pinMode(PIN_TRIG,OUTPUT);
  digitalWrite(PIN_TRIG, LOW); 
  pinMode(PIN_ECHO,INPUT);

  dist_min = _DIST_MIN; 
  dist_max = _DIST_MAX;
  timeout = (INTERVAL / 2) * 1000.0; 
  dist_raw = 0.0; 
  scale = 0.001 * 0.5 * SND_VEL;

  Serial.begin(57600);
  last_sampling_time = 0;
}

void loop() {
  if(millis() < last_sampling_time + INTERVAL) return;
  dist_raw = USS_measure(PIN_TRIG,PIN_ECHO);

  Serial.print("Min:0,");
  Serial.print("raw:");
  Serial.print(dist_raw);
  Serial.print(",");
  Serial.println("Max:400");

  if(dist_raw < dist_min || dist_raw > dist_max){
    analogWrite(PIN_LED, 255);
  }
  else{
    float brightness = 0;
    if (len > dist_raw){
      brightness = 255 * ((len - dist_raw) / len);
    }
    else{
      brightness = 255 * ((dist_raw - len) / len);
    }
    analogWrite(PIN_LED, brightness);
  } 
  last_sampling_time += INTERVAL;
}


float USS_measure(int TRIG, int ECHO)
{
  float reading;
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  reading = pulseIn(ECHO, HIGH, timeout) * scale; 
  if(reading < dist_min || reading > dist_max) reading = 0.0; 
  return reading;
}
