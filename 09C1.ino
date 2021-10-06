#define PIN_LED 9
#define PIN_TRIG 12
#define PIN_ECHO 13

#define SND_VEL 346.0 
#define INTERVAL 25 
#define _DIST_MIN 100 
#define _DIST_MAX 300 
#define _DIST_ALPHA 0.1
#define Num 30

//global variables
float timeout;
float dist_min, dist_max, dist_raw, median;
float m[100]; // store raws
int p,N; // indexing variable, numbers of array object
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
  p = 0;
  N = Num;

  Serial.begin(57600);
  last_sampling_time = 0;
}

void loop() {

  if(millis() < last_sampling_time + INTERVAL) return;
  dist_raw = USS_measure(PIN_TRIG,PIN_ECHO);

  m[p] = dist_raw;
  p++;

  if (p == N){
    float res = 0;
    for(int i = 0; i<N; i++){
      res += m[i];        
    }
    res = res/N;
    median = res;
    p = 0;
  }
  
  
  
  Serial.print("Min:0,");
  Serial.print("raw:");
  Serial.print(dist_raw);
  Serial.print(",");
  Serial.print("median:");
  Serial.print(map(median,0,400,100,500));
  Serial.print(",");
  Serial.println("Max:500");

  if(dist_raw < dist_min || dist_raw > dist_max) {
    analogWrite(PIN_LED, 255);
  }
  else {
    analogWrite(PIN_LED, 0);
  }

  last_sampling_time += INTERVAL;
}

float USS_measure(int TRIG, int ECHO)
{
  float reading;
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  reading = pulseIn(ECHO, HIGH, timeout) * scale; // unit: mm
  if(reading < dist_min || reading > dist_max) reading = 0.0; // return 0 when out of range.
  return reading;
}
