#define PIN7 7

void setup(){
  pinMode(PIN7, OUTPUT);
}

void loop(){
  digitalWrite(PIN7, LOW);
  delay(1000);

  int i = 0;
  for(;i<6;i++){
    digitalWrite(PIN7, LOW);
    delay(100);
    digitalWrite(PIN7, HIGH);
    delay(100);
  }
  digitalWrite(PIN7, HIGH);
  while(1){
  
  }
}
