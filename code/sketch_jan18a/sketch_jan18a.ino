
long currentTime, previousTime;

void setup() {
  Serial.begin(9600);
  pinMode(12, OUTPUT);

  previousTime = millis();

}

void loop() {
  currentTime = millis();

  if(currentTime > previousTime + 1) {

    if(digitalRead(12) == LOW){
      digitalWrite(12, HIGH);
    } else {
      digitalWrite(12, LOW);
    }
    previousTime == currentTime;  
  }
  
}
