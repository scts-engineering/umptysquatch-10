long currentTime, previousTime;
boolean isOn;
int blinkTime;

//#define SLOW_BLINK_TIME 1000
#define FAST_BLINK_TIME 250
//#define SOLID_LED 0

void setup() {
  Serial.begin(9600);
  pinMode(12, OUTPUT);

  previousTime = millis();
  isOn = false;

  #ifdef SLOW_BLINK_TIME 
  blinkTime = SLOW_BLINK_TIME;
  #endif

  #ifdef FAST_BLINK_TIME
  blinkTime = FAST_BLINK_TIME;
  #endif

  #ifdef SOLID_LED
  blinkTime = SOLID_LED;
  #endif
}

void loop() {
  currentTime = millis();

  if(currentTime > previousTime + blinkTime) {

    if(isOn){
      digitalWrite(12, LOW);
      isOn = false;
    } else {
      digitalWrite(12, HIGH);
      isOn = true;
    }
    previousTime = currentTime; 
     
  }
}
