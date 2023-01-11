#include "Servo.h"

#define UP_BUTTON_PIN A0
#define DOWN_BUTTON_PIN 13
#define MODE_BUTTON_PIN 8
#define EXTRA_BUTTON_1_PIN 7
#define DEPTH_LED_PIN 12
#define ACTUATOR_A_PIN 13
#define ACTUATOR_B_PIN A0
#define PUMP_A_PIN 2
#define PUMP_B_PIN 4
#define JOYSTICK_X_PIN A2
#define JOYSTICK_Y_PIN A3
#define SERVO_1_PIN 9
#define SERVO_2_PIN 6
#define SERVO_3_PIN 5
#define SERVO_4_PIN 3

Servo servos[4];

void setPinModes() {

    //set the pins to be either an input, or output
    pinMode(UP_BUTTON_PIN, INPUT);
    pinMode(DOWN_BUTTON_PIN, INPUT);
    pinMode(MODE_BUTTON_PIN, INPUT);
    pinMode(EXTRA_BUTTON_1_PIN, INPUT);
    pinMode(DEPTH_LED_PIN, OUTPUT);
    pinMode(PUMP_A_PIN, OUTPUT);
    pinMode(PUMP_B_PIN, OUTPUT);
    pinMode(ACTUATOR_A_PIN, OUTPUT);
    pinMode(ACTUATOR_B_PIN, OUTPUT);
   // pinMode(INTERRUPT_PIN, INPUT); //TODO: It may need to be put after mpu initialization to work
    pinMode(JOYSTICK_X_PIN, INPUT);
    pinMode(JOYSTICK_Y_PIN, INPUT);

    //set the servos to recieve pin input
    servos[0].attach(SERVO_1_PIN);
    servos[1].attach(SERVO_2_PIN);
    servos[2].attach(SERVO_3_PIN);
    servos[3].attach(SERVO_4_PIN);
}

void setServo(Servo servo, float degrees) {

    float microseconds = degrees * 9.9 + 870;

    servo.writeMicroseconds(microseconds);
}

void setup() {
  Serial.begin(9600);

  setPinModes();

}

void loop() {

  Serial.println("starting servo test");
  Serial.println("setting servo 1 to 90");
  setServo(servos[0], 90);
  delay(1000);
  Serial.println("Servo 1 completed");
  Serial.println("setting servo 2 to 90");
  setServo(servos[1], 90);
  delay(1000);
  Serial.println("Servo 2 completed");
  Serial.println("setting servo 3 to 90");
  setServo(servos[2], 90);
  delay(1000);
  Serial.println("Servo 3 completed");
  Serial.println("setting servo 4 to 90");
  setServo(servos[3], 90);
  delay(1000);
  Serial.println("Servo 4 completed");
  delay(3000);
  Serial.println("starting pump test");
  Serial.println("activating pump A");
  digitalWrite(PUMP_A_PIN, HIGH);
  Serial.println("pump A is now on");
  delay(3000);
  digitalWrite(PUMP_A_PIN, LOW);
  Serial.println("pump A is now off");
  delay(1000);
  Serial.println("activating pump B");
  digitalWrite(PUMP_B_PIN, HIGH);
  Serial.println("pump B is now on");
  delay(3000);
  digitalWrite(PUMP_B_PIN, LOW);
  Serial.println("pump B is now off");
  delay(3000);
  Serial.println("starting actuator test");
  Serial.println("activating Blow actuator");
  digitalWrite(ACTUATOR_B_PIN, HIGH);
  Serial.println("Blow actuator is now on");
  delay(3000);
  digitalWrite(ACTUATOR_B_PIN, LOW);
  Serial.println("deactivated Blow actuator");
  delay(1000);
  Serial.println("activating vent actuator");
  digitalWrite(ACTUATOR_A_PIN, HIGH);
  Serial.println("Vent actuator is now on");
  delay(3000);
  digitalWrite(ACTUATOR_A_PIN, LOW);
  Serial.println("deactivated Vent actuator");
}
