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
  Serial.println("testing servo 1");
  setServo(servos[0], 146);
  delay(1000);
  setServo(servos[0], 0);
  delay(1000);
  Serial.println("testing servo 2");
  setServo(servos[1], 146);
  delay(1000);
  setServo(servos[0], 0);
  delay(1000);
  Serial.println("testing servo 3");
  setServo(servos[2], 146);
  delay(1000);
  setServo(servos[0], 0);
  delay(1000);
  Serial.println("testing servo 4");
  setServo(servos[3], 146);
  delay(1000);
  setServo(servos[3], 0);
  delay(1000);

  Serial.println("activating pump A");
  digitalWrite(PUMP_A_PIN, HIGH);
  delay(2000);
  digitalWrite(PUMP_A_PIN, LOW);
  delay(1000);
  Serial.println("activating pump B");
  digitalWrite(PUMP_B_PIN, HIGH);
  delay(2000);
  digitalWrite(PUMP_B_PIN, LOW);
  delay(1000);
  Serial.println("activating Blow actuator");
  digitalWrite(ACTUATOR_B_PIN, HIGH);
  delay(1000);
  digitalWrite(ACTUATOR_B_PIN, LOW);
  delay(1000);
  Serial.println("activating vent actuator");
  digitalWrite(ACTUATOR_A_PIN, HIGH);
  delay(1000);
  digitalWrite(ACTUATOR_A_PIN, LOW);
  delay(1000);
}
