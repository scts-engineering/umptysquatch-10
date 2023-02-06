//#define USE_OLD_PINS //old board
#define USE_NEW_PINS //board revision 1

//#define USE_MAGNET_JOYSTICK
//#define ENABLE_AUTO_PUMPS

//#define DEBUG

#define arraylength(x) (sizeof(x) / sizeof((x)[0]))
#ifdef DEBUG
#define debugPrintln(x) Serial.println(x)
#define debugPrint(x) Serial.print(x)
#else
#define debugPrintln(x)
#define debugPrint(x)
#endif

#ifdef USE_OLD_PINS
#define INTERRUPT_PIN 2
#define UP_BUTTON_PIN 4
#define DOWN_BUTTON_PIN A1
#define MODE_BUTTON_PIN A0
#define POWER_BUTTON_PIN 13
#define DEPTH_LED_PIN 0
#define ACTUATOR_A_PIN 8
#define ACTUATOR_B_PIN 7
#define PUMP_A_PIN 11
#define PUMP_B_PIN 10
#define JOYSTICK_X_PIN A2
#define JOYSTICK_Y_PIN A3
#define SERVO_1_PIN 9
#define SERVO_2_PIN 6
#define SERVO_3_PIN 5
#define SERVO_4_PIN 3
#endif

#ifdef USE_NEW_PINS
#define BUTTON_1_PIN 11
#define BUTTON_2_PIN 10
#define MODE_BUTTON_PIN 8
#define POWER_BUTTON_PIN 7
#define DEPTH_LED_PIN 12
#define ACTUATOR_A_PIN 13
#define ACTUATOR_B_PIN A0
#define PUMP_A_PIN 2
#define PUMP_B_PIN 4
#define JOYSTICK_X_PIN A2
#define JOYSTICK_Y_PIN A3
#define SERVO_1_PIN 3
#define SERVO_2_PIN 5
#define SERVO_3_PIN 6
#define SERVO_4_PIN 9
#endif

enum BlinkInterval {
    SLOW = 1000,
    FAST = 250,
    SOLID = 0
};

enum Mode {
    AUTO,
    PUMP,
    ACTUATOR
};
