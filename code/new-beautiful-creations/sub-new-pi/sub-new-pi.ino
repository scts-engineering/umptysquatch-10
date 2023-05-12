#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"


#include "MS5837.h"
#include <Servo.h>
#include "Sodaq_LSM303AGR.h"

#include "shared/submarine.h"
#include "buttons.h"

extern "C" {
  #include <hardware/watchdog.h>
};

Servo servos[4];

MPU6050 gyroscope;

MS5837 depthSensor;

uint8_t mpuIntStatus;
uint16_t packetSize;
float ypr[3];

int modeButtonState;
int lastModeButtonState;
int reading;

volatile bool mpuInterrupt = false;

int blinkSpeed; //sets the blink interval of the LED
float holdDepth; // the depth recorded after going into auto mode, must maintain this depth closely

long currentTime, previousTime;
boolean isOn = false;

BlinkInterval blinkInterval;
Mode mode;

boolean pumpMode = true; //always defaults to pump mode for the first time it is put into manual mode

void dmpDataReady() {
    mpuInterrupt = true;
    //Serial.println("dmp data ready called");
}

DebouncedSwitch modeSwitch;

void setPinModes() {

    //set the pins to be either an input, or output
    pinMode(VENT_BUTTON_PIN, INPUT); // labeled "vent/pump AFT", used to control the compressed air in actuator mode, or the FWD pump in pump mode
    pinMode(BLOW_BUTTON_PIN, INPUT); // labeled "blow/pump FWD", used to control the vent in actuator mode, or the AFT pump in pump mode

    attachSwitch(&modeSwitch, MODE_SWITCH_PIN, 50, changeMode);
 
    pinMode(DEPTH_LED_PIN, OUTPUT);
    pinMode(PUMP_A_PIN, OUTPUT);
    pinMode(PUMP_B_PIN, OUTPUT);
    pinMode(ACTUATOR_A_PIN, OUTPUT);
    pinMode(ACTUATOR_B_PIN, OUTPUT);

    pinMode(JOYSTICK_X_PIN, INPUT);
    pinMode(JOYSTICK_Y_PIN, INPUT);
    Serial.println("Set pin modes for analog joystick");

#ifdef MAIN_BAT_VOLTAGE
    pinMode(MAIN_BAT_VOLTAGE, INPUT);
#endif

    //set the servos to recieve pin input
    servos[0].attach(SERVO_1_PIN, 870, 2320);
    servos[1].attach(SERVO_2_PIN, 870, 2320);
    servos[2].attach(SERVO_3_PIN, 870, 2320);
    servos[3].attach(SERVO_4_PIN, 870, 2320);
}

void setupGyro() {

    debugPrintln("Starting gyro initialization");

    bool dmpReady = false;
    uint8_t devStatus;

    debugPrintln("running gyroscope.initialize()");
    gyroscope.initialize();

    Serial.println(gyroscope.testConnection() ? "Connection good" : "Connection bad");


    debugPrintln("running gyroscope.dmpInitalize()");
    //load and configure the DMP
    devStatus = gyroscope.dmpInitialize();

    //TODO: see if these offsets need to be changed for maximum accuracy
    gyroscope.setXGyroOffset(220);
    gyroscope.setYGyroOffset(76);
    gyroscope.setZGyroOffset(-85);
    gyroscope.setZAccelOffset(1788);

    //TODO: retry initalization if failed
    if (devStatus == 0) {

        //TODO: Might need gyro calibration methods if results are off

        gyroscope.setDMPEnabled(true);

        mpuIntStatus = gyroscope.getIntStatus();

        dmpReady = true; //allows the main loop method to run

        packetSize = gyroscope.dmpGetFIFOPacketSize();

        debugPrintln("Sucessful gyro initialization");
    } else {
        //delay(1000);
        //setupGyro(); // haha recursion temp
        debugPrintln("Failed gyro initialzation");
    }
}

void setupDepthSensor() {

    //TODO: have a timeout and error message if initilization fails
    if (!depthSensor.init()) {
        debugPrintln("waiting on depth sensor...");
        delay(1000);
    } else {
        debugPrintln("depth sensor initilized successfully");
    }
    
    // depthSensor.setModel(MS5837::MS5837_30BA); // set depth sensor model to the MS5837, 30 bar
    depthSensor.setFluidDensity(997); // set fluid density to 997 kilograms per meter cubed (freshwater)
}

void setServo(Servo *servo, float degrees) {

    // convert 0-146 deg to "0-180 deg" for library
    float fakeDegrees = degrees / 0.8111f;
    servo->write(fakeDegrees);
}

void changeMode(int switchState) {
    Serial.println("I am changing the mode");
    if(switchState == HIGH && mode != AUTO) { //only runs once every time auto gets swtiched on

        holdDepth = depthSensor.depth();

        if(pumpMode) {
            pumpMode = false;
        } else {
            pumpMode = true;
        }

        mode = AUTO;
        debugPrintln("mode switched to AUTO");

    } else if(switchState == LOW && mode == AUTO) { //only runs once every time manual gets switched on
        
        turnOffAllDevices();
        
        if(pumpMode) {
            mode = PUMP;
            debugPrintln("mode switched to PUMP");
        } else {
            mode = ACTUATOR;
            debugPrintln("mode switched to ACTUATOR");
        }
    }
}

void setInitialMode() { //note that this method only runs during the startup to determine the starting mode

    if(digitalRead(MODE_SWITCH_PIN) == HIGH) {

        depthSensor.read();
        holdDepth = depthSensor.depth();
        mode = AUTO;
        debugPrintln("initial mode set to AUTO");

    } else {
        mode = PUMP; //if set to manual mode, the sub will default to pump mode
        debugPrintln("initial mode set to PUMP (manual default)");
    }
    reading = MODE_SWITCH_PIN;
    modeButtonState = MODE_SWITCH_PIN;
    lastModeButtonState = modeButtonState;
}

void processGyroData() {

    uint16_t fifoCount;
    uint8_t fifoBuffer[64];

    Quaternion q;
    VectorFloat gravity;
 
    //TODO: create error message if setup fails (dmpReady)

    if(gyroscope.dmpGetCurrentFIFOPacket(fifoBuffer)) {
            
        // display Euler angles in degrees
        gyroscope.dmpGetQuaternion(&q, fifoBuffer);
        gyroscope.dmpGetGravity(&gravity, &q);
        gyroscope.dmpGetYawPitchRoll(ypr, &q, &gravity);

        
    }
}

void processSteering() { // read the joystick, then set the servo angles

    float servoAngles[4];

    float x = (analogRead(JOYSTICK_X_PIN) - 512) / 512.0f;
    float y = (analogRead(JOYSTICK_Y_PIN) - 512) / 512.0f;
    // if it goes the opposite x direction remove this
    x = -x;

    int deadMin = 85;
    int deadMax = 95;
    int minAngle = 45; //minimum angle required for x-pattern
    int maxAngle = 135; //maximum angle required for x-pattern

    servoAngles[0] = (atan(x - y) * -180 / PI) + 90;
    servoAngles[1] = (atan(x + y) * -180 / PI) + 90;
    servoAngles[2] = (atan(y - x) * -180 / PI) + 90;
    servoAngles[3] = (atan(x + y) * 180 / PI) + 90;

    for (int i = 0; i < arraylength(servos); i++ ) {

        if (servoAngles[i] > maxAngle){
            servoAngles[i] = maxAngle;
        } else if (servoAngles[i] < minAngle) {
            servoAngles[i] = minAngle;
        }

        if (servoAngles[i] > deadMin && servoAngles[i] < deadMax) servoAngles[i] = 90;

        servoAngles[i] = round(servoAngles[i]);

#ifdef DEBUG
       /* Serial.print("Setting servo ");
        Serial.print(i);
        Serial.print(" to ");
        Serial.println(servoAngles[i]); */
#endif
        setServo(&servos[i], servoAngles[i]);
    }
}


void maintainEquilibrium() {

    float roll = (ypr[2] * 180 / M_PI);
    Serial.println(roll);
    depthSensor.read();
    float depth = depthSensor.depth();
    Serial.print("Depth: ");
    Serial.println(depth);
    Serial.print("holdDepth: ");
    Serial.println(holdDepth);
    DepthState isDepthOff;

    if(depth < holdDepth + 1 && depth > holdDepth - 1) {
        isDepthOff = STEADY;  
    } else {
        if(depth < holdDepth + 1) isDepthOff = TOO_LOW;
        if(depth > holdDepth - 1) isDepthOff = TOO_HIGH;
    }

    if(roll < 2.5 && roll > -2.5 && !isDepthOff) { //if the sub is in equalibrium then LED is set to SOLID
        blinkInterval = SOLID;
    } else { // if its not in equalibrium then it blinks the LED with SLOW interval
        blinkInterval = SLOW;
    }

    Serial.print("Roll: ");
    Serial.println(roll);

    if ((roll) < 2.5 and (roll) > -2.5) {

        digitalWrite(PUMP_A_PIN, LOW);
        digitalWrite(PUMP_B_PIN, LOW);
    }

    if ((roll) > 2.5) {

        digitalWrite(PUMP_A_PIN, HIGH);
        digitalWrite(PUMP_B_PIN, LOW);
    }

    if ((roll) < -2.5) {

        digitalWrite(PUMP_A_PIN, LOW);
        digitalWrite(PUMP_B_PIN, HIGH);
    }

    //TODO: depth offset equilibrium needs to be figured out(1 meter for temporary solution)

    //if there is a depth offset sync the appropriate actuator to the blinking of the LED
    if(isDepthOff == TOO_HIGH) { //if the sub is too high, which activates the vent
       // Serial.println("hi");
        if(isOn) {
            digitalWrite(ACTUATOR_A_PIN, HIGH); 
            digitalWrite(ACTUATOR_B_PIN, LOW);
        } else {
            digitalWrite(ACTUATOR_A_PIN, LOW);
            digitalWrite(ACTUATOR_B_PIN, LOW);
        }

    } else if(isDepthOff == TOO_LOW) { //if the sub is too low, which activates the blow

        if(isOn) {
            digitalWrite(ACTUATOR_A_PIN, LOW);
            digitalWrite(ACTUATOR_B_PIN, HIGH);
        } else {
            digitalWrite(ACTUATOR_A_PIN, LOW);
            digitalWrite(ACTUATOR_B_PIN, LOW);
        }

    } else {
        Serial.println("both off");
        digitalWrite(ACTUATOR_A_PIN, LOW);
        digitalWrite(ACTUATOR_B_PIN, LOW);
    }
}

void processPumpInput() {

    if (digitalRead(VENT_BUTTON_PIN) == HIGH) {
   
        digitalWrite(PUMP_A_PIN, HIGH);
        digitalWrite(PUMP_B_PIN, LOW);

    } else if (digitalRead(BLOW_BUTTON_PIN) == HIGH) {

        digitalWrite(PUMP_A_PIN, LOW);
        digitalWrite(PUMP_B_PIN, HIGH);

    } else {
        digitalWrite(PUMP_A_PIN, LOW);
        digitalWrite(PUMP_B_PIN, LOW);
    }
}

void processActuatorInput() {

    digitalWrite(PUMP_A_PIN, LOW); //change this to be better 
    digitalWrite(PUMP_B_PIN, LOW);

    if (digitalRead(VENT_BUTTON_PIN) == HIGH) {
        Serial.println("hi");
        digitalWrite(ACTUATOR_A_PIN, HIGH);
        digitalWrite(ACTUATOR_B_PIN, LOW);

    } else if (digitalRead(BLOW_BUTTON_PIN) == HIGH) {

        digitalWrite(ACTUATOR_A_PIN, LOW);
        digitalWrite(ACTUATOR_B_PIN, HIGH);

    } else {

        digitalWrite(ACTUATOR_A_PIN, LOW);
        digitalWrite(ACTUATOR_B_PIN, LOW);
    }
}

void processLED() {
 
  if(blinkInterval == SOLID) {
      if(!isOn) {
        digitalWrite(DEPTH_LED_PIN, HIGH);
        isOn = true;
      }
  } else {
    currentTime = millis();
  
    if(currentTime > previousTime + blinkInterval) {
        if(isOn) {
          digitalWrite(DEPTH_LED_PIN, LOW);
          isOn = false;
        } else {
          digitalWrite(DEPTH_LED_PIN, HIGH);
          isOn = true;
        }
    
        previousTime = currentTime;
    }
  }
}

void turnOffAllDevices() {
    digitalWrite(ACTUATOR_A_PIN, LOW);
    digitalWrite(ACTUATOR_B_PIN, LOW);
    digitalWrite(PUMP_A_PIN, LOW);
    digitalWrite(PUMP_B_PIN, LOW);
}

void setup() {
    Serial.begin(9600);
     if(watchdog_caused_reboot()) { delay(5000); }
    Wire.setSDA(0);
    Wire.setSCL(1);
    Wire.begin();

    //Wire.setClock(100000);
    //delay(5000);

    Serial.println("hello there");
    //return;
    
    blinkInterval = FAST;
    previousTime = millis();

    setPinModes();
    Serial.println("I am done setting pin modes");

    setupGyro();
    Serial.println("I am done setting up gyro");

    setupDepthSensor();
    Serial.println("I am done setting up depth sensor");

    setInitialMode();

        watchdog_enable(2000, 1);

   Serial.println("I am done setting up");
}

void loop() {
        watchdog_update();

    // temporary battery voltage reading thing (NOT CALIBRATED AT ALL!!!!)
    /*int batVoltageReading = analogRead(MAIN_BAT_VOLTAGE_PIN);
    Serial.print("12v supply: ");
    Serial.println((float)batVoltageReading/54);*/

    if(mode == AUTO) {

        processGyroData();

        maintainEquilibrium();
    } else if(mode == PUMP) {
        //Serial.println("I am now in manual pump mode");
        blinkInterval = SOLID;
        processPumpInput();
    } else {
        blinkInterval = FAST;
        processActuatorInput();
    }

    processSteering();

    processLED();

    tickSwitch(&modeSwitch);
    
}
