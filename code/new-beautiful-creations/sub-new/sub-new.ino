#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "MS5837.h"
#include "Servo.h"
#include "Sodaq_LSM303AGR.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE // copied from mr jrowberg, guess it only imports the wire library if it needs it
#include "Wire.h"
#endif

#include "shared/submarine.h"

Servo servos[4];

#ifdef USE_MAGNET_JOYSTICK
Sodaq_LSM303AGR magnetometer;
#endif

MPU6050 gyroscope;

MS5837 depthSensor;

uint8_t mpuIntStatus;
uint16_t packetSize;
float ypr[3];

volatile bool mpuInterrupt = false;

int blinkSpeed; //sets the blink interval of the LED
long depth; // the current depth of the submarine
long holdDepth; // the depth recorded after going into auto mode, must maintain this depth closely

long currentTime, previousTime;
boolean isOn = false;

BlinkInterval blinkInterval;
Mode mode;

boolean pumpMode = true; //always defaults to pump mode for the first time it is put into manual mode

void dmpDataReady() {
    mpuInterrupt = true;
    //Serial.println("dmp data ready called");
}

void setPinModes() {

    //set the pins to be either an input, or output
    pinMode(BUTTON_1_PIN, INPUT); // labeled "blow/pump FWD", used to control the compressed air in actuator mode, or the FWD pump in pump mode
    pinMode(BUTTON_2_PIN, INPUT); // labeled "vent/pump AFT", used to control the vent in actuator mode, or the AFT pump in pump mode
    pinMode(MODE_BUTTON_PIN, INPUT);
    pinMode(POWER_BUTTON_PIN, INPUT);
    pinMode(DEPTH_LED_PIN, OUTPUT);
    pinMode(PUMP_A_PIN, OUTPUT);
    pinMode(PUMP_B_PIN, OUTPUT);
    pinMode(ACTUATOR_A_PIN, OUTPUT);
    pinMode(ACTUATOR_B_PIN, OUTPUT);
   // pinMode(INTERRUPT_PIN, INPUT);
    pinMode(JOYSTICK_X_PIN, INPUT);
    pinMode(JOYSTICK_Y_PIN, INPUT);

    //set the servos to recieve pin input
    servos[0].attach(SERVO_1_PIN);
    servos[1].attach(SERVO_2_PIN);
    servos[2].attach(SERVO_3_PIN);
    servos[3].attach(SERVO_4_PIN);
}

void setupGyro() {

    debugPrintln("Starting gyro initialization");

    bool dmpReady = false;
    uint8_t devStatus;

    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    //TODO: set baud rate

    gyroscope.initialize();

    //load and configure the DMP
    devStatus = gyroscope.dmpInitialize();

    //TODO: see if these offsets need to be changed for maximum accuracy
    gyroscope.setXGyroOffset(220);
    gyroscope.setYGyroOffset(76);
    gyroscope.setZGyroOffset(-85);
    gyroscope.setZAccelOffset(1788);

    //TODO: have error message if initialization fails
    //TODO: retry initalization if failed
    if (devStatus == 0) {

        //TODO: Might need gyro calibration methods if results are off

        gyroscope.setDMPEnabled(true);

        //enable Arduino interrupt detection
        //attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = gyroscope.getIntStatus();

        dmpReady = true; //allows the main loop method to run

        packetSize = gyroscope.dmpGetFIFOPacketSize();

        debugPrintln("Sucessful gyro initialization");
    } else {

        debugPrintln("Failed gyro initialzation");
    }
}

void setupDepthSensor() {

    //TODO: have a timeout and error message if initilization fails
    if (!depthSensor.init()) {
        delay(1000);
    }

    // depthSensor.setModel(MS5837::MS5837_30BA); // set depth sensor model to the MS5837, 30 bar
    depthSensor.setFluidDensity(997); // set fluid density to 997 kilograms per meter cubed (freshwater)
}

void setServo(Servo servo, float degrees) {

    float microseconds = degrees * 9.9 + 870;

    servo.writeMicroseconds(microseconds);
}

void setInitialMode() { //note that this method only runs during the startup to determine the starting mode

    if(digitalRead(MODE_BUTTON_PIN) == HIGH) {
        processDepthData();
        holdDepth = depthSensor.depth();
        mode = AUTO;
    } else {
        mode = PUMP; //if set to manual mode, the sub will default to pump mode
    }
}

void setMode() { //used to change the mode during operation

    if(digitalRead(MODE_BUTTON_PIN) == HIGH && mode != AUTO) { //only runs once every time auto gets swtiched on

        holdDepth = depthSensor.depth();

        if(pumpMode) {
            pumpMode = false;
        } else {
            pumpMode = true;
        }

        mode = AUTO;

    } else if(digitalRead(MODE_BUTTON_PIN) == LOW && mode == AUTO) { //only runs once every time manual gets switched on

        if(pumpMode) {
            mode = PUMP;
        } else {
            mode = ACTUATOR;
        }
    }
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

#ifdef USE_MAGNET_JOYSTICK
void setupMagnetometer() {
    
    magnetometer.rebootMagnetometer();
    delay(1000);
    magnetometer.enableMagnetometer(Sodaq_LSM303AGR::MagHighResMode, Sodaq_LSM303AGR::Hz100, Sodaq_LSM303AGR::Continuous);
    uint8_t axes = Sodaq_LSM303AGR::MagX;
    magnetometer.enableMagnetometerInterrupt(axes, -400);
}
#endif

void processSteering() { // read the joystick, then set the servo angles

    float servoAngles[4];

#ifdef USE_MAGNET_JOYSTICK
    float x = magnetometer.getMagX() / 3000;
    float y = magnetometer.getMagY() / 3000;
    //#ifdef DEBUG
    Serial.print(x);
    Serial.print(" ");
    Serial.println(y);
    //#endif

#else
    float x = (analogRead(JOYSTICK_X_PIN) - 512) / 512.0f;
    float y = (analogRead(JOYSTICK_Y_PIN) - 512) / 512.0f;
    // if it goes the opposite x direction remove this
    x = -x;
#endif

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
/*
#ifdef DEBUG
        Serial.print("Setting servo ");
        Serial.print(i);
        Serial.print(" to ");
        Serial.println(servoAngles[i]);
#endif*/
        setServo(servos[i], servoAngles[i]);
    }
}

void processDepthData() {
    depthSensor.read();
}

void maintainEquilibrium() {

    float roll = (ypr[2] * 180 / M_PI);
    depth = depthSensor.depth();

    if(roll < 2.5 && roll > -2.5 && depth < holdDepth + 1 && depth > holdDepth - 1) { //if the sub is in equalibrium then LED is set to SOLID
        blinkInterval = SOLID;
    } else { // if its not in equalibrium then it blinks the LED with SLOW interval
        blinkInterval = SLOW;
    }

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
    if(depth > holdDepth + 1) {

        //TODO: make this activate the actuator in a safe way

    } else if(depth < holdDepth - 1) {

        //TODO: make this activate thmoe actuator in a safe way

    } else {

        digitalWrite(ACTUATOR_A_PIN, LOW);
        digitalWrite(ACTUATOR_B_PIN, LOW);
    }
}

void processPumpInput() {

    if (digitalRead(BUTTON_1_PIN) == HIGH) {

        digitalWrite(PUMP_A_PIN, HIGH);
        digitalWrite(PUMP_B_PIN, LOW);

    } else if (digitalRead(BUTTON_2_PIN) == HIGH) {

        digitalWrite(PUMP_A_PIN, LOW);
        digitalWrite(PUMP_B_PIN, HIGH);

    } else {
        Serial.println("here");
        digitalWrite(PUMP_A_PIN, LOW);
        digitalWrite(PUMP_B_PIN, LOW);
    }
}

void processActuatorInput() {

    digitalWrite(PUMP_A_PIN, LOW); //change this to be better 
    digitalWrite(PUMP_B_PIN, LOW);

    if (digitalRead(BUTTON_1_PIN) == HIGH) {

        digitalWrite(ACTUATOR_A_PIN, HIGH);
        digitalWrite(ACTUATOR_B_PIN, LOW);

    } else if (digitalRead(BUTTON_2_PIN) == HIGH) {

        digitalWrite(ACTUATOR_A_PIN, LOW);
        digitalWrite(ACTUATOR_B_PIN, HIGH);

    } else {

        digitalWrite(ACTUATOR_A_PIN, LOW);
        digitalWrite(ACTUATOR_B_PIN, LOW);
    }
}

void processLED() {
 
  if(blinkInterval == 0) {
      if(!isOn) {
        digitalWrite(12, HIGH);
        isOn = true;
      }
  } else {
    currentTime = millis();
  
    if(currentTime > previousTime + blinkInterval) {
        if(isOn) {
          digitalWrite(12, LOW);
          isOn = false;
        } else {
          digitalWrite(12, HIGH);
          isOn = true;
        }
    
        previousTime = currentTime;
    }
  }
}

void setup() {
    Serial.begin(9600);

    blinkInterval = FAST;
    previousTime = millis();

    setPinModes();

    setupGyro();

    setupDepthSensor();

#ifdef USE_MAGNET_JOYSTICK
    setupMagnetometer();
#endif

    setInitialMode();
   
}

void loop() {

    if(mode == AUTO) {

        processGyroData();

        processDepthData();

        maintainEquilibrium();
    } else if(mode == PUMP) {
        Serial.println("I am now in manual pump mode");
        blinkInterval = SOLID;
        processPumpInput();
    } else {
        blinkInterval = FAST;
        processActuatorInput();
    }

    processSteering();

    processLED();

    setMode();

    // char thing[100];
    // sprintf(thing, "Up button: %d. Down button: %d. Mode button: %d. Power button: %d.\n", digitalRead(BUTTON_1_PIN), digitalRead(BUTTON_2_PIN), digitalRead(MODE_BUTTON_PIN), digitalRead(POWER_BUTTON_PIN));
    // Serial.print(thing);
    
}
