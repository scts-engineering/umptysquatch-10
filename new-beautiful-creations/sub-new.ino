#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "MS5837.h"
#include "Servo.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE // copied from mr jrowberg, guess it only imports the wire library if it needs it
#include "Wire.h"
#endif

#define INTERRUPT_PIN 2
#define arraylength(x) (sizeof(x) / sizeof((x)[0]))

Servo servos[4];

MPU6050 gyroscope;

MS5837 depthSensor;

bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer

Quaternion q;
VectorFloat gravity;
float ypr[3];

float roll;
int holddepth;
int depth;

int depthled = 0; // depth level indicator led pin
int depthbutton = 1; // depth set button pin
int upbutton = 4; // add air to ballast tank/add water to fore trim tank pin
int downbutton = A1; // release air from ballast tank/add water to aft trim tank pin
int modebutton = A0; // change manual control mode pin
int extrabutton1 = 12; // extra button one pin
int extrabutton2 = 13; // extra button two pin
int acta = 8; // add air to ballast actuator pin
int actb = 7; // release air from ballast actuator pin
int pumpa = 11; // fore pump pin
int pumpb = 10; // aft pump pin
int xjoystick = A2; // joystick x input
int yjoystick = A3; // joystick y input

int depthsetbutton;

int servoangles[4];

int maxangle = 135; //maximum angle needed for x-pattern
int minangle = 45; //minimum angle needed for x-pattern

volatile bool mpuInterrupt = false;

void dmpDataReady() {
    mpuInterrupt = true;
}

void setServos(Servo servo, float degrees) {

    float microseconds = degrees * 9.9 + 870

    servo.writeMicroseconds(microseconds);

}

void setup() {

    //set the pins to be either an input, or output
    pinMode(depthbutton, INPUT);
    pinMode(upbutton, INPUT);
    pinMode(downbutton, INPUT);
    pinMode(modebutton, INPUT);
    pinMode(extrabutton1, INPUT);
    pinMode(extrabutton2, INPUT);
    pinMode(depthled, OUTPUT);
    pinMode(pumpa, OUTPUT);
    pinMode(pumpb, OUTPUT);
    pinMode(acta, OUTPUT);
    pinMode(actb, OUTPUT);
    pinMode(INTERRUPT_PIN, INPUT); //TODO: It may need to be put after mpu initialization to work
    pinMode(xjoystick, INPUT);
    pinMode(yjoystick, INPUT);

    //set the servos to recieve pin input
    servos[0].attach(9);
    servos[1].attach(6);
    servos[2].attach(5);
    servos[3].attach(3);

    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    //TODO: set baud rate

    mpu.initialize();

    //TODO: have a timeout and error message if initilization fails
    if (!sensor.init()) {
        delay(1000);
    }

    sensor.setModel(MS5837::MS5837_30BA); // set depth sensor model to the MS5837, 30 bar
    sensor.setFluidDensity(997); // set fluid density to 997 kilograms per meter cubed (freshwater)

    //load and configure the DMP
    devStatus = mpu.dmpInitialize();

    //TODO: see if these offsets need to be changed for maximum accuracy
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788);

    //TODO: have error message if initialization fails
    if (devStatus == 0) {

        //TODO: Might need gyro calibration methods if results are off

        mpu.setDMPEnabled(true);

        //enable Arduino interrupt detection
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        dmpReady = true; //allows the main loop method to run

        packetSize = mpu.dmpGetFIFOPacketSize();
    }
}

void loop() {

    //TODO: create error message if setup fails (dmpReady)

    //while loop copied from mr jrowberg
    while (!mpuInterrupt && fifoCount < packetSize) {
        if (mpuInterrupt && fifoCount < packetSize) {
          // try to get out of the infinite loop
          fifoCount = mpu.getFIFOCount();
        }
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    fifoCount = mpu.getFIFOCount();

    //TODO: See if the following is correct
    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        fifoCount = mpu.getFIFOCount();


        // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();


        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);


        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    }

    // setting the servo angles
    float x = (analogRead(xjoystick) - 512) / 512.0f;
    float y = (analogRead(yjoystick) - 512) / 512.0f;

    int deadmin = 85;
    int deadmax = 95;

    float servoangles[0] = (atan(x - y) * -180 / PI) + 90;
    float servoangles[1] = (atan(x + y) * -180 / PI) + 90;
    float servoangles[2] = (atan(y - x) * -180 / PI) + 90;
    float servoangles[3] = (atan(x + y) * 180 / PI) + 90;

    for (int i = 0, i < arraylength(servos), i++ ) {

        if (servoangles[i] > maxangle){
            servoangles[i] = maxangle;
        } else if (servoangles[i] < minangle) {
            servoangles[i] = minangle;
        }

        if (servoangles[i] > deadmin && servosangles[i] < deadmax) servoangles[i] = 90;

        servoangles[i] = round(servoangles[i]);

        setServo(servos[i], servoangles[i]);
    }

    // TODO: figure out whether we need the vertical depth offset stuff

    //automatic tilt bouyancy system

    if (digitalRead(extrabutton1) == LOW) { // auto bouyancy is on

        roll = (ypr[2] * 180 / M_PI);

        if ((roll) < 2.5 and (roll) > -2.5) {

            digitalWrite(pumpa, LOW);
            digitalWrite(pumpb, LOW);
        }

        if ((roll) > 2.5) {

            digitalWrite(pumpa, HIGH);
            digitalWrite(pumpb, LOW);
        }

        if ((roll) < -2.5) {

            digitalWrite(pumpa, LOW);
            digitalWrite(pumpb, HIGH);
        }

    } else { // auto bouyancy is off

        if (digitalRead(modebutton) == LOW) { // mode button is switched to manual pump control

            if (upbutton == HIGH) {

                digitalWrite(pumpa, HIGH);
                digitalWrite(pumpb, LOW);
            }

            if (downbutton == HIGH) {

                digitalWrite(pumpa, LOW);
                digitalWrite(pumpb, HIGH);
            }
        }

        if (digitalRead(modebutton) == HIGH) { // mode button is switched to manual actuator control

            if (upbutton == HIGH) {

                digitalWrite(acta, HIGH);
                digitalWrite(actb, LOW);
            }

            if (downbutton == HIGH) {

                digitalWrite(acta, LOW);
                digitalWrite(actb, HIGH);
            }
        }
    }
}
