#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "MS5837.h"
#include "Servo.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE // copied from mr jrowberg, guess it only imports the wire library if it needs it
#include "Wire.h"
#endif

Servo servos[4];

MPU6050 gyroscope;

MS5837 depthSensor;


