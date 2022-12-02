#include "I2Cdev.h" // include I2C(a form of serial sensor communication) library


#include "MPU6050_6Axis_MotionApps20.h" //include MPU6050(accelerometer) six axis library


#include "MS5837.h" // include MS5837(depth sensor) library


#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE // does the I2C library need the Arduino Wire library?
#include "Wire.h" // if so, include Wire(another I2C library, for Arduino-type programs) library
#endif // end the conditional statement


#include "Servo.h" // include servo motor library


Servo servoa; // make object for servo one from servo class in servo library
Servo servob; // make object for servo two from servo class in servo library
Servo servoc; // make object for servo three from servo class in servo library
Servo servod; // make object for servo four from servo class in servo library


MPU6050 mpu; // make object for accelerometer from MPU5050 library


MS5837 sensor; // make object for depth sensor from MS5837 library


#define OUTPUT_READABLE_YAWPITCHROLL // define a "keyword" from the MPU6050 library for outputting human readable angular measurement data from the accelerometer


#define INTERRUPT_PIN 2  // define a "keyword" which stands for "2", the interrupt pin for the MPU6050 library, needed in I2C communication
#define 13 // define a "keyword" which stands for "13", the status led pin for the MPU6050 library
bool blinkState = false; // boolean which defines whether the MPU6050 status led is on or off and set it to false


// MPU6050 control/status variables
bool dmpReady = false;  // boolean which defines whether the MPU6050 DMP(digital motion processing) is on or off and set it to false
uint8_t mpuIntStatus;   // unsigned integer of 1 byte, or 8 bits, in length, which holds the interrupt status of the MPU6050
uint8_t devStatus;      // unsigned integer of 1 byte, or 8 bits, in length, which holds the operation return status of the MPU6050
uint16_t packetSize;    // unsigned integer of 2 bytes, or 16 bits, in length, which holds the expected data packet size for the DMP(digital motion processing)
uint16_t fifoCount;     // unsigned integer of 2 bytes, or 16 bits, in length, which holds the counter which states the number of all bytes in the FIFO(first in - first out) buffer
uint8_t fifoBuffer[64]; // array of 64 unsigned integers of 1 byte, or 8 bits, acting as a storage area for a FIFO(first in - first out) buffer


// MPU6050 orientation/motion variables
Quaternion q;           // [w, x, y, z]         container which holds a quaternion(x, y, z coordinates with a rotational axis, w) for the MPU6050 measurements
VectorInt16 aa;         // [x, y, z]            three dimensional vector of 2 byte, or 16 bits, in length integers, to hold the MPU6050 acceleration data
VectorInt16 aaReal;     // [x, y, z]            three dimensional vector of 2 byte, or 16 bits, in length integers, to hold the gravity removed MPU6050 acceleration data
VectorInt16 aaWorld;    // [x, y, z]            three dimensional vector of 2 byte, or 16 bits, in length integers, to hold the MPU6050 acceleration data corrected to be within the world frame
VectorFloat gravity;    // [x, y, z]            three dimensional floating point vector to hold the gravity direction
float euler[3];         // [psi, theta, phi]    array of three floating point values to hold the euler angles for the MPU6050
float ypr[3];           // [yaw, pitch, roll]   array of three floating point values to hold the angles in radians for the MPU6050, as yaw, pitch, and roll


float roll; // submarine tilt angle floating point value


int holddepth; // maintained depth integer
int depth; // current depth integer


int depthled = 0; // depth level indicator led pin integer
int depthbutton = 1; // depth set button pin integer
int upbutton = 4; // add air to ballast tank/add water to fore trim tank pin integer
int downbutton = A1; // release air from ballast tank/add water to aft trim tank pin integer
int modebutton = A0; // change manual control mode pin integer
int extrabutton1 = 12; // extra button one pin integer
int extrabutton2 = 13; // extra button two pin integer
int acta = 8; // add air to ballast actuator pin integer
int actb = 7; // release air from ballast actuator pin integer
int pumpa = 11; // fore pump pin integer
int pumpb = 10; // aft pump pin integer
int xpos = analogRead(A2); // joystick x-axis analog input pin integer
int ypos = analogRead(A3); // joystick y-axis analog input pin integer


int depthsetbutton; // depth set button boolean value as integer


int servoadegrees; // servo one angle integer
int servobdegrees; // servo two angle integer
int servocdegrees; // servo three angle integer
int servoddegrees; // servo four angle integer


int maxangle = 180; // maximum angle for each servo integer
int minangle = 0; // minimum angle for each servo integer


int average; // average value between the maximum and minimum angles for each servo integer


volatile bool mpuInterrupt = false; // volatile(low-priority in program optimization when compiling) boolean which is an indicator for the state of the interrupt pin for the MPU6050
void dmpDataReady() { // start of function which sets the interrupt pin state indicator to a value of the for the MPU6050
  mpuInterrupt = true; // set the interrupt pin state indicator to true
} // end of function which sets the interrupt pin state indicator to a value of the for the MPU6050

void setup() { // start of the area to define necessary parameters of the program


  pinMode(depthbutton, INPUT); // set the depth button pin to an input
  pinMode(upbutton, INPUT); // set the add air to ballast tank/add water to fore trim tank pin to an input
  pinMode(downbutton, INPUT); // set the release air from ballast tank/add water to aft trim tank pin to an input
  pinMode(modebutton, INPUT); // set the change manual control mode pin to an input
  pinMode(extrabutton1, INPUT); // set the extra button one pin to an input
  pinMode(extrabutton2, INPUT); // set the extra button two pin to an input
  pinMode(depthled, OUTPUT); // set the depth level indicator led pin to an output
  pinMode(pumpa, OUTPUT); // set the fore pump pin to an output
  pinMode(pumpb, OUTPUT); // aft pump pin to an output
  pinMode(acta, OUTPUT); // add air to ballast actuator pin integer
  pinMode(actb, OUTPUT); // release air from ballast actuator pin integer


  servoa.attach(9); // set the object for servo one from servo class in servo library to be attached to pin number 9
  servob.attach(6); // set the object for servo two from servo class in servo library to be attached to pin number 6
  servoc.attach(5); // set the object for servo three from servo class in servo library to be attatched to pin number 5
  servod.attach(3); // set the object for servo four from servo class in servo library to be attached to pin number 3


  Serial.begin(9600); // initialize serial data output (to communicate with another computer) at a baud rate(communication rate) of 9600


  // join I2C serial communication data bus
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE // does the I2C library need the Arduino Wire library?  If so, do the following: 
  Wire.begin(); // begin communication
  Wire.setClock(400000); // wz, through the Wire library
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE // does the I2C library have a built-in wire(fast wire) system?
  Fastwire::setup(400, true); // if so, setup the fastwire system to run at a clock speed of 400 kiloHertz
#endif // end the conditional statement


  mpu.initialize(); // initialize MPU6050 through object for accelerometer from MPU5050 library
  pinMode(INTERRUPT_PIN, INPUT); // set the interrupt pin to an input


  if (!sensor.init()) { // has the MS5837 not been initialized? (from the sensor library)
    delay(1000); // if so, delay the program for 1000 milliseconds (one second)
  } // end the conditional statement


  sensor.setModel(MS5837::MS5837_30BA); // set the depth sensor model in the sensor library to the MS5837, 30 bar
  sensor.setFluidDensity(997); // set the fluid density in the sensor library to the fluid density of freshwater, which is 997 kilograms per meter cubed


  devStatus = mpu.dmpInitialize(); // set the operation return status for the MPU6050, via the mpu object, to the returned value of the function which initializes the DMP(digital motion processing), which is from the MPU6050 library


  mpu.setXGyroOffset(220); // set the x axis offset of the MPU6050 gyroscope to 220 through the mpu object and the corresponding function from the MPU6050 library
  mpu.setYGyroOffset(76); // set the y axis offset of the MPU6050 gyroscope to 76 through the mpu object and the corresponding function from the MPU6050 library
  mpu.setZGyroOffset(-85); // set the z axis offset of the MPU6050 gyroscope to -85 through the mpu object and the corresponding function from the MPU6050 library
  mpu.setZAccelOffset(1788); // set the z axis acceleration offset of the MPU6050 to 220 through the mpu object and the corresponding function from the MPU6050 library


  if (devStatus == 0) { // is the operation return status false, or zero? If so, do the following:


    mpu.setDMPEnabled(true); // set the DMP(digital motion processing) to true, or enable it, through the MPU6050 library


    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING); // set the interrupt pin of the Arduino microcontroller to the interrupt pin, within the function which turns it from a digital pin to interrupt, have it run the ISR(interrupt data routine) of the dmpDataReady function, and have it triggered on the rising edge (low to high) of the signal to the pin
    mpuIntStatus = mpu.getIntStatus(); // set the interrupt status of the MPU6050 to the status returned by the corresponding function within the MPU6050 library


    dmpReady = true; // set the DMP(digital motion processing) of the MPU6050 to true


    packetSize = mpu.dmpGetFIFOPacketSize(); // set the DMP(digital motion processing)data packet size to the return of the function to get the FIFO(first-in first-out) data packet size, from the MPU6050 library
  } // end of the conditional statement


} // end of the area to define necessary parameters of the program




void loop() { // start of the main loop of the program


  if (!dmpReady) // is the DMP(digital motion processing) not ready (false)?
  { // start of the conditional statement


    delay(30); // if so, delay the program for 30 milliseconds, or 0.03 seconds


  } // end of the conditional statement


  while (!mpuInterrupt && fifoCount < packetSize) {
    if (mpuInterrupt && fifoCount < packetSize) {
      // try to get out of the infinite loop
      fifoCount = mpu.getFIFOCount();
    }


  }


  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();


  // get current FIFO count
  fifoCount = mpu.getFIFOCount();


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


#ifdef OUTPUT_READABLE_YAWPITCHROLL
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
#endif


  } // end of the conditional statement


  ypos = analogRead(A2); // set the position on the y axis of the joystick to the analog read of the joystick y axis pin, which is analog input pin two, thus automatically mapping it to a 10 bit value, or an integer between 0 and 1023
  servoadegrees = map(ypos, 0, 1023, minangle, maxangle); // linearly map the joystick value to servo angles


  ypos = analogRead(A2); // set the position on the y axis of the joystick to the analog read of the joystick y axis pin, which is analog input pin two, thus automatically mapping it to a 10 bit value, or an integer between 0 and 1023
  servobdegrees = map(ypos, 0, 1023, maxangle, minangle); // linearly map the joystick value to servo angles


  average = ((maxangle - minangle) / 2); // average the maximum and minimum angles


  xpos = analogRead(A3); // set the position on the x axis of the joystick to the analog read of the joystick x axis pin, which is analog input pin three, thus automatically mapping it to a 10 bit value, or an integer between 0 and 1023
  xpos = map(xpos, 0, 1023, minangle, maxangle); // linearly map the joystick value to servo angles


  if (xpos < average - 4) // if the maximum to minimum angle mapped joystick x axis position is less than the maximum and minimum angle value average minus four (check if the joystick is off center in the x axis in one direction), do the following:
  { // start of the conditional statement


    xpos = analogRead(A3); // set the position on the x axis of the joystick to the analog read of the joystick x axis pin, which is analog input pin three, thus automatically mapping it to a 10 bit value, or an integer between 0 and 1023
    xpos = map(xpos, 0, 1023, maxangle, minangle); // linearly map the joystick value to servo angles


    servobdegrees = servobdegrees + abs(xpos - average); // correct the servo angle for the absolute value of the mapped joystick xpos minus the average


    xpos = analogRead(A3); // set the position on the x axis of the joystick to the analog read of the joystick x axis pin, which is analog input pin three, thus automatically mapping it to a 10 bit value, or an integer between 0 and 1023
    xpos = map(xpos, 0, 1023, minangle, maxangle); // linearly map the joystick value to servo angles


    servoadegrees = servoadegrees + abs(xpos - average); // correct the servo angle for the absolute value of the mapped joystick xpos minus the average


  } // end of the conditional statement


  if (xpos > average + 4) // if the maximum to minimum angle mapped joystick x axis position is more than the maximum and minimum angle value average plus four (check if the joystick is off center in the x axis in one direction), do the following:
  { // start of the conditional statement


    xpos = analogRead(A3); // set the position on the x axis of the joystick to the analog read of the joystick x axis pin, which is analog input pin three, thus automatically mapping it to a 10 bit value, or an integer between 0 and 1023
    xpos = map(xpos, 0, 1023, minangle, maxangle); // linearly map the joystick value to servo angles


    servoadegrees = servoadegrees - abs(xpos - average); // correct the servo angle for the absolute value of the mapped joystick xpos minus the average


    xpos = analogRead(A3); // set the position on the x axis of the joystick to the analog read of the joystick x axis pin, which is analog input pin three, thus automatically mapping it to a 10 bit value, or an integer between 0 and 1023
    xpos = map(xpos, 0, 1023, maxangle, minangle); // linearly map the joystick value to servo angles


    servobdegrees = servobdegrees - abs(xpos - average); // correct the servo angle for the absolute value of the mapped joystick xpos minus the average


  } // end of the conditional statement


  ypos = analogRead(A2); // set the position on the y axis of the joystick to the analog read of the joystick y axis pin, which is analog input pin two, thus automatically mapping it to a 10 bit value, or an integer between 0 and 1023
  servocdegrees = map(ypos, 0, 1023, minangle, maxangle); // linearly map the joystick value to servo angles


  ypos = analogRead(A2); // set the position on the y axis of the joystick to the analog read of the joystick y axis pin, which is analog input pin two, thus automatically mapping it to a 10 bit value, or an integer between 0 and 1023
  servoddegrees = map(ypos, 0, 1023, maxangle, minangle); // linearly map the joystick value to servo angles


  average = ((maxangle - minangle) / 2); // average the maximum and minimum angles


  xpos = analogRead(A3); // set the position on the x axis of the joystick to the analog read of the joystick x axis pin, which is analog input pin three, thus automatically mapping it to a 10 bit value, or an integer between 0 and 1023
  xpos = map(xpos, 0, 1023, minangle, maxangle); // linearly map the joystick value to servo angles


  if (xpos < average - 4) // if the maximum to minimum angle mapped joystick x axis position is less than the maximum and minimum angle value average minus four (check if the joystick is off center in the x axis in one direction), do the following:
  { // start of the conditional statement


    xpos = analogRead(A3); // set the position on the x axis of the joystick to the analog read of the joystick x axis pin, which is analog input pin three, thus automatically mapping it to a 10 bit value, or an integer between 0 and 1023
    xpos = map(xpos, 0, 1023, maxangle, minangle); // linearly map the joystick value to servo angles


    servoddegrees = servoddegrees - abs(xpos - average); // correct the servo angle for the absolute value of the mapped joystick xpos minus the average


    xpos = analogRead(A3); // set the position on the x axis of the joystick to the analog read of the joystick x axis pin, which is analog input pin three, thus automatically mapping it to a 10 bit value, or an integer between 0 and 1023
    xpos = map(xpos, 0, 1023, minangle, maxangle); // linearly map the joystick value to servo angles


    servocdegrees = servocdegrees - abs(xpos - average); // correct the servo angle for the absolute value of the mapped joystick xpos minus the average


  } // end of the conditional statement


  if (xpos > average + 4) // if the maximum to minimum angle mapped joystick x axis position is more than the maximum and minimum angle value average plus four (check if the joystick is off center in the x axis in one direction), do the following:
  { // start of the conditional statement


    xpos = analogRead(A3); // set the position on the x axis of the joystick to the analog read of the joystick x axis pin, which is analog input pin three, thus automatically mapping it to a 10 bit value, or an integer between 0 and 1023
    xpos = map(xpos, 0, 1023, minangle, maxangle); // linearly map the joystick value to servo angles


    servocdegrees = servocdegrees + abs(xpos - average); // correct the servo angle for the absolute value of the mapped joystick xpos minus the average


    xpos = analogRead(A3); // set the position on the x axis of the joystick to the analog read of the joystick x axis pin, which is analog input pin three, thus automatically mapping it to a 10 bit value, or an integer between 0 and 1023
    xpos = map(xpos, 0, 1023, maxangle, minangle); // linearly map the joystick value to servo angles


    servoddegrees = servoddegrees + abs(xpos - average); // correct the servo angle for the absolute value of the mapped joystick xpos minus the average


  } // end of the conditional statement




  if (servoadegrees > maxangle) // is the angle, in degrees, of servo one more than the maximum angle?
  { // start of the conditional statement


    servoadegrees = maxangle; // if so, set the angle of servo one to the maximum angle


  } // end of the conditional statement


  if (servoadegrees < minangle) // is the angle, in degrees, of servo one less than the minimum angle?
  { // start of the conditional statement


    servoadegrees = minangle; // if so, set the angle of servo one to the minimum angle


  } // end of the conditional statement


  if (servobdegrees > maxangle) // is the angle, in degrees, of servo two more than the maximum angle?
  { // start of the conditional statement


    servobdegrees = maxangle; // if so, set the angle of servo two to the maximum angle


  } // end of the conditional statement


  if (servobdegrees < minangle) // is the angle, in degrees, of servo two less than the minimum angle?
  { // start of the conditional statement


    servobdegrees = minangle; // if so, set the angle of servo two to the minimum angle


  } // end of the conditional statement


  if (servocdegrees > maxangle) // is the angle, in degrees, of servo three more than the maximum angle?
  { // start of the conditional statement


    servocdegrees = maxangle; // if so, set the angle of servo three to the maximum angle


  } // end of the conditional statement


  if (servocdegrees < minangle) // is the angle, in degrees, of servo three less than the minimum angle?
  { // start of the conditional statement


    servocdegrees = minangle; // if so, set the angle of servo three to the minimum angle


  } // end of the conditional statement


  if (servoddegrees > maxangle) // is the angle, in degrees, of servo four more than the maximum angle?
  { // start of the conditional statement


    servoddegrees = maxangle; // if so, set the angle of servo four to the maximum angle


  } // end of the conditional statement


  if (servoddegrees < minangle) // is the angle, in degrees, of servo four less than the minimum angle?
  { // start of the conditional statement


    servoddegrees = minangle; // if so, set the angle of servo four to the minimum angle


  } // end of the conditional statement


  servoa.write(servoadegrees); // set the angle of servo one, in degrees, to its corresponding angle varaible, through the servo library
  servob.write(servobdegrees); // set the angle of servo two, in degrees, to its corresponding angle varaible, through the servo library
  servoc.write(servocdegrees); // set the angle of servo three, in degrees, to its corresponding angle varaible, through the servo library
  servod.write(servoddegrees); // set the angle of servo four, in degrees, to its corresponding angle varaible, through the servo library


  if (digitalRead(extrabutton1) == LOW) // handle manual override button
  {


    roll = (ypr[2] * 180 / M_PI); // set the submarine tilt value to the yaw(second value of the yaw, pitch, roll, array), converted from degrees to radians by multiplying it by 180/pi
    sensor.read(); // read the depth sensor through the MS5837 library
    depth = sensor.pressure(); // set the current submarine depth value to the recorded pressure


    depthsetbutton = digitalRead(depthbutton); // set the recorded value of the depth level set button to


    if (depthsetbutton == HIGH) { // if the recorded value of the boolean which holds the state of the depth set button is high, or if the depth set button is being pressed, do the following:


      holddepth = depth; // set the maintained depth level to the current recorded depth level
      digitalWrite(depthled, HIGH); // set the depth level indicator led to high, or turn it on


    } else { // if not, do the following:


      digitalWrite(depthled, LOW); // set the value of the depth level indicator led to low, or shut the led off


    } // end of the conditional statement


    if (depth > holddepth + 5) { // is there a vertical depth offset?  If so, adjust the actuators with the following:


      digitalWrite(acta, HIGH);
      digitalWrite(actb, LOW);


    } // end of the conditional statement


    if (depth < holddepth - 5) { // is there a vertical depth offset?  If so, adjust the actuators with the following:




      digitalWrite(acta, LOW);
      digitalWrite(actb, HIGH);


    } // end of the conditional statement


    if (depth < holddepth + 5 && depth > holddepth - 5) { // is there a vertical depth offset?  If so, adjust the actuators with the following:




      digitalWrite(acta, LOW);
      digitalWrite(actb, LOW);


    } // end of the conditional statement




    if ((roll) < 2.5 and (roll) > -2.5) { // is there an angular depth offset?  If so, adjust the pumps with the following:


      digitalWrite(pumpa, LOW);
      digitalWrite(pumpb, LOW);


    } // end of the conditional statement


    if ((roll) > 2.5) { // is there an angular depth offset?  If so, adjust the pumps with the following:


      digitalWrite(pumpa, HIGH);
      digitalWrite(pumpb, LOW);


    } // end of the conditional statement


    if ((roll) < -2.5) { // is there an angular depth offset?  If so, adjust the pumps with the following:


      digitalWrite(pumpa, LOW);
      digitalWrite(pumpb, HIGH);


    } // end of the conditional statement


  } else { // the following is all for the manual override


    if (digitalRead(modebutton) == LOW)
    {


      if (digitalRead(upbutton) == HIGH)
      {


        digitalWrite(pumpa, HIGH);
        digitalWrite(pumpb, LOW);


      }


      if (digitalRead(downbutton) == HIGH)
      {


        digitalWrite(pumpa, LOW);
        digitalWrite(pumpb, HIGH);


      }


    }


    if (digitalRead(modebutton) == HIGH)
    {


      if (digitalRead(upbutton) == HIGH)
      {


        digitalWrite(acta, HIGH);
        digitalWrite(actb, LOW);


      }


      if (digitalRead(downbutton) == HIGH)
      {


        digitalWrite(acta, LOW);
        digitalWrite(actb, HIGH);


      }


    }


  }


} // end of the main loop of the program
