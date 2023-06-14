#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"


#include "MS5837.h"
#include <Servo.h>
#include "Sodaq_LSM303AGR.h"

#include "shared/submarine.h"
#include "buttons.h"
#include "Moving_Average.h"

#ifdef USE_WIFI
#include <WiFi.h>
//#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
WiFiClient client;
WiFiServer server(12241);
char abuf[128];
boolean readyForPrint = false;
#endif

extern "C" {
  #include <hardware/watchdog.h>
  #include <pico/cyw43_arch.h>
};

boolean setupWifi() {
  //delay(5000);
  Serial.println("starting wifi...");
      WiFi.mode(WIFI_STA);
      WiFi.setHostname("u10-ota");
  WiFi.begin("OpenWrt", "goodbyecenturylink");
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
      Serial.println("wifi connection failed");
      pinMode(LED_BUILTIN, OUTPUT);
      digitalWrite(LED_BUILTIN, HIGH);
      return false;
  }
  
  cyw43_pm_value(CYW43_NO_POWERSAVE_MODE, 20, 1, 1, 1);
  WiFi.noLowPowerMode();

    ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else {  // U_FS
      type = "filesystem";
    }

    // NOTE: if updating FS this would be the place to unmount FS using FS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });
  ArduinoOTA.begin();
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  return true;
}


Servo servos[4];

MPU6050 gyroscope;

MS5837 depthSensor;

uint8_t mpuIntStatus;
uint16_t packetSize;
float ypr[3];

volatile bool mpuInterrupt = false;

int blinkSpeed; //sets the blink interval of the LED
float holdDepth; // the depth recorded after going into auto mode, must maintain this depth closely

long currentTime, previousTime;


BlinkInterval blinkInterval;
Mode mode;

boolean pumpMode = true; //always defaults to pump mode for the first time it is put into manual mode

DebouncedSwitch modeSwitch;




float floatArrMean(float arr[], int size) {
    float value;
    for(int i = 0; i < size; ++i) {
        value += arr[i];  
    }
    return value / size;
}

void setPumps(bool fwd, bool aft) {
    digitalWrite(PUMP_A_PIN, fwd ? HIGH : LOW);
    digitalWrite(PUMP_B_PIN, aft ? HIGH : LOW);
}

void setActuators(bool vent, bool blow) {
    digitalWrite(ACTUATOR_A_PIN, vent ? HIGH : LOW);
    digitalWrite(ACTUATOR_B_PIN, blow ? HIGH : LOW);
}

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

    // set servos to resting position from the start

    setServo(&servos[0], 90);
    setServo(&servos[1], 90);
    setServo(&servos[2], 90);
    setServo(&servos[3], 90);
    
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

        gyroscope.CalibrateAccel(6);
        gyroscope.CalibrateGyro(6);

        gyroscope.setDMPEnabled(true);

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
        debugPrintln("waiting on depth sensor...");
        delay(1000);
    } else {
        debugPrintln("depth sensor initilized successfully");
    }
    
    depthSensor.setModel(MS5837::MS5837_30BA); // set depth sensor model to the MS5837, 30 bar
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

        depthSensor.read();
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

void setInitialMode() { // note that this method only runs during the startup to determine the starting mode
    changeMode(digitalRead(MODE_SWITCH_PIN));
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

bool isLedOn = false;

void setLEDPWM(float pwmVal) {
    analogWrite(DEPTH_LED_PIN, pwmVal);    
}

void setLED(bool on) {
    digitalWrite(DEPTH_LED_PIN, on ? HIGH : LOW);
    isLedOn = on;
}

void processSteering() { // read the joystick, then set the servo angles

    float servoAngles[4];

    float x = (analogRead(JOYSTICK_X_PIN) - 512) / 512.0f;
    float y = (analogRead(JOYSTICK_Y_PIN) - 512) / 512.0f;
    // if it goes the opposite x direction remove this
    x = -x;

    const float deadSize = 0.4f;
    if(x < deadSize && x > -deadSize && y < deadSize && y > -deadSize) { x = 0; y = 0;}
    
    static float valuesX[1000];
    static float valuesY[1000];
    static int valuesIndex;
    valuesX[valuesIndex] = x;
    valuesY[valuesIndex++] = y;

    static Moving_Average<float, double, 20> maX;
    static Moving_Average<float, double, 20> maY;

    if(valuesIndex == 1000) {

      valuesIndex = 0;

      x = floatArrMean(valuesX, 1000);
      y = floatArrMean(valuesY, 1000);

      x = maX(x);
      y = maY(y);
    
    /*Serial.print(x);
    Serial.print(" ");
    Serial.println(y);*/
    

    const float angleTEMP = 7.f;
    const float divisorTEMP = 1 / (angleTEMP * (1.f/45.f));
    const int minAngle = 45; //minimum angle required for x-pattern
    const int maxAngle = 135; //maximum angle required for x-pattern

    servoAngles[0] = ((atan(x - y) * -180 / PI) / divisorTEMP) + 90;
    servoAngles[1] = ((atan(x + y) * -180 / PI) / divisorTEMP) + 90;
    servoAngles[2] = ((atan(y - x) * -180 / PI) / divisorTEMP) + 90;
    servoAngles[3] = ((atan(x + y) * 180 / PI) / divisorTEMP) + 90;
   
    for (int i = 0; i < arraylength(servos); i++) {

        if (servoAngles[i] > maxAngle){
            servoAngles[i] = maxAngle;
        } else if (servoAngles[i] < minAngle) {
            servoAngles[i] = minAngle;
        }

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


}


void maintainEquilibrium() {

    float roll = (ypr[2] * 180 / M_PI);
    depthSensor.read();
    float depth = depthSensor.depth();


    if(!readyForPrint) {
        sprintf(abuf, "Depth: %.6f\nholdDepth: %.6f\n", depth, holdDepth);
        readyForPrint = true;
    }

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

    if (roll < 2.5 && roll > -2.5) {
        setPumps(false, false);
    }

    if (roll > 2.5) {
        setPumps(true, false);
    }

    if (roll < -2.5) {
        setPumps(false, true);
    }

    //if there is a depth offset sync the appropriate actuator to the blinking of the LED
    if(isDepthOff == TOO_HIGH) { //if the sub is too high, which activates the vent

        if(isLedOn) {
            setActuators(true, false);
        } else {
            setActuators(false, false);
        }

    } else if(isDepthOff == TOO_LOW) { //if the sub is too low, which activates the blow

        if(isLedOn) {
            setActuators(false, true);
        } else {
            setActuators(false, false);
        }

    } else {
        setActuators(false, false);
    }
}

void processPumpInput() {

    if (digitalRead(VENT_BUTTON_PIN) == HIGH) {
   
        setPumps(true, false);

    } else if (digitalRead(BLOW_BUTTON_PIN) == HIGH) {

        setPumps(false, true);

    } else {
        setPumps(false, false);
    }
}


void processActuatorInput() {

    setPumps(false, false);

    if (digitalRead(VENT_BUTTON_PIN) == HIGH) {

        setActuators(true, false);

    } else if (digitalRead(BLOW_BUTTON_PIN) == HIGH) {

        setActuators(false, true);

    } else {

        setActuators(false, false);
    }
}

void processLED() {
 
  if(blinkInterval == SOLID) {
      if(!isLedOn) {
        setLED(true);
      }
  } else {
    currentTime = millis();
  
    if(currentTime > previousTime + blinkInterval) {
        if(isLedOn) {
          setLED(false);
        } else {
          setLED(true);
        }
    
        previousTime = currentTime;
    }
  }
}

void turnOffAllDevices() {
    setPumps(false, false);
    setActuators(false, false);
}

bool canSetup1RunYet = false;
bool isDoneSettingUp = false;

void setup1() {

    while(!canSetup1RunYet) delay(5);
    
    float before = millis();
    float now = before;
    while(!isDoneSettingUp) {
        now = millis();
        // 1hz breathing
        setLEDPWM(127 * (sin(5.f*6.28f*now) + 1));
        delay(10);
    }

}

void setup() {
  #ifndef USE_WIFI
    Serial.begin(9600);
    #endif
    if(watchdog_caused_reboot()) { delay(5000); }
    Wire.setSDA(0);
    Wire.setSCL(1);
    Wire.begin();


    Serial.println("hello there");


    
    blinkInterval = FAST;
    previousTime = millis();

    setPinModes();
    Serial.println("I am done setting pin modes");

    canSetup1RunYet = true;

          
#ifdef USE_WIFI
    if(setupWifi()) {
        server.begin();
    }
    /*Serial.println(client.connect("192.168.1.162", 12241));
    if(!client.connected()) Serial.println("no connected to server"); else Serial.println("connected to server");
    client.setNoDelay(true);*/
#endif


    setupGyro();
    Serial.println("I am done setting up gyro");



    setupDepthSensor();
    // read a value because the first one is sometimes wrong
    depthSensor.read();
    depthSensor.depth();
    
    Serial.println("I am done setting up depth sensor");




    setInitialMode();

    watchdog_enable(2000, 1);


   Serial.println("I am done setting up");
   isDoneSettingUp = true;
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

    processLED();

    tickSwitch(&modeSwitch);
    
}

void loop1() {

  if(!client.connected()) { 
      client = server.available();
      client.setNoDelay(true);
  } else if(readyForPrint) {
      client.print(abuf);
      readyForPrint = false;
      client.flush();
  }

  
#ifdef USE_WIFI
    ArduinoOTA.handle();
#endif
    processSteering();
}
