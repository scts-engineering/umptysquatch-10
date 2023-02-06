#include <Sodaq_LSM303AGR.h>

Sodaq_LSM303AGR accel;

//volatile bool magInterruptFlag = true;

//void magInterrupt() 
//{
  //  magInterruptFlag = true;
//}

void setup() 
{
    Serial.begin(57600);
    delay(1000);

    Serial.println("BEGIN");
    Wire.begin();
    delay(1000);

    if (accel.checkWhoAmI()) {
        Serial.println("FOUND ACCEL!");
    }
    else {
        Serial.println("NO ACCEL!");
    }
    
    accel.rebootMagnetometer();
    delay(1000);

    accel.enableMagnetometer(Sodaq_LSM303AGR::MagHighResMode, Sodaq_LSM303AGR::Hz100, Sodaq_LSM303AGR::Continuous);

    uint8_t axes = Sodaq_LSM303AGR::MagX;
    accel.enableMagnetometerInterrupt(axes, -400);

//    pinMode(MAG_INT, INPUT_PULLDOWN);
  //  attachInterrupt(MAG_INT, magInterrupt, RISING);
}

void loop() 
{
    delay(10);

    Serial.print(accel.getMagX());
    Serial.print(" ");
    Serial.print(accel.getMagY());
    Serial.print(" ");
    Serial.println(accel.getMagZ());

/*    if (magInterruptFlag) {
        Serial.println("INTERRUPT");
        magInterruptFlag = false;
    }*/
}
