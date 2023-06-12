#include <Sodaq_LSM303AGR.h>
#include <Adafruit_MCP4728.h>
#include <Wire.h>


Sodaq_LSM303AGR magnetometer;

Adafruit_MCP4728 dac;

double magCalX = 0;
double magCalY = 0;

void setup() {
  
    dac.begin(0x61);
    
    magnetometer.rebootMagnetometer();
    delay(1000);
    magnetometer.enableMagnetometer(Sodaq_LSM303AGR::MagHighResMode, Sodaq_LSM303AGR::Hz100, Sodaq_LSM303AGR::Continuous);

    uint8_t axes = Sodaq_LSM303AGR::MagX;
    magnetometer.enableMagnetometerInterrupt(axes, -400);

    //sets zero of magnetometer, used for correction of dimensional offset
    magCalX = magnetometer.getMagX();
    magCalY = magnetometer.getMagY();

}

void loop() {

    float x = (magnetometer.getMagX() - magCalX) / 3000;
    float y = (magnetometer.getMagY() - magCalY) / 3000;

    // convert -1 to 1 normalized values into 0-4096
    x *= 2048;
    y *= 2048;
    x += 2048;
    y += 2048;

    if(x > 4095) x = 4095; if(x < 0) x = 0;
    if(y > 4095) y = 4095; if(y < 0) y = 0; 
    
    dac.setChannelValue(MCP4728_CHANNEL_A, x);
    dac.setChannelValue(MCP4728_CHANNEL_B, y);
}
