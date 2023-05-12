#include <Sodaq_LSM303AGR.h>
#include <Adafruit_MCP4728.h>
#include <Wire.h>


Sodaq_LSM303AGR magnetometer;

Adafruit_MCP4728 dac;

double magCalX = 0;
double magCalY = 0;

void setup() {

    dac.begin();

    magnetometer.rebootMagnetometer();

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
    if(x > 1) x = 1; else if(x < -1) x = -1;
    if(y > 1) x = 1; else if(y < -1) y = -1;

    // convert -1 to 1 normalized values into 0-4096
    x *= 2048;
    y *= 2048;
    x += 2048;
    y += 2048;
    dac.setChannelValue(MCP4728_CHANNEL_A, x);
    dac.setChannelValue(MCP4728_CHANNEL_B, y);
}
