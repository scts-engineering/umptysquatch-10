#include "buttons.h"
#include <Arduino.h>

void attachSwitch(DebouncedSwitch *sw, int pinNum, int debounceDelay, void (*callback)(int)) {
    sw->lastDebounceTime = 0;
    sw->debounceDelay = debounceDelay;
    sw->bounceCheckReading = 0;
    sw->potentialReading = 0;
    sw->realReading = 0;
  
    sw->pinNum = pinNum;
    sw->callback = callback;
    pinMode(sw->pinNum, INPUT);
}

boolean tickSwitch(DebouncedSwitch *sw) {
    sw->bounceCheckReading = digitalRead(sw->pinNum);
    
    if(sw->potentialReading != sw->bounceCheckReading) {
        sw->lastDebounceTime = millis();
    }
    
    if((millis() - sw->lastDebounceTime) > sw->debounceDelay) {
        
        if(sw->potentialReading != sw->realReading) {
            
            sw->realReading = sw->potentialReading;
            
            sw->callback(sw->realReading);

            return true;
        }
    }
    return false;
}
