#include "buttons.h"
#include <Arduino.h>

void attachSwitch(DebouncedSwitch *sw, int pinNum, int debounceDelay, void (*callback)(int)) {
    sw->debounceDelay = debounceDelay;
    sw->buttonState = 1;
    sw->lastButtonState = 1;
    sw->reading = 1;
  
    sw->pinNum = pinNum;
    sw->callback = callback;
    pinMode(sw->pinNum, INPUT);
}

boolean tickSwitch(DebouncedSwitch *sw) {

    long lastDebounceTime = 0;
    
    sw->reading = digitalRead(sw->pinNum);
        
    if(sw->reading != sw->lastButtonState) {
        lastDebounceTime = millis();
    }
    
    if((millis() - lastDebounceTime) > sw->debounceDelay) {
        if(sw->reading != sw->buttonState) {
            
            sw->buttonState = sw->reading;

            sw->callback(sw->buttonState);
            
            return true;
        }
    }
    sw->lastButtonState = sw->reading;
    return false;
}
