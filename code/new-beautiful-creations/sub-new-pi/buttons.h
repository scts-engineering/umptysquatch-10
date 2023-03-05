#include <Arduino.h>

typedef struct DebouncedSwitch {
    long lastDebounceTime;
    long debounceDelay;
    int bounceCheckReading; // the second one to check the first one
    int potentialReading; // the initial one that we aren't sure of
    int realReading; // our final conclusion
    
    int pinNum;
    // the function to run when state changes, with the int holding the new state
    void (*callback)(int);
} DebouncedSwitch;

void attachSwitch(DebouncedSwitch *sw, int pinNum, int debounceDelay, void (*callback)(int));

// returns true when there is a CHANGE in state,
// also runs the callback in the DebouncedSwitch
boolean tickSwitch(DebouncedSwitch *sw);

