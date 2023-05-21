// this is to be the format for the new (and hopefully improved) submarine code:



// start of the program

// library imports

// object definitions (servos, accelerometer, depth sensor)

// keyword definition

// variable declaration

// start of setup()

    // set the pins to all necessary inputs and outputs

    // set all servo objects to take output from pins

    // initialization of components

    // set gyro offsets

    if (/*the setup happened*/) {
        //set DMP to true
    }

// start of loop()

    if (/*the setup didn't happen*/) {
        // delay the loop start
    }

    //weird stuff here that I need to figure out

    //set x and y to the output of the joystick pins

    //set the angle of each servo to correspond to the x and y of the joystick

    for (//all the servos) {
        if (angle > maxangle) {
            angle = maxangle;
        } else if (angle < minangle) {
            angle = minangle;

        //set the deadzones for the joystick

        //round each servo angle

        //set the servos to their calculated angle

    }

    if (/*automatic bouyancy is switched to on*/) {

        //set the submarine pitch angle
        //read the depth sensor
        //set the depth variable to the recorded value from the sensor

        if (/*the depth set button is being pressed*/) {

            //set the depth variable to current depth
            //turn on the depth set LED (if that even exists)

        } else { //if the depth set button is not being pressed

            //turn off the depth set LED (again, if it even exists)
        }

        if (/*there is a difference between the vertical depth variable and the depth recorded after pressing the depth set button*/) {

            //adjust the actuators accordingly
        }

        if (/*there is tilt detected*/) {

            //adjust the pumps accordingly
        }

    } else { //auto bouyancy is not switched to on

        if (/*modebutton is switched to off*/) { //used to manually control pumps

            if (/*the manual up button is pressed*/) {

                //change the PUMPS accordingly
            }

            if (/*the manual down button is pressed*/) {

                //change the PUMPS accordingly
            }

        }

        if (/*modebutton is switched to on*/) { //used to manually control actuators

            if (/*the manual up button is pressed*/) {

                //change the ACTUATORS accordingly
            }

            if (/*the manual down button is pressed*/) {

                //change the ACTUATORS accordingly
            }
        }
    }
}
