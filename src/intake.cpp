#include "main.h"

//Function for intake driver control
void IntakeControl() {

        //If L1 is pressed, spin intake forward
        if (master.get_digital(DIGITAL_L1)) {
            intakeBottom.move(127);
            intakeTop.move(-127);
        } 

        //If L2 is pressed, spin intake backward
        else if (master.get_digital(DIGITAL_R1)) {
            intakeBottom.move(127);
            intakeTop.move(127);
        } 

        else if (master.get_digital(DIGITAL_R2)) {
            intakeBottom.move(-127);
            intakeTop.move(-127);
        }

        //If no button is pressed, stop intake
        else {
            intakeBottom.move(0);
            intakeTop.move(0);
        }
}