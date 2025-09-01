#include "main.h"

//Parallel function to change Right Antenna status
void RightAntennaRaise(bool RightAntennaState) {
    rightAntenna.set(RightAntennaState);
}

void RightAntennaControl() {
    //If Right Arrow is pressed, toggle Right Antenna state
    rightAntenna.button_toggle(master.get_digital(DIGITAL_RIGHT));
}