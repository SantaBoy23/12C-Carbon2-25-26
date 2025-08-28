#include "main.h"

void MatchLoadControl() {
    //If L2 is pressed, toggle Match Load state
    matchLoad.button_toggle(master.get_digital(DIGITAL_L2));
}