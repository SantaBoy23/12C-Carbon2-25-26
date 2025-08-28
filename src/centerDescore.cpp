#include "main.h"

void CenterDescoreControl() {
    //If L2 is pressed, toggle Center Descore state
    centerDescore.button_toggle(master.get_digital(DIGITAL_UP));
}