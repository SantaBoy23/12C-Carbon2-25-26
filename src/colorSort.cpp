#include "main.h"
/*
//Create a set of color modes
enum ColorMode {
  BlueMode,
  RedMode,
  NoSortMode
};

//Create a variable to store the current color mode, default to no sort
inline ColorMode current_mode = NoSortMode;

//Function to toggle between color modes
void ToggleColorMode() {
    if (current_mode == BlueMode) {
      current_mode = RedMode;
    } else if (current_mode == RedMode) {
      current_mode = NoSortMode;
    } else {
      current_mode = BlueMode;
    }

    //print current color mode on brain screen
    pros::lcd::print(6, "Mode: %s", current_mode == BlueMode ? "Blue" : (current_mode == RedMode ? "Red" : "No Sort"));
}

//Function to sort opposite color of the current mode
void ColorSort() {

  //Wait for 2 seconds to permit initialization of other components
  pros::delay(2000);

  //Always have this function running in the background
  while (true) {

    //Variable to store intake color sensor hue reading
    double IntakeColorHue = intakeColor.get_hue();

    //Variable to store intake color sensor proximity reading
    int IntakeColorProximity = intakeColor.get_proximity();

    //Run this loop if it is autonomous mode
    if (ColorSortAuto == true) {
    
      //Run this loop if the current mode is blue
      if (current_mode == BlueMode) {  

        //Run this loop if the color sensor detects a hue <17 and the promixity is ≤300
        if ((IntakeColorHue) < 25 && (IntakeColorProximity <= 300)) {

          pros::delay(80);

          //Reverse intake
          IntakeMove(-127);

          //Delay for 75ms
          pros::delay(65); //used to be 55

          //Restart driver intake control
          IntakeMove(127);
        } 
      }

      //Run this loop if the current mode is red
      else if (current_mode == RedMode) {

        //Run this loop if the color sensor detects a hue >190 and <350 and the promixity is ≤255
        if (((IntakeColorHue > 190) && (IntakeColorHue) < 350) && (IntakeColorProximity <= 255)) {

          pros::delay(40);

          //Reverse intake
          IntakeMove(-127);

          //Delay for 75ms
          pros::delay(100); //used to be 55

          //Restart driver intake control
          IntakeMove(127);
        } 
      }
    }

    //Run this loop if it is driver control
    if (ColorSortDriver == true) {

      //Run this loop if the current mode is blue
      if (current_mode == BlueMode) {  

        //Run this loop if the color sensor detects a hue <17 and the promixity is ≤300
        if ((IntakeColorHue) < 25 && (IntakeColorProximity <= 300)) {

          pros::delay(80);

          //Stop driver intake control
          IntakeControlActive = false;

          //Reverse intake
          IntakeMove(-127);

          //Delay for 75ms
          pros::delay(65);

          //Restart driver intake control
          IntakeControlActive = true;
        }
      }

      //Run this loop if the current mode is red
      else if (current_mode == RedMode) {

        //Run this loop if the color sensor detects a hue >190 and <350 and the promixity is ≤255
        if (((IntakeColorHue > 190) && (IntakeColorHue) < 350) && (IntakeColorProximity <= 255)) {

          pros::delay(40);

          //Reverse intake
          IntakeControlActive = false;

          //Reverse intake
          IntakeControl(-127);

          //Delay for 75ms
          pros::delay(100); //used to be 65

          //Restart driver intake control
          IntakeControlActive = true;
        } 
      }
    }

    //Delay to prevent overloading the CPU
    pros::delay(ez::util::DELAY_TIME);
  }
}

//Parallel function to set the color sensor light level
void ColorSensorLight(int LightLevel) {
  intakeColor.set_led_pwm(LightLevel);
}

void ColorSortKill () {
  if (master.get_digital(DIGITAL_UP) && master.get_digital(DIGITAL_LEFT)) {
    ColorSortDriver = false;
  }
}
*/