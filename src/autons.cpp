#include "main.h"

const int DRIVE_SPEED = 110;
const int TURN_SPEED = 90;
const int SWING_SPEED = 110;
const int INTAKE_SPEED = 127;



void default_constants() {
  // P, I, D, and Start I
  chassis.pid_drive_constants_forward_set(9.5, 0, 6.5);         // Forward constants, used for odom and non odom motions
  chassis.pid_drive_constants_backward_set(9.5, 0, 6.5);         // Reverse constants, used for odom and non odom motions
  chassis.pid_heading_constants_set(11.0, 0.0, 20.0);        // Holds the robot straight while going forward without odom
  chassis.pid_turn_constants_set(1.7, 0.0, 2.5);     // Turn in place constants
  chassis.pid_swing_constants_set(3.0, 0.0, 1.6);           // Swing constants //Needs a tad more D
  chassis.pid_odom_angular_constants_set(11.5, 0.0, 68.5);
  chassis.pid_odom_boomerang_constants_set(1.0, 0.01, 11.5);  // Angular control for boomerang motions1 .01 11.5 

  // Exit conditions
  chassis.pid_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);
  chassis.pid_odom_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 750_ms);
  chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 750_ms);
  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  // Slew constants
  chassis.slew_turn_constants_set(3_deg, 70);
  chassis.slew_drive_constants_set(3_in, 70);
  chassis.slew_swing_constants_set(3_in, 80);

  // The amount that turns are prioritized over driving in odom motions
  // - if you have tracking wheels, you can run this higher.  1.0 is the max
  chassis.odom_turn_bias_set(0.97); //used to be 0.9

  chassis.odom_look_ahead_set(7_in);           // This is how far ahead in the path the robot looks at
  chassis.odom_boomerang_distance_set(16_in);  // This sets the maximum distance away from target that the carrot point can be
  chassis.odom_boomerang_dlead_set(0.625);     // This handles how aggressive the end of boomerang motions are

  chassis.pid_angle_behavior_set(ez::shortest);  // Defaults turning behavior to shortest path
}

void red_right_7_auto() {
  //set starting angle
  chassis.drive_angle_set(-90_deg);

  chassis.odom_xyt_set(95_in, 14.5_in, 270_deg);

  //Drive backwards towards match loader
  chassis.pid_drive_set(-30.75_in, DRIVE_SPEED, true);//was -30.2 //was -30.75
  chassis.pid_wait();
  pros::delay(50);

  //Turn to face match loader
  chassis.pid_turn_set(-6.5_deg, 90); //was 60 //was-6 //was-6.5
  chassis.pid_wait();
  pros::delay(50);

  //Set starting angle to 0
  chassis.drive_angle_set(0_deg);

  // Start bottom intake and  reverse top intake and drop match loader
  intakeTop.move(-127);
  intakeBottom.move(127);
  MatchLoadDrop(true);

  // Move into match loader once
  chassis.pid_drive_set(-9.75_in, 60); //was 90
  chassis.pid_wait_quick_chain();
  pros::delay(75); 

  //Push into loader, and wait till blocks are in intake
  chassis.pid_drive_set(-.4_in, DRIVE_SPEED); //was -.525
  chassis.pid_wait();
  pros::delay(50); //was 950
  chassis.pid_drive_set(-.3_in, DRIVE_SPEED);  //was -.25
  chassis.pid_wait();
  pros::delay(800); //was 1000 //was 850

  //Move sraight back into long goal, start top intake lift match load mech
  chassis.pid_drive_set(29.25_in, 127);
  chassis.pid_wait();
  MatchLoadDrop(false);
  chassis.pid_drive_set(1.25_in, 127); //was .75
  chassis.pid_wait();
  intakeTop.move(127);
  pros::delay(1800); //was 1950 //was 1850

  //turn
  // chassis.pid_turn_set(50_deg, 90); //was 60 //was-6
  // chassis.pid_wait();

  chassis.pid_swing_behavior_set(ez::shortest);

  //swing into four blocks and reverse top intake
  chassis.pid_swing_set(ez::RIGHT_SWING, 137_deg, 127, -23); //was -11 //was-20 //was -23 //was 135_deg
  chassis.pid_wait();
  intakeTop.move(-127);

  //move into 4 blocks
  chassis.pid_drive_set(-4.5_in, 127); //was 4
  chassis.pid_wait();
  MatchLoadDrop(true);
  intakeTop.move(-127);
  chassis.pid_drive_set(-1.5_in, 127); //was -1.5
  chassis.pid_wait();

  //turn towards long goal
  chassis.pid_turn_set(115_deg, 90); //was 60 //was-6 //was-6.5
  chassis.pid_wait();

  //move towards long goal
  chassis.pid_drive_set(24_in, 127); //was 22
  chassis.pid_wait();

  //turn to face long goal
  chassis.pid_turn_set(3_deg, 90); //was 6 //was 5
  chassis.pid_wait();
  pros::delay(25);

  //move into long goal and start top intake
  chassis.pid_drive_set(12_in, 127); //was -1.5
  chassis.pid_wait();
  intakeTop.move(127);
}

void red_left_auto() {
  //set starting angle
  chassis.drive_angle_set(90_deg);

  //Drive backwards towards match loader
  chassis.pid_drive_set(-30.2_in, DRIVE_SPEED, true);//was -30.2 //was -30.5
  chassis.pid_wait();
  pros::delay(50);

  //Turn to face match loader
  chassis.pid_turn_set(7.5_deg, 90); //was 60 //was-6.5
  chassis.pid_wait();
  pros::delay(50);

  //Set starting angle to 0
  chassis.drive_angle_set(0_deg);

  // Start bottom intake and  reverse top intake and drop match loader
  intakeTop.move(-127);
  intakeBottom.move(127);
  MatchLoadDrop(true);

  // Move into match loader once
  chassis.pid_drive_set(-9.75_in, 60); //was 90
  chassis.pid_wait_quick_chain();
  pros::delay(75); 

  //Push into loader, and wait till blocks are in intake
  chassis.pid_drive_set(-.25_in, DRIVE_SPEED); //was -.525 //was -.45 //was -.3
  chassis.pid_wait();
  pros::delay(50); //was 950
  chassis.pid_drive_set(-.25_in, DRIVE_SPEED);  //was .25
  chassis.pid_wait();
  pros::delay(300); //was 350

  //Move sraight back into long goal, start top intake lift match load mech
  chassis.pid_drive_set(29.25_in, 127);
  chassis.pid_wait();
  MatchLoadDrop(false);
  chassis.pid_drive_set(1.25_in, 127); //was .75
  chassis.pid_wait();
  intakeTop.move(127);
  pros::delay(1400); //was 1150

  //turn
  // chassis.pid_turn_set(50_deg, 90); //was 60 //was-6
  // chassis.pid_wait();

  chassis.pid_swing_behavior_set(ez::shortest);

  //swing into four blocks and reverse top intake
  chassis.pid_swing_set(ez::LEFT_SWING, -137_deg, 127, -13); //was -11 //was -135
  chassis.pid_wait();
  

  //move forward into the balls and drop match loader
  chassis.pid_drive_set(-4.5_in, 127); //was 4
  chassis.pid_wait();
  MatchLoadDrop(true);
  intakeTop.move(-127);
  chassis.pid_drive_set(-1.5_in, 127); //was 4
  chassis.pid_wait();

  //do a 180
  chassis.pid_turn_set(-315_deg, 90); //was -317 //was-6
  chassis.pid_wait();

  //move froward to middle goal
  chassis.pid_drive_set(24.5_in, 127); //was 19
  chassis.pid_wait();

  //reverse top intake and lift match loader
  MatchLoadDrop(false);

  //move away from goal and come back. Start top intake and drop intake
  chassis.pid_drive_set(-5_in, 127); //was 4
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(5.5_in, 127); //was 4
  chassis.pid_wait_quick_chain();
  IntakeLiftDrop(true);
  intakeTop.move(127);
  
  //Lift middle intake


}

void skills_auto() {
  //set starting angle
  chassis.drive_angle_set(-90_deg);

  chassis.odom_xyt_set(95_in, 14.5_in, 270_deg);

  //Drive backwards towards match loader
  chassis.pid_drive_set(-30.75_in, DRIVE_SPEED, true);//was -30.2
  chassis.pid_wait();
  pros::delay(50);

  //Turn to face match loader
  chassis.pid_turn_set(-6.5_deg, 90); //was 60 //was-6
  chassis.pid_wait();
  pros::delay(50);

  //Set starting angle to 0
  chassis.drive_angle_set(0_deg);

  // Start bottom intake and  reverse top intake and drop match loader
  intakeTop.move(-127);
  intakeBottom.move(127);
  MatchLoadDrop(true);

  // Move into match loader once
  chassis.pid_drive_set(-9.75_in, 60); //was 90
  chassis.pid_wait_quick_chain();
  pros::delay(75); 

  //Push into loader, and wait till blocks are in intake
  chassis.pid_drive_set(-.525_in, DRIVE_SPEED); //was -1
  chassis.pid_wait();
  pros::delay(50); //was 950
  chassis.pid_drive_set(-.25_in, DRIVE_SPEED);  //was .25
  chassis.pid_wait();
  pros::delay(1000); //was 0

  //Move sraight back into long goal, start top intake lift match load mech
  chassis.pid_drive_set(29.25_in, 127);
  chassis.pid_wait();
  MatchLoadDrop(false);
  chassis.pid_drive_set(1.25_in, 127); //was .75
  chassis.pid_wait();
  intakeTop.move(127);
  pros::delay(1950); //was 1850

  //turn
  // chassis.pid_turn_set(50_deg, 90); //was 60 //was-6
  // chassis.pid_wait();

  chassis.pid_swing_behavior_set(ez::shortest);

  //swing into four blocks and reverse top intake
  chassis.pid_swing_set(ez::RIGHT_SWING, 135_deg, 127, -11); //was -11
  chassis.pid_wait();
  intakeTop.move(-127);

  //swing to 180 deg
  chassis.pid_swing_set(ez::RIGHT_SWING, 180_deg, 127, 5);
  chassis.pid_wait();

  //Move straight back towards other side
  // chassis.pid_drive_set(-29.25_in, 127);
  // chassis.pid_wait();
}

void red_right_solo() {
  //set starting angle
  chassis.drive_angle_set(-90_deg);


  //Drive backwards towards match loader
  chassis.pid_drive_set(-30.2_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  pros::delay(50);

  //Turn to face match loader
  chassis.pid_turn_set(-6.5_deg, 90); //was 60 //was-6
  chassis.pid_wait();
  pros::delay(50);

  //Set starting angle to 0
  chassis.drive_angle_set(0_deg);

  // Start bottom intake and  reverse top intake and drop match loader
  intakeTop.move(-127);
  intakeBottom.move(127);
  MatchLoadDrop(true);

  // Move into match loader once
  chassis.pid_drive_set(-9.75_in, 60); //was 90
  chassis.pid_wait_quick_chain();
  pros::delay(75); 

  //Push into loader, and wait till blocks are in intake
  chassis.pid_drive_set(-.525_in, DRIVE_SPEED); //was -1
  chassis.pid_wait();
  pros::delay(75); //was 75
  chassis.pid_drive_set(.25_in, DRIVE_SPEED);  //was .25
  chassis.pid_wait();
  pros::delay(0);

  //Move sraight back into long goal, start top intake lift match load mech
  chassis.pid_drive_set(29.25_in, 127);
  chassis.pid_wait();
  MatchLoadDrop(false);
  chassis.pid_drive_set(1.25_in, 127); //was .75
  chassis.pid_wait();
  intakeTop.move(127);
  pros::delay(1850);

  //Turn towards blocks
  chassis.pid_turn_set(99_deg, TURN_SPEED);
  chassis.pid_wait();

  //Move to balls and reverse top intake
  chassis.pid_drive_set(-7.5_in, 127); //was-7
  chassis.pid_wait();
  intakeTop.move(-127);

  //Move forward more to grab all blocks
  chassis.pid_drive_set(-30_in, 127); //was -24
  chassis.pid_wait();

  //Turn towards other 3 blocks
  chassis.pid_turn_set(75_deg, TURN_SPEED); //was 79
  chassis.pid_wait();

  //Move to 3 balls
  chassis.pid_drive_set(-15_in, 127); //was-15.5
  chassis.pid_wait();

  //move forward again
  chassis.pid_drive_set(-8_in, 127); //was -7
  chassis.pid_wait();

  //turn towards middle goal
  chassis.pid_turn_set(40_deg, TURN_SPEED); //was 40
  chassis.pid_wait();

  //move to middle goal and deposit 2 balls
  chassis.pid_drive_set(17_in, 127); //was 16
  chassis.pid_wait();
  IntakeLiftDrop(true);
  intakeTop.move(127);
  pros::delay(875);
  intakeTop.move(-127);
  IntakeLiftDrop(false);

  //turn away from center goal
  chassis.pid_turn_set(52_deg, TURN_SPEED);
  chassis.pid_wait();

  //move away from middle goal and turn towards long goal
  chassis.pid_drive_set(-47_in, 127); //was 46
  chassis.pid_wait();
  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  //move forward to long goal and start top intake
  chassis.pid_drive_set(10_in, 127);
  chassis.pid_wait();
  intakeTop.move(127);



  // chassis.pid_swing_set(ez::LEFT_SWING, 90_deg, SWING_SPEED, 45);
  // chassis.pid_wait();

  // //Pull away from long goal and turn towards three blocks. Start bottom intake
  // chassis.pid_drive_set( -14_in, DRIVE_SPEED, true);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_turn_set(133_deg, TURN_SPEED); //was 132
  // chassis.pid_wait();
  // intakeBottom.move(127);

  // //Move towards three balls. When there, drop match load mech
  // chassis.pid_drive_set( -29_in, DRIVE_SPEED, true); //was -29.5
  // chassis.pid_wait();
  // MatchLoadDrop(true);
  // chassis.pid_drive_set( -6_in, DRIVE_SPEED, true);
  // chassis.pid_wait();

  // //Move forward 3 inches. Turn toward left three balls and lift match load mech
  // chassis.pid_turn_set(90_deg, TURN_SPEED);
  // chassis.pid_wait();
  // MatchLoadDrop(false);

  // //Move to other three balls and drop match load mech
  // chassis.pid_drive_set( -37_in, DRIVE_SPEED, true); //was 38
  // chassis.pid_wait();
  // chassis.pid_drive_set( -3.5_in, DRIVE_SPEED, true);
  // chassis.pid_wait();
  // MatchLoadDrop(true);

}


void red_right() {
  //set starting angle
  chassis.drive_angle_set(-90_deg);


  //Drive backwards towards match loader
  chassis.pid_drive_set(-30.2_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  pros::delay(50);

  //Drop match loader mech
  MatchLoadDrop(true);

  //Turn to face match loader
  chassis.pid_turn_set(-6.5_deg, 90); //was 60 //was-6
  chassis.pid_wait();

  //Set starting angle to 0
  chassis.drive_angle_set(0_deg);

  // Start bottom intake and  reverse top intake
  intakeTop.move(-127);
  intakeBottom.move(127);

  // Move into match loader once
  chassis.pid_drive_set(-9.75_in, 60); //was 90
  chassis.pid_wait_quick_chain();
  pros::delay(450);

  //Push into loader, and wait till blocks are in intake
  chassis.pid_drive_set(-1_in, DRIVE_SPEED);
  chassis.pid_wait();
  pros::delay(450);
  chassis.pid_drive_set(.25_in, DRIVE_SPEED);  //was 1
  chassis.pid_wait();
  pros::delay(50);

  //Move sraight back into long goal, start top intake lift match load mech
  chassis.pid_drive_set(29.25_in, 127);
  chassis.pid_wait();
  MatchLoadDrop(false);
  chassis.pid_drive_set(0.75_in, 127);
  chassis.pid_wait();
  intakeTop.move(127);
  pros::delay(1850);

  //Reverse intake. Pull back and push into long goal again
  intakeTop.move(-127);
  intakeBottom.move(-127);
  chassis.pid_drive_set(-5_in, 127);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(6.25_in, 127);
  chassis.pid_wait();

  //Pull away from long goal and turn towards three blocks. Start bottom intake
  chassis.pid_drive_set( -14_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(133_deg, TURN_SPEED); //was 132
  chassis.pid_wait();
  intakeBottom.move(127);

  //Move towards three balls. When there, drop match load mech
  chassis.pid_drive_set( -29.5_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  MatchLoadDrop(true);

  //move forward towards lower center goal, delay, lift match loader, move forward into goal, delay, and outake
  chassis.pid_drive_set( -15_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  pros::delay(200);
  MatchLoadDrop(false);
  chassis.pid_drive_set( -7_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  pros::delay(200);
  chassis.pid_drive_set( 1.5_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  intakeBottom.move(-127);
}

///
// Drive Example
///
void drive_example() {
  // The first parameter is target inches
  // The second parameter is max speed the robot will drive at
  // The third parameter is a boolean (true or false) for enabling/disabling a slew at the start of drive motions
  // for slew, only enable it when the drive distance is greater than the slew distance + a few inches

  // TopIntakeMove(IntakeSpeed);

  chassis.pid_drive_set(15_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(15_in, DRIVE_SPEED);
  chassis.pid_wait();


  pros::delay(1000);

  chassis.pid_drive_set(-15_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-15_in, DRIVE_SPEED);
  chassis.pid_wait();
}

///
// Turn Example
///
void turn_example() {
  // The first parameter is the target in degrees
  // The second parameter is max speed the robot will drive at

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();
}

///
// Combining Turn + Drive
///
void drive_and_turn() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Wait Until and Changing Max Speed
///
void wait_until_change_speed() {
  // pid_wait_until will wait until the robot gets to a desired position

  // When the robot gets to 6 inches slowly, the robot will travel the remaining distance at full speed
  chassis.pid_drive_set(24_in, 30, true);
  chassis.pid_wait_until(6_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // When the robot gets to -6 inches slowly, the robot will travel the remaining distance at full speed
  chassis.pid_drive_set(-24_in, 30, true);
  chassis.pid_wait_until(-6_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();
}

///
// Swing Example
///
void swing_example() {
  // The first parameter is ez::LEFT_SWING or ez::RIGHT_SWING
  // The second parameter is the target in degrees
  // The third parameter is the speed of the moving side of the drive
  // The fourth parameter is the speed of the still side of the drive, this allows for wider arcs

  chassis.pid_swing_set(ez::LEFT_SWING, 90_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 90_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::LEFT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();
}

///
// Motion Chaining
///
void motion_chaining() {
  // Motion chaining is where motions all try to blend together instead of individual movements.
  // This works by exiting while the robot is still moving a little bit.
  // To use this, replace pid_wait with pid_wait_quick_chain.
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // Your final motion should still be a normal pid_wait
  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Auto that tests everything
///
void combining_movements() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, -45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Interference example
///
void tug(int attempts) {
  for (int i = 0; i < attempts - 1; i++) {
    // Attempt to drive backward
    printf("i - %i", i);
    chassis.pid_drive_set(-12_in, 127);
    chassis.pid_wait();

    // If failsafed...
    if (chassis.interfered) {
      chassis.drive_sensor_reset();
      chassis.pid_drive_set(-2_in, 20);
      pros::delay(1000);
    }
    // If the robot successfully drove back, return
    else {
      return;
    }
  }
}

// If there is no interference, the robot will drive forward and turn 90 degrees.
// If interfered, the robot will drive forward and then attempt to drive backward.
void interfered_example() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  if (chassis.interfered) {
    tug(3);
    return;
  }

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
}

///
// Odom Drive PID
///
void odom_drive_example() {
  // This works the same as pid_drive_set, but it uses odom instead!
  // You can replace pid_drive_set with pid_odom_set and your robot will
  // have better error correction.

  chassis.pid_odom_set({{0_in, 24_in}, fwd, DRIVE_SPEED}, true);
  chassis.pid_wait();
  
  // chassis.pid_odom_set({{0_in, 0_in, 90_deg}, fwd, 110});
  // chassis.pid_wait();

  // chassis.pid_odom_set({{0_in, 0_in, 0_deg}, fwd, 110});
  // chassis.pid_wait();

  // chassis.pid_odom_set(24_in, DRIVE_SPEED, true);
  // chassis.pid_wait();

  // chassis.pid_odom_set(-12_in, DRIVE_SPEED);
  // chassis.pid_wait();

  // chassis.pid_odom_set(-12_in, DRIVE_SPEED);
  // chassis.pid_wait();
}

///
// Odom Pure Pursuit
///
void odom_pure_pursuit_example() {
  /*
  
  chassis.pid_odom_set({{0_in, 24_in}, fwd, DRIVE_SPEED});
  chassis.pid_wait();

  chassis.pid_odom_set({{24_in, 24_in}, fwd, DRIVE_SPEED});
  chassis.pid_wait();
  */

  // Drive to 0, 30 and pass through 6, 10 and 0, 20 on the way, with slew
  chassis.pid_odom_set({{{0_in, 24_in}, fwd, DRIVE_SPEED},
                        {{24_in, 48_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait();

  chassis.pid_odom_set({{{0_in, 24_in}, rev, DRIVE_SPEED},
                        {{0_in, 0_in}, rev, DRIVE_SPEED}},
                       true);
  chassis.pid_wait();
}

///
// Odom Pure Pursuit Wait Until
///
void odom_pure_pursuit_wait_until_example() {
  chassis.pid_odom_set({{{0_in, 24_in}, fwd, DRIVE_SPEED},
                        {{12_in, 24_in}, fwd, DRIVE_SPEED},
                        {{24_in, 24_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait_until_index(1);  // Waits until the robot passes 12, 24
  // Intake.move(127);  // Set your intake to start moving once it passes through the second point in the index
  chassis.pid_wait();
  // Intake.move(0);  // Turn the intake off
}

///
// Odom Boomerang
///
void odom_boomerang_example() {
  chassis.pid_odom_set({{0_in, 24_in, 45_deg}, fwd, DRIVE_SPEED},
                       true);
  chassis.pid_wait();

  chassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Odom Boomerang Injected Pure Pursuit
///
void odom_boomerang_injected_pure_pursuit_example() {
  chassis.pid_odom_set({{{0_in, 24_in, 45_deg}, fwd, DRIVE_SPEED},
                        {{12_in, 24_in}, fwd, DRIVE_SPEED},
                        {{24_in, 24_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait();

  chassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Calculate the offsets of your tracking wheels
///
void measure_offsets() {
  // Number of times to test
  int iterations = 10;

  // Our final offsets
  double l_offset = 0.0, r_offset = 0.0, b_offset = 0.0, f_offset = 0.0;

  // Reset all trackers if they exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->reset();
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->reset();
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->reset();
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->reset();
  
  for (int i = 0; i < iterations; i++) {
    // Reset pid targets and get ready for running an auton
    chassis.pid_targets_reset();
    chassis.drive_imu_reset();
    chassis.drive_sensor_reset();
    chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
    chassis.odom_xyt_set(0_in, 0_in, 0_deg);
    double imu_start = chassis.odom_theta_get();
    double target = i % 2 == 0 ? 90 : 270;  // Switch the turn target every run from 270 to 90

    // Turn to target at half power
    chassis.pid_turn_set(target, 63, ez::raw);
    chassis.pid_wait();
    pros::delay(250);

    // Calculate delta in angle
    double t_delta = util::to_rad(fabs(util::wrap_angle(chassis.odom_theta_get() - imu_start)));

    // Calculate delta in sensor values that exist
    double l_delta = chassis.odom_tracker_left != nullptr ? chassis.odom_tracker_left->get() : 0.0;
    double r_delta = chassis.odom_tracker_right != nullptr ? chassis.odom_tracker_right->get() : 0.0;
    double b_delta = chassis.odom_tracker_back != nullptr ? chassis.odom_tracker_back->get() : 0.0;
    double f_delta = chassis.odom_tracker_front != nullptr ? chassis.odom_tracker_front->get() : 0.0;

    // Calculate the radius that the robot traveled
    l_offset += l_delta / t_delta;
    r_offset += r_delta / t_delta;
    b_offset += b_delta / t_delta;
    f_offset += f_delta / t_delta;
  }

  // Average all offsets
  l_offset /= iterations;
  r_offset /= iterations;
  b_offset /= iterations;
  f_offset /= iterations;

  // Set new offsets to trackers that exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->distance_to_center_set(l_offset);
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->distance_to_center_set(r_offset);
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->distance_to_center_set(b_offset);
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->distance_to_center_set(f_offset);
}

// . . .
// Make your own autonomous functions here!
// . . .

// r: 2.73 2.74 2.72 2.77 2.71 - 2.73
// l: 3.38 3.37 3.41 3.40 3.34 - 3.38