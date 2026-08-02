#include "vex.h"

/**
 * Resets the constants for auton movement.
 * Modify these to change the default behavior of functions like
 * drive_distance(). For explanations of the difference between
 * drive, heading, turning, and swinging, as well as the PID and
 * exit conditions, check the docs.
 */




 
void default_constants(){
  // Each constant set is in the form of (maxVoltage, kP, kI, kD, startI).
  chassis.set_drive_constants(10, 0.6, 0, 8, 0);
  chassis.set_heading_constants(6, .5, 0, 0.9, 0);
  chassis.set_turn_constants(12, .4, .03, 3, 15);
  chassis.set_swing_constants(12, .3, .003, 2.1, 15);

  // Each exit condition set is in the form of (settle_error, settle_time, timeout).
  chassis.set_drive_exit_conditions(0.5, 300, 5000);
  chassis.set_turn_exit_conditions(0.5, 300, 3000);
  chassis.set_swing_exit_conditions(1, 300, 3000);
}

/**
 * Sets constants to be more effective for odom movements.
 * For functions like drive_to_point(), it's often better to have
 * a slower max_voltage and greater settle_error than you would otherwise.
 */

void odom_constants(){
  default_constants();
  chassis.heading_max_voltage = 10;
  chassis.drive_max_voltage = 8;
  chassis.drive_settle_error = 3;
  chassis.boomerang_lead = .5;
  chassis.drive_min_voltage = 0;
}

/**
 * The expected behavior is to return to the start position.
 */




/**
 * The expected behavior is to return to the start angle, after making a complete turn.
 */


/**
 * Should swing in a fun S shape.
 */


/**
 * A little of this, a little of that; it should end roughly where it started.
 */

void tracker_test() {
  double startingPosition =
      chassis.get_ForwardTracker_position();

  while (true) {
    double currentPosition =
        chassis.get_ForwardTracker_position();

    Brain.Screen.clearScreen();

    Brain.Screen.printAt(
        5, 20,
        "Start: %.2f",
        startingPosition);

    Brain.Screen.printAt(
        5, 40,
        "Current: %.2f",
        currentPosition);

    Brain.Screen.printAt(
        5, 60,
        "Change: %.2f",
        currentPosition - startingPosition);

    wait(100, msec);
  }
}

void PID_test() {
  chassis.drive_max_voltage = 10;

  chassis.drive_distance(36);

}

void full_test(){
  chassis.drive_distance(24);
  chassis.turn_to_angle(-45);
  chassis.drive_distance(-36);
  chassis.right_swing_to_angle(-90);
  chassis.drive_distance(24);
  chassis.turn_to_angle(0);
}

/**
 * Doesn't drive the robot, but just prints coordinates to the Brain screen 
 * so you can check if they are accurate to life. Push the robot around and
 * see if the coordinates increase like you'd expect.
 */

void odom_test(){
  chassis.set_coordinates(0, 0, 0);
  while(1){
    Brain.Screen.clearScreen();
    Brain.Screen.printAt(5,20, "X: %f", chassis.get_X_position());
    Brain.Screen.printAt(5,40, "Y: %f", chassis.get_Y_position());
    Brain.Screen.printAt(5,60, "Heading: %f", chassis.get_absolute_heading());
    Brain.Screen.printAt(5,80, "ForwardTracker: %f", chassis.get_ForwardTracker_position());
    Brain.Screen.printAt(5,100, "SidewaysTracker: %f", chassis.get_SidewaysTracker_position());
    task::sleep(20);
  }
}

/**
 * Should end in the same place it began, but the second movement
 * will be curved while the first is straight.
 */

void tank_odom_test(){
  odom_constants();
  chassis.set_coordinates(0, 0, 0);
  chassis.turn_to_point(24, 24);
  chassis.drive_to_point(24,24);
  chassis.drive_to_point(0,0);
  chassis.turn_to_angle(0);
}

/**
 * Drives in a square while making a full turn in the process. Should
 * end where it started.
 */

void holonomic_odom_test(){
  odom_constants();
  chassis.set_coordinates(0, 0, 0);
  chassis.holonomic_drive_to_pose(0, 18, 90);
  chassis.holonomic_drive_to_pose(18, 0, 180);
  chassis.holonomic_drive_to_pose(0, 18, 270);
  chassis.holonomic_drive_to_pose(0, 0, 0);
}

void first_skills(){}
 
void second_skills(){}
  
void third_skills(){
  
}
void fourth_skills(){

}
void skills(){
  
}
void forwardDrive(){
  chassis.drive_distance(-3);
}

void left_side(){
  

}

void skillsBarrierCross(){
  

}

void solo_awp(){
  
}

void right_side(){
 
  
}