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
  chassis.set_drive_constants(10, 1, 0, 11.5, 0);
  chassis.set_heading_constants(6, .45, 0, 0.9, 0);
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


void drive_test(){
  default_constants();
  chassis.right_swing_to_angle(90);
}

/**
 * The expected behavior is to return to the start angle, after making a complete turn.
 */


/**
 * Should swing in a fun S shape.
 */


/**
 * A little of this, a little of that; it should end roughly where it started.
 */

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
void first_skills(){
  Wings.set(true);
  //grab ball
  intake.spin(fwd, 100, percentUnits::pct);
  topGoal.spin(reverse, 100, percentUnits::pct);
  chassis.right_swing_to_angle(-35, 12, 1, 300, 700, .3, .003, 2.1, 15);
  chassis.drive_distance(19.5, -35, 8, 8, 1, 300, 800);
  
  //wait(400, msec);
  intake.stop();
  
  //score top mid
  chassis.turn_to_angle(-135, 12, 1, 300, 700);
  intake.spin(reverse, 10, percentUnits::pct);
  topGoal.spin(reverse, 30, percentUnits::pct);
  matchLoad.set(true);
  chassis.drive_distance(-17, -135, 10, 10, 1, 300, 700);
  chassis.DriveL.spin(reverse, 1, voltageUnits::volt);
  chassis.DriveR.spin(reverse, 1, voltageUnits::volt);
   
  DigitalOutC.set(true);
  topGoal.stop();
  intake.spin(fwd, 80, percentUnits::pct);
  wait(600, msec);
  intake.stop();
  chassis.DriveL.stop(hold);
  chassis.DriveR.stop(hold);

  //grab matchloads1
  chassis.drive_distance(41, -135, 10, 10, 1, 300, 1200);
  DigitalOutC.set(false);
  chassis.turn_to_angle(180, 12, 1, 300, 800);
  intake.spin(fwd, 100, percentUnits::pct);
  chassis.drive_distance(15, 180, 8, 8, 1, 300, 900);
  wait(1400, msec);
  //score top
  chassis.drive_distance(-16, 180, 10, 10, 1, 300, 700);
// first half
  topGoal.stop();
  chassis.turn_to_angle(-225, 12, 1, 300, 800);
  chassis.drive_distance(-16, -225, 10, 10, 1, 300, 700);
  chassis.turn_to_angle(180, 12, 1, 300, 800);
  matchLoad.set(false);
  chassis.drive_distance(-46, 180, 10, 10, 1, 300, 1200);
  chassis.turn_to_angle(225, 12, 1, 300, 800);
  chassis.drive_distance(-14, 225, 10, 10, 1, 300, 700);
  intake.stop();
  chassis.turn_to_angle(360, 12, 1, 300, 800);
  chassis.drive_distance(-13, 360, 10, 10, 1, 300, 700);
  matchLoad.set(true);
  topGoal.spin(fwd, 100, percentUnits::pct);
  intake.spin(fwd, 100, percentUnits::pct);
  wait(1900, msec);
  topGoal.stop();
}
void second_skills(){
  Wings.set(true);
  matchLoad.set(true);
  topGoal.stop(hold);
  intake.spin(fwd, 100, percentUnits::pct);
  chassis.drive_distance(25.5, 0, 6, 6, 1, 300, 1500);
  wait(1400, msec);
  chassis.drive_distance(-25.5, 0, 9, 9, 1, 300, 800);
  topGoal.spin(fwd, 100, percentUnits::pct);
  wait(1700, msec);
  matchLoad.set(false);
  topGoal.stop();
  intake.spin(fwd, 100, percentUnits::pct);
  chassis.drive_distance(16, 0, 10, 10, 1, 300, 800);
  chassis.left_swing_to_angle(65, 12, 1, 300, 800, .3, .003, 2.1, 15);
  chassis.drive_distance(20, 65, 8, 8, 1, 300, 800);
  chassis.turn_to_angle(80, 6, 1, 300, 600);

  chassis.drive_distance(18, 80, 10, 10, 1, 300, 900);
  matchLoad.set(true);
  chassis.drive_distance(35, 80, 8, 8, 1, 300, 1500);
  
  intake.stop();
  topGoal.stop();
  chassis.turn_to_angle(90, 6, 1, 300, 600);
  chassis.drive_distance(-16, 90, 3, 3, 1, 300, 1500);
  chassis.left_swing_to_angle(180, 12, 1, 300, 800, .3, .003, 2.1, 15);
  matchLoad.set(true);
  chassis.drive_distance(26, 180, 7, 7, 1, 300, 1000);
  
  intake.spin(reverse, 10, percentUnits::pct);
  topGoal.spin(reverse, 30, percentUnits::pct);
  chassis.turn_to_angle(45, 12, 1, 300, 800);
  chassis.drive_distance(-18.7, 45, 10, 10, 1, 300, 700);
  DigitalOutC.set(true);
  intake.spin(fwd, 100, percentUnits::pct);
  topGoal.spin(fwd, 100, percentUnits::pct);
  wait(3000, msec);
  DigitalOutC.set(false);
}
void third_skills(){

}
void fourth_skills(){

}
void skills(){
  Wings.set(true);
  //grab ball
  intake.spin(fwd, 100, percentUnits::pct);
  topGoal.spin(reverse, 100, percentUnits::pct);
  chassis.right_swing_to_angle(-35, 12, 1, 300, 700, .3, .003, 2.1, 15);
  chassis.drive_distance(19.5, -35, 8, 8, 1, 300, 800);
  
  //wait(400, msec);
  intake.stop();
  
  //score top mid
  chassis.turn_to_angle(-135, 12, 1, 300, 700);
  intake.spin(reverse, 10, percentUnits::pct);
  topGoal.spin(reverse, 30, percentUnits::pct);
  matchLoad.set(true);
  chassis.drive_distance(-17, -135, 10, 10, 1, 300, 700);
   
  DigitalOutC.set(true);
  topGoal.stop();
  intake.spin(fwd, 90, percentUnits::pct);
  wait(1000, msec);
  DigitalOutC.set(false);
 
  //grab matchloads1
  chassis.drive_distance(42, -135, 10, 10, 1, 300, 1200);
  chassis.turn_to_angle(180, 12, 1, 300, 1000);
  chassis.drive_distance(12.5, 180, 6, 10, 1, 300, 700);
  wait(1500, msec);
  //score top
  chassis.drive_distance(-17, 180, 10, 10, 1, 300, 700);
// first half
  topGoal.stop();
  chassis.turn_to_angle(-225, 12, 1, 300, 800);
  chassis.drive_distance(-16, -225, 10, 10, 1, 300, 700);
  chassis.turn_to_angle(180, 12, 1, 300, 800);
  matchLoad.set(false);
  chassis.drive_distance(-45, 180, 10, 10, 1, 300, 1200);
  chassis.turn_to_angle(225, 12, 1, 300, 800);
  chassis.drive_distance(-13, 225, 10, 10, 1, 300, 700);
  intake.stop();
  chassis.turn_to_angle(360, 12, 1, 300, 800);
  chassis.drive_distance(-13, 360, 10, 10, 1, 300, 700);
  matchLoad.set(true);
  topGoal.spin(fwd, 100, percentUnits::pct);
  intake.spin(fwd, 100, percentUnits::pct);
  wait(1900, msec);
  topGoal.stop();
  
  chassis.drive_distance(26, 2, 6, 6, 1, 300, 1500);
  wait(1500, msec);
  chassis.drive_distance(-25.5, 2, 9, 9, 1, 300, 800);
  topGoal.spin(fwd, 100, percentUnits::pct);
  wait(1900, msec);
  matchLoad.set(false);
  topGoal.stop();
  intake.spin(fwd, 100, percentUnits::pct);
  chassis.drive_distance(16, 2, 10, 10, 1, 300, 700);
  chassis.left_swing_to_angle(65, 12, 1, 300, 800, .3, .003, 2.1, 15);
  chassis.drive_distance(20, 65, 8, 8, 1, 300, 800);
  chassis.turn_to_angle(80, 13, 1, 300, 500);

  chassis.drive_distance(18, 80, 10, 10, 1, 300, 700);
  chassis.drive_distance(35, 80, 8, 8, 1, 300, 1700);
  
  intake.stop();
  topGoal.stop();
  chassis.turn_to_angle(90, 12, 1, 300, 800);
  chassis.drive_distance(-16, 90, 3, 3, 1, 300, 1500);
  chassis.left_swing_to_angle(180, 12, 1, 300, 800, .3, .003, 2.1, 15);
  matchLoad.set(true);
  chassis.drive_distance(26, 180, 7, 7, 1, 300, 1000);
  
  intake.spin(reverse, 10, percentUnits::pct);
  topGoal.spin(reverse, 30, percentUnits::pct);
  chassis.turn_to_angle(45, 12, 1, 300, 800);
  chassis.drive_distance(-18.7, 45, 10, 10, 1, 300, 700);
  DigitalOutC.set(true);
  intake.spin(fwd, 100, percentUnits::pct);
  topGoal.spin(fwd, 100, percentUnits::pct);
  wait(3000, msec);
  DigitalOutC.set(false);
  chassis.drive_distance(42, 45, 10, 10, 1, 300, 1500);
  topGoal.stop();
  chassis.turn_to_angle(0, 12, 1, 300, 1000);
  chassis.drive_distance(13, 0, 6, 6, 1, 300, 700);
  wait(1500, msec);
  //score top
  chassis.drive_distance(-16, 0, 10, 10, 1, 300, 700);
  topGoal.stop();
  chassis.turn_to_angle(-45, 12, 1, 300, 800);
  chassis.drive_distance(-16, -45, 10, 10, 1, 300, 700);
  chassis.turn_to_angle(0, 12, 1, 300, 800);
  matchLoad.set(true);
  chassis.drive_distance(-35, 0, 10, 10, 1, 300, 1200);
  chassis.turn_to_angle(35, 12, 1, 300, 800);
  chassis.drive_distance(-12, 45, 10, 10, 1, 300, 700);
  intake.stop();
  chassis.turn_to_angle(180, 12, 1, 300, 800);
  chassis.drive_distance(-13, 180, 10, 10, 1, 300, 700);
  topGoal.spin(fwd, 100, percentUnits::pct);
  intake.spin(fwd, 100, percentUnits::pct);
  wait(1500, msec);
  topGoal.stop();
  matchLoad.set(true);
  chassis.drive_distance(27, 182, 6, 6, 1, 300, 1500);
  wait(1500, msec);
  chassis.drive_distance(-26, 182, 9, 9, 1, 300, 800);
  topGoal.spin(fwd, 100, percentUnits::pct);
  wait(1500, msec);
  matchLoad.set(false);
  topGoal.stop();
  intake.spin(reverse, 100, percentUnits::pct);
  chassis.drive_distance(15, 180, 10, 10, 1, 300, 700);
  chassis.left_swing_to_angle(245, 12, 1, 300, 800, .3, .003, 2.1, 15);
  chassis.drive_distance(26, 245, 8, 8, 1, 300, 800);
  chassis.turn_to_angle(260, 13, 1, 300, 500);
  chassis.drive_distance(18, 260, 12, 12, 1, 300, 700);
  chassis.drive_distance(12, 260, 12, 12, 1, 300, 1700);


}
void forwardDrive(){
  chassis.drive_distance(-3);
}

void left_side(){
  Wings.set(true);
  matchLoad.set(true);
  intake.spin(fwd, 100, percentUnits::pct);
  chassis.drive_distance(30.5, 0, 10, 10, 1, 300, 900);
  chassis.turn_to_angle(-90, 12, 1, 300, 1000);
  chassis.drive_distance(12, -90, 3, 3, 1, 300, 1500);
  wait(180, msec);
  chassis.drive_distance(-27, -90, 10, 10, 1, 300, 1000);
  topGoal.spin(fwd, 100, percentUnits::pct);
  wait(1000, msec);
  matchLoad.set(false);
  topGoal.stop();
  
  chassis.drive_distance(19, -90, 10, 10, 1, 300, 900);
  chassis.turn_to_angle(135, 12, 0.5, 300, 1000);
  chassis.drive_distance(35, 135, 6, 6, 0.5, 300, 1500);
  chassis.turn_to_angle(315, 12, 1, 300, 1000);
  intake.spin(reverse, 5, percentUnits::pct);
  topGoal.spin(reverse, 100, percentUnits::pct);
  chassis.drive_distance(-14.5, 315, 10, 10, 1, 300, 1000);
  topGoal.stop();
  DigitalOutC.set(true);
  intake.spin(fwd, 90, percentUnits::pct);
  wait(1000, msec);
  Wings.set(false);
  chassis.drive_distance(33, 315, 10, 10, 1, 300, 1000);
  chassis.turn_to_angle(90, 12, 1, 300, 1000);
  chassis.drive_distance(24, 90, 6, 6, 0.5, 300, 1200);
  chassis.turn_to_angle(145, 8, 1, 300, 1000);

}

void skillsBarrierCross(){
  Wings.set(true);
  intake.spin(fwd, 100, percentUnits::pct);
   chassis.drive_distance(16, 2, 10, 10, 1, 300, 700);
  chassis.left_swing_to_angle(65, 12, 1, 300, 800, .3, .003, 2.1, 15);
  chassis.drive_distance(20, 65, 8, 8, 1, 300, 800);
  chassis.turn_to_angle(80, 13, 1, 300, 500);

  chassis.drive_distance(18, 80, 10, 10, 1, 300, 700);
  chassis.drive_distance(35, 80, 8, 8, 1, 300, 1700);
  
  intake.stop();
  topGoal.stop();
  chassis.turn_to_angle(90, 12, 1, 300, 800);
  chassis.drive_distance(-16, 90, 3, 3, 1, 300, 1500);
  chassis.left_swing_to_angle(180, 12, 1, 300, 800, .3, .003, 2.1, 15);
  chassis.drive_distance(26, 180, 7, 7, 1, 300, 1000);
  
  intake.spin(reverse, 10, percentUnits::pct);
  topGoal.spin(reverse, 15, percentUnits::pct);
  chassis.turn_to_angle(45, 12, 1, 300, 800);
  chassis.drive_distance(-18.7, 45, 10, 10, 1, 300, 700);
  DigitalOutC.set(true);
  intake.spin(fwd, 85, percentUnits::pct);
  topGoal.spin(fwd, 100, percentUnits::pct);
  wait(3000, msec);
  DigitalOutC.set(false);
  chassis.drive_distance(42, 45, 10, 10, 1, 300, 1500);
  topGoal.stop();
  chassis.turn_to_angle(0, 12, 1, 300, 1000);
  chassis.drive_distance(13, 0, 6, 6, 1, 300, 700);
  wait(1500, msec);
  //score top
  chassis.drive_distance(-16, 0, 10, 10, 1, 300, 700);
  topGoal.stop();
  chassis.turn_to_angle(-45, 12, 1, 300, 800);
  chassis.drive_distance(-16, -45, 10, 10, 1, 300, 700);
  chassis.turn_to_angle(0, 12, 1, 300, 800);
  matchLoad.set(true);
  chassis.drive_distance(-45, 0, 10, 10, 1, 300, 1200);
  chassis.turn_to_angle(45, 12, 1, 300, 800);
  chassis.drive_distance(-13, 45, 10, 10, 1, 300, 700);
  intake.stop();
  chassis.turn_to_angle(180, 12, 1, 300, 800);
  chassis.drive_distance(-13, 180, 10, 10, 1, 300, 700);
  topGoal.spin(fwd, 100, percentUnits::pct);
  intake.spin(fwd, 100, percentUnits::pct);
  wait(1500, msec);
  topGoal.stop();
  matchLoad.set(true);
  chassis.drive_distance(27, 182, 6, 6, 1, 300, 1500);
  wait(1500, msec);
  chassis.drive_distance(-26, 182, 9, 9, 1, 300, 800);
  topGoal.spin(fwd, 100, percentUnits::pct);
  wait(1500, msec);
  matchLoad.set(false);
  topGoal.stop();
  intake.spin(reverse, 100, percentUnits::pct);
  chassis.drive_distance(15, 180, 10, 10, 1, 300, 700);
  chassis.left_swing_to_angle(245, 12, 1, 300, 800, .3, .003, 2.1, 15);
  chassis.drive_distance(26, 245, 8, 8, 1, 300, 800);
  chassis.turn_to_angle(260, 13, 1, 300, 500);
  chassis.drive_distance(18, 260, 12, 12, 1, 300, 700);
  chassis.drive_distance(12, 260, 12, 12, 1, 300, 1700);

}

void right_side(){
  Wings.set(true);
  matchLoad.set(true);
  intake.spin(fwd, 100, percentUnits::pct);
  chassis.drive_distance(30.5, 0, 10, 10, 1, 300, 900);
  chassis.turn_to_angle(90, 12, 1, 300, 1000);
  chassis.drive_distance(12.5, 90, 6, 6, 1, 300, 800);
  wait(250, msec);
  chassis.drive_distance(-26.5, 90, 10, 10, 1, 300, 800);
  topGoal.spin(fwd, 100, percentUnits::pct);
  wait(1200, msec);
  chassis.turn_to_angle(135, 12, 1, 300, 700);
  chassis.drive_distance(8, 135, 10, 10, 1, 300, 700);
  chassis.turn_to_angle(90, 12, 1, 300, 700);
  Wings.set(false);
  chassis.drive_distance(-24, 90, 6, 6, 1, 300, 1000);
  
}