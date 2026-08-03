#include "vex.h"
#include <cmath>      // fabs
#include <cstdlib>    // std::abs (sometimes)


using namespace vex;
using namespace cte;
competition Competition;

// These devices are defined in robot-config.cpp. Keeping the declarations here
// also allows this file to compile if robot-config.h has not yet been updated.
extern motor Cascade_1;
extern motor Cascade_2;
extern digital_out claw;
extern digital_out toggleL;
extern digital_out toggleR;

motor_group CascadeMotors(Cascade_1, Cascade_2);

/*---------------------------------------------------------------------------*/
/*                             VEXcode Config                                */
/*                                                                           */
/*  Before you do anything else, start by configuring your motors and        */
/*  sensors. In VEXcode Pro V5, you can do this using the graphical          */
/*  configurer port icon at the top right. In the VSCode extension, you'll   */
/*  need to go to robot-config.cpp and robot-config.h and create the         */
/*  motors yourself by following the style shown. All motors must be         */
/*  properly reversed, meaning the drive should drive forward when all       */
/*  motors spin forward.                                                     */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/*                             JAR-Template Config                           */
/*                                                                           */
/*  Where all the magic happens. Follow the instructions below to input      */
/*  all the physical constants and values for your robot. You should         */
/*  already have configured your motors.                                     */
/*---------------------------------------------------------------------------*/



Drive chassis(

//Pick your drive setup from the list below:
//ZERO_TRACKER_NO_ODOM
//ZERO_TRACKER_ODOM
//TANK_ONE_FORWARD_ENCODER
//TANK_ONE_FORWARD_ROTATION
//TANK_ONE_SIDEWAYS_ENCODER
//TANK_ONE_SIDEWAYS_ROTATION
//TANK_TWO_ENCODER
//TANK_TWO_ROTATION
//HOLONOMIC_TWO_ENCODERS
//HOLONOMIC_TWO_ROTATION
//
//Write it here:
TANK_ONE_FORWARD_ROTATION,

//Add the names of your Drive motors into the motor groups below, separated by commas, i.e. motor_group(Motor1,Motor2,Motor3).
//You will input whatever motor names you chose when you configured your robot using the sidebar configurer, they don't have to be "Motor1" and "Motor2".

//Left Motors:
motor_group(leftF, leftM, leftB),

//Right Motors:
motor_group(rightF, rightM, rightB),

//Specify the PORT NUMBER of your inertial sensor, in PORT format (i.e. "PORT1", not simply "1"):
PORT18,

//Input your wheel diameter. (4" omnis are actually closer to 4.125"):
3.25,

//External ratio, must be in decimal, in the format of input teeth/output teeth.
//If your motor has an 84-tooth gear and your wheel has a 60-tooth gear, this value will be 1.4.
//If the motor drives the wheel directly, this value is 1:
1.667,

//Gyro scale, this is what your gyro reads when you spin the robot 360 degrees.
//For most cases 360 will do fine here, but this scale factor can be very helpful when precision is necessary.
360,

/*---------------------------------------------------------------------------*/
/*                                  PAUSE!                                   */
/*                                                                           */
/*  The rest of the drive constructor is for robots using POSITION TRACKING. */
/*  If you are not using position tracking, leave the rest of the values as  */
/*  they are.                                                                */
/*---------------------------------------------------------------------------*/

//If you are using ZERO_TRACKER_ODOM, you ONLY need to adjust the FORWARD TRACKER CENTER DISTANCE.

//FOR HOLONOMIC DRIVES ONLY: Input your drive motors by position. This is only necessary for holonomic drives, otherwise this section can be left alone.
//LF:      //RF:    
PORT1,     -PORT2,

//LB:      //RB: 
PORT3,     -PORT4,

//If you are using position tracking, this is the Forward Tracker port (the tracker which runs parallel to the direction of the chassis).
//If this is a rotation sensor, enter it in "PORT1" format, inputting the port below.
//If this is an encoder, enter the port as an integer. Triport A will be a "1", Triport B will be a "2", etc.
PORT4,

//Input the Forward Tracker diameter (reverse it to make the direction switch):
-2.0,

//Input Forward Tracker center distance (a positive distance corresponds to a tracker on the right side of the robot, negative is left.)
//For a zero tracker tank drive with odom, put the positive distance from the center of the robot to the right side of the drive.
//This distance is in inches:
0,

//Input the Sideways Tracker Port, following the same steps as the Forward Tracker Port:
21,

//Sideways tracker diameter (reverse to make the direction switch):
-2.75,

//Sideways tracker center distance (positive distance is behind the center of the robot, negative is in front):
5.5

);

int current_auton_selection = 0;
volatile bool auto_started = false;

//Cascade Constants
constexpr double CASCADE_KP = 0.021;
constexpr double CASCADE_KI = 0.000;
constexpr double CASCADE_KD = 0.0000;
constexpr double CASCADE_MAX_PID_VOLTAGE = 12;
constexpr double CASCADE_MIN_POSITION_DEG = 0.0;
constexpr double CASCADE_MAX_POSITION_DEG = 7560.0;
constexpr double CASCADE_HOLD_VOLTAGE = 0.5;
constexpr double CASCADE_PID_DT_SECONDS = 0.010;
constexpr double CASCADE_I_ZONE_DEG = 100.0;
constexpr double CASCADE_INTEGRAL_LIMIT = 300.0;

class CascadePIDController {
 private:
  double integral = 0.0;
  double previousError = 0.0;
  double filteredDerivative = 0.0;

 public:
  void reset(double currentError) {
    integral = 0.0;
    previousError = currentError;
    filteredDerivative = 0.0;
  }

  double compute(double error) {
    if (std::fabs(error) <= CASCADE_I_ZONE_DEG) {
      integral += error * CASCADE_PID_DT_SECONDS;

      if (integral > CASCADE_INTEGRAL_LIMIT) {
        integral = CASCADE_INTEGRAL_LIMIT;
      }
      if (integral < -CASCADE_INTEGRAL_LIMIT) {
        integral = -CASCADE_INTEGRAL_LIMIT;
      }
    } 
    
    else {
      integral = 0.0;
    }

    double rawDerivative = (error - previousError) / CASCADE_PID_DT_SECONDS;

    filteredDerivative = 0.8 * filteredDerivative + 0.2 * rawDerivative;

    previousError = error;

    return CASCADE_KP * error + CASCADE_KI * integral + CASCADE_KD * filteredDerivative;
  }
};

CascadePIDController cascadePIDController;
volatile double cascadeTargetDeg = 0.0;
volatile bool cascadePIDEnabled = false;
volatile double cascadeLastErrorDeg = 0.0;
volatile double cascadeLastVoltage = 0.0;

// two bar constants
constexpr double TWOBAR_KP = 0.021;
constexpr double TWOBAR_KI = 0.000;
constexpr double TWOBAR_KD = 0.0000;

constexpr double TWOBAR_MAX_VOLTAGE = 5;
constexpr double TWOBAR_MIN_MOVE_VOLTAGE = 3.0;
constexpr double TWOBAR_SYNC_KP = 0.020;
constexpr double TWOBAR_SYNC_MAX_CORRECTION_VOLTAGE = 1.5;
constexpr double TWOBAR_PID_DT_SECONDS = 0.010;
constexpr double TWOBAR_I_ZONE_DEG = 100.0;
constexpr double TWOBAR_INTEGRAL_LIMIT = 300.0;
constexpr double TWOBAR_HOLD_ERROR_DEG = 20.0;
constexpr double TWOBAR_SYNC_HOLD_ERROR_DEG = 30.0;
constexpr double TWOBAR_MIN_VOLTAGE_ERROR_DEG = 40.0;
constexpr double TWOBAR_STALL_ERROR_DEG = 200.0;
constexpr double TWOBAR_STALL_VELOCITY_PCT = 1.0;
constexpr double TWOBAR_STALL_TIMEOUT_MSEC = 700.0;

constexpr double TWOBAR_FRONT_DEG = 0.0;
constexpr double TWOBAR_MOTOR_DEG_PER_ARM_DEG = 48.0 / 12.0;


//arm angles at each Cascade position
constexpr double R2_ARM_SCORE_ANGLE_DEG = -250;     
constexpr double DOWN_ARM_SCORE_ANGLE_DEG = -230;   
constexpr double RIGHT_ARM_SCORE_ANGLE_DEG = -210;  
constexpr double Y_ARM_SCORE_ANGLE_DEG = -185;      
constexpr double B_ARM_SCORE_ANGLE_DEG = -185;     

constexpr double armAngleToMotorDegrees(double armAngleDeg) {
  return armAngleDeg * TWOBAR_MOTOR_DEG_PER_ARM_DEG;
}

constexpr double TWOBAR_FRONT_TOLERANCE_DEG = 50.0;
constexpr double TWOBAR_SYNC_READY_TOLERANCE_DEG = 80.0;
constexpr double CASCADE_SEQUENCE_READY_TOLERANCE_DEG = 100.0;

class TwoBarPIDController {
 private:
  double integral = 0.0;
  double previousError = 0.0;
  double filteredDerivative = 0.0;

 public:
  void reset(double currentError) {
    integral = 0.0;
    previousError = currentError;
    filteredDerivative = 0.0;
  }

  double compute(double error) {
    if (std::fabs(error) <= TWOBAR_I_ZONE_DEG) {
      integral += error * TWOBAR_PID_DT_SECONDS;

      if (integral > TWOBAR_INTEGRAL_LIMIT) {
        integral = TWOBAR_INTEGRAL_LIMIT;
      }

      if (integral < -TWOBAR_INTEGRAL_LIMIT) {
        integral = -TWOBAR_INTEGRAL_LIMIT;
      }
    } 

    else {
      integral = 0.0;
    }

    double rawDerivative = (error - previousError) / TWOBAR_PID_DT_SECONDS;

    filteredDerivative = 0.8 * filteredDerivative + 0.2 * rawDerivative;

    previousError = error;

    return TWOBAR_KP * error + TWOBAR_KI * integral + TWOBAR_KD * filteredDerivative;
  }
};

TwoBarPIDController twoBarPIDController;
volatile double twoBarTargetDeg = TWOBAR_FRONT_DEG;
volatile bool twoBarPIDEnabled = false;
volatile double twoBarLastPositionDeg = 0.0;
volatile double twoBarLastLeftPositionDeg = 0.0;
volatile double twoBarLastRightPositionDeg = 0.0;
volatile double twoBarLastErrorDeg = 0.0;
volatile double twoBarLastLeftVoltage = 0.0;
volatile double twoBarLastRightVoltage = 0.0;
volatile bool twoBarStallFault = false;

constexpr bool CLAW_OPEN_STATE = true;
constexpr bool CLAW_CLAMPED_STATE = false;

/**
 * Function before autonomous. It prints the current auton number on the screen
 * and tapping the screen cycles the selected auton by 1. Add anything else you
 * may need, like resetting pneumatic components. You can rename these autons to
 * be more descriptive, if you like.
 */

void pre_auton() {
  default_constants();

  while(!auto_started){
    Brain.Screen.clearScreen();
    Brain.Screen.printAt(5, 20, "JAR Template v1.2.0");
    Brain.Screen.printAt(5, 40, "Battery Percentage:");
    Brain.Screen.printAt(5, 60, "%d", Brain.Battery.capacity());
    Brain.Screen.printAt(5, 80, "Chassis Heading Reading:");
    Brain.Screen.printAt(5, 100, "%f", chassis.get_absolute_heading());
    Brain.Screen.printAt(5, 120, "Selected Auton:");
    switch(current_auton_selection){
      case 0:
        Brain.Screen.printAt(5, 140, "Auton 1");
        break;
      case 1:
        Brain.Screen.printAt(5, 140, "Auton 2");
        break;
      case 2:
        Brain.Screen.printAt(5, 140, "Auton 3");
        break;
      case 3:
        Brain.Screen.printAt(5, 140, "Auton 4");
        break;
      case 4:
        Brain.Screen.printAt(5, 140, "Auton 5");
        break;
      case 5:
        Brain.Screen.printAt(5, 140, "Auton 6");
        break;
      case 6:
        Brain.Screen.printAt(5, 140, "Auton 7");
        break;
      case 7:
        Brain.Screen.printAt(5, 140, "Auton 8");
        break;
    }
    if(Brain.Screen.pressing()){
      while(Brain.Screen.pressing()) {}
      current_auton_selection ++;
    } else if (current_auton_selection == 8){
      current_auton_selection = 0;
    }
    task::sleep(10);
  }
}

/**
 * Auton function, which runs the selected auton. Case 0 is the default,
 * and will run in the brain screen goes untouched during preauton. Replace
 * drive_test(), for example, with your own auton function you created in
 * autons.cpp and declared in autons.h.
 */


int CPIDTask() {
  while (true) {
    if (cascadePIDEnabled) {
      double currentPos = rotationCascade.position(degrees);
      double target = cascadeTargetDeg;
      double error = target - currentPos;

      double voltage = cascadePIDController.compute(error);

      if (target > CASCADE_MIN_POSITION_DEG + 5.0) {
        voltage += CASCADE_HOLD_VOLTAGE;
      }

      if (voltage > CASCADE_MAX_PID_VOLTAGE) {
        voltage = CASCADE_MAX_PID_VOLTAGE;
      }

      if (voltage < -CASCADE_MAX_PID_VOLTAGE) {
        voltage = -CASCADE_MAX_PID_VOLTAGE;
      }

      if (currentPos <= CASCADE_MIN_POSITION_DEG && voltage < 0.0) {
        voltage = 0.0;
      }
      
      if (currentPos >= CASCADE_MAX_POSITION_DEG && voltage > 0.0) {
        voltage = 0.0;
      }

      cascadeLastErrorDeg = error;
      cascadeLastVoltage = voltage;

      if (voltage >= 0.0) {
        CascadeMotors.spin(forward, voltage, volt);
      } 
      
      else {
        CascadeMotors.spin(reverse, -voltage, volt);
      }
    }
    wait(10, msec);
  }

  return 0;
}

void spinMotorVoltage(motor &motorToSpin, double voltage) {
  if (voltage >= 0.0) {
    motorToSpin.spin(forward, voltage, volt);
  } 
  else {
    motorToSpin.spin(reverse, -voltage, volt);
  }
}

int TwoBarPIDTask() {
  timer stallTimer;
  bool stallTimerRunning = false;

  while (true) {
    if (twoBarPIDEnabled) {
      double leftPosition = twoBar_1.position(degrees);
      double rightPosition = twoBar_2.position(degrees);
      double averagePosition = (leftPosition + rightPosition) / 2.0;

      double target = twoBarTargetDeg;
      double error = target - averagePosition;
      double baseVoltage = twoBarPIDController.compute(error);

      double syncError = leftPosition - rightPosition;
      double syncCorrection = TWOBAR_SYNC_KP * syncError;

      if (syncCorrection > TWOBAR_SYNC_MAX_CORRECTION_VOLTAGE) {
        syncCorrection = TWOBAR_SYNC_MAX_CORRECTION_VOLTAGE;
      }
      if (syncCorrection < -TWOBAR_SYNC_MAX_CORRECTION_VOLTAGE) {
        syncCorrection = -TWOBAR_SYNC_MAX_CORRECTION_VOLTAGE;
      }

      double leftVoltage = baseVoltage - syncCorrection;
      double rightVoltage = baseVoltage + syncCorrection;

      if (std::fabs(error) > TWOBAR_MIN_VOLTAGE_ERROR_DEG) {
        if (error > 0.0) {
          if (leftVoltage < TWOBAR_MIN_MOVE_VOLTAGE) {
            leftVoltage = TWOBAR_MIN_MOVE_VOLTAGE;
          }

          if (rightVoltage < TWOBAR_MIN_MOVE_VOLTAGE) {
            rightVoltage = TWOBAR_MIN_MOVE_VOLTAGE;
          }
        } 
        
        else {
          if (leftVoltage > -TWOBAR_MIN_MOVE_VOLTAGE) {
            leftVoltage = -TWOBAR_MIN_MOVE_VOLTAGE;
          }

          if (rightVoltage > -TWOBAR_MIN_MOVE_VOLTAGE) {
            rightVoltage = -TWOBAR_MIN_MOVE_VOLTAGE;
          }
        }
      }

      if (leftVoltage > TWOBAR_MAX_VOLTAGE) {
        leftVoltage = TWOBAR_MAX_VOLTAGE;
      }

      if (leftVoltage < -TWOBAR_MAX_VOLTAGE) {
        leftVoltage = -TWOBAR_MAX_VOLTAGE;
      }

      if (rightVoltage > TWOBAR_MAX_VOLTAGE) {
        rightVoltage = TWOBAR_MAX_VOLTAGE;
      }

      if (rightVoltage < -TWOBAR_MAX_VOLTAGE) {
        rightVoltage = -TWOBAR_MAX_VOLTAGE;
      }


      twoBarLastPositionDeg = averagePosition;
      twoBarLastLeftPositionDeg = leftPosition;
      twoBarLastRightPositionDeg = rightPosition;
      twoBarLastErrorDeg = error;
      twoBarLastLeftVoltage = leftVoltage;
      twoBarLastRightVoltage = rightVoltage;

      double leftVelocity = std::fabs(twoBar_1.velocity(percent));
      double rightVelocity = std::fabs(twoBar_2.velocity(percent));

      bool movementRequested = std::fabs(error) > TWOBAR_STALL_ERROR_DEG;
      bool eitherMotorStopped = leftVelocity < TWOBAR_STALL_VELOCITY_PCT || rightVelocity < TWOBAR_STALL_VELOCITY_PCT;

      if (movementRequested && eitherMotorStopped) {
        if (!stallTimerRunning) {
          stallTimer.reset();
          stallTimerRunning = true;
        } 
        
        else if (
          stallTimer.time(msec) >= TWOBAR_STALL_TIMEOUT_MSEC) {
            twoBar_1.stop(hold);
            twoBar_2.stop(hold);
            twoBarPIDEnabled = false;
            twoBarStallFault = true;
            stallTimerRunning = false;

          wait(10, msec);
          continue;
          }
      }  
      
      else {
        stallTimerRunning = false;
      }

      if (std::fabs(error) <= TWOBAR_HOLD_ERROR_DEG && std::fabs(syncError) <= TWOBAR_SYNC_HOLD_ERROR_DEG) {
        twoBar_1.stop(hold);
        twoBar_2.stop(hold);
      } 
      
      else {
        spinMotorVoltage(twoBar_1, leftVoltage);
        spinMotorVoltage(twoBar_2, rightVoltage);
      }
    } 
    
    else {
      stallTimerRunning = false;
    }

    wait(10, msec);
  }

  return 0;
}




void autonomous() {
  auto_started = true;
  good_side();
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/



// Five scoring stages. Zero is reserved for the open-claw pickup state.
const double CASCADE_STAGE_1 = 850;
const double CASCADE_STAGE_2 = 1500;
const double CASCADE_STAGE_3 = 3000;
const double CASCADE_STAGE_4 = 4900;
const double CASCADE_STAGE_5 = 7560.0;

//Pick-up constants
constexpr double CLAW_OPEN_CASCADE_HEIGHT = 0.0;
constexpr double CLAW_CLOSED_CASCADE_HEIGHT = 850.0;
constexpr double CLAW_CLOSE_LIFT_DELAY_MSEC = 400.0;

constexpr double SCORE_RELEASE_DELAY_MSEC = 200.0;
constexpr double SCORE_CASCADE_BUMP_DEG = 1000.0;
constexpr double SCORE_ARM_READY_TOLERANCE_DEG = 100.0;

enum ScoreReturnState {
  SCORE_RETURN_IDLE = 0,
  SCORE_RETURN_WAIT_FOR_ARM,
  SCORE_RETURN_WAIT_AFTER_RELEASE,
  SCORE_RETURN_RAISE_CASCADE,
  SCORE_RETURN_ARM_TO_FRONT,
  SCORE_RETURN_CASCADE_TO_DEFAULT
};

void setCascadeTarget(double target) {
  if (target < CASCADE_MIN_POSITION_DEG) {
    target = CASCADE_MIN_POSITION_DEG;
  }
  if (target > CASCADE_MAX_POSITION_DEG) {
    target = CASCADE_MAX_POSITION_DEG;
  }

  cascadePIDEnabled = false;
  cascadeTargetDeg = target;

  double currentError = target - rotationCascade.position(degrees);
  cascadePIDController.reset(currentError);

  cascadePIDEnabled = true;
}

void setTwoBarTarget(double target) {
  twoBarPIDEnabled = false;
  twoBarStallFault = false;

  twoBar_1.stop(brake);
  twoBar_2.stop(brake);

  twoBarTargetDeg = target;

  double currentPosition = (twoBar_1.position(degrees) + twoBar_2.position(degrees)) / 2.0;

  twoBarPIDController.reset(target - currentPosition);
  twoBarPIDEnabled = true;
}

// One shared stage table for both driver control and autonomous.
// armScoreTargetMotorDeg is already converted through the 1:4 gear ratio.
bool getMechanismStageTargets(
    int stage,
    double &cascadeHeightDeg,
    double &armScoreTargetMotorDeg) {
  switch (stage) {
    case 1:
      cascadeHeightDeg = CASCADE_STAGE_1;
      armScoreTargetMotorDeg =
          armAngleToMotorDegrees(R2_ARM_SCORE_ANGLE_DEG);
      return true;
    case 2:
      cascadeHeightDeg = CASCADE_STAGE_2;
      armScoreTargetMotorDeg =
          armAngleToMotorDegrees(DOWN_ARM_SCORE_ANGLE_DEG);
      return true;
    case 3:
      cascadeHeightDeg = CASCADE_STAGE_3;
      armScoreTargetMotorDeg =
          armAngleToMotorDegrees(RIGHT_ARM_SCORE_ANGLE_DEG);
      return true;
    case 4:
      cascadeHeightDeg = CASCADE_STAGE_4;
      armScoreTargetMotorDeg =
          armAngleToMotorDegrees(Y_ARM_SCORE_ANGLE_DEG);
      return true;
    case 5:
      cascadeHeightDeg = CASCADE_STAGE_5;
      armScoreTargetMotorDeg =
          armAngleToMotorDegrees(B_ARM_SCORE_ANGLE_DEG);
      return true;
    default:
      return false;
  }
}

bool waitForCascadePIDTarget(
    double targetDeg,
    double toleranceDeg,
    double timeoutMsec) {
  timer timeoutTimer;
  timeoutTimer.reset();

  while (timeoutTimer.time(msec) < timeoutMsec) {
    if (std::fabs(
            targetDeg - rotationCascade.position(degrees))
        <= toleranceDeg) {
      return true;
    }

    wait(10, msec);
  }

  // Keep the PID enabled so it can continue holding/approaching its target.
  return false;
}

bool waitForTwoBarPIDTarget(
    double targetMotorDeg,
    double positionToleranceDeg,
    double syncToleranceDeg,
    double timeoutMsec) {
  timer timeoutTimer;
  timeoutTimer.reset();

  while (timeoutTimer.time(msec) < timeoutMsec) {
    double leftPosition = twoBar_1.position(degrees);
    double rightPosition = twoBar_2.position(degrees);
    double averagePosition =
        (leftPosition + rightPosition) / 2.0;
    double syncError = leftPosition - rightPosition;

    if (std::fabs(targetMotorDeg - averagePosition)
            <= positionToleranceDeg &&
        std::fabs(syncError) <= syncToleranceDeg) {
      return true;
    }

    // The two-bar PID disables itself if it detects a stall. Do not let that
    // freeze the rest of autonomous.
    if (twoBarStallFault) {
      return false;
    }

    wait(10, msec);
  }

  return false;
}

bool autonSetPickupPosition(double timeoutMsec) {
  // Open claw.
  claw.set(CLAW_OPEN_STATE);

  // Return arm to the front first.
  setTwoBarTarget(TWOBAR_FRONT_DEG);

  if (!waitForTwoBarPIDTarget(
          TWOBAR_FRONT_DEG,
          TWOBAR_FRONT_TOLERANCE_DEG,
          TWOBAR_SYNC_READY_TOLERANCE_DEG,
          timeoutMsec)) {
    return false;
  }

  // Lower cascade after the arm is safely in front.
  setCascadeTarget(CLAW_OPEN_CASCADE_HEIGHT);

  return waitForCascadePIDTarget(
      CLAW_OPEN_CASCADE_HEIGHT,
      CASCADE_SEQUENCE_READY_TOLERANCE_DEG,
      timeoutMsec);
}

bool autonMoveTwoBarToAngle(
    double armAngleDeg,
    double timeoutMsec) {

  // Converts actual arm angle through the 1:4 gearing.
  double targetMotorDeg =
      armAngleToMotorDegrees(armAngleDeg);

  setTwoBarTarget(targetMotorDeg);

  return waitForTwoBarPIDTarget(
      targetMotorDeg,
      TWOBAR_HOLD_ERROR_DEG,
      TWOBAR_SYNC_READY_TOLERANCE_DEG,
      timeoutMsec);
}

// Autonomous state 1: arm at the front and cascade at the selected holding
// height. Stages 1-5 use the exact same presets as the driver buttons.
bool autonSetHoldingHeight(int stage, double timeoutMsec) {
  double cascadeHeightDeg = 0.0;
  double unusedArmScoreTarget = 0.0;

  if (!getMechanismStageTargets(
          stage, cascadeHeightDeg, unusedArmScoreTarget)) {
    return false;
  }

  setTwoBarTarget(TWOBAR_FRONT_DEG);
  if (!waitForTwoBarPIDTarget(
          TWOBAR_FRONT_DEG,
          TWOBAR_FRONT_TOLERANCE_DEG,
          TWOBAR_SYNC_READY_TOLERANCE_DEG,
          timeoutMsec)) {
    return false;
  }

  setCascadeTarget(cascadeHeightDeg);
  return waitForCascadePIDTarget(
      cascadeHeightDeg,
      CASCADE_SEQUENCE_READY_TOLERANCE_DEG,
      timeoutMsec);
}

// Nonblocking: the background PID keeps moving the arm while autonomous
// immediately continues to driving or another mechanism command.
void autonStartTwoBarToAngle(double armAngleDeg) {
  setTwoBarTarget(armAngleToMotorDegrees(armAngleDeg));
}

// Optional pickup helper: clamp first, wait for the pneumatic clamp to finish,
// then run the normal holding-height state.
bool autonClampAndSetHoldingHeight(int stage, double timeoutMsec) {
  claw.set(CLAW_CLAMPED_STATE);
  wait(CLAW_CLOSE_LIFT_DELAY_MSEC, msec);
  return autonSetHoldingHeight(stage, timeoutMsec);
}

// Autonomous state 2: ensure the cascade is at the selected stage, then move
// the arm to that stage's independently tuned scoring angle.
bool autonMoveToScoringPosition(int stage, double timeoutMsec) {
  double cascadeHeightDeg = 0.0;
  double armScoreTargetMotorDeg = 0.0;

  if (!getMechanismStageTargets(
          stage, cascadeHeightDeg, armScoreTargetMotorDeg)) {
    return false;
  }

  if (std::fabs(
          cascadeHeightDeg - rotationCascade.position(degrees))
      > CASCADE_SEQUENCE_READY_TOLERANCE_DEG) {
    if (!autonSetHoldingHeight(stage, timeoutMsec)) {
      return false;
    }
  }

  setTwoBarTarget(armScoreTargetMotorDeg);
  return waitForTwoBarPIDTarget(
      armScoreTargetMotorDeg,
      SCORE_ARM_READY_TOLERANCE_DEG,
      TWOBAR_SYNC_READY_TOLERANCE_DEG,
      timeoutMsec);
}

// Autonomous state 3: the same full animation that opening the claw starts in
// driver control: release, wait, bump the cascade, return the arm to zero, and
// return the cascade to its default closed-claw height.
bool autonRunReturnAnimation(double timeoutMsec) {
  claw.set(CLAW_OPEN_STATE);

  // Finish any scoring-arm movement before starting the release delay.
  double scoringArmTarget = twoBarTargetDeg;
  if (!waitForTwoBarPIDTarget(
          scoringArmTarget,
          SCORE_ARM_READY_TOLERANCE_DEG,
          TWOBAR_SYNC_READY_TOLERANCE_DEG,
          timeoutMsec)) {
    return false;
  }

  wait(SCORE_RELEASE_DELAY_MSEC, msec);

  double bumpTarget = cascadeTargetDeg + SCORE_CASCADE_BUMP_DEG;
  if (bumpTarget > CASCADE_MAX_POSITION_DEG) {
    bumpTarget = CASCADE_MAX_POSITION_DEG;
  }

  setCascadeTarget(bumpTarget);
  if (!waitForCascadePIDTarget(
          bumpTarget,
          CASCADE_SEQUENCE_READY_TOLERANCE_DEG,
          timeoutMsec)) {
    return false;
  }

  setTwoBarTarget(TWOBAR_FRONT_DEG);
  if (!waitForTwoBarPIDTarget(
          TWOBAR_FRONT_DEG,
          TWOBAR_FRONT_TOLERANCE_DEG,
          TWOBAR_SYNC_READY_TOLERANCE_DEG,
          timeoutMsec)) {
    return false;
  }

  setCascadeTarget(CLAW_CLOSED_CASCADE_HEIGHT);
  return waitForCascadePIDTarget(
      CLAW_CLOSED_CASCADE_HEIGHT,
      CASCADE_SEQUENCE_READY_TOLERANCE_DEG,
      timeoutMsec);
}

void handleThreePressSequence(
    int stage,
    int &activeButton,
    int &sequenceStep,
    bool &cascadeMoveQueued,
    double &queuedCascadeTarget,
    double &queuedArmScoreTarget,
    bool &scoreMoveQueued) {

  double cascadeHeight = 0.0;
  double armScoreTarget = 0.0;

  if (!getMechanismStageTargets(
          stage, cascadeHeight, armScoreTarget)) {
    return;
  }

  int buttonId = stage - 1;

  if (activeButton != buttonId || sequenceStep == 0) {
    activeButton = buttonId;
    sequenceStep = 1;
    scoreMoveQueued = false;
    queuedCascadeTarget = cascadeHeight;
    queuedArmScoreTarget = armScoreTarget;
    cascadeMoveQueued = true;
    setTwoBarTarget(TWOBAR_FRONT_DEG);
  }
  
  else if (sequenceStep == 1) {
    sequenceStep = 2;
    scoreMoveQueued = true;
  }
  else {
    // Ignore extra presses 
  }
}




void usercontrol(void) {
  int activeSequenceButton = -1;
  int sequenceStep = 0;
  bool cascadeMoveQueued = false;
  double queuedCascadeTarget = CASCADE_STAGE_1;
  double queuedArmScoreTarget =
      armAngleToMotorDegrees(R2_ARM_SCORE_ANGLE_DEG);
  bool scoreMoveQueued = false;

  bool previousDown = false;
  bool previousRight = false;
  bool previousY = false;
  bool previousB = false;
  bool previousR2 = false;
  bool previousClawOpen = false;
  bool clawLiftPending = false;
  bool defaultCascadeMode = true;
  timer clawCloseTimer;
  ScoreReturnState scoreReturnState = SCORE_RETURN_IDLE;
  timer scoreReturnTimer;
  double scoreReturnLiftTarget = CASCADE_STAGE_1;

  // Begin above 0 deg
  setCascadeTarget(CLAW_CLOSED_CASCADE_HEIGHT);

  while (true) {
    chassis.control_arcade();

    bool currentDown = controller67.ButtonDown.pressing();
    bool currentRight = controller67.ButtonRight.pressing();
    bool currentY = controller67.ButtonY.pressing();
    bool currentB = controller67.ButtonB.pressing();
    bool currentL1 = controller67.ButtonL1.pressing();
    bool currentL2 = controller67.ButtonL2.pressing();
    bool currentR1 = controller67.ButtonR1.pressing();
    bool currentR2 = controller67.ButtonR2.pressing();


    bool clawOpen = currentR1;

    claw.set(clawOpen ? CLAW_OPEN_STATE : CLAW_CLAMPED_STATE);

    bool scoreReleaseJustRequested = clawOpen && !previousClawOpen && sequenceStep == 2 && scoreReturnState == SCORE_RETURN_IDLE;

    if (scoreReleaseJustRequested) {
      //cascade can't just go to 0 deg while scoring
      clawLiftPending = false;
      scoreReturnState = SCORE_RETURN_WAIT_FOR_ARM;
    } else if (
        clawOpen != previousClawOpen &&
        scoreReturnState == SCORE_RETURN_IDLE) {
      if (defaultCascadeMode) {
        //Cancel unfinished scoring
        activeSequenceButton = -1;
        sequenceStep = 0;
        cascadeMoveQueued = false;
        scoreMoveQueued = false;

        if (clawOpen) {
          clawLiftPending = false;
          setCascadeTarget(CLAW_OPEN_CASCADE_HEIGHT);
        } 
        
        else {
          clawCloseTimer.reset();
          clawLiftPending = true;
        }
      } 
      else {
        if (clawOpen) {
          clawLiftPending = false;
        } 
        
        else {
          clawCloseTimer.reset();
          clawLiftPending = true;
        }
      }

    }

    previousClawOpen = clawOpen;

    // running while the claw finishes clamping.
    if (clawLiftPending && !clawOpen && clawCloseTimer.time(msec) >= CLAW_CLOSE_LIFT_DELAY_MSEC) {
      clawLiftPending = false;

      // Only default height can trigger the lifting movement
  
      if (defaultCascadeMode && !cascadeMoveQueued) {
        setCascadeTarget(CLAW_CLOSED_CASCADE_HEIGHT);
      }
    }

    toggleL.set(currentL1);
    toggleR.set(currentL2);

    //fuck ah three stage

    if (scoreReturnState == SCORE_RETURN_IDLE && currentR2 && !previousR2) {
      defaultCascadeMode = true;


      if (clawOpen) {
        clawLiftPending = false;
        setCascadeTarget(CLAW_OPEN_CASCADE_HEIGHT);
      }

      handleThreePressSequence(1, activeSequenceButton, sequenceStep, cascadeMoveQueued, queuedCascadeTarget, queuedArmScoreTarget, scoreMoveQueued);
    }

    if (scoreReturnState == SCORE_RETURN_IDLE &&currentDown && !previousDown) {
      defaultCascadeMode = false;
      handleThreePressSequence(2, activeSequenceButton, sequenceStep, cascadeMoveQueued, queuedCascadeTarget, queuedArmScoreTarget, scoreMoveQueued);
    }

    if (scoreReturnState == SCORE_RETURN_IDLE && currentRight && !previousRight) {
      defaultCascadeMode = false;
      handleThreePressSequence(3, activeSequenceButton, sequenceStep, cascadeMoveQueued, queuedCascadeTarget, queuedArmScoreTarget, scoreMoveQueued);
    }

    if (scoreReturnState == SCORE_RETURN_IDLE && currentY && !previousY) {
      defaultCascadeMode = false;
      handleThreePressSequence(4, activeSequenceButton, sequenceStep, cascadeMoveQueued, queuedCascadeTarget, queuedArmScoreTarget, scoreMoveQueued);
    }

    if (scoreReturnState == SCORE_RETURN_IDLE && currentB && !previousB) {
      defaultCascadeMode = false;
      handleThreePressSequence(5, activeSequenceButton, sequenceStep, cascadeMoveQueued, queuedCascadeTarget, queuedArmScoreTarget, scoreMoveQueued);
    }

    previousDown = currentDown;
    previousRight = currentRight;
    previousY = currentY;
    previousB = currentB;
    previousR2 = currentR2;

    if (scoreReturnState == SCORE_RETURN_IDLE &&
        clawOpen &&
        sequenceStep == 2) {
      clawLiftPending = false;
      scoreReturnState = SCORE_RETURN_WAIT_FOR_ARM;
    }

    double leftTwoBarPosition = twoBar_1.position(degrees);
    double rightTwoBarPosition = twoBar_2.position(degrees);
    double averageTwoBarPosition =
        (leftTwoBarPosition + rightTwoBarPosition) / 2.0;

    double twoBarSyncError =
        leftTwoBarPosition - rightTwoBarPosition;

    bool twoBarAtFront =
        std::fabs(averageTwoBarPosition - TWOBAR_FRONT_DEG)
            <= TWOBAR_FRONT_TOLERANCE_DEG
        &&
        std::fabs(twoBarSyncError)
            <= TWOBAR_SYNC_READY_TOLERANCE_DEG;

    if (cascadeMoveQueued &&
        (!clawOpen || scoreReturnState != SCORE_RETURN_IDLE) &&
        !clawLiftPending &&
        twoBarAtFront) {
      setCascadeTarget(queuedCascadeTarget);
      cascadeMoveQueued = false;
    }

    bool cascadeAtTarget = !cascadeMoveQueued && std::fabs(cascadeTargetDeg - rotationCascade.position(degrees)) <= CASCADE_SEQUENCE_READY_TOLERANCE_DEG;

    if (scoreMoveQueued && cascadeAtTarget) {
      setTwoBarTarget(queuedArmScoreTarget);
      scoreMoveQueued = false;
    }

    //go back to front pos after scoring

    bool armAtScoringTarget = !scoreMoveQueued && std::fabs(averageTwoBarPosition - queuedArmScoreTarget) <= SCORE_ARM_READY_TOLERANCE_DEG;

    if (scoreReturnState == SCORE_RETURN_WAIT_FOR_ARM && armAtScoringTarget) {
      scoreReturnTimer.reset();
      scoreReturnState = SCORE_RETURN_WAIT_AFTER_RELEASE;
    }

    if (scoreReturnState == SCORE_RETURN_WAIT_AFTER_RELEASE && scoreReturnTimer.time(msec) >= SCORE_RELEASE_DELAY_MSEC) {
      scoreReturnLiftTarget = cascadeTargetDeg + SCORE_CASCADE_BUMP_DEG;

      if (scoreReturnLiftTarget > CASCADE_MAX_POSITION_DEG) {
        scoreReturnLiftTarget = CASCADE_MAX_POSITION_DEG;
      }

      setCascadeTarget(scoreReturnLiftTarget);
      scoreReturnState = SCORE_RETURN_RAISE_CASCADE;
    }

    bool scoreBumpReached = std::fabs(scoreReturnLiftTarget -rotationCascade.position(degrees)) <= CASCADE_SEQUENCE_READY_TOLERANCE_DEG;

    if (scoreReturnState == SCORE_RETURN_RAISE_CASCADE && scoreBumpReached) {
      setTwoBarTarget(TWOBAR_FRONT_DEG);
      scoreReturnState = SCORE_RETURN_ARM_TO_FRONT;
    }

    if (scoreReturnState == SCORE_RETURN_ARM_TO_FRONT && twoBarAtFront) {
      setCascadeTarget(CLAW_CLOSED_CASCADE_HEIGHT);
      scoreReturnState = SCORE_RETURN_CASCADE_TO_DEFAULT;
    }

    bool cascadeAtDefault = std::fabs(CLAW_CLOSED_CASCADE_HEIGHT - rotationCascade.position(degrees)) <= CASCADE_SEQUENCE_READY_TOLERANCE_DEG;

    if (scoreReturnState == SCORE_RETURN_CASCADE_TO_DEFAULT && cascadeAtDefault) {
      scoreReturnState = SCORE_RETURN_IDLE;
      activeSequenceButton = -1;
      sequenceStep = 0;
      cascadeMoveQueued = false;
      scoreMoveQueued = false;
      defaultCascadeMode = true;
    }


    wait(10, msec);
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  vexcodeInit();

  claw.set(CLAW_CLAMPED_STATE);
  toggleL.set(false);
  toggleR.set(false);

  rotationCascade.resetPosition();
  twoBar_1.resetPosition();
  twoBar_2.resetPosition();

  twoBar_1.setStopping(hold);
  twoBar_2.setStopping(hold);

  cascadeTargetDeg = rotationCascade.position(degrees);
  cascadePIDController.reset(0.0);
  cascadePIDEnabled = true;

  twoBarTargetDeg = TWOBAR_FRONT_DEG;
  twoBarPIDController.reset(0.0);
  twoBarPIDEnabled = true;

  task cascadeTask(CPIDTask);
  task twoBarTask(TwoBarPIDTask);

  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  pre_auton();

  while (true) {
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print(
      "Position: %.1f",
      rotationCascade.position(degrees)
    );
    Brain.Screen.setCursor(2, 1);
    Brain.Screen.print("Target:   %.1f", cascadeTargetDeg);
    Brain.Screen.setCursor(3, 1);
    Brain.Screen.print("Error:    %.1f", cascadeLastErrorDeg);
    Brain.Screen.setCursor(4, 1);
    Brain.Screen.print("Voltage:  %.2f", cascadeLastVoltage);
    Brain.Screen.setCursor(6, 1);
    Brain.Screen.print("2Bar pos: %.1f", twoBarLastPositionDeg);
    Brain.Screen.setCursor(7, 1);
    Brain.Screen.print("2Bar tgt: %.1f", twoBarTargetDeg);
    Brain.Screen.setCursor(8, 1);
    Brain.Screen.print("2Bar err: %.1f", twoBarLastErrorDeg);
    Brain.Screen.setCursor(9, 1);
    Brain.Screen.print(
      "2Bar V: %.1f/%.1f",
      twoBarLastLeftVoltage,
      twoBarLastRightVoltage
    );
    Brain.Screen.setCursor(10, 1);
    Brain.Screen.print(
      "2Bar L/R: %.0f/%.0f",
      twoBarLastLeftPositionDeg,
      twoBarLastRightPositionDeg
    );
    Brain.Screen.setCursor(11, 1);
    Brain.Screen.print(
      "2Bar stall: %d",
      twoBarStallFault
    );

    wait(100, msec);
  }
}