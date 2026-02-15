#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen.
brain  Brain;
motor leftF = motor(PORT3,ratio6_1, true);
motor leftM = motor(PORT12,ratio6_1, true);
motor leftB = motor(PORT2,ratio6_1, true);
motor rightF = motor(PORT9,ratio6_1, false);
motor rightM = motor(PORT20,ratio6_1, false);
motor rightB = motor(PORT10,ratio6_1, false);
motor intake = motor(PORT11, ratio6_1, true);
motor topGoal = motor(PORT19, ratio6_1, false);
digital_out matchLoad = digital_out(Brain.ThreeWirePort.A);
digital_out DigitalOutC = digital_out(Brain.ThreeWirePort.C);
controller controller67 = controller(primary);
digital_out Wings = digital_out(Brain.ThreeWirePort.B);     
inertial inertialS = inertial(PORT13);
distance distanceB = distance(PORT15);
distance distanceL = distance(PORT4);


//The motor constructor takes motors as (port, ratio, reversed), so for example
//motor LeftFront = motor(PORT1, ratio6_1, false);

//Add your devices below, and don't forget to do the same in robot-config.h:


void vexcodeInit( void ) {
  // nothing to initialize
}