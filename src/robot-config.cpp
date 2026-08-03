#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen.
brain  Brain;
motor leftF = motor(PORT1,ratio18_1, true);
motor leftM = motor(PORT12,ratio18_1, true);
motor leftB = motor(PORT11,ratio18_1, true);

motor rightF = motor(PORT5,ratio18_1, false );
motor rightM = motor(PORT19,ratio18_1, false);
motor rightB = motor(PORT20,ratio18_1, false);

motor Cascade_1 = motor(PORT2,ratio18_1, false);
motor Cascade_2 = motor(PORT9,ratio18_1, true);
motor twoBar_1 = motor(PORT15,ratio18_1, false);
motor twoBar_2 = motor(PORT17,ratio18_1, true);

digital_out claw(Brain.ThreeWirePort.B);
digital_out toggleL(Brain.ThreeWirePort.H);
digital_out toggleR(Brain.ThreeWirePort.C);
controller controller67 = controller(primary);   
inertial inertialS = inertial(PORT18);
rotation rotationCascade = rotation(PORT3);



//The motor constructor takes motors as (port, ratio, reversed), so for example
//motor LeftFront = motor(PORT1, ratio6_1, false);

//Add your devices below, and don't forget to do the same in robot-config.h:


void vexcodeInit( void ) {
  // nothing to initialize
}