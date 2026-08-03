using namespace vex;
using namespace cte;
extern brain Brain;
extern motor leftF;
extern motor leftM;
extern motor leftB;
extern motor rightF;
extern motor rightM;
extern motor rightB;
extern motor intake;
extern motor topGoal;
extern controller controller67;
extern digital_out matchLoad;
extern digital_out DigitalOutC;
extern digital_out claw;
extern inertial inertialS;
extern distance distanceB;
extern motor intakeL;
extern motor intakeR;
extern motor Cascade;
extern rotation rotationCascade;
extern rotation rotationTwoBar;
extern motor twoBar_1;
extern motor twoBar_2;
//To set up a extern called LeftFront here, you'd use
//extern extern LeftFront;

//Add your devices below, and don't forget to do the same in robot-config.cpp:



void  vexcodeInit( void );