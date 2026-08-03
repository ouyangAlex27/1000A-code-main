#pragma once
#include "JAR-Template/drive.h"

class Drive;

extern Drive chassis;
bool autonSetHoldingHeight(
    int stage,
    double timeoutMsec = 5000.0);

bool autonClampAndSetHoldingHeight(
    int stage,
    double timeoutMsec = 5000.0);

bool autonMoveToScoringPosition(
    int stage,
    double timeoutMsec = 5000.0);

bool autonRunReturnAnimation(
    double timeoutMsec = 5000.0);


bool autonMoveTwoBarToAngle(
    double armAngleDeg,
    double timeoutMsec = 2500.0);
bool autonSetPickupPosition(double timeoutMsec = 4000.0);
void autonStartTwoBarToAngle(double armAngleDeg);
void default_constants();
void individual_motor_test();
void PID_test();
void turn_test();
void swing_test();
void full_test();
void odom_test();
void tank_odom_test();
void holonomic_odom_test();
void first_skills();
void second_skills();
void third_skills();
void skills();
void skillsBarrierCross();
void good_side();
void forwardDrive();
void right_side();
void solo_awp();