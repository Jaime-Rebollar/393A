#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller umisha;

motor flywheelMotorA = motor(PORT13, ratio6_1, false);
motor flywheelMotorB = motor(PORT14, ratio6_1, false);
motor_group flywheel = motor_group(flywheelMotorA, flywheelMotorB);

motor leftcMotorA = motor(PORT9, ratio6_1, true);
motor leftcMotorB = motor(PORT7, ratio6_1, true);
motor_group leftc = motor_group(leftcMotorA, leftcMotorB);

motor rightcMotorA = motor(PORT3, ratio6_1, false);
motor rightcMotorB = motor(PORT2, ratio6_1, false);
motor_group rightc = motor_group(rightcMotorA, rightcMotorB);

motor indexer = motor(PORT20, ratio18_1, true);
motor intake = motor( PORT10, ratio6_1, true);

inertial sensor = inertial(PORT17);
digital_out expand = digital_out(Brain.ThreeWirePort.A);



// VEXcode generated functions



/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}