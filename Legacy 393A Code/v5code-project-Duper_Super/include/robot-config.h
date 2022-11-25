using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller umisha;


extern motor flywheelMotorA;
extern motor flywheelMotorB;
extern motor_group flywheel;


extern motor leftcMotorA;
extern motor leftcMotorB;
extern motor_group leftc;

extern motor rightcMotorA;
extern motor rightcMotorB;
extern motor_group rightc;


extern motor indexer;


extern motor intake;


extern inertial sensor;


extern digital_out expand;



/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );