#include "vex.h"
#include "intake.h"

//Helper Functions 
void setIntake(int power) {
    intake.spin(forward, power, volt);
}

//Driver Control Functions
void setIntakeMotors() {

  if (umisha.ButtonL2.pressing()) {
    setIntake(12.0);
  }
  else if (umisha.ButtonL1.pressing()) {
    setIntake(-7.0);
  }
  else {
    intake.stop(hold);
  }
}