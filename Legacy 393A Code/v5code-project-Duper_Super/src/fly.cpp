#include "vex.h"
#include "fly.h"

// bool enable_flyPIDf = true;

// double error = 0;
// double target = 0;
// double input = 0;
// double derivative = 0;
// double prev_error = 0;
// double final_power = 0;
// float kp = 0;
// float kd = 0;
// float kf = 0;

// double get_vel() { return (fly.velocity(rpm) * 5); }
// void set_rpm(double input) { target = input; }

// void set_fly_constants(float KP, float KD, float KF) {
//   kp = KP;
//   kd = KD;
//   kf = KF;
// }

// int flyPID () {
  
//   while (enable_flyPIDf) {
//     error = target - get_vel(); //proportion
//     derivative = error - prev_error;
//     prev_error = error;

//     final_power = (error * kp) + (derivative * kd) + (target * kf);
//     if (final_power < 0) final_power = 0;
//     fly.spin(directionType::fwd, (final_power * (12000.0 / 12.0)), voltageUnits::mV);
//     //Fly.spin(directionType::fwd, ((12000)), voltageUnits::mV);



//     //printf("rpm:%f", get_vel());
//     Brain.Screen.setPenColor(blue);
//     Brain.Screen.clearLine(1);
//     Brain.Screen.printAt(5, 20, "rpm: %f", get_vel());
//     vex::task::sleep(5);
//   }
//   return 1;
// }

bool enable_flyPIDf = true;
int targetrpm;

double kp;
double kd;
double kf;

double error;
double prevError;
double derivative;
double rawPower;
double maxVolts = 12.0;
double minVolts = 12.0;
int cappedSpeed = -1;
double gearRatio = 1 / 5.0;
double rpmPerTick = 2.0; // motor rpm / motor ticks 600 / 300



//Helper Functions
void setFlywheel(int power) {
  flywheel.spin(forward, power, volt);
}

double getFlywheelAVG() {
  double avg = (fabs(flywheelMotorA.velocity(rpm)) +
                (fabs(flywheelMotorB.velocity(rpm))) / 2.0) / gearRatio;
  return avg;
}

void setFlywheelConstants(float KP, float KD, float KF) {
  kp = KP;
  kd = KD;
  kf = KF;
}

void setTargetrpm(double input) {
  targetrpm = input;
}




//Driver Control Functions
int setFlywheelMotors() {

  while(enable_flyPIDf) {

    error = targetrpm - getFlywheelAVG();
    derivative = error - prevError;
    prevError = error;

    cappedSpeed = error / abs(int(error));

    if (fabs(rawPower) <= fabs(minVolts)) {
      rawPower = minVolts * cappedSpeed;
    }

    if (fabs(rawPower) >= fabs(maxVolts)) {
      rawPower = maxVolts * cappedSpeed;
    }


  rawPower = (kp * error) + (kd * derivative) + (kf * targetrpm);


  printf("rpm:%f", getFlywheelAVG());
  Brain.Screen.setPenColor(blue);
  Brain.Screen.clearLine(1);
  Brain.Screen.printAt(5, 20, "rpm: %f", getFlywheelAVG());
  task::sleep(5);
  setFlywheel(rawPower);
  }

  return 1;
}
