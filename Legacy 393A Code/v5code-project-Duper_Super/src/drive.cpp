#include "drive.h"
#include "vex.h"

using namespace vex;

// Helper Functions

void setDrive(int left, int right) {
  leftc.spin(fwd, left, volt);
  rightc.spin(fwd, right, volt);
}

void resetEncoders() {
  leftc.resetPosition();
  rightc.resetPosition();
  sensor.resetRotation();
}

double getEncoderAvgL() {
  double avg = ((fabs(leftcMotorA.position(rotationUnits::raw))) +
                (fabs(leftcMotorB.position(rotationUnits::raw)))) / 2.0;
  return avg;
}

double getEncoderAvgR() {
  double avg = ((fabs(rightcMotorA.position(rotationUnits::raw))) +
                (fabs(rightcMotorB.position(rotationUnits::raw)))) / 2.0;
  return avg;
}

double getIMUDeg() { 
  return fabs(sensor.rotation(deg));
}

void setChassisBrake(int type) {
  if (type == 1) {
    leftc.setStopping(coast);
    rightc.setStopping(coast);
  } 
  else if (type == 2) {
    leftc.setStopping(hold);
    rightc.setStopping(hold);
  }
  else if (type == 3) {
    leftc.setStopping(brake);
    rightc.setStopping(brake);
  }
}

// Driver Control Functions
void setDriveMotors() {
  double turnImportance = 0.5;
  setChassisBrake(1);

  double turnVal = umisha.Axis4.position();
  double forwardVal = umisha.Axis2.position();
  double turnVolts = turnVal * -0.12;
  double forwardVolts = forwardVal * 0.12 * (1 - (fabs(turnVolts) / 12.0) * turnImportance);
  setDrive(forwardVolts - turnVolts, forwardVolts + turnVolts);
}

// Autonomous Functons

void pdForward(int target, int maxSpeed, int minSpeed) {
  double kp = 0.55;
  double kd = 0.1;

  double errorL;
  double errorR;
  double prevErrorL;
  double prevErrorR;
  double derivativeL;
  double derivativeR;
  double rawPowerL;
  double rawPowerR;
  double maxVolts = maxSpeed * 12.0;
  double minVolts = minSpeed * 12.0;
  int cappedSpeed = -1;
  double gearRatio = 3 / 5.0;
  double wheelCircumfrence = M_PI * 4.125; // 
  double targetValue = (300.0 / (wheelCircumfrence * gearRatio)) * target;
  int DELAY_TIME = 10;
  int errorTimer;

  resetEncoders();
  setChassisBrake(2);

  while ((targetValue > getEncoderAvgL()) && (targetValue > getEncoderAvgR())) {

    errorL = targetValue - getEncoderAvgL();
    errorR = targetValue - getEncoderAvgR();
    derivativeL = (errorL - prevErrorL);
    derivativeR = (errorR - prevErrorR);
    prevErrorL = errorL;
    prevErrorR = errorR;

    rawPowerL = (errorL * kp) + (derivativeL * kd) - sensor.rotation(deg);
    rawPowerR = (errorR * kp) + (derivativeR * kd) + sensor.rotation(deg);

    cappedSpeed = errorL / abs(int(errorL));

    if (fabs(rawPowerL) <= fabs(minVolts)) {
      rawPowerL = minVolts * cappedSpeed;
    }

    if (fabs(rawPowerL) >= fabs(maxVolts)) {
      rawPowerL = maxVolts * cappedSpeed;
    }

    if (fabs(rawPowerR) <= fabs(minVolts)) {
      rawPowerR = minVolts * cappedSpeed;
    }

    if (fabs(rawPowerR) >= fabs(maxVolts)) {
      rawPowerR = maxVolts * cappedSpeed;
    }

    if (abs(int((errorL))) <= 5) { // && abs(turnError) <= 4
      errorTimer += DELAY_TIME;
      if (errorTimer > 60) {
        errorTimer = 0;
          break;
      }
    } 
    else {
      errorTimer = 0;
    }

    printf("error: %f\n", errorL);
    setDrive(-rawPowerL, -rawPowerR);
    wait(10, msec);
  }
}

void pdBackward(int target, int maxSpeed, int minSpeed) {
  double kp = 0.55;
  double kd = 0.1;

  double errorL;
  double errorR;
  double prevErrorL;
  double prevErrorR;
  double derivativeL;
  double derivativeR;
  double rawPowerL;
  double rawPowerR;
  double maxVolts = maxSpeed * 12.0;
  double minVolts = minSpeed * 12.0;
  int cappedSpeed = -1;
  double gearRatio = 3 / 5.0;
  double wheelCircumfrence = M_PI * 4.125; // 12.56
  double targetValue = (300.0 / (wheelCircumfrence * gearRatio)) * target;
  int DELAY_TIME = 10;
  int errorTimer;

  resetEncoders();
  setChassisBrake(2);

  while ((targetValue > getEncoderAvgL()) && (targetValue > getEncoderAvgR())) {

    errorL = targetValue - getEncoderAvgL();
    errorR = targetValue - getEncoderAvgR();
    derivativeL = (errorL - prevErrorL);
    derivativeR = (errorR - prevErrorR);
    prevErrorL = errorL;
    prevErrorR = errorR;

    rawPowerL = (errorL * kp) + (derivativeL * kd) - sensor.rotation(deg);
    rawPowerR = (errorR * kp) + (derivativeR * kd) + sensor.rotation(deg);

    cappedSpeed = errorL / abs(int(errorL));

    if (fabs(rawPowerL) <= fabs(minVolts)) {
      rawPowerL = minVolts * cappedSpeed;
    }

    if (fabs(rawPowerL) >= fabs(maxVolts)) { 
      rawPowerL = maxVolts * cappedSpeed;
    }

    if (fabs(rawPowerR) <= fabs(minVolts)) {
      rawPowerR = minVolts * cappedSpeed;
    }

    if (fabs(rawPowerR) >= fabs(maxVolts)) { 
      rawPowerR = maxVolts * cappedSpeed;
    }

    if (abs(int((errorL))) <= 5) { // && abs(turnError) <= 4
      errorTimer += DELAY_TIME;
      if (errorTimer > 60) {
        errorTimer = 0;
          break;
      }
    } 
    else {
      errorTimer = 0;
    }

    printf("error: %f\n", errorL);
    setDrive(rawPowerL, rawPowerR);
    wait(10, msec);
  }
}

void pdTurnLeft(int target, int maxSpeed, int minSpeed) {
  double kp = 0.111;
  double kd = 0.0;

  double error;
  double prevError;
  double derivative;
  double rawPower;
  double maxVolts = maxSpeed * 12.0;
  double minVolts = minSpeed * 12.0;
  int cappedSpeed = -1;
  int DELAY_TIME = 10;
  int errorTimer;

  resetEncoders();
  setChassisBrake(2);
  while (target > getIMUDeg()) {

    error = target - getIMUDeg();
    derivative = (error - prevError);
    prevError = error;
    rawPower = (error * kp) + (derivative * kd);
    cappedSpeed = error / abs(int(error));

    if (fabs(rawPower) <= fabs(minVolts)) {
      rawPower = minVolts * cappedSpeed;
    }

    if (fabs(rawPower) >= fabs(maxVolts)) {
      rawPower = maxVolts * cappedSpeed;
    }

    if (abs(int((error))) <= 5) { // && abs(turnError) <= 4
      errorTimer += DELAY_TIME;
      if (errorTimer > 60) {
        errorTimer = 0;
          break;
      }
    } else {
      errorTimer = 0;
    }

    printf("error: %f\n", error);
    setDrive(-rawPower, rawPower);
    wait(10, msec);
  }
}

void pdTurnRight(int target, int maxSpeed, int minSpeed) {
  double kp = 0.111;
  double kd = 0.0;

  double error;
  double prevError;
  double derivative;
  double rawPower;
  double maxVolts = maxSpeed * 12.0;
  double minVolts = minSpeed * 12.0;
  int cappedSpeed = -1;
  int DELAY_TIME = 10;
  int errorTimer;

  resetEncoders();
  setChassisBrake(2);

  while (target > getIMUDeg()) {

    error = target - getIMUDeg();
    derivative = (error - prevError);
    prevError = error;
    rawPower = (error * kp) + (derivative * kd);
    cappedSpeed = error / abs(int(error));

    if (fabs(rawPower) <= fabs(minVolts)) {
      rawPower = minVolts * cappedSpeed;
    }

    if (fabs(rawPower) >= fabs(maxVolts)) {
      rawPower = maxVolts * cappedSpeed;
    }

    if (abs(int((error))) <= 5) { // && abs(turnError) <= 4
      errorTimer += DELAY_TIME;
      if (errorTimer > 60) {
        errorTimer = 0;
          break;
      }
    } 
    else {
      errorTimer = 0;
    }

    printf("error: %f\n", error);
    setDrive(rawPower, -rawPower);
    wait(10, msec);
  }
}
