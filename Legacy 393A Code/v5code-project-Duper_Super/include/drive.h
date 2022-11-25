#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>


#include "v5.h"
#include "v5_vcs.h"

#include "robot-config.h"



//Helper Functions
void setDrive(int left, int right);

void resetEncoders();

double getEncoderAvgL();

double getEncoderAvgR();

double getIMUDeg();

void setChassisBrake(int type);

//Driver Control Functions
void setDriveMotors();

//Autonomous Functons

void pdForward(int target, int maxSpeed, int minSpeed); //target in inches

void pdBackward(int target, int maxSpeed, int minSpeed); //target in inches

void pdTurnRight(int target, int maxSpeed, int minSpeed); //target in degrees

void pdTurnLeft(int target, int maxSpeed, int minSpeed); //target in degrees