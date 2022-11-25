#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>


#include "v5.h"
#include "v5_vcs.h"

#include "robot-config.h"

//Helper Functions
void setFlywheel(int power);

double getFlywheelAVG();

void setFlywheelConstants(float KP, float KD, float KF);

void setTargetrpm(double input);


//Driver Control Functions
int setFlywheelMotors();
