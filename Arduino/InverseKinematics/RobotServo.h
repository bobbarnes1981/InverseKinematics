#ifndef RobotServo_h
#define RobotServo_h

#include "Arduino.h"

class RobotServo
{
  public:
    double Angle;
    double Min;
    double Max;
    RobotServo();
    RobotServo(double angle, double min_angle, double max_angle);
    bool Valid();
};

#endif
