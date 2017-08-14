#include "Arduino.h"
#include "RobotServo.h"

RobotServo::RobotServo()
{
  Angle = 0;
  Min = 0;
  Max = 0;
}

RobotServo::RobotServo(double angle, double min_angle, double max_angle)
{
  Angle = angle;
  Min = min_angle;
  Max = max_angle;
}

bool RobotServo::Valid()
{
  return Angle >= Min && Angle <= Max;
}

