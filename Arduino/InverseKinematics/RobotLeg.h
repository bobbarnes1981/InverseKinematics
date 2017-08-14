#ifndef RobotLeg_h
#define RobotLeg_h

#include "Arduino.h"
#include "RobotServo.h"

class RobotLeg
{
  public:
    RobotLeg(double ab_length, double bc_length);
    void Solve(double x, double y, double z);
    RobotServo Servo0;
    RobotServo Servo1;
    RobotServo Servo2;
  private:
    double ab_length;
    double bc_length;
    void solve3DOF(double x, double y, double z);
    void solve2DOF(double x, double y);
    double radToDeg(double rad);
};

#endif
