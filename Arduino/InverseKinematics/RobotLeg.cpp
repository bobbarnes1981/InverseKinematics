#include "Arduino.h"
#include "RobotLeg.h"
#include "RobotServo.h"
  
RobotLeg::RobotLeg(double ab_length, double bc_length)
{
  ab_length = ab_length;
  bc_length = bc_length;
  
  Servo0 = RobotServo(0.0, -45.0, +45.0);
  Servo1 = RobotServo(60, -45, +180);
  Servo2 = RobotServo(120, +45, +170);
}

void RobotLeg::Solve(double x, double y, double z)
{
  solve3DOF(x, y, z);
}

// target x-side y-forward z-up
void RobotLeg::solve3DOF(double tx, double ty, double tz)
{
  Servo0.Angle = this->radToDeg(atan(ty / tx));

  double AT = sqrt(pow(tx, 2) + pow(ty, 2));

  solve2DOF(AT, tz);
}

// target x-side, y-up
void RobotLeg::solve2DOF(double tx, double ty)
{
  double cx = tx;
  double cy = ty;

  double AC = sqrt(pow(cx, 2) + pow(cy, 2));
  double ac_angle = this->radToDeg(atan(cy / cx));

  //s=(AB+BC+AC)/2
  double s = (ab_length + bc_length + AC) / 2;

  //S=sqr(s*(s-AB)(s-BC)(s-AC))
  double S = sqrt(s * (s - ab_length) * (s - bc_length) * (s - AC));

  //A=asin(2S/(AB*AC))
  double A = this->radToDeg(asin((2 * S) / (ab_length * AC)));

  //B=asin(2S/(AB*BC))
  double B = this->radToDeg(asin((2 * S) / (ab_length * bc_length)));

  // Check if Hypoteneuse (AC) is big enough to mean triangle is obtuse
  // as rule of sines doesn't work on obtuse angles, could use rule of
  // cosines
  if (AC > sqrt(pow(ab_length, 2) + pow(bc_length, 2))) {
    B = 180 - B;
  }

  Servo1.Angle = ac_angle + A;

  Servo2.Angle = B;
}

double RobotLeg::radToDeg(double rad)
{
  return rad * (180 / PI);
}

