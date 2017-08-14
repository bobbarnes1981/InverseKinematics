#include <math.h>
#include "RobotLeg.h"
#include "RobotServo.h"

RobotLeg leg(1.0, 1.0);

void setup() {
}

void loop() {
  // put your main code here, to run repeatedly:
  leg.Solve(2.5, 0, 0);

  // offset for servo position
  double angle0 = leg.Servo0.Angle;
  double angle1 = leg.Servo1.Angle;
  double angle2 = leg.Servo2.Angle;
}
