#include <math.h>

double ab_length;

double bc_length;

double servo0_angle = 0;
double servo0_min = -45;
double servo0_max = +45;

double servo1_angle = 60;
double servo1_min = -45;
double servo1_max = +180;

double servo2_angle = 120;
double servo2_min = +45;
double servo2_max = +170;

void setup() {
  // put your setup code here, to run once:
}

void loop() {
  // put your main code here, to run repeatedly:
  solve3DOF(2.5, 0, 0);
}

// target x-side y-forward z-up
void solve3DOF(double tx, double ty, double tz) {
  servo0_angle = radToDeg(atan(ty / tx));

  double AT = sqrt(pow(tx, 2) + pow(ty, 2));

  solve2DOF(AT, tz);
}

// target x-side, y-up
void solve2DOF(double tx, double ty) {
  double cx = tx;
  double cy = ty;

  double AC = sqrt(pow(cx, 2) + pow(cy, 2));
  double ac_angle = radToDeg(atan(cy / cx));

  //s=(AB+BC+AC)/2
  double s = (ab_length + bc_length + AC) / 2;

  //S=sqr(s*(s-AB)(s-BC)(s-AC))
  double S = sqrt(s * (s - ab_length) * (s - bc_length) * (s - AC));

  //A=asin(2S/(AB*AC))
  double A = radToDeg(asin((2 * S) / (ab_length * AC)));

  //B=asin(2S/(AB*BC))
  double B = radToDeg(asin((2 * S) / (ab_length * bc_length)));

  // Check if Hypoteneuse (AC) is big enough to mean triangle is obtuse
  // as rule of sines doesn't work on obtuse angles, could use rule of
  // cosines
  if (AC > sqrt(pow(ab_length, 2) + pow(bc_length, 2))) {
    B = 180 - B;
  }

  servo1_angle = ac_angle + A;

  servo2_angle = B;
}

double radToDeg(double rad) {
  return rad * (180 / PI);
}
