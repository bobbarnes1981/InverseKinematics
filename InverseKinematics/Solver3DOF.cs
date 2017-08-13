using System;

namespace InverseKinematics
{
    class Solver3DOF
    {
        private double AB;

        private double BC;

        public Solver3DOF(double ab, double bc)
        {
            AB = ab;
            BC = bc;
        }

        public double[] SolveArduinoStyle(double x, double y, double z)
        {
            double[] servos = new double[3];

            double newX = Math.Sqrt(Math.Pow(x, 2) + Math.Pow(y, 2));

            double newY = z;

            double AC = Math.Sqrt(Math.Pow(newX, 2) + Math.Pow(newY, 2));
            double ac_angle = radiansToDegrees(Math.Atan(newY / newX));

            //s=(AB+BC+AC)/2
            double s = (AB + BC + AC) / 2;

            //S=sqr(s*(s-AB)(s-BC)(s-AC))
            double S = Math.Sqrt(s * (s - AB) * (s - BC) * (s - AC));

            //A=asin(2S/(AB*AC))
            double A = radiansToDegrees(Math.Asin((2 * S) / (AB * AC)));
            //B=asin(2S/(AB*BC))
            double B = radiansToDegrees(Math.Asin((2 * S) / (AB * BC)));

            servos[0] = radiansToDegrees(Math.Atan(y / x));
            servos[1] = ac_angle + A;
            servos[2] = B;

            return servos;
        }

        public Servos Solve(double tx, double ty, double tz)
        {
            return Solve3DOF(tx, ty, tz);
        }

        // x-side, y-forward, z-up
        private Servos Solve3DOF(double tx, double ty, double tz)
        {
            double servo0 = radiansToDegrees(Math.Atan(ty / tx));

            double AT = Math.Sqrt(Math.Pow(tx, 2) + Math.Pow(ty, 2));

            Tuple<double, double> servo1and2 = Solve2DOF(AT, tz);

            return new Servos
            {
                Servo0 = servo0,
                Servo1 = servo1and2.Item1,
                Servo2 = servo1and2.Item2
            };
        }

        // x-side, y-up
        private Tuple<double, double> Solve2DOF(double tx, double ty)
        {
            double cx = tx;
            double cy = ty;

            double AC = Math.Sqrt(Math.Pow(cx, 2) + Math.Pow(cy, 2));
            double ac_angle = radiansToDegrees(Math.Atan(cy / cx));

            //s=(AB+BC+AC)/2
            double s = (AB + BC + AC) / 2;

            //S=sqr(s*(s-AB)(s-BC)(s-AC))
            double S = Math.Sqrt(s * (s - AB) * (s - BC) * (s - AC));

            //A=asin(2S/(AB*AC))
            double A = radiansToDegrees(Math.Asin((2 * S) / (AB * AC)));
            //B=asin(2S/(AB*BC))
            double B = radiansToDegrees(Math.Asin((2 * S) / (AB * BC)));

            return new Tuple<double, double>(ac_angle + A, B);
        }

        private double radiansToDegrees(double radians)
        {
            return radians * (180 / Math.PI);
        }
    }
}
