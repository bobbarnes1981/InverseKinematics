using System;

namespace InverseKinematics
{
    class Solver3DOF
    {
        private double AB;

        private double BC;

        private Servo[] m_servos = new Servo[]
        {
            new Servo { Min = -90, Max = +90 },
            new Servo { Min = -45, Max = +90 },
            new Servo { Min = +45, Max = +170 }
        };

        public Solver3DOF(double ab, double bc)
        {
            AB = ab;
            BC = bc;
        }

        public Servo[] Solve(double tx, double ty, double tz)
        {
            Solve3DOF(tx, ty, tz);

            return m_servos;
        }

        // x-side, y-forward, z-up
        private void Solve3DOF(double tx, double ty, double tz)
        {
            m_servos[0].Angle = radiansToDegrees(Math.Atan(ty / tx));

            double AT = Math.Sqrt(Math.Pow(tx, 2) + Math.Pow(ty, 2));

            Solve2DOF(AT, tz);
        }

        // x-side, y-up
        private void Solve2DOF(double tx, double ty)
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

            // Check if Hypoteneuse (AC) is big enough to mean triangle is obtuse
            // for some reason angle B went between 0-90 the back again instead
            // of 0-180. Can we just use a different equation to B=asin(2S/(AB*BC))?
            if (AC > Math.Sqrt(Math.Pow(AB, 2) + Math.Pow(BC, 2)))
            {
                B = 180 - B;
            }

            m_servos[1].Angle = ac_angle + A;

            m_servos[2].Angle = B;
        }

        private double radiansToDegrees(double radians)
        {
            return radians * (180 / Math.PI);
        }
    }
}
