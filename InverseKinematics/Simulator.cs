namespace InverseKinematics
{
    class Simulator
    {
        private Solver3DOF m_solver;
        /// <summary>
        /// width
        /// </summary>
        public double X { get; set; }

        /// <summary>
        /// depth
        /// </summary>
        public double Y { get; set; }

        /// <summary>
        /// height
        /// </summary>
        public double Z { get; set; }

        private float m_speed = 0.4f;

        private State m_state = State.Z;

        private Direction m_direction = Direction.B;

        public Simulator(double ab, double bc)
        {
            m_solver = new Solver3DOF(ab, bc);
        }

        public void Update(float seconds)
        {
            switch (m_state)
            {
                case State.X:
                    switch (m_direction)
                    {
                        case Direction.B:
                            X += (m_speed * seconds);
                            if (X > 3)
                            {
                                X = 3;
                                m_direction = Direction.F;
                            }
                            break;
                        case Direction.F:
                            X -= (m_speed * seconds);
                            if (X < 2.5)
                            {
                                X = 2.5;
                                m_direction = Direction.B;
                                m_state = State.Y;
                            }
                            break;
                    }
                    break;
                case State.Y:
                    switch (m_direction)
                    {
                        case Direction.B:
                            Y += (m_speed * seconds);
                            if (Y > 2)
                            {
                                Y = 2;
                                m_direction = Direction.F;
                            }
                            break;
                        case Direction.F:
                            Y -= (m_speed * seconds);
                            if (Y < 0)
                            {
                                Y = 0;
                                m_direction = Direction.B;
                                m_state = State.Z;
                            }
                            break;
                    }
                    break;
                case State.Z:
                    switch (m_direction)
                    {
                        case Direction.B:
                            Z -= (m_speed * seconds);
                            if (Z < -1)
                            {
                                Z = -1;
                                m_direction = Direction.F;
                            }
                            break;
                        case Direction.F:
                            Z += (m_speed * seconds);
                            if (Z > 2)
                            {
                                Z = 2;
                                m_direction = Direction.B;
                                m_state = State.X;
                            }
                            break;
                    }
                    break;
            }
        }

        public void Update(float seconds, float x, float y, float z)
        {
            X += seconds * m_speed * x;
            Y += seconds * m_speed * y;
            Z += seconds * m_speed * z;
        }

        public Servo[] GetServos()
        {
            // 1 width, 1 depth, 0 height
            Servo[] servos = m_solver.Solve(X, Y, Z);

            return servos;
        }

        private enum State
        {
            X,
            Y,
            Z
        }

        private enum Direction
        {
            F,
            B
        }
    }
}
