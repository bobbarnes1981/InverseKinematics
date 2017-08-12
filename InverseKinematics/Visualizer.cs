using SdlDotNet.Core;
using SdlDotNet.Graphics;
using SdlDotNet.Graphics.Primitives;
using System;
using System.Drawing;

namespace InverseKinematics
{
    class Visualizer
    {
        private Surface m_video;

        private Simulator m_simulator;

        private double AB = 2;

        private double BC = 2;

        private int m_width = 800;

        private int m_height = 600;

        private int m_multiplier = 100;

        public void Run()
        {
            m_video = Video.SetVideoMode(m_width, m_height, 32, false);

            m_simulator = new Simulator(AB, BC);
            m_simulator.X = 2.5;
            m_simulator.Y = 0;
            m_simulator.Z = 0;

            Events.Tick += Events_Tick;
            Events.Quit += Events_Quit;

            Events.Run();
        }

        private void Events_Quit(object sender, QuitEventArgs e)
        {
            Events.QuitApplication();
        }

        private void Events_Tick(object sender, TickEventArgs e)
        {
            m_video.Fill(Color.Black);

            SdlDotNet.Graphics.Font font = new SdlDotNet.Graphics.Font("C:\\Windows\\Fonts\\ARIAL.TTF", 12);

            // update sim

            m_simulator.Update(e.SecondsElapsed);

            Servos s = m_simulator.GetServos();

            // origin (A)

            Point A = new Point(m_width / 2, m_height / 2);

            // foot (C)

            double cx = Math.Sqrt(Math.Pow(m_simulator.X, 2) + Math.Pow(m_simulator.Y, 2));

            double cy = m_simulator.Z;

            Point C = new Point((int)(cx * m_multiplier) + A.X, A.Y - (int)(cy * m_multiplier));

            // elbow (B)

            // a = cos(theta)*h
            double bx = Math.Cos(degreesToRadians(s.Servo1)) * AB;

            // o = sin(theta)*h
            double by = Math.Sin(degreesToRadians(s.Servo1)) * AB;

            Point B = new Point((int)(bx * m_multiplier) + A.X, A.Y - (int)(by * m_multiplier));

            // draw AC

            m_video.Draw(new Line(A, C), Color.Green, true);
            m_video.Blit(font.Render(string.Format("({0:0.0},{1:0.0}) {2:0.00}°", 0, 0, s.Servo1), Color.Red), A);

            // draw AB

            m_video.Draw(new Line(A, B), Color.Red, true);
            m_video.Blit(font.Render(string.Format("({0:0.0},{1:0.0}) {2:0.00}°", bx, by, s.Servo2), Color.Blue), B);

            // draw BC

            m_video.Draw(new Line(B, C), Color.Blue, true);
            m_video.Blit(font.Render(string.Format("({0:0.0},{1:0.0})", cx, cy), Color.Green), C);

            // text

            m_video.Blit(font.Render("Inverse Kinematics", Color.White), new Point(50, 50));

            m_video.Blit(font.Render(string.Format("Location {0:0.0},{1:0.0},{2:0.0}", m_simulator.X, m_simulator.Y, m_simulator.Z), Color.White), new Point(50, 70));

            m_video.Blit(font.Render(string.Format("Origin location (a) {0:0.0},{1:0.0}", 0, 0), Color.White), new Point(50, 90));

            m_video.Blit(font.Render(string.Format("Elbow location (b) {0:0.0},{1:0.0}", bx, by), Color.White), new Point(50, 110));

            m_video.Blit(font.Render(string.Format("Foot location (c) {0:0.0},{1:0.0}", cx, cy), Color.White), new Point(50, 130));

            m_video.Blit(font.Render(string.Format("ac_angle {0:0.00}°", s.Servo1 - s.Servo2), Color.Green), new Point(50, 150));

            m_video.Blit(font.Render(string.Format("ab_angle {0:0.00}°", s.Servo1), Color.Red), new Point(50, 170));

            m_video.Blit(font.Render(string.Format("bc_angle {0:0.00}°", s.Servo2), Color.Blue), new Point(50, 190));

            m_video.Blit(font.Render(string.Format("s0 (front/back) {0:0.00}°", s.Servo0), Color.White), new Point(50, 210));

            m_video.Blit(font.Render(string.Format("s1 (up/down hip) {0:0.00}°", s.Servo1), Color.White), new Point(50, 230));

            m_video.Blit(font.Render(string.Format("s2 (up/down knee) {0:0.00}°", s.Servo2), Color.White), new Point(50, 250));

            // update display

            m_video.Update();
        }

        private double degreesToRadians(double degrees)
        {
            return degrees * (Math.PI / 180);
        }
    }
}
