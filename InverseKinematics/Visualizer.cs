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

        private int m_multiplier = 50;

        private SdlDotNet.Graphics.Font m_font;

        private bool m_demo = true;

        private int m_xMove = 0;
        private int m_yMove = 0;
        private int m_zMove = 0;

        public void Run()
        {
            m_video = Video.SetVideoMode(m_width, m_height, 32, false);

            Video.WindowCaption = "Inverse Kinematics of 3 Degrees of Freedom Robot Leg";
            
            m_simulator = new Simulator(AB, BC);
            m_simulator.X = 2.5;
            m_simulator.Y = 0;
            m_simulator.Z = 0;

            m_font = new SdlDotNet.Graphics.Font("C:\\Windows\\Fonts\\ARIAL.TTF", 12);

            Events.Tick += Events_Tick;
            Events.Quit += Events_Quit;
            Events.KeyboardDown += Events_KeyboardDown;
            Events.KeyboardUp += Events_KeyboardUp;

            Events.Run();
        }

        private void Events_KeyboardUp(object sender, SdlDotNet.Input.KeyboardEventArgs e)
        {
            switch(e.Key)
            {
                case SdlDotNet.Input.Key.LeftArrow:
                    if (m_xMove < 0)
                    {
                        m_xMove = 0;
                    }
                    break;

                case SdlDotNet.Input.Key.RightArrow:
                    if (m_xMove > 0)
                    {
                        m_xMove = 0;
                    }
                    break;

                case SdlDotNet.Input.Key.UpArrow:
                    if (m_zMove > 0)
                    {
                        m_zMove = 0;
                    }
                    break;

                case SdlDotNet.Input.Key.DownArrow:
                    if (m_zMove < 0)
                    {
                        m_zMove = 0;
                    }
                    break;

                case SdlDotNet.Input.Key.Q:
                    if (m_yMove > 0)
                    {
                        m_yMove = 0;
                    }
                    break;

                case SdlDotNet.Input.Key.A:
                    if (m_yMove < 0)
                    {
                        m_yMove = 0;
                    }
                    break;
            }
        }

        private void Events_KeyboardDown(object sender, SdlDotNet.Input.KeyboardEventArgs e)
        {
            switch (e.Key)
            {
                case SdlDotNet.Input.Key.D:
                    m_demo = !m_demo;
                    break;

                case SdlDotNet.Input.Key.Escape:
                    Events.QuitApplication();
                    break;

                case SdlDotNet.Input.Key.LeftArrow:
                    m_xMove = -1;
                    break;

                case SdlDotNet.Input.Key.RightArrow:
                    m_xMove = +1;
                    break;

                case SdlDotNet.Input.Key.UpArrow:
                    m_zMove = +1;
                    break;

                case SdlDotNet.Input.Key.DownArrow:
                    m_zMove = -1;
                    break;

                case SdlDotNet.Input.Key.Q:
                    m_yMove = +1;
                    break;

                case SdlDotNet.Input.Key.A:
                    m_yMove = -1;
                    break;
            }
        }

        private void Events_Quit(object sender, QuitEventArgs e)
        {
            Events.QuitApplication();
        }

        private void Events_Tick(object sender, TickEventArgs e)
        {
            int gridSize = 200;
            int gridStep = 10;

            m_video.Fill(Color.Black);

            // update sim
            if (m_demo)
            {
                m_simulator.Update(e.SecondsElapsed);
            }
            else
            {
                m_simulator.Update(e.SecondsElapsed, m_xMove, m_yMove, m_zMove);
            }

            Servo[] s = m_simulator.GetServos();

            // origin (A) (side)
            Point sideA = new Point(550, 150);
            drawGrid(sideA.X-gridStep, sideA.Y-(gridSize/2), gridSize, gridStep, "X-Y plane (rear, follows plane)");

            // knee (B) (side)
            double side_bx = Math.Cos(degreesToRadians(s[1].Angle)) * AB; // a = cos(theta)*h
            double side_by = Math.Sin(degreesToRadians(s[1].Angle)) * AB; // o = sin(theta)*h
            Point sideB = new Point((int)(side_bx * m_multiplier) + sideA.X, sideA.Y - (int)(side_by * m_multiplier));

            // foot (C) (side)
            double side_cx = Math.Sqrt(Math.Pow(m_simulator.X, 2) + Math.Pow(m_simulator.Y, 2));
            double side_cy = m_simulator.Z;
            Point sideC = new Point((int)(side_cx * m_multiplier) + sideA.X, sideA.Y - (int)(side_cy * m_multiplier));

            // draw AC (rear)
            m_video.Draw(new Line(sideA, sideC), Color.Green, true);
            m_video.Blit(m_font.Render(string.Format("({0:0.0},{1:0.0}) {2:0.00}°", 0, 0, s[1].Angle), Color.Red), sideA);

            // draw AB (rear)
            m_video.Draw(new Line(sideA, sideB), Color.Red, true);
            m_video.Blit(m_font.Render(string.Format("({0:0.0},{1:0.0}) {2:0.00}°", side_bx, side_by, s[2].Angle), Color.Blue), sideB);

            // draw BC (rear)
            m_video.Draw(new Line(sideB, sideC), Color.Blue, true);
            m_video.Blit(m_font.Render(string.Format("({0:0.0},{1:0.0})", side_cx, side_cy), Color.Green), sideC);

            // origin (A) (top)
            Point topA = new Point(sideA.X, sideA.Y+gridSize+(gridStep*2));
            drawGrid(topA.X - gridStep, topA.Y - (gridSize / 2), gridSize, gridStep, "Z plane (above)");

            // knee (B) (top)
            double top_bh = side_bx;
            double top_bx = Math.Cos(degreesToRadians(s[0].Angle)) * top_bh; // cos(theta) * h = a
            double top_by = Math.Sin(degreesToRadians(s[0].Angle)) * top_bh; // sin(theta) * h = o
            Point topB = new Point((int)(top_bx * m_multiplier) + topA.X, topA.Y - (int)(top_by * m_multiplier));

            // foot (C) (top)
            double top_ch = side_cx;
            double top_cx = Math.Cos(degreesToRadians(s[0].Angle)) * top_ch; // cos(theta) * h = a
            double top_cy = Math.Sin(degreesToRadians(s[0].Angle)) * top_ch; // sin(theta) * h = o
            Point topC = new Point((int)(top_cx * m_multiplier) + topA.X, topA.Y - (int)(top_cy * m_multiplier));

            // draw BC (top)
            m_video.Draw(new Line(topB, topC), Color.Blue, true);
            m_video.Blit(m_font.Render(string.Format("({0:0.0},{1:0.0})", top_bx, top_by), Color.Blue), new Point(topB.X, topB.Y-20));

            // draw AB (top) - draw after BC so it appears on top of
            m_video.Draw(new Line(topA, topB), Color.Red, true);
            m_video.Blit(m_font.Render(string.Format("({0:0.0},{1:0.0}) {2:0.00}°", 0, 0, s[0].Angle), Color.Red), topA);

            // draw AC (top)
            m_video.Blit(m_font.Render(string.Format("({0:0.0},{1:0.0})", top_cx, top_cy), Color.Green), topC);

            // text
            m_video.Blit(m_font.Render("Inverse Kinematics", Color.White), new Point(50, 50));
            m_video.Blit(m_font.Render(string.Format("Location (w,d,h) {0:0.0},{1:0.0},{2:0.0}", m_simulator.X, m_simulator.Y, m_simulator.Z), Color.White), new Point(50, 70));

            m_video.Blit(m_font.Render(string.Format("s0 (front/back hip) {0:0.00}°", s[0].Angle), Color.White, s[0].Valid ? Color.Black : Color.Red), new Point(50, 90));
            m_video.Blit(m_font.Render(string.Format("s1 (up/down hip) {0:0.00}°", s[1].Angle), Color.White, s[1].Valid ? Color.Black : Color.Red), new Point(50, 110));
            m_video.Blit(m_font.Render(string.Format("s2 (up/down knee) {0:0.00}°", s[2].Angle), Color.White, s[2].Valid ? Color.Black : Color.Red), new Point(50, 130));

            m_video.Blit(m_font.Render(string.Format("Origin location (a) {0:0.0},{1:0.0}", 0, 0), Color.White), new Point(300, 70));
            m_video.Blit(m_font.Render(string.Format("Knee location (b) {0:0.0},{1:0.0}", side_bx, side_by), Color.White), new Point(300, 90));
            m_video.Blit(m_font.Render(string.Format("Foot location (c) {0:0.0},{1:0.0}", side_cx, side_cy), Color.White), new Point(300, 110));
            m_video.Blit(m_font.Render(string.Format("ac_angle {0:0.00}°", s[1].Angle - s[2].Angle), Color.Green), new Point(300, 130));
            m_video.Blit(m_font.Render(string.Format("ab_angle {0:0.00}°", s[1].Angle), Color.Red), new Point(300, 150));
            m_video.Blit(m_font.Render(string.Format("bc_angle {0:0.00}°", s[2].Angle), Color.Blue), new Point(300, 170));

            m_video.Blit(m_font.Render(string.Format("Origin location (a) {0:0.0},{1:0.0}", 0, 0), Color.White), new Point(300, 290));
            m_video.Blit(m_font.Render(string.Format("Knee location (b) {0:0.0},{1:0.0}", top_bx, top_by), Color.White), new Point(300, 310));
            m_video.Blit(m_font.Render(string.Format("Foot location (c) {0:0.0},{1:0.0}", top_cx, top_cy), Color.White), new Point(300, 330));

            // update display
            m_video.Update();
        }

        private void drawGrid(int startX, int startY, int size, int step, string label)
        {
            Color color = Color.FromArgb(0x20, 0xC0, 0xC0, 0xC0);

            for (int x = startX; x <= startX + size; x += step)
            {
                m_video.Draw(new Line((short)x, (short)(startY - (step / 2)), (short)x, (short)(startY + size + (step / 2))), color);
            }

            for (int y = startY; y <= startY + size; y += step)
            {
                m_video.Draw(new Line((short)(startX - (step / 2)), (short)y, (short)(startX + size + (step / 2)), (short)y), color);
            }

            m_video.Blit(m_font.Render(label, Color.White), new Point(startX, startY + size - step));
        }

        private double degreesToRadians(double degrees)
        {
            return degrees * (Math.PI / 180);
        }
    }
}
