namespace InverseKinematics
{
    struct Servo
    {
        public double Angle { get; set; }
        public bool Valid
        {
            get
            {
                return Angle > Min && Angle < Max;
            }
        }
        public double Max { get; set; }
        public double Min { get; set; }
    }
}
