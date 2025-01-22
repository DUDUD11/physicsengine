using System;


namespace FlatPhysics
{
    public readonly struct FlatVector
    {
        public readonly float X;
        public readonly float Y;

        public static readonly FlatVector Zero = new FlatVector(0f, 0f);


        public FlatVector(float x, float y)
        {
            this.X = x;
            this.Y = y;
        }

        public static FlatVector operator +(FlatVector a, FlatVector b)
        {
            return new FlatVector(a.X + b.X, a.Y + b.Y);
        }

        public static FlatVector operator -(FlatVector a, FlatVector b)
        {
            return new FlatVector(a.X - b.X, a.Y - b.Y);
        }

        public static FlatVector operator -(FlatVector a)
        {
            return new FlatVector(-a.X , -a.Y);
        }

        public static FlatVector operator *(FlatVector a, float s)
        {
            return new FlatVector(a.X * s, a.Y * s);
        }

        public static FlatVector operator *(float s, FlatVector a)
        {
            return new FlatVector(a.X * s, a.Y * s);
        }

        public static FlatVector operator /(FlatVector a, float s)
        {
            return new FlatVector(a.X / s, a.Y / s);
        }

        public bool Equals(FlatVector other)
        {
            return this.X == other.X && this.Y == other.Y;
        }

        internal static FlatVector Transform(FlatVector v, FlatTransform transform)
        {
            return new FlatVector(
               transform.Cos * v.X - transform.Sin * v.Y + transform.PositionX,
               transform.Sin * v.X + transform.Cos * v.Y + transform.PositionY);



        }

        public override bool Equals(object obj)
        {
            if (obj is FlatVector other)
            {
                return this.Equals(other);
            }
            return false;
        }

        public override int GetHashCode()
        {
            return new { this.X, this.Y }.GetHashCode();
        }

        public override string ToString()
        {
            return $"X: {this.X}, Y: {this.Y}";
        }

    }
}
