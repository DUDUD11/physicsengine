using System;
using System.Runtime.Intrinsics;


namespace FlatPhysics
{
    public static class FlatMath
    {

        public static readonly float VerySmallAmount = 0.005f;
        public static readonly float UltraMicroAmount = 0.000005f;

        public static float Clamp(float value, float min, float max)
        {
            if (min == max)
            {
                return min;
            }

            if (min > max)
            {
                throw new ArgumentOutOfRangeException("min is greater than the max.");
            }

            if (value < min)
            {
                return min;
            }

            if (value > max)
            {
                return max;
            }

            return value;


        }

        public static float LengthSquared(FlatVector v)
        { 
            return v.X*v.X+v.Y*v.Y;
        }

        public static float DistanceSquared(FlatVector a, FlatVector b)
        { 
            float dx = a.X - b.X;
            float dy = a.Y - b.Y;

            return dx*dx + dy*dy;
        }

        public static int Clamp(int value, int min, int max)
        {
            if (min == max)
            {
                return min;
            }

            if (min > max)
            {
                throw new ArgumentOutOfRangeException("min is greater than the max.");
            }

            if (value < min)
            {
                return min;
            }

            if (value > max)
            {
                return max;
            }

            return value;


        }




        public static float Length(FlatVector v)
        { 
            return MathF.Sqrt(v.X*v.X+v.Y*v.Y);
        }

        public static float Distance(FlatVector v1, FlatVector v2)
        {
            return Length(v1 - v2);
        }

        public static FlatVector Normalize(FlatVector v)
        {
            return v / Length(v);

        }

        public static float Dot(FlatVector v1, FlatVector v2)
        {
            return v1.X*v2.X+v1.Y*v2.Y;

        }

        public static bool NearlyEqual(float a, float b)
        {
            return MathF.Abs(a - b) < FlatMath.VerySmallAmount;



        }

        public static bool NearlyEqual(FlatVector a, FlatVector b)
        {
            return FlatMath.DistanceSquared(a, b) < FlatMath.VerySmallAmount;
        }

        public static bool AbsoultelyEqual(FlatVector a, FlatVector b)
        {
            return FlatMath.DistanceSquared(a, b) < FlatMath.UltraMicroAmount;
        }


        public static float Cross(FlatVector a, FlatVector b)
        {

            return a.X * b.Y - a.Y * b.X; 
        }
        


    }
}
