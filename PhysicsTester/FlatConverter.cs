using System;
using System.Runtime.CompilerServices;
using Microsoft.Xna.Framework;
using FlatPhysics;

namespace PhysicsTester
{
    public static class FlatConverter
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static Vector2 ToVector2(FlatVector v)
        { 
            return new Vector2(v.X, v.Y);   
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static FlatVector ToFlatVector(Vector2 v)
        {
            return new FlatVector(v.X, v.Y);
        }

        public static void ToVector2Array(FlatVector[] src, ref Vector2[] dst)
        {
            if (dst is null || src.Length != dst.Length)
            { 
                dst = new Vector2[src.Length];
            }

            for (int i = 0; i < src.Length; i++)
            {
                    FlatVector v = src[i];
                dst[i] = new Vector2(v.X, v.Y);
            }

        }
    }
}
