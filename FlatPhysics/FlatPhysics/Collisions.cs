using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using static FlatPhysics.FlatBody;

namespace FlatPhysics
{
    public static class Collisions
    {
        public static void PointSegmentDistance(FlatVector p, FlatVector a, FlatVector b,
            out float distancesquared, out FlatVector cp)
        {
            FlatVector ab = b - a;
            FlatVector ap = p - a;
            float proj = FlatMath.Dot(ap, ab);
            float abLenSq = FlatMath.LengthSquared(ab);
            float d = proj / abLenSq;

            if (d <= 0f)
            {
                cp = a;
            }

            else if (d > 1f)
            {
                cp = b;
            }

            else
            {
                cp = a + ab * d;
            }

            distancesquared = FlatMath.DistanceSquared(p, cp);

        }


        public static bool IntersectAABBs(FlatAABB aabb1, FlatAABB aabb2)
        { 
            if(aabb1.Max.X <= aabb2.Min.X || aabb2.Max.X <= aabb2.Min.X) return false;

            if (aabb1.Max.Y <= aabb2.Min.Y || aabb2.Max.Y <= aabb2.Min.Y) return false;

            return true;
        }

        public static bool Collide(FlatBody bodyA, FlatBody bodyB, out FlatVector normal, out float depth)
        {
            normal = FlatVector.Zero;
            depth = 0f;

            ShapeType shapeTypeA = bodyA.shapeType;
            ShapeType shapeTypeB = bodyB.shapeType;

            if (shapeTypeA is ShapeType.Box)
            {
                if (shapeTypeB is ShapeType.Box)
                {
                    return Collisions.IntersectPolygons(bodyA.Position, bodyB.Position, bodyA.GetTransformedVertices(),
                        bodyB.GetTransformedVertices(), out normal, out depth);

                }

                else if (shapeTypeB is ShapeType.Circle)
                {
                    bool result = Collisions.IntersectCirclePolygon(bodyB.Position, bodyB.radius, bodyA.Position,
                        bodyA.GetTransformedVertices(), out normal, out depth);
                    normal = -normal;

                    return result;

                }

            }

            else if (shapeTypeA is ShapeType.Circle)
            {
                if (shapeTypeB is ShapeType.Box)
                {
                    return Collisions.IntersectCirclePolygon(bodyA.Position, bodyA.radius, bodyB.Position,
                        bodyB.GetTransformedVertices(), out normal, out depth);
                }

                else if (shapeTypeB is ShapeType.Circle)
                {
                    return Collisions.IntersectCircles(bodyA.Position, bodyA.radius, bodyB.Position, bodyB.radius, out normal, out depth);

                }

            }

            return false;


        }

        private static void FindPolygonsContactPoints(FlatVector[] verticesA, FlatVector[] verticesB,
            out FlatVector contact1, out FlatVector contact2, out int contactCount)
        {
            contact1 = FlatVector.Zero;
            contact2 = FlatVector.Zero;
            contactCount = 0;

            float minDistSq = float.MaxValue;

            for (int i = 0; i < verticesA.Length; i++) { 
                FlatVector p = verticesA[i];
                for (int j = 0; j < verticesB.Length; j++)
                { 
                    FlatVector va = verticesB[j];
                    FlatVector vb = verticesB[(j+1)%verticesB.Length];

                    Collisions.PointSegmentDistance(p, va, vb, out float distSq, out FlatVector cp);

                    if (FlatMath.NearlyEqual(distSq,minDistSq))
                    {

                        if (!FlatMath.NearlyEqual(cp,contact1))
                        { 

                            contactCount = 2;
                            contact2 = cp;
                        }
                    }

                    else if (distSq < minDistSq)
                    {

                        minDistSq = distSq;
                        contactCount = 1;
                        contact1 = cp;
                    }

                }
            
            }

            for (int i = 0; i < verticesB.Length; i++)
            {
                FlatVector p = verticesB[i];
                for (int j = 0; j < verticesA.Length; j++)
                {
                    FlatVector va = verticesA[j];
                    FlatVector vb = verticesA[(j + 1) % verticesA.Length];

                    Collisions.PointSegmentDistance(p, va, vb, out float distSq, out FlatVector cp);

                    if (FlatMath.NearlyEqual(distSq, minDistSq))
                    {

                        if (!FlatMath.NearlyEqual(cp, contact1))
                        {

                            contactCount = 2;
                            contact2 = cp;
                        }
                    }

                    else if (distSq < minDistSq)
                    {

                        minDistSq = distSq;
                        contactCount = 1;
                        contact1 = cp;
                    }

                }

            }



        }


        private static void FindCirclePolygonContactPoints(FlatVector circleCenter, float circleRadius,
            FlatVector polygonCenter, FlatVector[] polygonVertices, out FlatVector cp)
        {

            float minDistSq = float.MaxValue;
            cp = FlatVector.Zero;


            for (int i = 0; i < polygonVertices.Length; i++)
            {
                FlatVector va = polygonVertices[i];
                FlatVector vb = polygonVertices[(i + 1)%polygonVertices.Length];

                Collisions.PointSegmentDistance(circleCenter, va, vb, out float distanceSquared, out FlatVector contact);

                if (minDistSq > distanceSquared)
                { 
                    minDistSq = distanceSquared;
                    cp = contact;
                }


            }

        }

        public static void FindContactPoints(FlatBody bodyA, FlatBody bodyB, 
            out FlatVector contact1, out FlatVector contact2, out int contactCount)
        {

            contact1 = FlatVector.Zero;
            contact2 = FlatVector.Zero;
            contactCount = 0;

            ShapeType shapeTypeA = bodyA.shapeType;
            ShapeType shapeTypeB = bodyB.shapeType;

            if (shapeTypeA is ShapeType.Box)
            {
                if (shapeTypeB is ShapeType.Box)
                {
                    Collisions.FindPolygonsContactPoints(bodyA.GetTransformedVertices(), bodyB.GetTransformedVertices(), out contact1, out contact2, out contactCount);

                }

                else if (shapeTypeB is ShapeType.Circle)
                {
                    Collisions.FindCirclePolygonContactPoints(bodyB.Position, bodyB.radius, bodyA.Position, bodyA.GetTransformedVertices(), out contact1);
                    contactCount = 1;
                }

            }

            else if (shapeTypeA is ShapeType.Circle)
            {
                if (shapeTypeB is ShapeType.Box)
                {
                    Collisions.FindCirclePolygonContactPoints(bodyA.Position, bodyA.radius, bodyB.Position, bodyB.GetTransformedVertices(), out contact1);
                    contactCount = 1;
                }

                else if (shapeTypeB is ShapeType.Circle)
                {
                    Collisions.FindCirclesContactPoint(bodyA.Position, bodyA.radius, bodyB.Position, out contact1);
                    contactCount = 1;
                }

            }


        }

        private static void FindCirclesContactPoint(FlatVector centerA, float radiusA, FlatVector centerB, out FlatVector cp)
        { 
            FlatVector ab = centerB - centerA;
            FlatVector dir = FlatMath.Normalize(ab);
            cp = centerA + dir* radiusA;
        
        }


        public static bool IntersectCirclePolygon(FlatVector circleCenter, float circleRadius,FlatVector polygonCenter,  FlatVector[] vertices, 
            out FlatVector normal, out float depth)
        {
            normal = FlatVector.Zero;
            depth = float.MaxValue;

            FlatVector axis = FlatVector.Zero;
            float axisDepth = 0f;
            float minA, maxA, minB, maxB;


            for (int i = 0; i < vertices.Length; i++)
            {
                FlatVector va = vertices[i];
                FlatVector vb = vertices[(i + 1) % (vertices.Length)];

                FlatVector edge = vb - va;
                axis = new FlatVector(-edge.Y, edge.X);
                axis = FlatMath.Normalize(axis);



                Collisions.ProjectVertices(vertices, axis, out minA, out maxA);
                Collisions.ProjectCircle(circleCenter, circleRadius, axis, out minB, out maxB);


                if (minA >= maxB || maxA <= minB)

                {
                    return false;
                }

                axisDepth = MathF.Min(maxB - minA, maxA - minB);
                if (axisDepth < depth)
                {
                    depth = axisDepth;
                    normal = axis;
                }

            }

            int cpIndex = Collisions.FindClosestOnPolygon(circleCenter, vertices);
            FlatVector cp = vertices[cpIndex];

            axis = cp - circleCenter;
            axis = FlatMath.Normalize(axis);

            Collisions.ProjectVertices(vertices, axis, out minA, out maxA);
            Collisions.ProjectCircle(circleCenter, circleRadius, axis, out minB, out maxB);

            if (minA >= maxB || maxA <= minB)

            {
                return false;
            }

            axisDepth = MathF.Min(maxB - minA, maxA - minB);

            if (axisDepth < depth)
            {
                depth = axisDepth;
                normal = axis;
            }

          
            FlatVector direction = polygonCenter - circleCenter;

            if (FlatMath.Dot(direction, normal) < 0)
            {
                normal = -normal;
            }

            return true;

        }
      

        private static int FindClosestOnPolygon(FlatVector circleCenter, FlatVector[] vertices)
        {
            int result = -1;
            float minDistance = float.MaxValue;

            for (int i = 0; i < vertices.Length; i++)
            { 
                FlatVector v= vertices[i];
                float distance = FlatMath.Distance(v, circleCenter);

                if (distance < minDistance)
                {

                    minDistance = distance;
                    result = i;
                }
            
            }

            return result;
        }

        public static bool IntersectPolygons(FlatVector centerA, FlatVector centerB, FlatVector[] verticesA, FlatVector[] verticesB, 
            out FlatVector normal, out float depth)
        {
            normal = FlatVector.Zero;
            depth = float.MaxValue;


            for (int i = 0; i < verticesA.Length; i++)
            {
                FlatVector va = verticesA[i];
                FlatVector vb = verticesA[(i + 1) % (verticesA.Length)];

                FlatVector edge = vb - va;
                FlatVector axis = new FlatVector(-edge.Y, edge.X);

                axis = FlatMath.Normalize(axis);

                Collisions.ProjectVertices(verticesA, axis, out float minA, out float maxA);
                Collisions.ProjectVertices(verticesB, axis, out float minB, out float maxB);

                if (minA >= maxB || maxA <= minB)

                {
                    return false;
                }

                float axisDepth = MathF.Min(maxB - minA, maxA - minB);
                if (axisDepth < depth)
                {
                    depth = axisDepth;
                    normal = axis;
                }

            }

            for (int i = 0; i < verticesB.Length; i++)
            {
                FlatVector va = verticesB[i];
                FlatVector vb = verticesB[(i + 1) % (verticesB.Length)];

                FlatVector edge = vb - va;
                FlatVector axis = new FlatVector(-edge.Y, edge.X);

                axis = FlatMath.Normalize(axis);

                Collisions.ProjectVertices(verticesA, axis, out float minA, out float maxA);
                Collisions.ProjectVertices(verticesB, axis, out float minB, out float maxB);

                if (minA >= maxB || maxA <= minB)

                {
                    return false;
                }

                float axisDepth = MathF.Min(maxB - minA, maxA - minB);
                if (axisDepth < depth)
                {
                    depth = axisDepth;
                    normal = axis;
                }

            }

            FlatVector direction = centerB - centerA;

            if (FlatMath.Dot(direction, normal) < 0)
            {
                normal = -normal;
            }


            return true;

        }

        public static void ProjectCircle(FlatVector center, float radius, FlatVector axis, out float min, out float max)
        {
            FlatVector direction = FlatMath.Normalize(axis);
            FlatVector directionAndRadius = direction * radius;

            FlatVector p1 = center + directionAndRadius;
            FlatVector p2 = center - directionAndRadius;

            min = FlatMath.Dot(p1, axis);
            max = FlatMath.Dot(p2, axis);

            if (min > max)
            {
                float t = min;
                min = max;
                max = t;
            }



        }


        private static void ProjectVertices(FlatVector[] vertices, FlatVector axis, out float min, out float max)
        {
            min = float.MaxValue;
            max = float.MinValue;

            foreach (FlatVector v in vertices)
            {
                float proj = FlatMath.Dot(v, axis);

                if (proj < min)
                {
                    min = proj;
                }

                if (proj > max)
                {
                    max = proj;
                }

            }
        
        
        }



        public static bool IntersectCircles(FlatVector centerA, float radiusA, FlatVector centerB, float radiusB,
            out FlatVector normal, out float depth)
        {
            normal = FlatVector.Zero;
            depth = 0f;

            float distance = FlatMath.Distance(centerA, centerB);
            float radii = radiusA + radiusB;

            if (distance >= radii)
            {
                return false;

            }

            normal = FlatMath.Normalize(centerB - centerA);
            depth = radii - distance;

            return true;


        }

    }
}
