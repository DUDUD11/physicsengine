using System;
using System.Linq.Expressions;
using System.Runtime.CompilerServices;
using System.Security.AccessControl;

namespace FlatPhysics
{
    public sealed class FlatBody
    {

        public enum ShapeType
        { 
            Circle = 0,
            Box = 1
        }

        private FlatVector position;
        private FlatVector linearVelocity;
        private float angle;
        private float angularVelocity;
        private FlatVector force;

        public readonly ShapeType shapeType;
        public readonly float density;
        public readonly float mass;
        public readonly float restitution; // bounce 0 to 1 
        public readonly float area;
        public readonly float Inertia;
        public readonly float invInertia;
        public readonly bool isStatic;
        public readonly float invMass;
        public readonly float radius;
        public readonly float width;
        public readonly float height;
        public readonly float StaticFriction;
        public readonly float DynamicFriction;
 
        private readonly FlatVector[] vertices;
     //   public readonly int[] triangles;
        private FlatVector[] transformedVertices;
        private FlatAABB aabb;


        private bool transformUpdateRequired;
        private bool aabbUpdateRequired;

   

        public FlatVector Position
           { get { return position; } }

        public FlatVector LinearVelocity
            { get { return linearVelocity; } 
            internal set { linearVelocity = value; }
        
        
        }

        public float Angle
        {
            get { return angle; }

        }
        public float AngularVelocity
        {
            get { return angularVelocity; }
            internal set { angularVelocity = value; }
        }




        private FlatBody(float _density, float _mass, float inertia, float _restitution, float _area, bool _isStatic,
            float _radius, float _width,float _height, FlatVector[] vertices, ShapeType _shapeType)
        { 
            this.position = FlatVector.Zero;
            this.linearVelocity = FlatVector.Zero;
            this.angle = 0f;
            this.angularVelocity = 0f;
            this.density = _density;
            this.mass = _mass;
            this.restitution = _restitution;
            this.area = _area;
            this.isStatic = _isStatic;
            this.radius = _radius;
            this.width = _width;
            this.height = _height;
            this.shapeType = _shapeType;
            this.force = FlatVector.Zero;
            this.Inertia = inertia;
            this.invMass = mass > 0f ? 1f/this.mass:0f;
            this.invInertia = inertia>0f? 1f/this.Inertia : 0f;
            this.StaticFriction = 0.6f;
            this.DynamicFriction = 0.4f;

            if (shapeType is ShapeType.Box)
            {
                this.vertices = vertices;
                //triangles = CreateBoxTriangles();
                this.transformedVertices = new FlatVector[vertices.Length];
                
           
            }

            else
            {
                vertices = null;
                transformedVertices = null;
              //  triangles = null;
            }

            aabbUpdateRequired = true;
            transformUpdateRequired = true;

        }

        private  static FlatVector[] CreateBoxVertices(float width, float height)
        {
            float left = -width / 2f;
            float right = left + width;
            float bottom = -height / 2f;
            float top = bottom + height;

            FlatVector[] vertices = new FlatVector[4];
            vertices[0] = new FlatVector(left, bottom);
            vertices[1] = new FlatVector(left, top);
            vertices[2] = new FlatVector(right, top);
            vertices[3] = new FlatVector(right, bottom);

            return vertices;
        }

        private static int[] CreateBoxTriangles()
        {
            int[] triangles = new int[6];
            triangles[0] = 0;
            triangles[1] = 0;
            triangles[2] = 2;
            triangles[3] = 0;
            triangles[4] = 2;
            triangles[5] = 3;

            return triangles;
        }

        public FlatVector[] GetTransformedVertices()
        {
            if (transformUpdateRequired)
            { 
                FlatTransform Transform = new FlatTransform(position,angle);

                for (int i = 0; i < vertices.Length; i++)
                { 
                    FlatVector v = vertices[i];
                    this.transformedVertices[i] = FlatVector.Transform(v, Transform);
                }

            }

            this.transformUpdateRequired = false;

            return transformedVertices;
        }

        public void Move(FlatVector amount)
        { 
            this.position += amount;
            this.transformUpdateRequired = true;
            this.aabbUpdateRequired = true;
        }

        public void MoveTo(FlatVector position)
        {
            this.position = position;
            this.transformUpdateRequired = true;
            this.aabbUpdateRequired = true;
        }

        public void Rotate(float amount)
        { 
            this.angle += amount;
            this.transformUpdateRequired = true;
            this.aabbUpdateRequired = true;
        }

        public void RotateTo(float angle)
        {
            this.angle = angle;
            this.transformUpdateRequired = true;
            this.aabbUpdateRequired = true;
        }

        public void AddForce(FlatVector amount)
        {
            force = amount;
        }

        public FlatAABB GetAABB()
        {
            if (aabbUpdateRequired)

            {


                float minX = float.MaxValue;
                float minY = float.MaxValue;
                float maxX = float.MinValue;
                float maxY = float.MinValue;


                if (shapeType is ShapeType.Box)
                {
                    FlatVector[] vertices = GetTransformedVertices();
                    for (int i = 0; i < vertices.Length; i++)
                    {
                        FlatVector v = vertices[i];

                        if (v.X < minX) minX = v.X;
                        if (v.X > maxX) maxX = v.X;
                        if (v.Y < minY) minY = v.Y;
                        if (v.Y > maxY) maxY = v.Y;

                    }


                }
                else if (this.shapeType is ShapeType.Circle)
                {
                    minX = position.X - radius;
                    minY = position.Y - radius;
                    maxX = position.X + radius;
                    maxY = position.Y + radius;
                }
                else
                {
                    throw new Exception("Unknown shapeType.");
                }

                return aabb = new FlatAABB(minX, minY, maxX, maxY);
            }

            aabbUpdateRequired = false; 

            return aabb;
        }

        internal void Step(float time,FlatVector gravity, int iterations)
        {
            if (this.isStatic) return;


            //  FlatVector acceleration = force / mass;

            //  this.linearVelocity += acceleration * time;

            time /= (float)iterations;

            this.linearVelocity += gravity * time;
            position += this.linearVelocity * time;
            angle += this.angularVelocity * time;

            this.force = FlatVector.Zero;

            this.transformUpdateRequired = true;
            this.aabbUpdateRequired = true;

     

        }

        public static bool CreateCircleBody(float radius, float density, bool isStatic, float restitution, out FlatBody body, out string errorMessage)
        {
            body = null;
            errorMessage = string.Empty;

            float area = radius * radius * MathF.PI;

            if (area < FlatWorld.MinBodySize)
            {
                errorMessage = $"Circle radius is too small. Min circle area is {FlatWorld.MinBodySize}.";
                return false;
            }

            if (area > FlatWorld.MaxBodySize)
            {
                errorMessage = $"Circle radius is too Big. Max circle area is {FlatWorld.MaxBodySize}.";
                return false;
            }

            if (density < FlatWorld.MinDensity)
            {
                errorMessage = $"density is too small. Min density is {FlatWorld.MinDensity}.";
                return false;
            }

            if (density > FlatWorld.MaxDensity)
            {
                errorMessage = $"density is too large. Max density is {FlatWorld.MaxDensity}.";
                return false;
            }

            restitution = FlatMath.Clamp(restitution, 0f, 1f);

            float mass = 0f;
            float inertia = 0f;

            if (!isStatic)
            {

                mass = area * density;
                inertia = (1f / 2) * mass * radius * radius;
            }


            body = new FlatBody(density, mass, inertia, restitution, area, isStatic, radius, 0f, 0f, null, ShapeType.Circle);

            return true;

        }

        public static bool CreateBoxBody(float width, float height, float density, bool isStatic, float restitution, out FlatBody body, out string errorMessage)
        {
            body = null;
            errorMessage = string.Empty;

            float area = width * height ;

            if (area < FlatWorld.MinBodySize)
            {
                errorMessage = $"area is too small. Min circle area is {FlatWorld.MinBodySize}.";
                return false;
            }

            if (area > FlatWorld.MaxBodySize)
            {
                errorMessage = $"area is too Big. Max circle area is {FlatWorld.MaxBodySize}.";
                return false;
            }

            if (density < FlatWorld.MinDensity)
            {
                errorMessage = $"density is too small. Min density is {FlatWorld.MinDensity}.";
                return false;
            }

            if (density > FlatWorld.MaxDensity)
            {
                errorMessage = $"density is too large. Max density is {FlatWorld.MaxDensity}.";
                return false;
            }

            restitution = FlatMath.Clamp(restitution, 0f, 1f);

            float mass = 0f;
            float inertia = 0f;

            if (!isStatic)
            {
                mass = area * density;
                inertia = (1f / 12) * mass * (width * width + height * height);
            }

            FlatVector[] vertices = FlatBody.CreateBoxVertices(width, height);

            body = new FlatBody(density, mass, inertia, restitution, area, isStatic, 0f, width, height, vertices,ShapeType.Box);

            return true;

        }

    }
}
