using Flat;
using Flat.Graphics;
using FlatPhysics;
using Microsoft.Xna.Framework;
using System;
using static FlatPhysics.FlatBody;


namespace PhysicsTester
{
    public sealed class FlatEntity
    {
        public readonly FlatBody Body;
        public readonly Color Color;

        public FlatEntity(FlatBody body)
        { 
            this.Body = body;
            Color = RandomHelper.RandomColor();
        }
     
        public FlatEntity(FlatBody body, Color color)
        {
            this.Body = body;
            this.Color = color;
        }

        public FlatEntity(FlatWorld world, float radius, bool isStatic, FlatVector position)
        {
            if (!FlatBody.CreateCircleBody(radius, 1f, isStatic, 0.5f, out FlatBody body, out string errorMessage))
            { 
                throw new Exception(errorMessage);
            }

            body.MoveTo(position);
            this.Body = body;
            world.AddBody(body);
            Color = RandomHelper.RandomColor();
        
        }

        public FlatEntity(FlatWorld world, float width, float height, bool isStatic, FlatVector position)
        {
            if (!FlatBody.CreateBoxBody(width,height, 1f, isStatic, 0.5f, out FlatBody body, out string errorMessage))
            {
                throw new Exception(errorMessage);
            }

            body.MoveTo(position);
            this.Body = body;
            world.AddBody(body);
            Color = RandomHelper.RandomColor();
        }

        public void Draw(Shapes shapes)
        {
            Vector2 position = FlatConverter.ToVector2(Body.Position);

            if (Body.shapeType is ShapeType.Circle)
            {
             //   Vector2 va = Vector2.Zero;
             //   Vector2 vb = new Vector2(Body.radius, 0f);
                Flat.FlatTransform transform = new Flat.FlatTransform(position, Body.Angle);
            //    va = FlatUtil.Transform(va, transform);
             //   vb = FlatUtil.Transform(vb, transform);


                shapes.DrawCircleFill(position, Body.radius, 26, Color);
                shapes.DrawCircle(position,Body.radius,26, Color.White);
               // shapes.DrawLine(va, vb,Color.White);
              
            }
            else if (Body.shapeType is ShapeType.Box)
            {
                shapes.DrawBoxFill(position, Body.width,Body.height, Body.Angle, Color);
                shapes.DrawBox(position, Body.width,Body.height, Body.Angle, Color.White);
            }
        }
    }
}
